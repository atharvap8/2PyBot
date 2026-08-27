# TMC2226 Driver Migration

The BaseLink actuation layer moved from TMC2208 to TMC2226 stepper drivers. This document records why the swap happened, what changed in the wiring and the firmware, how the drivers are configured now, and how to verify a board after flashing.

## 1. Why the swap

Two problems drove the change.

First, the TMC2208 UART never worked on this robot. Every configured value (current, microsteps, chopper mode) silently failed to reach the chips, and the drivers ran on their MS1/MS2 pin strapping and the Vref potentiometer instead. Encoder logs from 18 August proved the real microstepping was 1/8, not the 1/16 the firmware assumed.

Second, the motors stalled above roughly 9000 pulses per second at 1100 mA, which forced a conservative speed cap of 6000 steps per second. The TMC2208 offered no way to observe stall onset and little current headroom to push the ceiling higher.

The TMC2226 addresses both. It is a TMC2209 family chip in a better thermal package, rated for 2.0 A RMS continuous, with StallGuard4 load measurement and CoolStep adaptive current. The UART link was repaired as part of the migration and is now verified on every boot.

| Property | TMC2208 (old) | TMC2226 (new) |
|:---|:---|:---|
| Continuous drive current | about 1.4 A RMS | 2.0 A RMS |
| Stall and load telemetry | none | StallGuard4 (SG_RESULT register, DIAG pin) |
| Adaptive current scaling | none | CoolStep |
| UART node addressing | single fixed address | 2 bit address on MS1/MS2, up to 4 chips per bus |
| Standalone microstep strapping, both pins LOW | 1/8 | 1/8 (identical, so the fallback behavior is unchanged) |
| Maximum motor supply | 36 V | 29 V (irrelevant on the 3S pack, but it rules out a future 24 V upgrade with spikes) |
| TMCStepper class | `TMC2208Stepper` | `TMC2209Stepper` (register compatible) |

## 2. What stayed the same

- The STEP, DIR, and EN pin map in `config.h` is unchanged. Enable is still active LOW.
- The 20 kHz timer ISR and the Bresenham step generation are untouched.
- One dedicated hardware UART per driver: `Serial2` on pins 16/17 for the right motor, `Serial1` on pins 18/19 for the left, each with a 1 kOhm inline resistor on the TX line.
- `R_SENSE` stays 0.11 ohm.
- Microstepping stays at 1/8, so `STEPS_PER_M`, the encoder calibration, and every tuned LQR gain carried over without change.
- MT6816 encoder odometry, the safety limits, and the control law are untouched.

## 3. What changed in the firmware

### 3.1 Driver bindings
`stepper_control.h` now declares the drivers as `TMC2209Stepper` (TMCStepper drives the TMC2226 through its TMC2209 support) and constructs them with `DRV_UART_ADDR` (0b00). MS1 and MS2 are the UART address pins on this chip. Both modules strap them LOW, which selects address 0 and also keeps the standalone fallback at 1/8 microstepping, so no rewiring was needed.

### 3.2 Boot sequence with write verification
`setupDriver()` performs a full bring-up over UART: it disables the PDN function on the UART pin, switches microstep selection from the MS pins to the MRES register, switches current control from Vref to the IRUN/IHOLD registers, and programs current, microsteps, interpolation, and the chopper. It then runs two checks per driver: `test_connection()` must return 0, and the IFCNT transmission counter must advance by exactly one after a counted verification write. A healthy boot prints:

```
[STEP] RIGHT TMC2226: OK | 1500 mA, 1/8 usteps (readback 1/8), StealthChop, SGTHRS=77, CoolStep ON
```

A failure prints `TMC2226: COMM ERROR (conn=..., IFCNT delta=...)` and `BaseLink.ino` follows with `[MAIN] WARNING: TMC2226 UART unresponsive`.

### 3.3 Operating mode: StealthChop
The drivers now run StealthChop at all speeds (`TPWMTHRS 0`) because StallGuard4 and CoolStep require it on this chip family. The stall ceiling was measured in both chopper modes and came out identical, about 9000 microsteps per second at 1100 mA, so no top speed was lost. Setting `DRV_STEALTHCHOP 0` reverts to spreadCycle in one line, at the cost of StallGuard and CoolStep going dead.

### 3.4 StallGuard4
Thresholds were tuned on this hardware with the test sketch: `SGTHRS_LEFT 76` and `SGTHRS_RIGHT 77`. A stall flags when SG_RESULT drops below twice the threshold. Readings are only valid above `DRV_TCOOLTHRS` (300, which corresponds to roughly 1250 microsteps per second), so slow stalls are invisible to StallGuard by design. Optionally, the DIAG outputs can be wired to GPIO 34 (left) and GPIO 35 (right) and enabled with `USE_DIAG_PINS 1`. The main loop then prints report-only `[STALL]` counts. It never disables the motors automatically, because a false positive during a balance recovery would drop the robot.

### 3.5 CoolStep
CoolStep is enabled (`SEMIN 5`, `SEMAX 2`) with a current floor of half of IRUN. It is inactive below `DRV_TCOOLTHRS`, so standstill balancing always has full current. If balance ever feels soft against pushes while driving, set `COOLSTEP_ENABLE 0`.

### 3.6 Current and speed limits
Run current rose from 1100 mA to 1500 mA (`MOTOR_CURRENT_MA`), and the `M=` serial command still changes it live over UART. IHOLD is set to roughly half current at true standstill, which only takes effect when the robot is disarmed, since balancing steps continuously. The stall test confirmed the roughly 9000 microsteps per second ceiling is set by the motor and supply voltage, not the driver, so `MAX_SPEED_STEPS` was raised from 6000 to 8500 and `MAX_DRIVE_VEL_MS` from 0.55 to 0.70. Rerun the stall test after any current change before raising the cap further.

## 4. Configuration reference

| Constant | Value | Meaning |
|:---|:---|:---|
| `DRV_UART_ADDR` | `0b00` | UART address, MS1/MS2 LOW on both modules |
| `DRV_STEALTHCHOP` | 1 | StealthChop on; 0 reverts to spreadCycle |
| `SGTHRS_LEFT` / `SGTHRS_RIGHT` | 76 / 77 | StallGuard4 thresholds, tuned on this hardware |
| `DRV_TCOOLTHRS` | 300 | TSTEP threshold; StallGuard and CoolStep active above about 1250 usteps/s |
| `COOLSTEP_ENABLE` | 1 | CoolStep on, floor is IRUN/2 |
| `COOLSTEP_SEMIN` / `COOLSTEP_SEMAX` | 5 / 2 | CoolStep hysteresis window |
| `USE_DIAG_PINS` | 0 | Set 1 after wiring DIAG to GPIO 34/35 for `[STALL]` reporting |
| `MOTOR_CURRENT_MA` | 1500 | Run current; silicon limit 2000 mA RMS, bounded by the motor rating |
| `MICROSTEPS` | 8 | Programmed over UART, interpolated internally to 1/256 |
| `MAX_SPEED_STEPS` | 8500 | Raised from 6000 after the stall ceiling was confirmed |

## 5. Verification checklist after flashing

1. The boot log must show `TMC2226: OK` for both drivers, with a microstep readback of 1/8. This proves the UART writes land.
2. Send `M=1500`, then `S`. The reported current must reflect the change.
3. With the wheels off the ground, drive gently and confirm the debug value `v` equals `vCmd / STEPS_PER_M`. This catches any microstep or address mismatch.
4. Direction conventions are unchanged, but verify the sign checks in the `config.h` sign conventions section after any rewiring.
5. Run the stall test before raising `MOTOR_CURRENT_MA` or `MAX_SPEED_STEPS`, and give the drivers a thermal soak at balancing load after any current increase.
