# TMC2226 Driver Migration and Current Setup

Reference: [stepper_control.cpp](../../firmware/BaseLink/stepper_control.cpp) and [config.h](../../firmware/BaseLink/config.h).

## Migration background

Earlier TMC2208 modules ran with an unreliable UART link, leaving current and microstep settings dependent on Vref and pin straps. Historical encoder tests showed 1/8 microsteps and a motor speed ceiling near 9000 microsteps/s at 1100 mA.

The migration repaired UART communication and moved to TMC2226, a TMC2209-register-compatible driver with StallGuard4 and CoolStep. Historical tests recorded similar speed ceilings in StealthChop and spreadCycle. Those measurements describe that hardware setup, not a guaranteed limit for another motor or supply.

## Current wiring

| Signal | Right | Left |
| :--- | :--- | :--- |
| STEP | GPIO 33 | GPIO 27 |
| DIR | GPIO 25 | GPIO 14 |
| EN | GPIO 32, shared active LOW | GPIO 32, shared active LOW |
| UART | Serial2 | Serial1 |
| UART RX / TX | GPIO 16 / 17 | GPIO 18 / 19 |
| Optional DIAG | GPIO 35 | GPIO 34 |

Both drivers use UART address 0 with MS1/MS2 LOW. Each TX line uses a 1 kOhm series resistor for the single-wire UART connection. `R_SENSE` is 0.11 ohm. The old separate left enable on GPIO 26 is no longer present; GPIO 26 is camera pan.

## Boot configuration

`setupDriver()` uses `TMC2209Stepper` for both TMC2226s. It:

1. Clears status flags and selects UART configuration for current and microsteps.
2. Applies loaded `P.motorMa`, default 1500 mA.
3. Programs 1/8 microsteps with internal interpolation enabled.
4. Sets standstill-current and power-down registers.
5. Selects StealthChop at all speeds by default.
6. Programs compile-time StallGuard and CoolStep settings.
7. Checks `test_connection()==0` and an IFCNT increment of one after a verification write.

Example at compiled defaults:

```text
[STEP] RIGHT TMC2226: OK | 1500 mA, 1/8 usteps (readback 1/8), StealthChop, SGTHRS=77, CoolStep ON
```

`COMM ERROR` reports a failed connection/write check. It does not verify every configuration register and does not halt the sketch. `StepperControl::begin()` currently returns true even when a driver check fails.

## Compile-time and runtime settings

| Compiled setting | Default | Runtime key |
| :--- | :--- | :--- |
| `DRV_UART_ADDR` | 0 | None |
| `DRV_STEALTHCHOP` | 1 | None |
| `MICROSTEPS` | 8 | None |
| `MOTOR_CURRENT_MA` | 1500 mA | `CURRENT` |
| `SGTHRS_LEFT` / `SGTHRS_RIGHT` | 76 / 77 | `SGTHRSL` / `SGTHRSR` |
| `DRV_TCOOLTHRS` | 300 | `TCOOL` |
| `COOLSTEP_ENABLE` | 1 | Runtime disable through `CSSEMIN=0` |
| `COOLSTEP_SEMIN` / `COOLSTEP_SEMAX` | 5 / 2 | `CSSEMIN` / `CSSEMAX` |
| `USE_DIAG_PINS` | 0 | None |
| `MAX_SPEED_STEPS` | 8500 microsteps/s | `MAXSPD` |
| `MOTOR_ACCEL_LIMIT` | 20000 microsteps/s² | `MAXACC` |

`applyTuning()` writes SGTHRS, TCOOLTHRS, SEMIN, and SEMAX to both drivers after parameter commands. `CURRENT` also writes RMS current. The first hardware callback is skipped at boot, and initial driver tuning still uses compiled values. Send `PL` after initialization to apply saved StallGuard/CoolStep settings.

Use `P,CURRENT,1500` for a persistent parameter-store change. Legacy `M=1500` only changes hardware current; `S` does not report current. `P?` reports the stored current setting, not driver readback.

## StallGuard and CoolStep

Both features require StealthChop on this driver family. StallGuard compares load measurements against SGTHRS and is speed-dependent through TCOOLTHRS. CoolStep adjusts current with a configured floor of half IRUN when active.

Optional DIAG interrupts count events. The main loop prints changes at most every 250 ms. DIAG is report-only; it does not stop the balance controller. When disarmed, the shared enable line is HIGH and the drivers are disabled, so standstill-current registers do not provide a disarmed hold mode.

## Motion and odometry

The driver UART configures the chips; it does not command wheel motion. A 20 kHz timer ISR emits STEP pulses from signed wheel rates. Motor direction inversion is set in `config.h`.

MT6816 encoders provide actual wheel counts through PCNT. Geometry conversion uses runtime `WHEELR`, now default 0.050 m. Microstep resolution changes require rebuilding and checking the conversion/control assumptions.

## Verification

1. Read each driver's boot result and 1/8 microstep readback.
2. Confirm both enable inputs share GPIO 32 and both motor directions agree on forward motion.
3. Inspect `P?`, then `PL` if saved runtime driver tuning should be applied.
4. Compare measured `velF` with common commanded speed in debug telemetry under steady motion.
5. Check current, speed, and acceleration against the actual motor/supply setup before raising limits.

The current `AT,stall` routine has an independent motor-control path and is not cancelled by `X` or SELECT. Its implemented abort is `AT,stop`; see [Configuration and Tuning](Config_and_Tuning_Guide.md).
