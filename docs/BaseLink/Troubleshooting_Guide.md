# BaseLink Troubleshooting

Use USB serial at **460800 baud**. Start with `P?` and the boot logs. Source references below are in [firmware/BaseLink](../../firmware/BaseLink/).

## No serial data or garbled output

Confirm baud, port, newline handling, and exclusive ownership of the port. The current Radxa console uses 460800 baud and comma-separated protocol records. Check `2pybot-console.service` and `/api/status` for connection state.

The old desktop GUI expects 115200 baud, tab-separated data, and `$` commands, so its parser and controls do not match this firmware.

`radxa_brain.py` also uses 115200 baud. Its baud and wheel-radius conversion must be aligned before its motion gate can use current odometry.

## IMU initialization fails

Check SDA 21, SCL 22, common ground, sensor power, pull-ups, and the ISM6HG256X library. `IMUSensor::begin()` returns false if sensor initialization or accelerometer/gyro configuration fails. The sketch then halts setup.

`QMC5883L init called` is a log message, not a verified compass-detection result.

## Driver reports COMM ERROR

`setupDriver()` tests connection status and an IFCNT increment after one write. Check the driver address straps, PDN_UART routing, and the 1 kOhm TX resistor.

| Driver | UART | RX / TX |
| :--- | :--- | :--- |
| Right | Serial2 | 16 / 17 |
| Left | Serial1 | 18 / 19 |

Both modules use address 0 with MS1/MS2 LOW. Confirm `OK` and 1/8 microstep readback for both. A per-driver error does not halt setup: `StepperControl::begin()` returns true after initialization.

## START or E leaves the robot idle

`motorsRequested` can be set while the robot remains idle. Arming waits for `abs(pitchF) < ARMANG`, default 5 degrees. Check pitch signs and TRIM.

On connection, sticks must pass center calibration before buttons are processed. The first processed button state is adopted, so a held START does not count as a new press. Release it, then press again.

Host `V,...,1` does not arm. Use START or `E`. There is no `STATE_FALLEN`; a fall returns to idle and requires a new request.

## Immediate fall or rapid acceleration

Check sensor, encoder, and motor direction before changing gains:

- Forward tilt: positive `pitchF`.
- Forward tilt rate: positive `rateF`.
- Forward wheel movement: positive controller `velF`.
- Both motor directions: consistent forward travel.

Runtime sign keys are `PITCHSGN`, `RATESGN`, `ENCSGN`, `ACCSGN`, and `GYROSGN`. Motor direction flags remain in `config.h`. Use +1 or -1, not intermediate sign values.

## Oscillation or noisy recovery

Inspect `pitchF`, `rateF`, `satA`, `satV`, and `loopMaxUs`. Check mechanics, motor current, and IMU filter delay before tuning K3/K4. `IMUCUT` defaults to 50 Hz; a lower cutoff adds delay.

Compare with terrain adaptation disabled through `P,TERREN,0`. Wobble tuning reduces K3/K4 while measuring pitch RMS; it is not a full LQR gain redesign. There is no `PID_D_FILTER_ALPHA` setting in current firmware.

## Position drift or unexpected odometry scale

Check encoder counts from both wheels, loaded `WHEELR`, and `ENCSGN`. The compiled radius is 0.050 m. Saved NVS may contain another value. Position feedback comes from encoders, not generated steps.

Normal hold uses K1/K5 and a bounded position integral. Normal driving resets that integral. Near velocity saturation, the hold point moves toward the robot by design.

## Robot turns while trying to hold heading

Heading feedback uses right-minus-left encoder counts. Check encoder wiring/signs, `YAWKP`, `YAWKD`, `YAWMAX`, and differential-rate noise. `YAWRELCH` and `YAWTILT` re-latch heading after large error or tilt.

Compass offsets affect yaw telemetry, not the steering controller. Legacy `$INV_Y` and `$EN_Y` commands are not supported.

## Motors click, skip steps, or overheat

Check supply voltage, mechanical drag, current, and requested wheel rates. `MAXACC` limits common acceleration; `MAXSPD` limits common velocity. Steering can increase one wheel's rate beyond the common rate.

Use `P,CURRENT,value` to update both the parameter store and hardware. `M=value` only writes hardware. The current parameter bounds are 300 through 2000 mA; the actual motor rating still determines the usable value.

## Saved driver tuning does not return after reset

Boot uses `P.motorMa`, but StallGuard/CoolStep setup still reads compiled defaults. The first parameter hardware callback is skipped. After driver initialization, `PL` applies the saved driver values.

`CSSEMIN=0` disables CoolStep at runtime. Optional DIAG reporting requires GPIO 34/35 wiring and `USE_DIAG_PINS=1`. DIAG events are report-only and do not stop balancing.

## Gamepad input has no effect

Check connection/calibration logs and `RADXATMO`. A host that continues sending `V` commands owns drive priority even if both requested axes are zero. Stop the host's transmission and wait for freshness to expire.

Axis direction uses `PADFWD` and `PADSTR`. The active deadzone key is `STICKDZ`; `PADDZ` does not affect the radial mapping. Keep `STICKDZ > 0` and `SNAPOUT > SNAPIN` to avoid zero denominators.

## Camera travel or torch setting looks wrong

Pan/zoom pins are **26/0**, as defined in `config.h`; older source comments mention other pins. Requested servo positions can exceed 180 degrees, but `_servoWriteDeg()` clamps physical output to 180.

`SVIDLE` can stop servo pulses after inactivity. `SVRATE` is the software write rate, not the PWM frequency. `TRCHBOOT` does not initialize `_torchPct` in current code, and its boot print uses an incorrect integer format for a floating-point field. Inspect torch state through `S` or an explicit `B=value` command.

## Auto-tune will not finish or stop

- Radius: the busy check intercepts `AT,done`; use manual radius calibration until the parser is corrected.
- Stall: the routine controls motors while the balance state is idle. `X` and SELECT do not abort it; use `AT,stop`.
- Wobble/trim: leaving balancing or moving the drive input beyond 0.05 aborts the routine.
- Completion does not save NVS. Use `PS` only after reviewing values.

## Debug stream is too frequent

`DEBUG_STREAM` starts true. `DEBUG_PERIOD_MS=10` requests a nominal 100 Hz, not the older 10 Hz comment. Send `L` to toggle it. Odometry remains at 50 Hz. Logs are event-driven and are not disabled by `L`.

For more source-level checks, see [the detailed reference](100+_WHAT_AND_HOWS.md).
