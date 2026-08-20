# BaseLink_LQR — Package B (LQR / LQI)

Self-balancing firmware for the 2PyBot ESP32 using **optimal full-state feedback**: `u = −(K1·(x−xRef) + K2·v + K3·pitch + K4·pitchRate + K5·∫(x−xRef))`, where `u` is wheel acceleration, integrated into the 20 kHz stepper ISR velocity command. The gains were computed offline by solving the continuous algebraic Riccati equation for the linearized velocity-driven pendulum, and the position-integral (LQI) state absorbs the unknown CoG offset — that is what makes the drift disappear exactly, not approximately. Validated in simulation (see REPORT.md): 0.5 cm RMS station keeping under a 0.35° CoG error, 2.3° peak for a 45°/s shove, 0.2 cm steady-state error on a 0.5 m position command, and stable with a ±30 % wrong CoM-height estimate.

## Files

`BaseLink.ino` is the main sketch. `config.h` holds pins, limits and the LQR gain vector with the design record (Q, R, closed-loop poles). `imu_sensor.*`, `stepper_control.*`, `espnow_comm.h` are your original drivers, unchanged. BluetoothSerial and FastLED removed (radio conflict with ESP-NOW; not needed with the Radxa onboard).

## Libraries needed

TMCStepper, ISM6HG256XSensor, QMC5883LCompass. FastLED and BluetoothSerial are no longer required.

## Radxa serial protocol (USB serial, 115200)

Out, 50 Hz, always on: `O,<millis>,<encL>,<encR>,<pitchDeg>,<yawDeg>\n`.

In: `V,<forward>,<steering>,<enable>\n`, forward/steering −1..+1, enable 0/1. Forward maps to target velocity (±0.8 m/s default). Below 3 % stick the robot latches and holds its spot on the encoders (full LQI vector); while driving it runs the velocity-tracking subset K2..K4 (closed-loop poles −16.1, −7.2, −6.1 — verified stable). Radxa has drive authority while fresh (< 600 ms), then the ESP-NOW joystick, then zero.

## Tuner commands

`E` enable, `X` e-stop, `C` cal gyro, `S` settings, `L` debug stream, `R` reset, `?` help. `K1=`..`K5=` set gains live, `T=` pitch trim (deg), `M=` motor current (mA).

## Regenerating gains for different geometry

If your CoM height or wheel radius differ a lot from the design values (L_eff = 0.15 m, R = 42.5 mm), edit those constants at the top of `tools/simulate_controllers.py`, run it, and copy the printed K vector into `config.h`. In practice the design tolerates ±30 % error in L_eff unchanged, so measure the wheel radius carefully and don't stress about the CoM.

## First-time bring-up (same as Package A)

1. Set `WHEEL_RADIUS_M` in config.h to your measured wheel radius.
2. Send `L`; tilt forward by hand: `pF` must go positive, else flip `PITCH_FWD_SIGN`.
3. Motors off, push forward 1 m: O-stream `encL/encR` must increase, else flip `ENC_FWD_SIGN`.
4. Hold upright, send `E`; it arms within 5° of vertical. Keep a finger on `X`.

## Tuning feel guide

LQR rarely needs touching, but if it does: too stiff/buzzy → scale K3 and K4 down together by 10–20 %. Sluggish position return → raise |K1|. Slow residual creep before it corrects → raise |K5| (integral). Change one gain at a time over serial; nothing needs a reflash.
