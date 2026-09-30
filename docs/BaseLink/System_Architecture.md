# BaseLink System Architecture

Reference: [BaseLink firmware](../../firmware/BaseLink/). Balancing runs on the ESP32. The Radxa supplies drive commands; it does not close the balance loop.

## Data flow

```text
ISM6HG256X -> Mahony + pitch filter -> pitch and pitch rate
MT6816 encoders -> PCNT -> position, velocity, encoder difference
Accelerometer magnitude -> terrain estimate -> pitch-gain multiplier
Bluepad32 gamepad + USB V commands -> input arbitration
Measurements + commands + runtime parameters -> LQR/LQI controller
Acceleration -> integrated wheel velocity -> steering split
Left/right step rates -> 20 kHz timer ISR -> TMC2226 STEP/DIR
```

The QMC5883L provides tilt-compensated yaw telemetry. Heading control uses wheel encoder difference, not compass yaw.

## Execution rates

| Task | Default rate | Implementation |
| :--- | :--- | :--- |
| Sensor reads, control, input handling | 200 Hz | `BaseLink.ino::loop()` |
| Step generation | 20 kHz | `StepperControl::tick()` |
| Odometry stream | 50 Hz | `ODOM_PERIOD_MS=20` |
| Debug stream | Nominal 100 Hz | `DEBUG_PERIOD_MS=10`; enabled at boot |
| LED refresh | 50 Hz | `P.ledFps` |
| Servo writes | 50 Hz | `P.svRate`; LEDC pulses remain at 50 Hz |

The loop gates on 5000 microseconds, uses measured elapsed time, and caps `dt` at 0.05 s after stalls. These are scheduling targets, not guarantees of constant latency.

## Sensors and estimation

`imu_sensor.cpp` reads accelerometer and gyro data over 400 kHz I2C. It subtracts calibrated gyro bias, integrates a six-axis Mahony quaternion, and applies two cascaded EMA stages to pitch. When acceleration magnitude differs from 1 g by more than 0.2 g, the proportional correction is set to 0.1.

Compass offsets and signs are applied separately before tilt-compensated yaw calculation. Magnetometer measurements are not fed into the Mahony gravity correction.

Two PCNT units decode encoder A/B edges at four times the encoder pulse resolution. Overflow handlers extend the counters to 64 bits. The right count is negated in `getPositionR()` to match the left wheel convention.

`BaseLink.ino` converts average counts to metres and differentiates them for velocity. Velocity and encoder-difference rate use a 15 Hz EMA.

## Balance and steering

`runController()` selects one of these modes:

| Mode | Controller |
| :--- | :--- |
| Normal hold | Five-state LQI with clamped position error and integral |
| Driving | K2/K3/K4 velocity tracking; integral reset to zero |
| Stiff hold | S1 to S5 gain set while not driving |
| Climb | C1 to C5 gain set tracking a moving position reference |

Acceleration is limited to `P.aMax`, integrated into `vCmd`, and limited to `P.vMax`. Near velocity saturation, the hold reference and integral bleed toward the measured state.

Steering adds opposite rate offsets to the wheels. Active steering is slew-limited. With the steering stick centered, proportional and derivative feedback holds encoder difference. Large encoder error or excessive tilt re-latches heading instead of unwinding a possible slip.

## Terrain adaptation

`terrain.h` filters acceleration-magnitude deviation and a pitch-rate term into roughness. Low acceleration magnitude confirms an airborne state; touchdown starts a landing interval.

The gain multiplier applies to K3/K4 in normal hold and driving, and C3/C4 in climb. Stiff-hold S3/S4 are unchanged. With `AIRFRZ` enabled, airborne detection suppresses integral accumulation in hold and climb. Saturation bleed can still change the integral. Driving always resets it.

## Inputs and arming

Drive priority is fresh USB `V` commands, then fresh gamepad input, then zero. Default freshness windows are 600 ms and 200 ms respectively.

- START generates an arm request; SELECT generates a stop event.
- Serial `E` also requests arming; `X` stops balancing.
- A `V` enable transition from 1 to 0 requests a stop. A transition to 1 does not request arming.
- Arming requires `abs(pitchF) < P.armAngle`, default 5 degrees.
- A fall or rate spike disables balancing and returns to `STATE_IDLE`.
- Gamepad disconnect zeros its axes and stops refreshing its timestamp. It does not disarm balancing.

Gamepad sticks use center calibration, radial deadzone/rescaling, an angle-based steering taper, and exponential response. Buttons held on the first processed connection frame are adopted as the baseline.

## Parameters and persistence

`params.h` defines 106 fields and their defaults, bounds, groups, and descriptions. `params.cpp` owns the single `P` instance. Parameter writes recompute geometry conversions and call the hardware-update hook.

`PS` saves to the `2pybot` NVS namespace. `PL` loads saved values. `PD` restores compiled defaults in RAM. Per-value bounds do not validate relationships such as minimum versus maximum servo travel.

The first hardware-update callback is skipped during boot. Loaded motor current and IMU cutoff are used by initialization; StallGuard and CoolStep initialization still uses compiled defaults. `PL` after initialization applies the loaded driver tuning.

## Actuation and payload

TMC2226 drivers use `TMC2209Stepper`, separate 115200 baud UARTs, address 0, and a shared active-low enable on GPIO 32. Boot checks connection status and an IFCNT increment after one verification write. STEP/DIR pulses come from the timer ISR, not UART commands.

NeoPixelBus drives the WS2812 ring through RMT. LEDC drives pan, zoom, and torch on separate PWM channels. Payload updates continue while idle. The torch hard cap is 90% in the current code, and the physical servo pulse mapping clamps angles to 0 through 180 degrees.

## Host interfaces

The active firmware uses USB `Serial` at 460800 baud. The [Radxa console](../../software/radxa/console/README.md) owns that connection and serves browser/Android clients through HTTP and WebSocket. Its camera pipeline publishes H.264 through MediaMTX for WebRTC/WHEP clients.

There is no `SerialBT` command service or ESP-NOW receiver. See [PROTOCOL.md](../../firmware/BaseLink/PROTOCOL.md) for the wire format and [README.md](../../README.md) for host-tool compatibility.
