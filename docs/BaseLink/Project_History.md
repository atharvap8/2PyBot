# Project History

This records earlier designs and the changes visible in the repository history. Current behavior is described in [System Architecture](System_Architecture.md).

## Timer-driven actuation

The project moved step generation out of the main loop into a 20 kHz hardware timer ISR. Bresenham accumulators distribute STEP pulses for each requested wheel rate. Encoder feedback uses MT6816 A/B signals decoded by ESP32 PCNT hardware.

## IMU estimation and PID control

Earlier firmware used pitch PID control, separate heading/position options, Bluetooth serial tuning, and tab-separated telemetry. The desktop GUI and browser tuner still reflect that interface.

Sensor estimation moved to Mahony quaternion integration and cascaded pitch filtering. The current implementation uses accelerometer/gyro correction; the QMC5883L supplies separate tilt-compensated yaw telemetry.

## ESP-NOW joystick transmitter

`firmware/Controller/Controller.ino` reads an analog joystick and sends a packed forward/steering/enable packet at 50 Hz. An EvoFox bridge sketch was also added in the `feature/evofox_test` work and later removed from the active LQR branch.

The analog transmitter remains in the repository, but current BaseLink does not receive ESP-NOW packets.

## LQR/LQI balancing

The `system/LQR_Controller` work replaced pitch PID with full-state feedback and a position integral. Acceleration is integrated into wheel velocity rather than mapping pitch error directly to speed.

The controller added encoder position hold, velocity tracking, stiff hold, and climb reference tracking. Heading control moved to encoder differential. The branch was merged into `main` and tagged `v2.0.0`.

## Direct Bluetooth gamepad and status ring

Bluepad32 replaced the robot's ESP-NOW input path. The EVOFOX One S pairs directly to the robot, removing the bridge board. NeoPixelBus drives a 16-LED WS2812 ring through RMT for arm, fall, drive, hold, and gesture feedback.

## TMC2226 drivers

TMC2226 replaced TMC2208 through the register-compatible `TMC2209Stepper` class. Separate driver UARTs, address straps, connection tests, and an IFCNT write check made configuration visible at boot.

Historical hardware tests recorded a speed ceiling near 9000 microsteps/s at 1100 mA in both chopper modes. Current compiled defaults are 1500 mA, 1/8 microsteps, StealthChop, StallGuard4 thresholds 76/77, and a common speed cap of 8500 microsteps/s. See [TMC2226 Migration](TMC2226_Migration.md).

## Camera payload and motion I/O

Commit `cbdee62` added camera pan/zoom and torch controls. The left stick drives and steers; the right stick controls the payload. LB became a speed toggle, RB controls the torch, and D-pad buttons select hold modes or adjust brightness.

The shared motor enable moved both drivers onto GPIO 32. Current pan/zoom pins are GPIO 26/0. The wheel-radius and normal-gain update was committed as `e0c6554`.

## Runtime tuning and terrain adaptation

Commit `a8427f0` on `system/control-stack-v3` added:

- A shared 106-parameter store with bounds, metadata, derived conversions, and NVS persistence.
- Live settings for control, driver tuning, IMU, gamepad shaping, ring, and payload.
- Wobble, trim, radius, and stall auto-tune routines.
- Roughness, airborne, and landing detection with pitch-gain softening.
- Explicit START/SELECT events and synchronization after an automatic stop.
- Removal of arming from host `V` enable transitions.
- Terrain fields appended to comma-separated debug telemetry.

Radius completion, stall abort handling, saved driver tuning at boot, and legacy host-tool compatibility remain implementation gaps documented in the [tuning guide](Config_and_Tuning_Guide.md) and [protocol](../../firmware/BaseLink/PROTOCOL.md).

## Separate development branches

ROS workspace, ROS-hosted balancing, SLAM placeholder, and predictive fall-protection experiments exist on separate branches. They are not part of the current BaseLink architecture on `system/control-stack-v3`.
