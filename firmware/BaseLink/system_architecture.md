# BaseLink Module Map

The full [system architecture](../../docs/BaseLink/System_Architecture.md) describes control flow and timing. This file maps the current sketch modules.

| File | Responsibility |
| :--- | :--- |
| `BaseLink.ino` | Boot, 200 Hz loop, two-state balance machine, LQR/LQI, steering, serial parsing, drive arbitration, telemetry |
| `config.h` | Pin assignments, sensor mapping, PWM/timer settings, compiled parameter defaults |
| `params.h` | Parameter fields, bounds, keys, groups, metadata, declarations |
| `params.cpp` | Shared `P` instance, NVS persistence, parameter commands, derived conversions |
| `autotune.h` | Wobble, trim, radius, and stall-search state machine |
| `terrain.h` | Roughness filter, airborne/landing states, gain multiplier |
| `imu_sensor.h` / `imu_sensor.cpp` | ISM6HG256X reads, gyro calibration, six-axis Mahony, filtered pitch, compass yaw |
| `stepper_control.h` / `stepper_control.cpp` | TMC2226 UART setup, live driver tuning, PCNT encoder reads, 20 kHz STEP/DIR ISR |
| `bt_gamepad.h` | Bluepad32 pairing, stick shaping/calibration, button events, payload input |
| `led_ring.h` / `led_ring.cpp` | RMT-driven WS2812 status patterns |
| `payload.h` | LEDC pan/zoom servo pulses and torch brightness |
| `PROTOCOL.md` | USB command, parameter, and telemetry reference |

## Interfaces

- `IMUSensor` supplies pitch, gyro pitch rate, acceleration, and yaw.
- `StepperControl` supplies encoder counts and accepts signed left/right step rates.
- `P` supplies the live settings used by control and peripheral code.
- `btgamepad_takeEnableEvent()` supplies START/SELECT events independently of drive priority.
- `terrain_soften()` supplies the pitch-gain multiplier; `terrain_airborne()` gates integral accumulation.
- USB `V` messages supply normalized forward and steering requests. They do not arm the robot.
- [The Radxa console](../../software/radxa/console/README.md) reads parameter/telemetry records and exposes tuning, camera, and stream APIs to browser and Android clients. Its command whitelist does not include `V` or `E`.

There are no active `pid_controller`, `serial_tuner`, `espnow_comm`, or Bluetooth serial modules in this sketch. The legacy desktop GUI and PID browser tuner need protocol changes to support current BaseLink.
