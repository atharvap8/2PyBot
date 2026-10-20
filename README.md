![2PyBot: The Python-Powered Martian Rover Platform](assets/2pybot.png)

# 2PyBot: Self-Balancing Robot

[![MCU](https://img.shields.io/badge/MCU-ESP32-blue)](firmware/BaseLink/)
[![Control](https://img.shields.io/badge/Control-LQR%20%2F%20LQI-green)](docs/BaseLink/PID_Theory_and_Math.md)
[![Gamepad](https://img.shields.io/badge/Gamepad-Bluepad32-orange)](firmware/BaseLink/README.md)
[![Firmware CI](https://github.com/atharvap8/2PyBot/actions/workflows/firmware-ci.yml/badge.svg)](https://github.com/atharvap8/2PyBot/actions/workflows/firmware-ci.yml)
[![Python Lint](https://github.com/atharvap8/2PyBot/actions/workflows/python-lint.yml/badge.svg)](https://github.com/atharvap8/2PyBot/actions/workflows/python-lint.yml)
[![Docs Check](https://github.com/atharvap8/2PyBot/actions/workflows/docs-check.yml/badge.svg)](https://github.com/atharvap8/2PyBot/actions/workflows/docs-check.yml)

2PyBot v4.0.1 is an ESP32 self-balancing robot with two M0601 RS485 BLDC hub motors, integrated wheel feedback, and an ISM6HG256X IMU. The firmware uses **LQR/LQI state feedback** for balance and position hold. An EVOFOX One S gamepad pairs directly through Bluepad32. USB serial carries host drive commands, telemetry, and parameter tuning. See the [BLDC integration notes](docs/BLDC_V4.md) for wiring, polarity, full-stick control and turning changes.

## Current firmware

- 200 Hz control loop using pitch, pitch rate, wheel position, velocity, and position integral.
- One RS485 UART per wheel, with current control and absolute-position feedback.
- Encoder-differential heading hold and slew-limited steering.
- Normal hold, stiff hold, velocity tracking, and climb reference tracking.
- Terrain roughness, airborne, and landing detection with pitch-gain softening.
- Metadata-driven runtime parameters with bounded writes and separate BLDC NVS storage.
- Auto-tuning routines for wobble, trim, radius, and stall speed, with current limitations documented in the tuning guide.
- WS2812 status ring, camera pan/zoom servos, and PWM torch.

## Control and communication

The gamepad's left stick drives and steers. The right stick pans and zooms the camera. START requests arming; SELECT stops balancing. LB toggles LOW/HIGH speed, RB toggles the torch, and the D-pad controls hold modes and torch brightness.

Fresh host `V,<forward>,<steering>,<enable>` commands take priority over gamepad driving. `V` commands do not arm the robot. Use gamepad START or serial `E`. With no fresh input, drive commands become zero and balancing continues.

USB serial runs at **460800 baud**. The firmware emits `O` odometry and `D` debug telemetry at nominal 50 Hz, plus `HM` motor feedback at 10 Hz. Debug starts enabled; `L` toggles it. See the [serial protocol](firmware/BaseLink/PROTOCOL.md) for fields and commands.

## Repository layout

```text
firmware/
  BaseLink/                 Robot firmware and serial protocol
    BaseLink.ino            State machine, control, input arbitration
    config.h                Pins and compiled defaults
    params.h / params.cpp   Runtime parameters and NVS persistence
    autotune.h              Calibration and tuning routines
    terrain.h               Roughness, airborne, and landing detection
    bt_gamepad.h            Bluepad32 input mapping
    imu_sensor.h / .cpp      Mahony attitude estimation
    m0601.h / .cpp           RS485 framing, CRC and reply ownership
    hub_drive.h / .cpp       Two hub motors, feedback and mode handshake
    hub_control.h           Balance-priority common/turn current control
    led_ring.h / .cpp        WS2812 status display
    payload.h               Camera servos and torch
  Controller/               Legacy ESP-NOW joystick transmitter
software/
  android/                  Kotlin/Compose Android app
  radxa/
    console/                Current browser console and Android API
    deployment/             Installed services, MediaMTX, and device rules
  runtime/vision_controller/ Older collision guard and person follower
gui/                        Legacy PID desktop interface
tools/
  controller_lab/           Browser simulation lab
  tuner/                    Legacy Web Serial PID tuner
docs/                       Architecture, control theory, tuning, history
scripts/                    Model generation and Radxa utilities
hardware/cad/enclosure/rev5/ OpenSCAD enclosure source and STL exports
models/                     CAD assets
assets/                     Project media
```

## Build and first connection

1. Install the ESP32 + Bluepad32 board package and the libraries listed in the [firmware setup guide](firmware/BaseLink/README.md).
2. Select ESP32 Dev Module with the Huge APP partition scheme.
3. Open `firmware/BaseLink/BaseLink.ino`, review the pin assignments in `config.h`, and flash.
4. Open USB serial at 460800 baud with newline termination. Check the IMU and driver boot messages, then send `P?` to inspect loaded parameters.
5. Pair the EVOFOX One S in Home+B mode. Leave both sticks centered while calibration completes.
6. Use the [v4 hardware guide](docs/BLDC_V4.md) for the BLDC pin map, confirmed polarity, parameter schema and offline verification.

Saved v4 NVS parameters take precedence over compiled defaults in `2pybot-bldc4`. `PD` restores defaults in RAM; `PS` saves them with motors stopped. `ISGN` and `RPMSGN` are fixed at -1; `HUBVSRC` defaults to 1. Full-stick HIGH requests 1.00 m/s; `MAXSPD=0` leaves firmware speed uncapped. `VNOLOAD` is informational, not a driving limit.

## Host software status

| Component | Current compatibility |
| :--- | :--- |
| USB terminal or custom serial client | Supports the current parameter and telemetry protocol |
| `software/radxa/console/` | Current Radxa console: 460800 baud, live parameter metadata, browser UI, Android API, and WebRTC camera |
| `software/android/` | Kotlin/Compose client for console HTTP/WebSocket and MediaMTX WHEP; parameter groups are dynamic |
| `software/runtime/vision_controller/radxa_brain.py` | Uses `O` and `V` messages, but still has 115200 baud and 0.035 m radius constants; align both before use |
| `gui/robot_controller_ui.py` | Legacy PID UI: 115200 baud, tab-separated telemetry, and `$` commands; needs a protocol update |
| `tools/tuner/tuner.html` | Legacy Web Serial PID tuner; not a current parameter-store client |
| `firmware/Controller/Controller.ino` | Legacy ESP-NOW transmitter; current BaseLink has no ESP-NOW receiver |

The current console package was imported from the Cubie A7A. See its [setup and API guide](software/radxa/console/README.md) and [deployment layout](software/radxa/README.md). Its debug parser reads the 20 controller/terrain fields plus nine appended fall-prevention fields; a terrain parameter tab remains pending. **Driving view** shows the assembled CAD, measured VL53L5CX returns, trail and optional forward-reference guard, off by default. Live camera is available separately in that view and in Camera.

Local revision **3.9.0** adds a [predictive fall-prevention reference governor](docs/diagnostics/fall-prevention.md) with full recovery feedback and balance-priority steering allocation. Its read-only monitor is `/ui/drive-monitor.html`; a repeat physical reversal test is needed to measure fall reduction.
Local revision **3.10.0** adds [automatic load-aware climb](docs/diagnostics/auto-climb.md), gradual controller handover and a fix for mild-recovery integral washout. Automatic/manual controls are at `/ui/auto-climb.html`; the recorder includes actual climb-integral and detector state.
Current development revision **4.0.0** replaces the stepper drive with [M0601 BLDC hubs](docs/BLDC_V4.md). It retains the [adaptive Android cockpit](docs/ANDROID_COCKPIT_DESIGN.md) and [route autopilot](docs/ROUTE_AUTOPILOT.md). [Compiled-build reflashing](docs/FIRMWARE_REFLASH.md) is under **More → Setup → Controller firmware**; its tuning-restore path requires a matching hardware schema. [Camera stabilisation](docs/CAMERA_STABILISATION.md) covers the proposed pitch gimbal and isolated sensor pod.

[Boot coordination](scripts/radxa/BOOT_COORDINATION.md) holds the ESP32's USB reset until the Radxa camera, API and hotspot are ready, then releases a firmware startup interlock. A deliberate local START request is still required. [Printed mass and LQI calculations](hardware/dynamics/README.md) contain re-sliced estimates and updated estimated gains validated against geometry, filtering, latency and acceleration limits.

The older Radxa brain provides GUARD/FOLLOW modes. It owns the camera and serial port, so it cannot run alongside the console. See its [setup guide](software/runtime/vision_controller/README_RADXA.md).

The [Android app](software/android/README.md) has Drive, Live, Map, Tune, Auto, Camera, Data and Setup screens. It defaults to the Radxa hotspot address `10.42.0.1`. Drive uses a touch-navigable shaded world; Data records labelled terrain sessions; Map and Setup export reports. Its Tune screen includes all groups reported by firmware, including TERRAIN. See [terrain sessions and Drive](docs/terrain-sessions.md) for current behaviour.

## Next vision milestones

The [Objects viewer and SLAM notes](docs/ML_AND_SLAM.md) describe the lightweight
ONNX detector, temporary tracks, and the calibration/telemetry needed for later mapping.

The [guided camera calibration suite](docs/CAMERA_CALIBRATION.md) provides
target printing, automatic capture/analysis and corrected previews, with depth
health and frame timing. It is available in the console's Calibration tab and
Android Camera screen. Real target views are needed to obtain actual coefficients.

The [vision roadmap](docs/VISION_ROADMAP.md) tracks camera distortion/intrinsic
calibration, sensor extrinsics, synchronized camera/ToF distance overlays and
later ONNX detection/tracking. Camera geometric calibration is separate from
the existing colour/exposure tuning. The first proposed milestone is a measured
intrinsic calibration and validated undistorted preview.

## Revision Table

The [Revision Table](docs/REVISION_HISTORY.md) covers the main releases and
features, along with estimated dates for upcoming work. Smaller fixes are grouped
together. See the [vision roadmap](docs/VISION_ROADMAP.md) for the detailed task list.

## Updating the bot

See the [one-pull development SOP](docs/DEVELOPMENT_SOP.md) for coordinated Git
pulls, versioned host releases, firmware updates and deployment LED states.

The managed source checkout is `/home/radxa/projects/2PyBot`; the console runs from a versioned release snapshot. After one-time setup, `git pull --ff-only` waits for coordinated deployment and reports its outcome. Firmware updates use the existing application-only flash worker when their inputs change. CAD/documentation-only updates leave running services untouched. See the SOP for setup, status and recovery, or [repo-based deployment](scripts/radxa/README.md) for legacy installations.

## Hardware defaults

| Component | Configuration |
| :--- | :--- |
| Robot controller | ESP32 DevKit |
| Wheel drive | Two M0601 BLDC hubs, independent RS485 UARTs, current mode |
| Wheel feedback | Integrated absolute position; internal 4096-count/revolution units |
| Wheel radius | 0.052 m compiled default |
| Attitude | ISM6HG256X accelerometer/gyro, Mahony filter |
| Compass | QMC5883L, tilt-compensated yaw telemetry |
| Gamepad | EVOFOX One S through Bluepad32 |
| Status ring | 16 WS2812 LEDs on GPIO 15 |
| Camera payload | MG90S pan/zoom servos on GPIO 26/0; torch on GPIO 2 |

## Enclosure CAD

[Revision 5](hardware/cad/enclosure/rev5/README.md) includes the base tray, shell, face plate, and left/right motor covers as OpenSCAD source and five STL exports. The enclosure is **not yet printed or physically fit-tested**. Estimated dimensions are marked in the source. The imported face STL has three zero-area triangles that need re-export or repair before printing.

The existing simplified STEP model remains in `models/2pybot_simplified.step`; it is separate from the revision-5 enclosure package.

## Documentation

| Document | Contents |
| :--- | :--- |
| [Firmware setup](firmware/BaseLink/README.md) | Build, wiring, pairing, controls |
| [Serial protocol](firmware/BaseLink/PROTOCOL.md) | Commands, parameter records, telemetry fields |
| [Radxa console](software/radxa/console/README.md) | Browser UI, Android API, camera pipeline, deployment |
| [Android app](software/android/README.md) | Build, host connection, screens, parameter and video transport |
| [Vision roadmap](docs/VISION_ROADMAP.md) | Planned lens calibration, sensor geometry, depth fusion and ONNX perception |
| [Camera calibration](docs/CAMERA_CALIBRATION.md) | Guided capture, lens fitting, reports, depth health and frame timing |
| [Terrain sessions and Drive](docs/terrain-sessions.md) | Recording, 3D world coordinates, guard and range calibration |
| [Radxa deployment](scripts/radxa/README.md) | Git updates, selective restarts, systemd timer, health checks |
| [Enclosure CAD](hardware/cad/enclosure/rev5/README.md) | Revision-5 source, exports, dimensions, and current print status |
| [System architecture](docs/BaseLink/System_Architecture.md) | Sensors, control, actuation, host links |
| [LQR/LQI theory](docs/BaseLink/PID_Theory_and_Math.md) | State feedback, control modes, filtering, limits |
| [Configuration and tuning](docs/BaseLink/Config_and_Tuning_Guide.md) | Defaults, runtime settings, calibration |
| [Program flow](docs/BaseLink/Program_Flow_State_Machine.md) | Boot sequence and state transitions |
| [Troubleshooting](docs/BaseLink/Troubleshooting_Guide.md) | Common faults and source-level checks |
| [Detailed reference](docs/BaseLink/100+_WHAT_AND_HOWS.md) | 126 code-mapped checks |
| [TMC2226 migration](docs/BaseLink/TMC2226_Migration.md) | Driver configuration and migration history |
| [Project history](docs/BaseLink/Project_History.md) | PID, ESP-NOW, LQR/LQI, and runtime-tuning milestones |
| [Legacy transmitter](docs/Controller/System_Architecture.md) | ESP-NOW joystick packet and sampling |
