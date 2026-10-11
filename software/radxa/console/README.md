# 2PyBot Console

Browser tuning interface, telemetry, camera controls, and HTTP/WebSocket API for the Android app. Imported from the running Cubie A7A package at `/home/radxa/2pybot-console`.

The Kotlin/Compose client is in [software/android](../../android/README.md).

## Components

| Path | Responsibility |
| :--- | :--- |
| `console/server.py` | FastAPI endpoints, WebSocket feed, static browser UI |
| `console/serial_link.py` | Exclusive USB connection, parameter commands, telemetry parsing, dead reckoning |
| `console/config.py` | Serial, HTTP, camera, snapshot, and profile settings |
| `console/camera.py` | V4L2 controls, exposure/gain adjustment, profiles, JPEG snapshots |
| `console/stream.py` | Camera mode selection, encoder startup, progress metrics, process supervision |
| `console/tof.py`, `console/tof_worker.py` | Isolated VL53L5CX reader, 8×8 telemetry, optional forward-reference guard |
| `ui/` | Browser shell, shared parameter renderer, twelve tabs |
| `native/` | Allwinner Cedar H.264 publisher and probe source |
| `systemd/` | Installer templates for console and MediaMTX |

## Firmware connection

Serial defaults to `/dev/ttyUSB0` at **460800 baud** with DTR/RTS disabled and exclusive ownership. The reader requests `P?` after connecting and retries when the port is unavailable.

- `P` rows populate the parameter editor, including bounds, group, and description.
- `PR` rows provide live geometry conversions; odometry uses `COUNTSM` when available, otherwise a 0.050 m radius fallback.
- `P!` acknowledges a write and requests a new dump.
- `PS`, `PL`, and `PD` provide firmware persistence actions.
- `O` records update pose and telemetry.
- `D` records populate the original 15 debug fields plus acceleration magnitude, roughness, softening, terrain state and drop count.

The browser has parameter tabs for BALANCE, STEPPER, DRIVE, YAW, SAFETY, IMU, CLIMB, LED, and PAYLOAD. Firmware TERRAIN parameters are available through the API, but a terrain tab is not registered yet. Map dead reckoning uses the fixed 0.155 m track width from `config.py`.

The API command whitelist excludes `E` and `V`. The gamepad remains connected to the ESP32. `AT,` commands are allowed, so the firmware's current stall-tuning limitations still apply.

## Addresses

| Interface | Address |
| :--- | :--- |
| Browser console and API | `http://<radxa-ip>:8080` |
| WebSocket telemetry | `ws://<radxa-ip>:8080/ws` |
| WebRTC WHEP | `http://<radxa-ip>:8889/cam/whep` |
| Local camera publication | `rtsp://127.0.0.1:8554/cam` |
| MediaMTX administration | `http://127.0.0.1:9997` |

The installed hotspot normally uses `10.42.0.1`. Use your configured hotspot password; no device credentials are stored in this package.

## Install on the Radxa

The current bot uses the repository checkout at `/home/radxa/projects/2PyBot`, with this application under `software/radxa/console`. Use [repo-based deployment](../../../scripts/radxa/README.md) for normal updates and service monitoring.

The standalone installer below is for dependency/hotspot setup on a new board. It is not needed for routine Git updates. Its service templates substitute the application's installation directory.

The installer needs root access and a hotspot password in `PSK`:

```bash
cd /home/radxa/2pybot-console
read -rs -p 'Hotspot password: ' PSK
export PSK
sudo -E bash install.sh
```

It installs Python/stream dependencies, creates `/var/lib/2pybot`, sets dialout/video membership, installs the serial udev rule, configures a NetworkManager hotspot, and installs the two services. MediaMTX is downloaded if not already installed; the captured board version was **v1.9.3**.

The native Cedar publisher is built separately. See [native/README.md](native/README.md) for SDK and development-package requirements. The source `install.sh` does not build that executable or install its encoder rules.

The running board also has a camera startup override. Install the captured override and encoder rule when restoring that setup:

```bash
sudo mkdir -p /etc/systemd/system/2pybot-console.service.d
sudo cp 20-camera-natural.conf /etc/systemd/system/2pybot-console.service.d/
sudo cp native/99-2pybot-encoder.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo systemctl daemon-reload
sudo systemctl restart 2pybot-console
```

The deployed service definitions and live MediaMTX settings are also preserved in [deployment/](../deployment/). Those service definitions contain the original `/home/radxa/2pybot-console` path; the installer templates substitute the actual destination.

## Camera pipeline

For MJPEG modes, the console selects `native/2pybot_hwstream` when present. It decodes JPEG on the CPU, converts pixels, encodes H.264 with Cedar, publishes RTSP, and writes original JPEG snapshots to `/dev/shm/2pybot_snap.jpg`.

Without that executable, it probes FFmpeg hardware encoders and falls back to low-latency libx264. The supervisor restarts failed stream processes. `/api/stream` reports the selected mode, encoder, errors, and progress metrics.

Camera profiles are daylight, indoor, low_light, night, and driving. Built-in exposure adjustments cap absolute exposure at 300 units. User profiles live in `/var/lib/2pybot/camera_profiles.json`; the board's saved copy was downloaded as local runtime data.

## API

| Method and path | Purpose |
| :--- | :--- |
| `GET /api/status` | Connection, telemetry, debug, derived values, stream state, parameter count |
| `GET /api/params` | Parameters, groups, bounds, and dump status |
| `POST /api/params/set` | Send `{key, value}` |
| `POST /api/params/{save,load,defaults,refresh}` | Firmware parameter actions |
| `POST /api/command` | Send a whitelisted `{line}` |
| `POST /api/estop` | Send firmware `X` |
| `GET /api/path`, `POST /api/pose/reset` | Pose trail and local pose reset |
| `GET /api/log` | Recent console logs |
| `GET /api/camera/controls` | V4L2 controls, values, profiles, and camera modes |
| `POST /api/camera/set` | Set `{name, value}` |
| `POST /api/camera/profile/{name}` | Apply a camera profile |
| `POST /api/camera/autocalibrate` | Adjust exposure/gain from snapshot brightness |
| `GET /api/tof`, `POST /api/tof/guard` | Depth/guard status and explicit guard enable/disable |
| `POST /api/tof/calibrate` | Save a range bias from a measured stable central target |
| `GET /api/camera/snapshot`, `GET /api/camera/luma` | JPEG and brightness measurement |
| `GET /api/stream` | Stream state and metrics |
| `POST /api/stream/restart`, `POST /api/stream/stop` | Stream process control |
| `WS /ws` | 10 Hz telemetry, parameter acknowledgements, and logs |

`/api/camera/profile/save` precedes the generic profile route, so saved profiles
are handled by the correct endpoint. CSI profiles and controls use the native
publisher IPC and persist across restarts. See [camera tuning](native/CAMERA_TUNING.md).

## Operation

```bash
systemctl status 2pybot-console 2pybot-mediamtx
journalctl -u 2pybot-console -f
journalctl -u 2pybot-mediamtx -f
```

Run only one application against the ESP32 serial port and camera. The older `radxa_brain.py` owns both and should not run alongside this console.

The separate camera/ToF calibration service was not imported. Camera controls contained in this console are part of the downloaded application.
Geometric lens and sensor-pose calibration are future work in the
[vision roadmap](../../../docs/VISION_ROADMAP.md); exposure auto-calibration is
not intrinsic/distortion calibration.

## Driving view and ToF

The **Driving view** tab displays assembled CAD and accepted optical points,
with trail and speed/status. Its expandable Live camera section starts WHEP
on opening and stops on closing; Camera manages its own session. The toolbar has a non-shrinking
66 px navigation row and 44 px buttons; narrow screens scroll horizontally.

The depth worker uses `/dev/i2c-7`, address `0x29`, at 10 Hz. It runs as a separate
low-priority process with bounded initialization, I²C chunking, timeout/restart
handling and explicit 64-bit ctypes signatures. The board's existing ULD is
reused through `BOT_TOF_VENDOR`; `40-tof.conf` captures that setup. For a fresh
board install `vl53l5cx-ctypes==0.0.3` and `smbus2` for its Python/architecture,
or provide a compatible vendor directory. Console acquisition does not own the
camera or open the ESP32 port independently.

Autonomous ranging uses a 20 ms integration budget (four subframes per 8×8
frame) to improve return quality. `BOT_TOF_INTEGRATION_MS` can configure this;
the budget is checked against the requested frame rate. The API/grid also reports
valid-zone count; invalid returns are retained for diagnosis, not treated as clear space.

Install the environment override, then restart while idle:

```bash
sudo cp 40-tof.conf /etc/systemd/system/2pybot-console.service.d/
sudo systemctl daemon-reload
sudo systemctl restart 2pybot-console
```

Stop other ToF readers, including the earlier calibration sensor worker, before
starting console acquisition. The old calibration service remains separate.

`GET /api/tof` and 10 Hz WebSocket `{"type":"tof","tof":...}` events report raw
and bias-corrected millimetres, validity, age, errors, accepted points, corridor
depth, coverage and guard state. Zones require ULD status 5 or 9, a detected
target, 50–4000 mm and sigma 0–20 mm. Frames older than 500 ms are stale.
Invalid raw ranges remain in telemetry and recordings for diagnosis.

`POST /api/tof/guard` accepts `{"enabled":true|false}`. The guard is **OFF on every
console startup**, never persisted, and enabling requires the firmware's
`PR,TOFGUARD,1` capability. Confirmed returns in a projected wheel/turn corridor
set a reaction/braking cap, with 0.20 m close-obstacle clearance. From 3.4.0,
fresh missing/partial coverage imposes no fixed creep cap; no confirmed obstacle
leaves the normal `DRIVEV` limit. Stale/offline depth holds forward requests.
Unknown depth is not a clear-path observation. These are sensor/reference
limits, not measured bumper clearance or a proven physical stopping distance.

`POST /api/tof/calibrate` accepts a measured `reference_m` from the sensor face.
It requires stable central returns and a bias within ±0.10 m; the offset persists
in `/var/lib/2pybot/tof_calibration.json`. This does not calibrate mounting pose
or camera alignment. See [range calibration and guard](../../../docs/terrain-sessions.md).

The host sends `TG,<enabled 0|1>,<forward limit m/s>` at 10 Hz while enabled.
Firmware constrains the forward reference **after** host/gamepad/gesture
arbitration, including climb-mode speed conversion. Reverse and steering remain
available; balance recovery acceleration/wheel velocity are not clamped. An
enabled guard with a missing heartbeat blocks new forward requests after 500 ms.
Explicit `TG,0,0` releases it. Disabled acquisition does not send `V` overrides.

Zone orientation/coverage and actual stopping clearance need checking on the
mounted robot. This is an optional forward-reference limiter, not a guaranteed
collision stop or camera-ToF calibrated overlay.

## Boot coordination

See [Radxa / ESP32 boot coordination](../../../scripts/radxa/BOOT_COORDINATION.md).
The console exposes `/api/ready`, sends the readiness handshake after camera and
hotspot startup, and retries parameter discovery across an ESP32 reset/calibration.
`BOOTWAIT` is reported in the firmware SAFETY group. The ready flag never arms.
