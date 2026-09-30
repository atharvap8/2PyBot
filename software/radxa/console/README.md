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
- `D` records populate the original 15 debug fields. The five newer terrain fields are currently ignored.

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

Copy this directory to `/home/radxa/2pybot-console`. For an existing working board, keep its installed files until the imported package has been reviewed.

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
| `GET /api/camera/snapshot`, `GET /api/camera/luma` | JPEG and brightness measurement |
| `GET /api/stream` | Stream state and metrics |
| `POST /api/stream/restart`, `POST /api/stream/stop` | Stream process control |
| `WS /ws` | 10 Hz telemetry, parameter acknowledgements, and logs |

`/api/camera/profile/save` is defined after the generic profile route in the imported code. Route ordering needs review before relying on profile saving through HTTP.

## Operation

```bash
systemctl status 2pybot-console 2pybot-mediamtx
journalctl -u 2pybot-console -f
journalctl -u 2pybot-mediamtx -f
```

Run only one application against the ESP32 serial port and camera. The older `radxa_brain.py` owns both and should not run alongside this console.

The separate camera/ToF calibration service was not imported. Camera controls contained in this console are part of the downloaded application.
