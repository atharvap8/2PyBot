# 2PyBot Android App

Revision **4.0.0** targets the [BLDC hardware branch](../../docs/BLDC_V4.md).
Tune shows the MOTOR group and fixed wiring-polarity values as read-only.

Revision **3.40.0** fixed Routes rendering across rotation and short landscape
windows, with instance-owned WebView lifecycle, explicit viewport sizing and Reload.

Revision **3.30.0** introduced the [adaptive cockpit](../../docs/ANDROID_COCKPIT_DESIGN.md):
five primary navigation items, a compact landscape rail, a larger driving/video
workspace, direct Routes entry and responsive settings columns. The installed
controller firmware is independent of the Android UI revision.

**More → Setup → Controller firmware** can reflash the latest
compiled build registered on the bot, with exact build identity, durable progress
and rollback results. See [firmware reflashing](../../docs/FIRMWARE_REFLASH.md).

Native Kotlin and Jetpack Compose client for the [Radxa console](../radxa/console/README.md). The app uses HTTP for commands and settings, WebSocket for telemetry, and WebRTC/WHEP for camera video.

## Build configuration

| Setting | Value |
| :--- | :--- |
| Application ID / namespace | `in.twopybot.app` |
| Minimum Android version | API 26, Android 8.0 |
| Compile / target SDK | API 34 |
| Java target | 17 |
| Gradle wrapper | 8.9 |
| Android Gradle plugin | 8.5.2 |
| Kotlin | 1.9.24 |
| Compose compiler | 1.5.14 |
| Default robot host | `10.42.0.1` |

Open this directory in Android Studio with JDK 17 and Android SDK 34 installed. The Gradle wrapper scripts, properties, and JAR are included.

The imported Windows wrapper's two missing-Java branches were corrected to return immediately instead of falling through into Gradle startup.

From Windows:

```powershell
.\gradlew.bat :app:assembleDebug
```

From Linux or macOS:

```bash
bash gradlew :app:assembleDebug
```

Debug APK: `app/build/outputs/apk/debug/app-debug.apk`.

`assembleRelease` produces an unsigned release APK. No signing key is included. Machine-specific SDK paths belong in ignored `local.properties`, or use `ANDROID_HOME`.

Release-candidate tags keep root `VERSION` numeric (for example `3.4.0`) and
build with `-PreleaseVersion=3.4.0-rc.1`. APK versionName then includes the RC
suffix, while versionCode continues to derive from the numeric base version.

## Build helper and CI

`build.sh` supports debug, run, release, clean, setup, and doctor modes. Its setup commands use Arch Linux `pacman`; they are not a generic Linux installer.

```bash
bash build.sh doctor
bash build.sh
bash build.sh run
```

The repository's [Android workflow](../../.github/workflows/android.yml) builds debug and unsigned release APKs and runs unit tests when Android/version files change on `main` or `system/**`, on matching pull requests, or through manual dispatch. Artifacts are `2pybot-apk` and `2pybot-apk-release-unsigned`; version tags also attach named APKs to the GitHub release. App version/code derive from the repository's root `VERSION`.

## Connection

Join the Radxa hotspot or use its LAN address. Enter the host address in Setup. The app connects to:

| Interface | Address |
| :--- | :--- |
| HTTP API | `http://<host>:8080` |
| Telemetry WebSocket | `ws://<host>:8080/ws` |
| Video | `http://<host>:8889` plus the WHEP path reported by the console |

Default WHEP path is `/cam/whep`. The current Android network configuration permits cleartext traffic through its base configuration; it is not restricted to private IP ranges.

The app communicates with the Radxa, not directly with the ESP32. The console owns USB serial and relays firmware records and parameter writes.

## Screens

| Screen | Contents |
| :--- | :--- |
| Drive | Full-width dark shaded world, chase/orbit/pan/zoom, live pose, reference guidance, depth and guard/range calibration |
| Live | WHEP camera, attitude gauge, controller debug values, gesture controls |
| Map | Console dead-reckoned trail and pose; local pose reset; PNG/PDF exports |
| Tune | Parameter groups, values, bounds, descriptions, Save/Reload/Defaults |
| Routes | Original-command route teaching/replay; tuning routines remain available through the toolbar |
| Camera | V4L2 settings, named profiles, exposure/gain adjustment, stream settings |
| Camera → Lens calibration | Embedded progressive web wizard, printable targets, automatic capture/analysis, corrected previews and reports |
| Data | Labelled on-disk telemetry/ToF sessions, annotations, diagnostics and ZIP export |
| Setup | Robot address, stream status, stop control, console logs and log PDF export |

Portrait uses five bottom destinations: Drive, Live, Routes, Map and More. More
contains Tune, Camera, Sessions and Setup. Landscape uses a 72 dp navigation rail
with an anchored stop control. Content panels use available window width, bounded
tool panes and independent scrolling. See [cockpit design](../../docs/ANDROID_COCKPIT_DESIGN.md).

The display stays awake while the app is visible. Rotation supports portrait and
both landscape directions. Drive touch gestures change only the camera view;
they do not drive or arm the robot. Its references are not measured roads or
free-space claims. Detailed coordinates, recorder and guard behaviour are in
[terrain sessions and Drive](../../docs/terrain-sessions.md). Planned camera/ToF
calibration and ML work is tracked in the [vision roadmap](../../docs/VISION_ROADMAP.md).

## Parameters and commands

Tune builds its group list from the console parameter payload. Current firmware has **107 parameters in ten groups**, including TERRAIN. Unlike the browser's fixed tab list, the Android group list includes new groups automatically.

Parameter writes change firmware RAM. Save sends the NVS save action; Reload loads saved values; Defaults restores compiled settings without saving.

The console whitelist excludes `E` and `V`, so this app has no direct joystick-driving or balance-arm command. It can send gestures, hold-mode commands, and auto-tune routines. The displayed pad values come from controller debug after input arbitration, and the connection indicator is not a Bluetooth-gamepad pairing status.

The stop button calls `/api/estop`, which sends firmware `X`. That command stops balancing but does not abort the current firmware's stall-search routine. Use Auto's Abort action for `AT,stop`. Radius Done currently encounters the firmware busy-guard bug.

## Source map

| File | Responsibility |
| :--- | :--- |
| `MainActivity.kt` | Eight destinations, orientation/navigation, keep-awake and stop control |
| `data/BotRepository.kt` | HTTP/WebSocket models, parameter requests, progress logs, WHEP URL |
| `vm/BotViewModel.kt` | Saved host setting and UI command actions |
| `net/WhepClient.kt` | Receive-only video track, SDP exchange, decoder factory |
| `data/SessionRecorder.kt` | Raw timestamped events, labelled CSV, metadata/diagnostics and ZIP export |
| `ui/screens/` | Drive, Live, Map, Tune, Auto, Camera, Data and Setup |
| `ui/components/RobotWorldView.kt` | GLES shaded CAD/world and touch interaction |
| `ui/components/WorldFrame.kt`, `WorldMotion.kt`, `WorldCamera.kt` | Shared scene axes, calibrated pitch, pose and chase/explore camera |
| `ui/Responsive.kt` | Orientation helpers |
| `ui/components/Widgets.kt` | Reusable gauges, parameter rows, and status components |

Kotlin files are under `app/src/main/java/in/twopybot/app/`.

Launcher artwork and density variants are reproducible with
`python scripts/generate_launcher_icon.py` from the repository root (numpy,
trimesh, Pillow). The full-resolution world asset uses
`python scripts/generate_world_model.py` (numpy, trimesh).
