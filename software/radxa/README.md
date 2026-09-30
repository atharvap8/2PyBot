# Radxa Software

The current browser console and Android API are in [console/](console/README.md). The package was imported from `/home/radxa/2pybot-console` on the Cubie A7A.

## Layout

| Path | Contents |
| :--- | :--- |
| `console/console/` | FastAPI server, USB protocol client, camera control, stream supervisor |
| `console/ui/` | Browser interface and parameter tabs |
| `console/native/` | Cedar H.264 publisher source, build file, encoder device rules |
| `console/systemd/` | Installer service templates |
| `deployment/systemd/` | Installed service definitions and camera startup override captured from the board |
| `deployment/udev/` | Installed serial and encoder device rules |
| `deployment/mediamtx.yml` | Live MediaMTX configuration captured from `/etc/2pybot` |
| `deployment/bin/` | Local copy of the installed MediaMTX binary; ignored by Git |
| `deployment/runtime/` | Local copy of saved camera profiles; ignored by Git |

The running board used MediaMTX **v1.9.3**. The console uses **460800 baud**, requests firmware parameter metadata, and serves HTTP/WebSocket on port **8080**. MediaMTX exposes WHEP on **8889**, RTSP on **8554**, and its local administration API on **9997**.

The [Android app](../android/README.md) consumes the console API and MediaMTX WHEP stream. It builds parameter groups dynamically from the firmware metadata.

The bot runs this package from `/home/radxa/projects/2PyBot/software/radxa/console`. A [commit-aware deployment timer](../../scripts/radxa/README.md) applies relevant changes after a manual `git pull --ff-only`.

The existing [vision controller](../runtime/vision_controller/README_RADXA.md) is a separate older host application. It must not open the same camera or serial port while the console is using them.

## Import record

The original source, backup files, executables, and deployed settings were downloaded to a local snapshot before importing. Application source and deployment configuration are kept here. Generated executables, captures, and device-specific runtime profiles are retained locally and excluded from commits.

The separate camera/ToF calibration application is outside this package. The camera controls already built into the console remain included.
