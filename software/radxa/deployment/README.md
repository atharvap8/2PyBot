# Captured Cubie Deployment

Configuration downloaded from the running `2pybot` board with the console source. The original application path is `/home/radxa/2pybot-console` and the service user is `radxa`.

| Repository file | Original board path |
| :--- | :--- |
| `systemd/2pybot-console.service` | `/etc/systemd/system/2pybot-console.service` |
| `systemd/2pybot-console.service.d/20-camera-natural.conf` | `/etc/systemd/system/2pybot-console.service.d/20-camera-natural.conf` |
| `systemd/2pybot-mediamtx.service` | `/etc/systemd/system/2pybot-mediamtx.service` |
| `mediamtx.yml` | `/etc/2pybot/mediamtx.yml` |
| `udev/99-2pybot.rules` | `/etc/udev/rules.d/99-2pybot.rules` |
| `udev/99-2pybot-encoder.rules` | `/etc/udev/rules.d/99-2pybot-encoder.rules` |

These are the installed files, separate from the installer's templates. The camera drop-in is part of the deployed setup and applies natural-color controls before the console starts.

MediaMTX was **v1.9.3**, running from `/usr/local/bin/mediamtx`. Its downloaded executable is retained locally at `bin/mediamtx`, which is ignored by Git. Saved camera profiles from `/var/lib/2pybot` are retained locally under `runtime/`, also ignored.

The live MediaMTX settings enable TCP RTSP on 8554, WebRTC on 8889 with UDP media on 8189, and the administration API on localhost 9997. The `cam` path accepts a publisher. HLS, RTMP, and SRT are disabled.

Use [the console setup guide](../console/README.md) when installing or restoring the application. Review the absolute paths before applying these captured service definitions to another board.
