# Bot command environment

Commands work from **any directory** on the Radxa. The environment uses its own
Python venv; the CLI itself needs only the Python standard library.

## Install once

```sh
sudo python3 /home/radxa/projects/2PyBot/scripts/radxa/install_bot_cli.py --user radxa
```

The managed one-pull installer also installs/updates these commands. CLI setup
runs before controller READY checks, so an erased ESP does not block installing
the recovery tools. This installer does not access the ESP or restart services.

New **interactive SSH logins to the configured bot account automatically
activate the environment**, showing `(2pybot)` in the prompt. The installer adds
an idempotent block to that account's Bash startup files while retaining existing
content. Non-interactive SSH commands, file transfers and local logins do not
auto-activate; an already selected different virtualenv is respected.

Immediately use `bot --help` from any directory, even without activation.
For an existing terminal, enter `bot-env`; `exit` returns to your previous shell.
These commands do not change your directory.

Configuration: `/etc/2pybot-cli.json`. Command venv and backups:
`~/.local/share/2pybot/command-env` and `~/.local/share/2pybot/backups`.
Existing unrelated `/usr/local/bin` commands are never overwritten.

## Everyday commands

| Command | Operation |
| --- | --- |
| `bot status` | Host READY, controller state, telemetry and firmware versions |
| `bot check deploy-status` | Inspect deployment state |
| `deploy-status --watch` | Follow deployment progress |
| `bot update` | Managed `git pull --ff-only` in the configured checkout |
| `bot deploy` | Queue/retry the current commit without downloading source |
| `build-firmware` | Compile the checkout with the locked toolchain and register the image |
| `flash-fw` | Flash the latest registered image using the firmware worker; show progress |
| `flash-fw --no-wait` | Queue the same flash and return immediately |
| `bot firmware-status` | Inspect installed/latest build IDs and update job |
| `bot stop` | Operator E-STOP; never starts/arms motors |
| `bot logs firmware -f` | Follow worker logs |
| `bot logs deploy -n 100` | Last 100 deployment log lines |
| `bot restart console` | Restart the console on a stopped bot |
| `bot toolchain status` | Inspect Arduino CLI/core/libraries |
| `bot toolchain install` | Install the board core/libraries from the toolchain lock; requires internet |
| `diagnostics --output ~/bot-diagnostics.json` | Save one-shot telemetry/logs/USB/service/Wi-Fi/DNS report |

All direct commands are also available as `bot` subcommands, e.g.
`bot build-firmware`, `bot flash-fw`, `bot calibration` and `bot diagnostics`.
Direct backup/recovery aliases are `bot-backup` and `bot-restore`.
`bot doctor` is another name for diagnostics.

`build-firmware` needs `arduino-cli` installed and configured on the bot. It
compiles/registers only; it does not start a flash. `flash-fw` uses the current
API checks, maintenance lock, image verification and tuning restoration. Local
SELECT/IDLE and host readiness are required. Firmware jobs continue if you exit
the terminal. A successful update still requires local START to arm.

Builds report unresolved firmware/VERSION conflict markers before compilation.

## Calibration and settings

```sh
calibration imu                        # stationary gyro calibration, not angle-zero trim
calibration trim --upright             # hold chassis physically upright; set RAM TRIM
calibration trim --upright --save      # also request NVS save after parameter readback
calibration tof --reference 0.5        # target is measured 0.5 metres away
calibration camera                    # camera exposure auto-calibration, not lens intrinsics
bot params get TRIM
bot params list
bot params set DRIVEV 1.0
bot params save
bot params load
bot params defaults
bot params refresh
```

Mutating controller settings/calibration require a stopped, stationary, fresh
READY controller. Trim uses the measured raw upright angle, not a guessed sign
or offset. Parameter writes are range-checked and read back before optional save.

## Backups and recovery

```sh
bot backup
bot restore                           # list backups/builds; no writes
bot restore tuning                    # restore latest backup's RAM tuning
bot restore tuning --save             # also request NVS save
bot restore tuning --backup /path/to/backup
bot restore firmware                  # USB recovery, including an erased ESP32
bot restore firmware --backup /path/to/backup
bot restore firmware --build REGISTERED_BUILD_SHA256
```

`bot backup` copies the recorded installed application, a validated saved worker
boot/NVS prefix and available live settings into a checksummed folder. It does
**not** read the ESP's flash or change device state. The prefix is the saved NVS
from that worker job; current unsaved RAM tuning is a separate `params.json`.

`bot restore firmware` defaults to the recorded installed image plus the latest
valid worker prefix in `/var/lib/2pybot/firmware/jobs/`. With `--backup`, it uses
that checksummed backup's image and prefix instead. Recovery validates the ESP32
image and Huge APP layout, stops serial owners/deployment, restores the saved
64-KiB boot/partition/NVS prefix at `0x0` and application at `0x10000`, then starts
the boot gate, console, worker and deployment monitors. It replaces NVS with the
selected saved prefix; use `restore tuning` separately for the backup's live RAM
settings. Administrative service/USB operations use `sudo`.

Recovery refuses a balancing bot or active update job. If the console itself is
unavailable and you have stopped the bot, use `bot restore firmware --offline`.
Likewise, `bot restart console --offline` can restart an unavailable console on
a stopped bot. If a USB write fails, services remain stopped for another recovery
attempt. Keep the bot stationary for startup calibration after recovery and use
`bot status` to inspect READY; the USB write alone is not startup validation.

An erased chip with no saved prefix needs the original bootloader/partition
files; the CLI reports this rather than trying application-only flashing.

## Offline help

`bot --help` and each subcommand's `--help` work without the console, Wi-Fi or an
AI connection. Backups and recovery use local files. Firmware builds work offline
once the Arduino core/libraries are installed; `bot update` and toolchain setup
need internet. Diagnostics collect one snapshot, not a drive recorder/test.
