# Repo-Based Radxa Deployment

The bot runs the console from a Git checkout at `/home/radxa/projects/2PyBot`. A systemd timer checks the local commit about every ten seconds. It does not fetch or pull from GitHub.

## Update

```bash
cd /home/radxa/projects/2PyBot
git pull --ff-only
```

The deployment service waits for Git to finish and rejects tracked local edits. Build/dependency checks happen before service changes. A second Git update during preparation defers deployment to the next check.

## What causes a restart

| Change | Action |
| :--- | :--- |
| Console Python or browser JS/HTML/CSS | Check Python imports, restart console |
| `requirements.txt` | Update the dedicated environment, check imports, restart console |
| Native C/header/Makefile | Build the encoder, restart console |
| Console service template or camera override | Install changed file, reload systemd, restart console |
| MediaMTX source configuration or service | Install changed file, restart MediaMTX and console |
| Encoder device rules | Install/reload rules, restart console |
| Deployment timer/service templates | Update the monitor units |
| Firmware, Android, CAD, documentation | Record the commit without restarting handlers |

The live configuration is `software/radxa/console/mediamtx.yml`. Files under `software/radxa/deployment/` are captured deployment references, not active update inputs.

Firmware still needs a separate compile/flash step. An Android source update needs a separate APK build/install.

## Initial installation

The board needs Git, Python 3.9 or newer with venv/pip, make, the existing console dependencies, MediaMTX, and the Cedar SDK/development libraries described in [the native guide](../../software/radxa/console/native/README.md).

```bash
cd /home/radxa/projects/2PyBot
sudo python3 scripts/radxa/deploy.py --user radxa --force --install-monitor
```

This uses the console service templates with the checkout path and a dedicated Python environment. It does not run the hotspot installer or change NetworkManager configuration.

## Runtime paths

| Path | Contents |
| :--- | :--- |
| `/var/lib/2pybot/` | Existing user camera profiles |
| `/var/lib/2pybot-deploy/venv/` | Dedicated Python environment using available system/user packages |
| `/var/lib/2pybot-deploy/state.json` | Last successfully applied commit and actions |
| `/var/lib/2pybot-deploy/backups/` | Previous installed configuration files, grouped by target commit |
| `/run/2pybot-deploy.lock` | Lock preventing overlapping deployments |
| `/etc/2pybot/mediamtx.yml` | Installed streaming configuration |

State is updated only after commands and service health checks succeed. If video was running before a relevant update, its recovery is also checked. ESP32 serial connection is not required for deployment health.

The monitor runs installation commands as root but reads Git as the checkout owner. This supports the board's Git 2.30 without global ownership exceptions. The console service disables bytecode writes in the source checkout.

## Inspect or retry

```bash
systemctl status 2pybot-deploy.timer
journalctl -u 2pybot-deploy.service -n 50
cat /var/lib/2pybot-deploy/state.json
sudo systemctl start 2pybot-deploy.service
```

Preview pending actions without applying them:

```bash
python3 scripts/radxa/deploy.py --dry-run
```

If a deployment fails, the previous marker stays in place and the timer retries. The script does not reset the checkout or automatically roll back application code. Configuration backups are retained for manual recovery.

`pair2.sh` is the older Linux gamepad-pairing helper. Current BaseLink pairs its gamepad directly to the ESP32.
