# Windows → GitHub → one pull on the bot

## Contract

Development happens on Windows. The bot is a managed checkout on one configured
branch. After one-time setup, run as the checkout owner:

```sh
git pull --ff-only
```

The command displays Git output, then waits for deployment. A completed fetch is
not shown as deployment success. READY means the requested host/controller
installation passed its checks; physical balancing/driving validation is separate.

Deployment never sends E/U or an arming gesture. It waits for stationary, fresh
READY/IDLE telemetry and acquires FU,1 before restarting the serial owner. Pending
local arming can refuse that lock. A balancing robot is not force-stopped.

## One-time GitHub setup

1. Commit/push this implementation on the branch the bot will track. Keep binaries,
   NVS prefixes, recordings and private keys outside source history.
2. Enable **Bot deployment bundle**. It checks native logic, builds firmware using
   `scripts/firmware/toolchain.json`, builds Android and publishes:
   `bot-<commit>/2pybot-deploy-<commit>.tar.gz`.
3. Set repository secret **ANDROID_DEBUG_KEYSTORE_BASE64** to the existing local
   development signing keystore encoded as base64. This is needed for APK upgrades
   to retain the installed app's signing identity. Do not generate a new CI key
   every run or put its private contents in Git, chat or issues.
4. The workflow token needs Contents write for build assets. **One-pull deployment
   checks** separately tests orchestration on Python 3.9/3.12 without hardware.

The bundle binds the source commit, firmware input fingerprint, locked toolchain,
hardware/schema and every payload SHA-256. CI success is not physical validation.
Each push to a tracked branch gets a bundle, so a bot that skips several commits
can still obtain the exact final commit's artifacts. Unchanged firmware inputs
are not reflashed when that bundle is deployed.

## One-time bot setup

Prerequisites: the existing working camera/MediaMTX stack, Python/venv/pip, Git,
encoder SDK/build dependencies, and firmware worker with esptool/serial. The
controller must already be commissioned as BLDC schema 4 with **persisted BOOTWAIT=1**, FLASHLOCK and
STARTUPACK support and an accurately registered installed image. The normal
updater verifies that image against flash before writing another.
BOOTWAIT must also be saved in the controller's normal tuning store, so a USB
reset cannot reload a standalone-start setting during host maintenance.

The old host lacks the new atomic deployment-lock endpoint. For the one-time host
migration, disconnect controller USB with motor power safely disabled, then run:

```sh
sudo python3 scripts/radxa/install_one_pull.py --user radxa --bootstrap-host
hash -r
```

Reconnect after host setup. Pending updates wait for fresh stationary telemetry.
The bootstrap installs host support; it does not mark new firmware as installed.
If the host already exposes protocol 1 at `/api/deployment/status`, omit
`--bootstrap-host`. An unrelated existing `/usr/local/bin/git` is left untouched.

For a private repository, create root-owned mode-0600 `/etc/2pybot-deploy.env`
containing `BOT_DEPLOY_TOKEN=...`. Contents read grants release access; Actions
read enables early CI-failure reporting. Tokens stay in headers and are stripped
from cross-host redirects, never put in progress files or command URLs.

### Optional firmware builds on Radxa

CI artifacts are the default. For local firmware builds, supply a compatible
installed ARM64 arduino-cli and provision locked dependencies once:

```sh
sudo python3 scripts/radxa/install_one_pull.py --user radxa \
  --firmware-mode build --arduino-cli /path/to/arduino-cli --provision-toolchain
```

Android source/version changes still need the CI APK bundle. Local compilation
checks actual board/library versions against the lock. Firmware input changes
invalidate its cache; documentation-only changes do not reflash unchanged code.

## Everyday workflow

On Windows:

```powershell
git status
git diff
git add <intended-files>
git commit -m "Describe the change"
git push
```

On the bot's configured branch:

```sh
git pull --ff-only
```

The managed wrapper refuses tracked edits/rebase instead of discarding work.
Untracked notes such as the handoff are preserved. Other Git operations/repos
pass through to the real Git. A checkout to another branch is not auto-deployed.

## Deployment sequence

1. **Pulling:** the wrapper writes a heartbeat before the network fetch. Git has
   no standard pre-pull hook, so post-merge alone cannot show this phase.
2. **Checking/CI:** wait for the exact commit's bundle; verify identity, hashes,
   hardware/schema and toolchain. Report CI failure instead of claiming ready.
3. **Preparation:** stage a Git snapshot, prepare a dependency environment and
   check imports/native builds. The running application keeps its previous
   snapshot while the checkout changes.
4. **Maintenance:** acquire a durable token-owned hold through the existing serial
   owner. The hold survives host restart; no second owner races the console.
5. **Activation:** switch the `current` snapshot and restart only affected services.
   Boot-gate changes apply next boot, without restarting a live reset holder.
6. **Firmware, if changed:** skip a matching installed image; otherwise queue the
   pinned build on the existing worker. Preserve live tuning/calibration, verify
   the previous image, write application only, verify and restore locked IDLE.
7. **Android:** publish the checked APK in persistent downloads. `/ui/android.apk`
   resolves its version without hardcoded links.
8. **Handback:** release maintenance, check fresh READY metadata/host recovery,
   wait for the controller's unlock acknowledgement, record the applied
   commit/fingerprint and display success. Local START is
   still required for motion.

A shared operation lock serializes managed pulls and deployment activation;
a busy pull exits 75 without changing the checkout. An interrupted deployer waits for its already-running flash worker
instead of restarting it mid-write.

## Failures and recovery

- Preparation failure leaves the running release unchanged.
- Host activation failure can restore its previous snapshot/configuration while
  controller maintenance is held. Handback requires a healthy rollback; an
  unverified rollback keeps maintenance held.
- Firmware failures use the worker's verified previous-image rollback. Normal
  updates do not read a complete application partition for backup every time.
- Recovery-required/partial flash state keeps maintenance held.
- If firmware succeeded and a later operation fails before handback, keep its
  matched host/image and hold together rather than mix old host/new firmware.
- After handback, failures do not reset a controller the operator may have armed.
- Failed commits are latched to avoid reflashing every ten seconds. Fix the
  cause and issue another pull, or explicitly retry `deploy.py --force`.
- Waiting for IDLE/CI resumes on the timer without a second pull.
- An interrupted, unconfirmed lock acquisition is reported for recovery, never
  interpreted as permission to unlock or drive.

## Unrestricted full-stick diagnostic

Normal full stick now follows `DRIVEV` directly (currently 1.00 m/s). Free-speed
estimates and `SPDHEAD` do not reduce that target. `SPDHEAD` remains as an unused
legacy parameter so firmware updates can restore the existing tuning snapshot.
An explicitly configured nonzero `MAXSPD` remains an operator-selected ceiling.

The predictive reference governor is bypassed: no reference slew, risk-driven
velocity-reference cancellation or predictive steering reduction. Metadata reports
`PR,FALLPREVENT,0`; normal-mode `PR,VDRIVE` should equal `DRIVEV`. Existing tilt/rate
cutoffs, current limits, motor-link/fault stops and local START/SELECT stay active.
This diagnostic change does not establish the cause of the full-speed fall.

`python tools/sims/capture_control.py` only prepares/checks the recorder command;
it performs no API reads or recording. Start it with `--go` only after the operator
says GO. It then verifies the unrestricted firmware is live and records a fresh
timestamped directory under `build/control-audit/`, without issuing motor commands.

## Managed hotspot band

`scripts/radxa/hotspot.json` requests **2.4 GHz (`bg`), channel 6** on the existing
NetworkManager hotspot. A null `connection` selects the unique active Wi-Fi AP;
an explicit profile name can be used when multiple APs are active. A pull changing that file selects a hotspot
deployment. It waits for stationary controller maintenance, modifies only the
band/channel, reactivates that profile and verifies the live radio frequency.
SSID, password, IP addressing and uplink configuration are preserved. Fresh
installations also default to 2.4 GHz/channel 6.

The AP restart briefly disconnects Wi-Fi/SSH clients. Rejoin the same SSID and
reconnect SSH; deployment runs under systemd and continues independently of the
terminal. If band activation fails before handback, the previous band/channel
are restored. Original settings are retained under `hotspot-backups/<commit>.json`
in the deployment state directory. An already-correct radio is not restarted.

After reconnecting, check `/ui/deployment.html` and inspect the active interface:

```sh
nmcli -f NAME,TYPE connection show --active
iw dev
```

Expected live channel: **6**; expected frequency: **2437 MHz**. The change
does not alter firmware inputs; a first deployment after host-only bootstrap
may still have the initial controller update pending. Reconnect the controller
after bootstrap and leave the robot stationary for that coordinated deployment.

## Ring animations and ownership

The ESP32 ring uses the original stepper animation policy from commit `a8427f0`,
including its boot, idle, drive, gesture, fall and sleep animations. There is no
Radxa display lease or deployment overlay. `PR,HOSTLED,0` advertises this policy;
old `LS` messages are ignored. Both console and firmware worker stop sending them.

GPIO15 frame transmission uses NeoPixelBus I2S1 DMA instead of interrupt-refilled
RMT. Startup sends a complete black frame before the boot swirl, and normal
rendering skips a busy transfer instead of waiting in the control loop. This
targets the reported per-pixel colour corruption on START; physical confirmation
is required after deploying the change. The original GRB format and animations
are retained. Local tests were skipped at the owner's request.

The full stepper animation set is retained: rainbow boot swirl, green arming
sweeps, red fall flash/fade, HIGH blue laps, LOW green settling arc, gesture
rainbow, vision-brake pulses, violet climbing/grip dots, shy pink cheeks,
directional driving beam and amber reverse, stiff amber breathing, idle
breathing/twinkle/effort tint, green pickup prompt, sleeping ember/double-blink
and controller-connect greeting.

One-pull disables/stops `2pybot-deployment-leds.service` for the default setup,
including installations whose old configuration still says `esp32`. Fresh setups
default to `--led-backend firmware`. Deployment progress remains in the dashboard.

### Existing wiring and flash-time indication

The current ring is wired to **ESP32 GPIO 15** and stays firmware-controlled.
It cannot animate while that ESP32 is in its bootloader. Its last frame can remain
visible until the app runs again.

For continuous flash-time animation, use a separately wired Radxa SPI LED data
path/controller and select it:

```sh
sudo python3 scripts/radxa/install_one_pull.py --user radxa \
  --led-backend spi --spi-bus 0 --spi-device 0
```

This requires the enabled spidev node, `python3-spidev`, compatible electrical
levels/common ground and **one physical driver** on the ring data input. Two
push-pull outputs must not be connected together. A shared-data handoff needs
an appropriate hardware mux/isolator. After deployment the SPI service renders
normal robot feedback from telemetry. This explicit SPI option is separate from
the firmware-owned ESP32 ring.
Default geometry is sixteen pixels with bounded brightness.

## Inspect

- Browser: `/ui/deployment.html`
- API: `/api/deployment/status`
- Progress/applied state: `/var/lib/2pybot-deploy/progress.json`, `state.json`
- Active snapshot: `/var/lib/2pybot-deploy/current`
- Pull heartbeat: `/run/2pybot-deploy/pull.json`
- Durable hold: `/var/lib/2pybot/firmware/deployment-hold.json`
- Logs: `journalctl -u 2pybot-deploy.service -u 2pybot-deployment-leds.service`

Deployment write endpoints are loopback-only. If a local reverse proxy is added,
deny external access to `/api/internal/` in its configuration.

## Implementation verification (2026-10-20)

- Windows Python suite: **149 passed, 1 skipped**. The skipped test checks Linux
  advisory locking and is included in the Linux CI matrix.
- Six native BLDC/display checks passed, including lease expiry and UART/control
  regression checks.
- ESP32 4.1.0 built with the locked core/libraries; Android 4.1.0 debug APK built.
- Ruff undefined-name/syntax checks and `git diff --check` passed.

These are local working-tree checks. The implementation has not yet been
installed on Radxa or exercised as a GitHub-bundle-to-controller deployment.
The local dirty-tree build is not an immutable CI release bundle.

This work coordinates deployment/display. It does not certify the BLDC control
law, establish maximum motor performance or implement the MuJoCo twin. Those
remain separately measured/validated work.
