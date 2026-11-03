# Experimental front-bumper self-uprighting — 4.3.0

## Baseline and branch

- Owner-selected stable BLDC: `ed913418ca05759482a024b6c89794c2c5b10641`.
- Annotated rollback tag: **stable-bldc**, also published to origin.
- Experiment branch: **hardware/bldc-self-uprighting**.
- Ignored lifted-wheel work remains in stash **d4fc0d4**; no unloaded-wheel guard
  is integrated in this branch.

The measured front-rest raw pitch was -61.416 degrees. With fixed TRIM=-1.2603
and PITCHSGN=-1 this becomes **+60.156 degrees control pitch**, consistent with the
owner's latest 60-degree bumper-rest description. The sign is positive forward
lean; the command uses the stable image's confirmed current/wiring polarities.

## START is the fire button

1. After matching firmware is installed and READY, place the bot on its **front
   bumper and wheels**, about +60 degrees **control** pitch; centre the drive stick.
2. A fresh local gamepad **START** edge fires one attempt. Holding START does not
   repeat it. A normal upright START still arms the existing balancer.
3. The controller first holds zero torque for 200 ms to confirm a stationary rest.
4. It then applies equal robot-forward wheel currents using bounded attitude PD:
   `I = clamp(SRKP*pitch + SRKD*rate, -currentLimit, currentLimit)`, with SRSLEW.
   Forward wheel torque reacts against forward chassis lean. Negative rising
   pitch rate reduces/reverses current to brake the rise; no ground LQI position
   integral runs while the chassis is on the bumper.
5. CAPTURE starts below/predicted below +25 degrees. At |pitch| <=8 degrees,
   |rate| <=70 deg/s and |pitch + 0.06*rate| <=8 degrees it hands off to the stable
   balancer. Its virtual wheel velocity is seeded from measured velocity, and
   position/heading are re-latched. Handoff selects normal hold, not stiff/climb.

**SELECT or remote E-STOP/X aborts.** After a failed attempt it stays stopped;
another deliberate local START is required. START at another fallen posture is
refused instead of queuing an unexpected future arm. Serial E cannot fire recovery.

## Initial tuning and bounds

These are initial experimental settings, not a measured guarantee of standing up.

| Key | Default | Meaning |
| --- | ---: | --- |
| SREN | 1 | Enable front-rest START recovery; 0 restores normal arming behaviour |
| SRREST | 60 | Positive control-pitch rest angle; launch window +/-10 degrees |
| SRMAXMA | 2700 | Per-wheel recovery current ceiling, also clamped by CURRENT |
| SRKP | 0.07 | Attitude current gain, A/degree |
| SRKD | 0.025 | Pitch-rate current damping, A/(degree/s) |
| SRSLEW | 30 | Current slew, A/s |
| SRTIME | 2.0 | Total attempt duration, including 200 ms settling |
| SRDIST | 0.45 | Maximum displacement from start for either wheel, metres |
| SRSPD | 1.2 | Maximum wheel speed during the attempt only, m/s |

The existing CURRENT setting is not increased. Initial maximum remains 2.7 A per
wheel if CURRENT is 2700 mA. Settings are snapshotted at launch and cannot be
changed mid-attempt. These attempt-only speed/travel limits do not add a normal
ground-driving speed governor.

Launch additionally needs healthy current-mode motors, fresh IMU, connected local
pad, neutral drive inputs, known pitch/rate signs, |rate| <=15 deg/s and both wheel
speeds <=0.08 m/s. Active auto-tuning blocks launch.

Abort conditions include: SELECT/X, motor link/mode/fault, control interval >20 ms,
IMU sample age >30 ms, pad loss/staleness, drive input movement, host readiness
loss, timeout, wheel speed/travel bounds, pitch outside -12..+75 degrees, rate over
250 deg/s (or a lower configured MAXRATE), lean worsening 5 degrees, less than
3 degrees improvement after 0.75 s, or relapse above +35 during capture.
Abort clears motor control and sends zero DRIVE before idle QUERY; brake stays
false. Normal MAXTILT=55 remains in force after the handoff.

## Deploy this branch on the bot

Use the existing managed checkout with the bot stopped by SELECT and no update
currently running. The deployment manager pins a branch in its configuration;
switch both the checkout and that branch setting. Wait for the commit's **Bot
deployment bundle** GitHub build to succeed before deploying.

```sh
ssh radxa@10.42.0.1
bot stop
sudo systemctl stop 2pybot-deploy.timer 2pybot-deploy.path
cd /home/radxa/projects/2PyBot
git fetch origin
git switch hardware/bldc-self-uprighting
git merge --ff-only origin/hardware/bldc-self-uprighting
sudo python3 - <<'PY'
import json
from pathlib import Path
p = Path('/etc/2pybot-deploy.json')
c = json.loads(p.read_text())
c['branch'] = 'hardware/bldc-self-uprighting'
t = p.with_name(p.name + '.branch-new')
t.write_text(json.dumps(c, indent=2) + '\n')
t.chmod(p.stat().st_mode & 0o777)
t.replace(p)
PY
bot deploy
sudo systemctl start 2pybot-deploy.path 2pybot-deploy.timer
bot deploy-status --watch
bot firmware-status
bot params get SELFRIGHT
bot params get SRREST
```

`git switch` creates the local tracking branch on first use and reuses it later;
the fast-forward merge updates an existing local branch without discarding work.
The configuration edit preserves the other deployment settings. Queuing with
`bot deploy` before restarting the monitors avoids racing their operation lock.
The timer resumes the queued deployment, including matching host and MCU flashing.
After success, confirm **installed.version=4.3.0**, **SELFRIGHT=1**, **SRREST=60**.
Later updates on this selected branch use `bot update` normally.

## Readback and the first manual attempt

Before using START, read firmware version **4.3.0** and **SELFRIGHT=1**. Updating
the APK or switching the Windows branch alone does not install MCU code.

Record one short attempt with the app Data recorder and a side-view video. Useful
feedback is: whether wheels push forward, whether pitch falls toward zero, minimum
pitch reached, SR phase/reason, actual current/RPM/fault bytes, and whether handoff
stays upright or overshoots. Stop at the first result; there is no automatic retry.

The API exposes `telemetry.self_righting_status` and raw SR events.
The record also carries live robot_state (0 idle, 1 balance, 2 recovery), so retained
results cannot masquerade as active motion and host interlocks work with D off.
Phase codes: 0 idle, 1 settle, 2 lift, 3 capture, 4 handed off, 5 aborted. Reason codes:
0 none, 1 operator, 2 rest moved, 3 timeout, 4 no lift, 5 wrong way, 6 angle,
7 speed, 8 travel, 9 sample/timing, 10 relapse, 11 motor, 12 pad, 13 IMU,
14 drive input, 15 rate, 16 host readiness. `[SR]` logs print human-readable reasons.

Compilation and syntax checking are permitted by AGENTS.md. No automated test or
remote arming/motion is part of building this experiment; physical success requires
the operator's local START attempt after installation.
