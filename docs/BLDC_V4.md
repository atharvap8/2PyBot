# v4.0.0 — M0601 BLDC drive

Branch: `hardware/bldc-v4.0.0`. The supplied `2pybot-hub/BaseLinkHub`
program is integrated into the existing `firmware/BaseLink` build target.
The USB protocol, gamepad, LQI, navigation lease, climb controller and payload
remain in the same application. STEP/DIR, TMC2226 and external PCNT wheel
drivers are removed from this target.

## Wiring and confirmed settings

| Wheel | ID | UART | TX | RX | DE + /RE | Direction |
| :--- | ---: | :--- | ---: | ---: | ---: | ---: |
| Right | 2 | Serial2 | 19 | 18 | 23 | +1 |
| Left | 1 | Serial1 | 17 | 16 | 4 | -1 |

The supplied README has conflicting ID descriptions; its `config.h` pin map
is the chosen source. Motor links are 115200 8N1. USB remains 460800 8N1.
The harness uses I2C 21/22, LED ring 15, pan 26, zoom 0 and torch 2.

- **`ISGN = -1`, `RPMSGN = -1`**: confirmed by the owner. Both are fixed
  metadata values; loading NVS or restoring defaults cannot flip them.
- **`HUBVSRC = 1`**: position-derived wheel velocity, the owner's confirmed
  fix for fore–aft oscillation. This is the compiled default.
- `ENCSGN = +1`: wheel direction is applied to both commands and feedback
  at the hub boundary, avoiding a second feedback-only inversion.
- Radius 0.052 m, track 0.155 m, supply 12 V and feed-forward mass 2 kg
  follow the supplied program. They are tunable commissioning values.
- The supplied common velocity gains (`HUBKP=6`, `HUBKI=8`, `HUBILIM=1`),
  LQI gains, 0.60 m/s drive setting and 6 m/s² balance acceleration remain.

One internal count is 1/4096 revolution. `STEPSM == COUNTSM`; legacy field
names such as `MAXSPD` and `vCmdSteps` now describe encoder counts, not
microsteps. `MAXSPD=0` derives the ceiling from supply voltage. A positive
setting can lower that ceiling but cannot exceed the configured no-load speed.

## Turning changes

1. The supplied 7500 counts/s differential corresponds to about **7.7 rad/s**
   at this track. The v4 full-stick default is **1.5 rad/s**, with a 4 rad/s²
   steering slew. LOW mode scales the stick request by 0.5.
2. Current control operates in common and differential coordinates. Balance
   current is allocated first; turning uses only the remaining per-wheel
   current headroom. Independent wheel clipping cannot convert a large turn
   into an unintended common acceleration.
3. Common acceleration feed-forward no longer differentiates the steering
   command. The turn loop has its own `HUBYAWKP`, `HUBYAWKI`, `HUBYAWIL`
   and `HUBYAWSL` settings (defaults 1.5, 2, 0.35 A and 6 A/s).
4. Slew applies to commanded turns, reversals and heading hold. Releasing
   the stick decelerates the turn before latching the final heading, rather
   than abruptly demanding a counter-turn against residual wheel motion.
5. Navigation yaw requests retain rad/s units and small-turn deadband bypass,
   while respecting the same v4 differential limit.

## Feedback and timing

Each UART has one outstanding request. DRIVE and QUERY reply layouts cannot
be re-labelled by a newer request. An 8 ms reply timeout invalidates mode
and position and starts a quiet resynchronisation window. CRC errors and
timeouts are counted. Position uses unwrapped absolute DRIVE feedback, without
an RPM extrapolation/reset step between position packets.

Temperature QUERY frames are sent only while disabled. Armed polling uses
DRIVE replies continuously; temperature readings therefore represent the last
idle poll. Both motors must confirm current mode and valid position before
normal balance arming. Mode loss, motor faults, link loss or a control interval
over 20 ms disarms. There is no automatic self-righting/rearming path.

`P?` output is incremental and TX-buffer-aware. It no longer holds the 200 Hz
control loop while printing the complete metadata table. NVS writes, reloads,
defaults, and mode/direction/feedback-source changes require stopped motors.
The browser and Android Tune screen use the `MOTOR` group and show fixed
polarity entries as non-editable.

`HM` feedback is parsed into `telemetry.hub`; see the
[protocol](../firmware/BaseLink/PROTOCOL.md) for field order and frames.

## Persistence and build identity

v4 uses **`2pybot-bldc4`**. The old `2pybot` and experimental `2pybot-hub`
namespaces are left intact, but their motor settings are not imported.
The captured v3 defaults remain as historical data, outside the active store.
Camera calibration files are independent of this motor namespace.

Build manifests identify `hardware=m0601-rs485` and
`parameter_schema=bldc-v4`; firmware advertises `PR,HUBSCHEMA,4`. The automatic
updater rejects cross-hardware tuning restoration. First BLDC commissioning
is a separate manual installation, not a stepper rollback/restore operation.

## Offline verification

```sh
python scripts/firmware/check_bldc.py
python scripts/firmware/build.py --arduino-cli /path/to/arduino-cli \
  --output-dir build/BaseLink-v4.0.0
```

`check_bldc.py` runs production C++ control, parser, hub-drive and parameter
code against host-only UART/NVS stubs. Checks cover golden CRC/frame vectors,
partial/corrupt/late replies, sign application, pure-turn symmetry, balance
current priority, slew/release, delayed turn responses, timeout disable,
namespace isolation and incremental dumps. These checks do not access a bot.

This change is source/offline-build work only. Turning on real hardware has
not been verified; no controller flashing, arming or motor exercise is part
of this work.
