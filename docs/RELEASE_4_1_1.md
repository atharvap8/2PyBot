# 4.1.1 — direct steering and persistent video

## Firmware

- Current-only control, owner-configured 18 V / 5S and 2.6 kg profile retained.
- Confirmed **raw neutral TRIM = -1.4 degrees**. This is not a command to subtract
  -1.4 from an existing trim. Control pitch is `PITCHSGN * (raw pitch - TRIM)`.
- Default maximum steering is 3 rad/s, about 2915 differential encoder counts/s
  for the configured wheel/track geometry. `MAXSTR` remains adjustable up to 9000.
- `STRSLEW=0`, `HUBYAWSL=0`, `EXPOSTR=0`, `LOWSTR=1`: no deliberate steering/yaw
  current ramp, linear steering, equal steering authority in LOW and HIGH.
- Drive-stick cardinal snapping is bypassed; radial deadzone remains. Camera-stick
  snapping and the old pan curve are retained separately as `PANCURVE`.
  Heading hold latches the new heading instead of returning to
  the pre-turn heading, and yaw integral is cleared when a turn is released.
- The predictive fall-prevention governor is removed from production. Its old
  header is retained only under `tools/sims/legacy` for historical reproduction.
  Neither NVS nor app settings can turn that production governor back on.
- Physical balance, current, fault, link, tilt/rate cutoffs and explicitly enabled
  ToF/climb controls remain. Motor inertia and available torque still affect the
  real response; removing software slew does not make motion instantaneous.

The first `INPUTPROFILE=1` load sets these targeted RAM defaults. The updater
recognises the profile so it does not replay old positive trim, low MAXSTR or
old slew/expo values over them. Other calibration and the user's added system-map
work are retained. With the bot stopped, `bot params save` persists the profile.

## Android

- Version name 4.1.1 and a higher version code distinguish this APK from old 4.1.0
  builds. Setup and the workspace menu display the installed app version.
- Portrait and the two landscape poses must be held continuously for **2 seconds**.
  Returning to the current pose or leaving a valid orientation window cancels the
  pending switch. A timer completes the hold even if no further sensor events arrive.
- The ViewModel owns playback, not a screen composition or SurfaceView. Rotation
  and tab/layout changes detach/reattach the display without closing the peer.
- Transient ICE/network failures retry while playback is requested. Stop video
  cancels connection/retry work and clears requested playback. Camera source
  capture is not stopped as a side effect of phone orientation/lifecycle changes.
- Control pitch is used for attitude instruments/model pose; raw IMU pitch is
  separately labelled, so the raw mount offset is not mistaken for trim failure.
- Tunables have an exact-value Edit dialog in addition to the slider, including
  signed trim values and large MAXSTR values. Fixed hardware settings stay read-only.

The installable release build uses the stable project **development signing key**,
not a production/Play Store certificate. CI verifies its signature. The release
artifact is `2PyBot-v4.1.1.apk` under the `2pybot-apk-release` Actions artifact.

## Deploy

Install the new APK and deploy the matching bot source/firmware with `bot update`.
After deployment, with local SELECT and a stationary bot:

```sh
bot params get TRIM
bot params get MAXSTR
bot params get STRSLEW
bot params get HUBYAWSL
bot params get INPUTPROFILE
bot params save
```

Expected: -1.4, approximately 2915, 0, 0 and profile 1. A successful build/unit
check does not constitute a physical-device rotation/streaming or robot trial.
