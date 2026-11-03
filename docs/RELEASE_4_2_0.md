# 4.2.0 — exact firmware values and truthful saving

## Fixed requirements

- Raw neutral `TRIM` is **-1.2603 degrees**.
- Differential steering `MAXSTR` is **4000 encoder counts/sec**.

These are fixed firmware requirements, not just compiled defaults. Their metadata
range is a single value, load/recompute enforces them, legacy T= writes are
bounds-checked, and the updater never replays old values over them. Old NVS values
such as +1.54042 / 1457.37 cannot become effective values in this firmware. The app
shows the actual controller readback; it does not fake a display override.

## Persistence correction

Previously the HTTP endpoint reported success when it merely sent PS; the app
then claimed the values survived reboot. A controller rejection or unavailable
serial writer could therefore look like a successful save. Parameter redumps also
lost dirty flags. These behaviours are removed.

The live controller reproduced the failure: after matching RAM readback and
`PS!,saved`, `PL` reported `PL!,no NVS, using defaults` and reverted the values.
Thus the old save acknowledgement does not establish persistence. The underlying
storage failure still needs the updated MCU's checked writes/diagnostics; no
partition erase or claim of successful hardware repair is made from this result.

The new path is:

1. A parameter write waits for the MCU's matching P! acknowledgement.
2. Save waits for preceding operations and complete controller readback.
3. The MCU checks every NVS write, reopens NVS and reads all stored values back.
4. Only then does it emit PS!,saved. PE errors and missing acknowledgements fail.
5. The API returns confirmed/persisted success only after that acknowledgement and
   a complete readback. The app checks those fields before displaying success.

Save is unavailable while balancing, disconnected, updating, or while writes are
pending. Rejected operations show an error, never a false "saved" toast. Defaults
remain marked unsaved until NVS save succeeds. Dirty flags survive parameter dumps.
TRIM/MAXSTR remain fixed even if saving fails; the UI still reports storage failure.

## Matching host, firmware and app are required

The live check during this report found **firmware 4.1.0**, source `9e0745f`, still
reporting TRIM=1.54042 / MAXSTR=1457.37. An APK update alone cannot change the MCU.
The new app warns about old controller save support and does not trust the old
"command sent" HTTP response.

Press local SELECT, deploy the matching source/firmware and wait for success:

```sh
bot update
bot firmware-status
bot params get PARAMACK
bot params get TRIM
bot params get MAXSTR
bot params save
```

Expected firmware version: **4.2.0**; PARAMACK: **2**; TRIM: **-1.2603**; MAXSTR:
**4000**. Save now reports confirmed/persisted success or a concrete error.

Current-only 18 V / 2.6 kg control, removed predictive governor, direct steering,
two-second orientation hold, persistent video, bot commands and SSH activation
remain. Existing user system-map additions are retained.

## Yaw release and heading response

Heading hold and release braking no longer accumulate a differential-current
integral. A deliberate turn may still use its configured velocity integral, but
reversing a turn clears the previous direction's stored integral immediately.
Together with zero default STRSLEW/HUBYAWSL and final-heading latch after braking,
this removes command lag and stored torque that could pull against a new heading.
`DIRECTYAW=1` identifies this implementation. Physical handling is not inferred
from compilation or a stationary parameter readback.

## No automatic tests

Standing user preference is recorded in AGENTS.md. Automated tests in CI execute
only with explicit opt-in `BOT_RUN_TESTS=true`; compilation, packaging, signing
checks and operation-specific NVS verification remain. No robot motion test is
performed by building or publishing this release.

The release APK uses the stable project development signing key and is named
`2PyBot-v4.2.0.apk`.
