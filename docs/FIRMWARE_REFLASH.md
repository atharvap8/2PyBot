# Latest compiled-build reflashing

Android: **Setup → Controller firmware → Reflash latest compiled firmware**.
Browser: `http://10.42.0.1:8080/ui/firmware.html`.

The version, timestamp and SHA-256 identify the exact compiled image. “Latest”
means the latest successful build published to this bot, not a GitHub release
or a file on a disconnected development machine.

## Use

1. Connect, press local SELECT, and wait for fresh IDLE/READY telemetry. Leave
   the camera stream running for the host-readiness/calibration handshake.
2. Tap Reflash. The bot locks arming and raw diagnostics, snapshots live tuning,
   and transfers serial ownership to an independent unprivileged systemd worker.
3. Keep the robot still during restart/gyro calibration. The camera/API remain
   online. Closing the phone app does not cancel an accepted job.
4. Wait for success and READY. Local START is required afterwards.

Only the Huge APP application at **0x10000** is written. The worker backs up and
checks the boot/NVS prefix, verifies the registered previous image against the
actual installed flash, and retains that binary for rollback. It verifies the
new application and startup, restores unsaved RAM tuning without PS, and checks
saved calibration/profile integrity. Failed writes/startup attempt rollback;
failed rollback holds maintenance and reports recovery required. Jobs and logs
persist under `/var/lib/2pybot/firmware/jobs/<id>/`.

`FU,1`/`FU,0` are internal serial maintenance commands, announced by
`PR,FLASHLOCK,1`. Lock acquisition refuses active/pending arming rather than
stopping a balancing robot. Both normal arming and raw auto-tune drive are
blocked while locked, including with BOOTWAIT disabled. Unlock never arms.
These commands are not exposed through the remote tuner whitelist.

## Future builds

```sh
python scripts/firmware/build.py --arduino-cli /path/to/arduino-cli \
  --output-dir build/BaseLink --publish --host 2pybot
```

The wrapper compiles, validates the ESP32 application/segment checksum/SHA and
creates `manifest.json`. Publishing updates latest only after success, using SSH
known-host verification and an interactive password or `--password-stdin`.

For an already compiled directory:

```sh
python scripts/firmware/publish.py build/BaseLink --host 2pybot
```

Images are content-addressed under `builds/<sha256>/`. `latest.json` selects the
next build; `installed.json` records the last verified install. An externally
installed build may be registered once with `--installed-candidate`; it is still
verified on-chip before being used as rollback.

Install the worker once with `sudo python3 scripts/firmware/install_worker.py`.
It accepts repository/user/venv/flasher-interpreter overrides; esptool 4.8.1 and
pyserial must be available in that interpreter. Restart it after code upgrades
only when no job is active. Its separate service survives console/app disconnects
and recovers interrupted jobs. Host readiness is checked before touching serial
at boot, avoiding conflict with the early USB reset gate.

API: `GET /api/firmware/status`; `POST /api/firmware/reflash` with
`{"build_id":"<displayed-sha256>"}`. Stale build selections, busy/unsafe starts
and mutations during flashing return 409. The bridge resets its encoder baseline
on reconnect so an MCU reset does not inject a large false odometry displacement.

Verified on the bot: client closed during update; job continued and succeeded;
duplicate/mutation requests rejected; all 107 tuning values and calibration
preserved; camera remained near 37 FPS; controller returned idle/unlocked.
Write/rollback failure paths were exercised with simulated hardware faults.
