# Radxa / ESP32 boot coordination

## Installed behaviour

1. Firmware starts with `BOOTWAIT=1` and its host-ready flag false. Only serial,
   saved parameter loading/discovery and safe disabled output levels are prepared.
   IMU initialization/calibration, driver startup, gamepad, LEDs and payload are
   held until Radxa sends `BR,1`. An ACK never arms the robot.
2. Once Linux enumerates the CP2102 at `/dev/ttyUSB0`, `2pybot-boot-gate.service`
   opens it exclusively with DTR false and RTS true. On the existing ESP32
   auto-reset circuit this holds EN low while keeping GPIO0 in normal-boot mode.
3. The gate notifies systemd as soon as reset is asserted. The console can then
   start concurrently. It retries the serial connection while the gate owns it.
4. `GET /api/ready` becomes ready when the web API is reachable, the camera
   publisher reports frames and a fresh snapshot, and the hotspot address
   `10.42.0.1` is assigned. Serial is deliberately excluded to avoid a deadlock.
   Invalid depth readings do not prevent host startup; the optional guard handles
   unknown depth separately.
5. The gate releases RTS/reset, closes its serial connection and records
   `/run/2pybot-boot/ready.json`. The console acquires serial and can get `P?`
   before sensors initialize. It recognizes `PR,BOOTGATE,1` and sends `BR,1`.
6. Firmware replies `BR!,1`, then initializes hardware and runs the existing
   three-second settling and roughly one-second gyro sampling sequence. It emits
   `BS,WAIT_HOST`, `BS,INITIALIZING`, `BS,CALIBRATING` and finally `BS,READY`.
   Startup failure emits `BS,ERROR` and leaves motors disabled.
7. After `BS,READY`, put the bot upright and press local START. Early enable/auto-
   tune/actuator commands are not queued through startup. Host readiness depends
   on the host camera/API/hotspot, not fresh IMU telemetry, so the handshake cannot
   deadlock waiting for calibration it has not released yet.

You can leave the bot resting still while Linux boots; it need not be held
upright for those minutes. Gyro bias calibration needs stillness, not an upright
position. Stand it upright only when startup is ready and you want to arm.

Software cannot physically assert EN before Linux can control USB. Firmware's
firmware startup hold covers that initial power-on interval. The physical
USB reset sequence and the firmware handshake were verified on this board.

The six-second console `ExecStartPre` sleep is removed by `45-boot-ready.conf`.
The existing CSI camera power service still orders the console correctly; actual
readiness replaces the fixed delay. Camera/native changes were preserved.

## Observed power-on result

On the tested boot, reset was held for 5.948 seconds, then released at
**13.75 seconds of Linux uptime** with API, camera and hotspot ready. Firmware
then completed its own IMU/driver/gamepad initialization and acknowledged ready.
The bot remained idle and the optional obstacle guard remained off.

Full graphical boot still completed at 37.116 seconds; its HDMI/Desktop jobs
run separately. This is distinct from the robot-services readiness timestamp.
The earlier full graphical timing was 39.516 seconds.

The first software-reboot connectivity check did not regain the hotspot from
the laptop. Windows switched to its normal Wi-Fi. After the operator power-cycle
and hotspot reconnection, the boot record and live services were verified.

## Configuration and restoration

Tracked files:

- `scripts/radxa/boot_gate.py`
- `scripts/radxa/systemd/2pybot-boot-gate.service`
- `scripts/radxa/systemd/86-2pybot-boot-gate.rules`
- `software/radxa/console/45-boot-ready.conf`
- `software/radxa/console/console/readiness.py`
- Firmware `BOOTWAIT` parameter, `BR` handshake and `BOOTGATE` capability.

The deployment monitor substitutes the service template's user, checkout and
virtual-environment paths and installs its units/rules/overrides. On a clean
repo-based installation:

```bash
sudo python3 scripts/radxa/deploy.py --user radxa --force --install-monitor
```

For a live test, with the robot idle, stop the console, restart the boot gate,
then start the console. Do not start a new exclusive reset holder while the
console or a firmware flasher owns the serial port.

```bash
systemctl status 2pybot-boot-gate 2pybot-console
journalctl -b -u 2pybot-boot-gate
cat /run/2pybot-boot/ready.json
curl http://127.0.0.1:8080/api/ready
```

`BOT_BOOT_HOTSPOT` may be empty for a wired host that does not require a hotspot.
`BOOTWAIT=0` explicitly bypasses the firmware host-startup wait for a standalone
ESP32 setup; initialization/calibration must still finish before arming. The reset holder is
separate and must also be disabled for an independent ESP32-only setup.

## Recovery checkpoints

- Original tracked work: Git stash `59c0461f2ab430a68647bd724a94b0e736d53175`.
- Before boot coordination: stash `d5984c67b2c2128c31856eded82e6e2b07cba72a` and
  `/home/radxa/archives/pre-boot-coordination-20261011-021207.tar.gz`, including
  changed and untracked code.
- Latest firmware recovery: `/home/radxa/archives/pre-tof-firmware-20261011-031948/`.
  It contains live bootloader/NVS/partition/OTA bytes and an application rebuilt
  from the pre-gain-update checkpoint (`rollback-rebuilt.bin`, with a verified hash).
  That recovery application includes ToF/boot coordination and the original gains.
  Its application offset is recorded in `app-offset.txt` (0x10000 on this board).
- Previous parameter values before the updated K/S/C gains were saved:
  `/home/radxa/archives/before-lqr-update-20261011-032038/params.json`.

The ESP32 has 16 MB flash with two 1.25 MB OTA application partitions. Application
updates preserved its original bootloader, partitions, OTA selection and NVS.
The working flasher setup was esptool 4.8.1 at 230400 baud; 460800 flash access
was unreliable and bulk live-application reads were unusually slow. Firmware
recovery notes distinguish preserved build artifacts from raw flash dumps.
