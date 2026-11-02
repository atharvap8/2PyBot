# Current-mode baseline — 5S / 18 V / 2.6 kg

The velocity balancing experiment, velocity gain bank, VSIGN and driver RPM
command path have been removed. `DRVMODE` accepts only `1`; the wire driver
always selects native current mode. The protocol can still recognise a motor
reporting another mode so it can stop/re-handshake rather than misinterpret it.

Retained: bot CLI and automatic SSH environment, firmware recovery/deployment,
original ring animation policy with DMA output, GPIO32 status LED, zoom's
two-second detach, camera/payload and current-mode climbing controls.

## Baseline migration

On the first current/5S profile load, RAM receives these targeted defaults:

- Current mode 1; supply estimate 18 V; mass 2.6 kg.
- Published normal K1..K5 gains and 6 m/s^2 acceleration.
- 2700 mA per-motor current limit, MAXSPD=0 and FREERPM=0.

Trim, encoder/wheel calibration, polarity, IMU settings and payload calibration
are retained. No NVS erase is performed. `bot params save` persists this baseline
and its migration marker. Until saved, a cold boot reapplies the baseline in RAM.
Future saved current-mode tuning is then preserved normally.

The updater recognises `CURONLY=1` / `CURPROFILE=1`, discards removed velocity-only
keys, converts a captured DRVMODE=2 selection to 1 and uses the fresh baseline
instead of replaying experiment gains/acceleration. It preserves other live
tuning/calibration. The first migration starts with normal hold selection rather
than inherited STIFF. Later manual STIFF/climb selections remain available.

After successful deployment, while stopped:

```sh
bot params get DRVMODE
bot params get SUPPLYV
bot params get HUBMASS
bot params save
```

Expected values: 1, 18, 2.6. Supply is a configured estimate, not measured battery
voltage; a 5S pack's actual voltage varies with charge. There is no newly added
voltage/free-speed DRIVEV limiter. E-stop sends zero torque current; it does not
assert the motor's brake bit. Physical balance/climbing still require confirmation
on the bot after the hardware change; the software build alone cannot prove it.
