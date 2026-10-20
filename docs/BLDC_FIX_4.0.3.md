# BLDC 4.0.3 — analysis and fix (2026-10-20)

**Historical proposal:** the free-speed/torque explanation below was a model
hypothesis, not an established cause of the hardware falls. The current branch
removes its SPDHEAD-derived stick-speed cap and bypasses the predictive reference
governor for the requested unrestricted diagnostic. See `DEVELOPMENT_SOP.md`.

Status: **source fixed and native checks pass. Not flashed, not physically tested.**
This session had no shell on the PC and could not reach the bot (10.42.0.1 is
only reachable from your LAN), so nothing was built for the ESP32, flashed or armed.

## 1. Root causes

### A. Neutral-balance regression (4.0.1/4.0.2): the RS485 transport
The supplied BaseLinkHub sends one frame per motor every 5 ms cycle,
unconditionally. 4.0.1 changed this to "one outstanding request per UART":

- No new DRIVE frame is sent while the previous reply is still pending.
- A reply not parsed within 8 ms triggers a >=10 ms quiet resync, then a QUERY,
  and only then another DRIVE frame.

Consequences on the real robot:

- **Loop jitter halves the torque update rate.** If a cycle's compute time pushes
  the reply past the next `service()` (the 2026-10-20 capture shows 10 ms loop
  intervals), that wheel skips its next DRIVE frame.
- **One lost/late reply freezes that wheel's current for ~25 ms**, which is
  4–5 control cycles of stale torque during balance. The two wheels also
  desynchronise, because 4.0.2 then serviced each wheel independently.

Simulation (`tools/sims/bldc_transport_sim.cpp`, using the real
`hub_control.h`/`fall_prevention.h`). The model includes 3.5 ms debug-print
compute every 4th cycle, 1% 6 ms CPU spikes and 2% lost replies:

| Scenario (20 seeds) | 4.0.2 as flashed | 4.0.2 control + supplied transport |
|---|---|---|
| Arm 2.9°, −25°/s | 1 fall, peak 55° | 0 falls, peak 2.9° |
| Shove 0.04 N·m·s | 2 falls | 0 falls, peak 1.4° |
| Drive 0.40 m/s | 3 falls | 0 falls |
| Harsher (5% loss, 2% spikes): arm | **8/20 falls** | 0 falls |

With ideal timing, all variants balance. The control-law changes (4/4/0.5,
direct acceleration feed-forward, recovery gate) were **not** the cause in the
model. The 6/8/1 loop was actually *less* robust to plant-inertia uncertainty.
They are left unchanged so that only one variable changes.

### B. Full-stick FELL (original firmware, and still present in 4.0.2): physics
M0601 free speed at 12 V is 133 rpm ≈ **0.71–0.73 m/s** (supplied README §6).
Balancing needs spare speed and torque to catch a forward lean. At the free
speed the available torque is zero. The supplied firmware let the reference
reach 100% of free speed (0.725 m/s). The 4.0.1/4.0.2 builds asked for
1.00 m/s, which these motors cannot reach at 12 V at all. Simulation:

| Reference / free speed | Steady drive | Shove at top speed |
|---|---|---|
| 0.60–0.70 | stable | survives |
| 0.80–0.90 | stable | **falls 20/20** |
| 1.00 m/s @ 12 V (4.0.2) | **falls 20/20** | – |

No inner-loop gain change fixes this: the voltage limit itself is the constraint.

## 2. Changes in 4.0.3

| File | Change |
|---|---|
| `m0601.h/.cpp` | Removed send gating, resync and reconfirm-QUERY. A frame always goes out. A missing reply is counted (`timeouts`) and forgotten after 8 ms. When the outstanding requests differ in kind (DRIVE vs QUERY), a late reply updates only the common fields (mode, current, rpm, fault) and never position or temperature. |
| `hub_drive.h/.cpp` | `transmit()` sends both wheels every cycle. An unhealthy wheel gets the MODE/QUERY handshake, and a healthy wheel gets a zero-current DRIVE frame while disarmed. Fault-byte transitions are logged with their meaning. |
| `params.h/.cpp`, `config.h` | New `SPDHEAD` (0.70) and derived `driveVEff = min(DRIVEV, SPDHEAD × free speed, MAXSPD)`, read back as `PR,VDRIVE`. Free speed = `FREERPM` if measured, else `SUPPLYV × Kv`. |
| `BaseLink.ino` | All full-stick scaling uses `driveVEff`, and navigation velocity is clamped to it. Version 4.0.3. |
| tests | `test_m0601`, `test_bldc_drive`, `test_bldc_controller`, `test_bldc_params` updated for the new semantics. All 5 `check_bldc.py` executables pass (g++; the Linux stubs needed `-include cstddef`, unrelated to this change). |

Unchanged on purpose: ISGN/RPMSGN −1, HUBVSRC 1, LQR gains, HUBKP/KI/ILIM
4/4/0.5 (your NVS already holds these), MAXTILT/MAXRATE, CURRENT, and fault,
link-age and loop-overrun disarms.

## 3. What full stick means now

| Free speed source | Full-stick HIGH |
|---|---|
| 12 V estimate (0.725 m/s) | **0.51 m/s** |
| Measured 0.81 m/s (consistent with the capture's peak) | 0.57 m/s |
| 4S full, 16.8 V (~1.0 m/s free) | 0.70 m/s |
| 5S, 21 V (~1.24 m/s free) | 0.87 m/s |
| 1.00 m/s requires | free speed ≥ 1.43 m/s, or SPDHEAD raised (less catch margin) |

The M0601 is rated 18 V, maximum 24 V. Do not use 6S.

## 4. Bring-up procedure (you need to run this; it is hands-on)

1. Build with your usual command (`--output-dir build\BaseLink-v4.0.3`) and flash.
2. Disarmed: `S` shows both motors in mode 1, fault 0, and a rising frame count.
   `P?` shows `PR,VDRIVE`.
3. Wheels off the ground: `AT,stall` records `FREERPM`, then `PS`. This raises
   full stick to 0.70 × your real free speed.
4. Hand under the robot, arm at neutral and hold it for 30 s. In `S`
   afterwards, `timeout` counts should grow slowly at most. Any
   `[HUB] … fault 0x..` line is a real motor trip.
5. Only then try LOW, then HIGH full stick.

If neutral balance still fails, capture the `[MAIN] Motors OFF (...)` line and
the D/HM stream from the 2 s before it. The disarm reason now names the trigger.
