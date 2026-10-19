# BaseLink USB Serial Protocol

Reference: `BaseLink.ino::handleLine()`, `params.cpp`, and `autotune.h`.

## Transport

- USB `Serial`, **460800 baud**, 8N1.
- Newline-terminated text. The parser accepts LF or CR as a terminator.
- Receive buffer: 96 bytes, including the final null terminator. Keep commands within 95 characters.
- Use the uppercase command forms below. Parameter keys and auto-tune routine names are case-insensitive.
- There is no `$` prefix, Bluetooth serial command service, or ESP-NOW receive path.

The current [Radxa console](../../software/radxa/console/README.md) uses 460800 baud. The legacy desktop GUI and older Radxa brain still open at 115200; those host settings must be changed before they can communicate with this firmware.

## Unsolicited streams

| Record | Default rate | Meaning |
| :--- | :--- | :--- |
| `O` | 50 Hz | Encoder odometry and IMU orientation; emitted in both states |
| `D` | Nominal 50 Hz | Controller and terrain debug; enabled at boot, toggled by `L` |
| `HM` | 10 Hz | Hub currents, raw RPM, idle temperatures, faults and link/mode state |
| `[TAG] ...` | On events | Boot, state, gamepad, driver, and tuning logs |
| `A,...` | During auto-tune | Progress, completion, abort, or status |

### Odometry

```text
O,ms,encL,encR,pitchDeg,yawDeg
```

`encL` and `encR` are signed 64-bit counts from the unwrapped M0601 absolute
position, converted to 4096 counts/revolution and per-wheel forward signs.
`ENCSGN` is fixed at +1. Pitch is filtered `imu.getPitch()`, before balance
trim/sign correction. Yaw is tilt-compensated compass heading in degrees.

### Hub feedback (v4)

```text
HM,ms,iCmdL,iCmdR,iMeasL,iMeasR,rpmL,rpmR,tempL,tempR,faultL,faultR,saturated,linkOk,modeOk
```

Commands are robot-forward amps; measured currents and RPM retain motor-wire
signs. `RPMSGN=-1` maps raw RPM to position direction; `LDIR`/`RDIR` then map
to robot forward. Temperatures are last idle-query values (-1 until known);
there are no temperature queries during armed DRIVE polling. Faults are bit
masks. Link/mode and saturation fields are booleans. The host stores these in
`telemetry.hub`, separate from the append-compatible `D` record.

### Debug

```text
D,ms,pitchF,rateF,uOut,velF,ex,vCmdSteps,steerSteps,satV,satA,loopMaxUs,speedHi,balancing,cmdFwd,cmdSteer,accelMag,rough,soften,terrainState,drops,fpRisk,fpPred,fpFlags,driveReq,driveRef,driveEff,balScale,wheelL,wheelR,acEnabled,acState,acPolicy,acBlend,acError,acBias,acFollow,acFlags,clZ,clEx,clRef,fpHazard,clRecover,acStable
```

| Field | Units / meaning |
| :--- | :--- |
| `ms` | Milliseconds since boot |
| `pitchF`, `rateF` | Forward-positive degrees and degrees/s |
| `uOut`, `velF` | Acceleration in m/s², filtered velocity in m/s |
| `ex` | Normal reference error, or climb reference error once blend ≥0.5, metres; use `clEx` for unambiguous climb state |
| `vCmdSteps`, `steerSteps` | Common and allocated differential rates, encoder counts/s in v4 |
| `satV`, `satA` | Velocity/acceleration clamp flags |
| `loopMaxUs` | Largest measured loop interval since the previous debug record |
| `speedHi`, `balancing` | HIGH-speed flag and balance-state flag, 0 or 1 |
| `cmdFwd`, `cmdSteer` | Inputs after authority selection and gesture playback |
| `accelMag` | Acceleration magnitude in g |
| `rough`, `soften` | Roughness estimate and pitch-gain multiplier |
| `terrainState` | 0 flat, 1 rough, 2 airborne, 3 landing |
| `drops` | Confirmed airborne events since boot |
| `fpRisk` | Recovery risk, 0–1; attenuates operator tracking and yaw |
| `fpPred` | Maximum of current and 120 ms projected absolute tilt, degrees |
| `fpFlags` | Bit mask: shaping 1, reversal 2, recovery 4, speed margin 8, airborne 16, yaw limited 32 |
| `driveReq`, `driveRef`, `driveEff` | Requested, shaped operator, and effective recovery velocity references, m/s |
| `balScale` | Actually selected attitude-feedback multiplier, 1 during manoeuvre/recovery |
| `wheelL`, `wheelR` | Allocated wheel step-rate targets before integer conversion; zeroed on disable |
| `acEnabled` | Runtime automatic detection enabled, 0/1; manual policy can override it |
| `acState` | 0 NORMAL, 1 SUSPECT, 2 CLIMB, 3 LIMITED, 4 MANUAL, 5 DISABLED |
| `acPolicy` | 0 automatic, 1 forced normal, 2 forced climb, 3 forced stiff |
| `acBlend` | Actual climb-controller blend, 0–1; 0.75 s handover |
| `acError`, `acBias`, `acFollow` | Filtered reference/velocity error (m/s), pitch-baseline bias (deg), maximum per-wheel following mismatch (m/s) |
| `acFlags` | Intent 1, tracking error 2, pitch bias 4, dynamic inhibit 8, actuator mismatch 16, no progress 32, manual override 64, transition 128 |
| `clZ`, `clEx`, `clRef` | Actual climb integral (m·s), position error (m), shaped climb reference (m/s) |
| `fpHazard` | Immediate fall/speed-margin severity, excluding the mild recovery latch |
| `clRecover` | Climb reference/integral bleed factor, zero for mild latched recovery |
| `acStable` | Steady-request confirmation time, seconds |

The nine fall-prevention fields are appended in local revision 3.9.0; hosts accept
shorter records from older firmware. These targets are not measurements of actual
measured motor rates. When idle, controller/reference fields retain the last
balancing evaluation, except the explicit wheel targets are zeroed on disable.
`soften` remains the terrain proposal; `balScale` reports the selected multiplier.
See [fall prevention](../../docs/diagnostics/fall-prevention.md).
Revision 3.10.0 appends 14 automatic-climb fields. During partial controller
handover, `acBlend` is required to interpret the mixed control law. `clZ` is
recorded controller state; do not reconstruct it from a pure-climb equation
while blend is below 1. `balScale` reports normal-branch feedback blending;
the climb branch uses its full C3/C4 gains. See [auto climb](../../docs/diagnostics/auto-climb.md).

The post-3.10 schema included in 3.20.0 appends ten more fields after `acStable` (53 total):
`acKind,acExit,acExitBlock,acCooldown,dsLimit,dsExcess,dsScale,dsActive,dsGoal,acOverspeed`.
`acKind` is 0 none, 1 uphill/deficit, 2 downhill/braking. State codes additionally
include 6 DOWNCLIMB, 7 EXITING and 8 COOLDOWN. `acExit` is evidence/probe time
in seconds. Exit-block bits are load 1, following 2, dynamics 4, dwell/blend 8,
no clear cue 16. `dsLimit`, `dsExcess`, `dsGoal`, `acOverspeed` are m/s;
`dsScale` is reference scale 0–1 and `dsActive` is a boolean. Extra `acFlags`
bits: descent cue 256, exit evidence 512, re-entry hold 1024, overspeed 2048.
`PR,AUTODESCENT,1` announces this capability. Speed limits govern references;
actual wheel motion remains available for balance recovery.

Parse comma-separated fields. Do not use the legacy tab-separated PID telemetry layout.

## Parameter commands

| Command | Reply | Action |
| :--- | :--- | :--- |
| `P?` | `P#,n`, parameter rows, derived rows, `P.` | Dump all settings |
| `P,key,value` | `P!,key,value` or `PE,reason` | Set one parameter within its bounds |
| `PS` | `PS!,saved` | Save the current parameter store to NVS |
| `PL` | `PL!,loaded from NVS` or `PL!,no NVS, using defaults` | Load NVS when present |
| `PD` | `PD!,defaults restored (not saved)` | Restore compiled defaults in RAM |

Other hardware/log records can appear before a parameter-command reply.

```text
P#,n
P,key,value,min,max,group,description
PR,STEPSM,value
PR,COUNTSM,value
PR,VMAX,value
PR,AMAX,value
PR,VNOLOAD,value
PR,HUBMOTOR,1
PR,HUBSCHEMA,4
PR,TOFGUARD,1
PR,BOOTGATE,1
PR,STARTUPACK,1
PR,FALLPREVENT,1
PR,AUTOCLIMB,1
PR,FLASHLOCK,1
P.
```

Split a parameter row at its first six commas; descriptions may contain commas. Derived fields are read-only. Changes to `WHEELR`, `MAXSPD`, or `MAXACC` recompute the conversion and limit chain.

| Group | Parameters |
| :--- | :--- |
| BALANCE | 15 |
| MOTOR | 21 |
| DRIVE | 17 |
| YAW | 10 |
| SAFETY | 6 |
| IMU | 14 |
| CLIMB | 8 |
| LED | 3 |
| PAYLOAD | 17 |
| TERRAIN | 8 |
| **Total** | **119** |

`params.h` is the complete key/bounds reference. The current Radxa console builds parameter rows from this metadata. It has tabs for nine parameter groups; TERRAIN parameters are exposed by the API but do not yet have a tab. The legacy desktop GUI does not implement this protocol.

The [Android app](../../software/android/README.md) receives parameters through the console API and derives its group list dynamically, including TERRAIN.

Parameter writes reject unknown keys, malformed numeric text and out-of-range
values. `ISGN`/`RPMSGN` are fixed at -1; `ENCSGN` is fixed at +1. Direction
signs must be exactly ±1 and drive/source selectors exactly 0 or 1. NVS
save/load/default operations and mode/direction/radius/source changes require
stopped motors. `P?` is nonblocking, one row per cycle; records can interleave
with telemetry and repeated requests do not restart an unfinished dump.
`TORCH_MAX_PCT` remains a compile-time cap. See [v4 defaults](../../docs/BLDC_V4.md).

## Startup hold and Radxa acknowledgement

With `BOOTWAIT=1`, firmware leaves motors disabled and postpones IMU, driver,
gamepad, payload and LED startup until `BR,1`. `P?` works during the hold so the
host can discover `BOOTGATE`/`STARTUPACK` and send that acknowledgement without
depending on sensor telemetry. `BR!,1` acknowledges host readiness, not arming.

Startup status lines are `BS,WAIT_HOST`, `BS,INITIALIZING`, `BS,CALIBRATING`,
`BS,READY` and `BS,ERROR`. Only BR, read-only P? and stop X are handled before
startup completes; other commands are not queued for later execution.
After READY, local START or serial E is still required to arm. `BR,0` revokes
host readiness and stops an active controller. Saved `BOOTWAIT=0` is an explicit
standalone startup bypass; initialization must still complete before arming.

See [boot coordination](../../scripts/radxa/BOOT_COORDINATION.md).

### Firmware maintenance lock (included in 3.20.0)

`FU,1` requests an arming/raw-diagnostic lock and replies `FU!,1`. An active or
pending arm replies `FU?,BUSY` without stopping balance. `FU,0` replies `FU!,0`
and releases the lock without requesting arm. It also works during startup and
with BOOTWAIT disabled. These are internal bridge/worker commands, not remote
tuner commands. See [app reflashing](../../docs/FIRMWARE_REFLASH.md).

## Drive, state, and expression commands

| Command | Action |
| :--- | :--- |
| `V,fwd,steer,en` | Update host drive input and timestamp; fwd/steer clamped to -1 through 1 |
| `E` | Request arming when upright |
| `X` | Stop balancing |
| `C` | Stop balancing and run blocking gyro-bias calibration |
| `R` | Reset controller dynamics and gestures; retains runtime auto/manual policy; does not disarm |
| `S` | Print gains, state, speed/hold mode, and payload status |
| `L` | Toggle periodic debug records |
| `?` | Print command help |
| `G,yes` / `G,no` / `G,spin` / `G,dance` | Start a gesture while balancing and outside climb mode |
| `G,stop` | Stop gesture playback |
| `H,0` / `H,1` / `H,2` | Force normal / stiff hold / climb; manual policy persists across disarm/rearm |
| `H,3` / `AC,1` | Enable automatic climb and release manual override |
| `AC,0` | Disable automatic climb and select normal; gain transition is blended |
| `A,value` | Heading look offset, clamped to -1 through 1 |
| `F` | Toggle torch |
| `N` | Center camera servos |
| `B=value` | Set torch brightness preset, clamped to runtime minimum and compile-time cap |
| `P=value` | Set requested pan angle |
| `Z=value` | Set requested zoom angle |

Fresh `V` messages override gamepad drive regardless of `en` before a navigation
session. Once a navigation token has been issued, its cancellation latches gamepad
priority until a new explicit navigation grant or controller reset. The enable
field only triggers a balancing stop on a 1-to-0 transition. It never arms. A first
`V,0,0,0` is therefore not an unconditional stop.

### Supervised navigation lease (3.20.0)

`PR,NAVIGATION,1` advertises support. These internal host commands cannot arm:

- `AN,<nonzero-token>,1`: grant only while balancing, local gamepad connected,
  drive sticks neutral and attitude stable. Replies `AN!,<token>` or
  `AN?,<token>`. A cancelled token cannot be reused.
- `AU,<token>,<sequence>,<forward-m/s>,<yaw-rad/s>`: finite 0–0.60 m/s forward,
  at most ±0.75 rad/s yaw, monotonically increasing sequence (32-bit wrap supported).
  The host currently plans at most ±0.45 rad/s. Commands expire after 500 ms.
- `AN,<token>,0`: release that lease without disarming. Local drive-stick input,
  disconnect, disarm, unsafe attitude and timeout also revoke it; stale commands
  cannot restore authority. SELECT remains an immediate motor stop.

Separate 10 Hz feedback leaves the 53-field debug schema unchanged:
`NR,<ms>,<token>,<state>,<sequence>,<command-age-ms>,<pad-connected>,<reserved-zero>`.
States: 0 off, 1 granted, 2 driving, 3 takeover, 4 timeout, 5 stopped, 6 disabled.
Camera/torch/payload position and controls do not gate or revoke navigation.
ToF is advisory for routes: the optional manual-drive guard does not cap a granted
navigation reference. Native turns use physical yaw units without the gamepad deadband. See
[route autopilot](../../docs/ROUTE_AUTOPILOT.md) for host supervision policy.

## Legacy tuning forms

`K1=value` through `K5=value` and `T=value` write the gain/trim fields directly. They bypass parameter bounds but share the same storage saved by `PS`. Prefer `P,K1,value` and `P,TRIM,value`.

`M=value` writes driver current directly without updating `P.motorMa`, bounds checking, or persistence. Use `P,CURRENT,value` for a parameter-store update.

`P=value` controls camera pan. It does not set a balance proportional gain. `I=value`, `D=value`, `$KP=...`, and keyboard `#` drive commands are not current balance commands.

## Auto-tune commands

| Command | Behavior |
| :--- | :--- |
| `AT,wobble` | Measure pitch RMS and reduce K3/K4 in bounded iterations; requires balancing |
| `AT,trim` | Average corrected pitch and adjust TRIM; requires balancing |
| `AT,radius` | Begin a wheel-radius measurement |
| `AT,done` | Intended radius completion; currently blocked by the busy guard during a radius run |
| `AT,stall` | Ramp raw wheel speed from idle and compare measured velocity |
| `AT,stop` | Abort the active routine and restore its saved gain/trim/speed values |
| `AT,?` | Report routine and step |

Routines time out after 45 seconds and do not save to NVS. Use `PS` to save a reviewed result. Wobble and trim abort when balancing stops or drive input exceeds 0.05. Radius and stall have different stop conditions.

Stall tuning drives motors while the balance state is idle. `X` and SELECT do not cancel it in the current code; `AT,stop` is its implemented abort command. See the [tuning guide](../../docs/BaseLink/Config_and_Tuning_Guide.md) for the remaining implementation limits.
