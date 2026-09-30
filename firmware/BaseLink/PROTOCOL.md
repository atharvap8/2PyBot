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
| `D` | Nominal 100 Hz | Controller and terrain debug; enabled at boot, toggled by `L` |
| `[TAG] ...` | On events | Boot, state, gamepad, driver, and tuning logs |
| `A,...` | During auto-tune | Progress, completion, abort, or status |

### Odometry

```text
O,ms,encL,encR,pitchDeg,yawDeg
```

`encL` and `encR` are signed 64-bit encoder counts from `StepperControl`; the right getter already negates its hardware count. They do not include `P.encSign`. Pitch is filtered `imu.getPitch()`, before balance trim/sign correction. Yaw is tilt-compensated compass heading in degrees.

### Debug

```text
D,ms,pitchF,rateF,uOut,velF,ex,vCmdSteps,steerSteps,satV,satA,loopMaxUs,speedHi,balancing,cmdFwd,cmdSteer,accelMag,rough,soften,terrainState,drops
```

| Field | Units / meaning |
| :--- | :--- |
| `ms` | Milliseconds since boot |
| `pitchF`, `rateF` | Forward-positive degrees and degrees/s |
| `uOut`, `velF` | Acceleration in m/s², filtered velocity in m/s |
| `ex` | Unclamped `posM-xRef`, metres |
| `vCmdSteps`, `steerSteps` | Common and differential step rates, microsteps/s |
| `satV`, `satA` | Velocity/acceleration clamp flags |
| `loopMaxUs` | Largest measured loop interval since the previous debug record |
| `speedHi`, `balancing` | HIGH-speed flag and balance-state flag, 0 or 1 |
| `cmdFwd`, `cmdSteer` | Inputs after authority selection and gesture playback |
| `accelMag` | Acceleration magnitude in g |
| `rough`, `soften` | Roughness estimate and pitch-gain multiplier |
| `terrainState` | 0 flat, 1 rough, 2 airborne, 3 landing |
| `drops` | Confirmed airborne events since boot |

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
P#,106
P,key,value,min,max,group,description
PR,STEPSM,value
PR,COUNTSM,value
PR,VMAX,value
PR,AMAX,value
P.
```

Split a parameter row at its first six commas; descriptions may contain commas. Derived fields are read-only. Changes to `WHEELR`, `MAXSPD`, or `MAXACC` recompute the conversion and limit chain.

| Group | Parameters |
| :--- | :--- |
| BALANCE | 15 |
| STEPPER | 9 |
| DRIVE | 17 |
| YAW | 10 |
| SAFETY | 5 |
| IMU | 14 |
| CLIMB | 8 |
| LED | 3 |
| PAYLOAD | 17 |
| TERRAIN | 8 |
| **Total** | **106** |

`params.h` is the complete key/bounds reference. The current Radxa console builds parameter rows from this metadata. It has tabs for nine parameter groups; TERRAIN parameters are exposed by the API but do not yet have a tab. The legacy desktop GUI does not implement this protocol.

The [Android app](../../software/android/README.md) receives parameters through the console API and derives its group list dynamically, including TERRAIN.

Writes reject unknown keys and out-of-range values. Numeric text is parsed with `atof`, so malformed text may become zero. Bounds are per parameter and do not enforce relationships between settings. `TORCH_MAX_PCT` remains a compile-time cap, not a runtime parameter.

## Drive, state, and expression commands

| Command | Action |
| :--- | :--- |
| `V,fwd,steer,en` | Update host drive input and timestamp; fwd/steer clamped to -1 through 1 |
| `E` | Request arming when upright |
| `X` | Stop balancing |
| `C` | Stop balancing and run blocking gyro-bias calibration |
| `R` | Reset controller reference, integral, velocity, steering, gestures, and hold modes; does not disarm |
| `S` | Print gains, state, speed/hold mode, and payload status |
| `L` | Toggle periodic debug records |
| `?` | Print command help |
| `G,yes` / `G,no` / `G,spin` / `G,dance` | Start a gesture while balancing and outside climb mode |
| `G,stop` | Stop gesture playback |
| `H,0` / `H,1` / `H,2` | Normal hold / stiff hold / climb mode |
| `A,value` | Heading look offset, clamped to -1 through 1 |
| `F` | Toggle torch |
| `N` | Center camera servos |
| `B=value` | Set torch brightness preset, clamped to runtime minimum and compile-time cap |
| `P=value` | Set requested pan angle |
| `Z=value` | Set requested zoom angle |

Fresh `V` messages override gamepad drive regardless of `en`. The enable field only triggers a balancing stop on a 1-to-0 transition. It never arms. A first `V,0,0,0` is therefore not an unconditional stop.

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
