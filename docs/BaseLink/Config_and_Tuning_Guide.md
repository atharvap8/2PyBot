# Configuration and Tuning Guide

Reference: [config.h](../../firmware/BaseLink/config.h), [params.h](../../firmware/BaseLink/params.h), [params.cpp](../../firmware/BaseLink/params.cpp), and [BaseLink.ino](../../firmware/BaseLink/BaseLink.ino).

## Defaults, live settings, and saved settings

`config.h` defines pins, sensor axes, peripheral frequencies, and compiled defaults. `params.h` exposes 106 runtime fields. The controller and peripheral modules read the shared `P` store.

At boot, saved NVS values replace defaults when the stored version is recognized. Changing `config.h` does not override an existing saved value.

Connect USB serial at **460800 baud** with newline termination:

```text
P?
P,TRIM,0.071
P,CURRENT,1500
PS
```

The [Radxa console](../../software/radxa/console/README.md) provides the current browser parameter editor and NVS actions. It owns the USB connection while running. TERRAIN keys are available through its API; the browser terrain tab is pending.

- `P?`: current values, bounds, descriptions, and derived conversions.
- `P,key,value`: bounded write; read the `P!` or `PE` reply.
- `PS`: save to NVS.
- `PL`: reload NVS. If no saved version exists, current RAM values are retained.
- `PD`: reset RAM to compiled defaults. Follow with `PS` to replace saved settings.

Legacy `K1=` through `K5=` and `T=` bypass bounds. `M=` changes hardware current without updating the saved parameter field. Prefer the `P,` forms. `P=` now controls camera pan, not a PID gain.

## Hardware and geometry

| Setting | Default | Runtime key |
| :--- | :--- | :--- |
| Shared active-low motor enable | GPIO 32 | Compile-time |
| Motor microsteps | 8 | Compile-time |
| Encoder counts/revolution | 4096 | Compile-time |
| Wheel radius | 0.050 m | `WHEELR` |
| Track width | 0.155 m | `TRACKW` |
| Motor current | 1500 mA | `CURRENT` |
| Common speed ceiling | 8500 microsteps/s | `MAXSPD` |
| Acceleration ceiling | 20000 microsteps/s² | `MAXACC` |
| Full forward command | 0.70 m/s | `DRIVEV` |
| Step timer | 20 kHz | Compile-time |

The radius default corresponds to a 100 mm diameter, despite the older 70 mm comment in `config.h`. Use the loaded radius for odometry calculations. At default geometry, `STEPSM` is about 5093, `COUNTSM` 13038, `VMAX` 1.67 m/s, and `AMAX` 3.93 m/s².

The final left/right split includes steering and is separately limited by the timer frequency. `MAXSPD` is the common velocity ceiling, not a final per-wheel clamp.

## Verify signs before tuning gains

1. Read debug `pitchF` and `rateF` with the motors disabled. Forward tilt and forward rotation should produce positive values in the controller frame.
2. Push both wheels forward and inspect `O` counts. The right count is already negated in the getter. Inspect `velF` for the effect of `ENCSGN`.
3. Verify motor direction with a supported chassis. `LEFT_DIR_INVERT` and `RIGHT_DIR_INVERT` are compile-time flags.
4. Verify left-stick forward and steering direction. `PADFWD` and `PADSTR` are runtime signs.

Use exactly -1 or +1 for sign fields. Their numeric bounds allow intermediate values, but those values scale the measurements instead of simply changing direction.

## Balance parameters

| Keys | Default | Purpose |
| :--- | :--- | :--- |
| `K1` to `K5` | -7.0547, -6.5334, -34.5881, -4.2854, -2.1082 | Normal LQI gain vector |
| `TRIM` | 0.071 degrees | Pitch offset before controller sign correction |
| `ZLIM` | 0.5 m·s | Normal integral clamp |
| `EXCLAMP` | 0.15 m | Position-error/reference bound |
| `VSATBLD` | 3/s | Reference bleed near velocity saturation |
| `BRAKELK` | 0.30 s | Driving hold-reference lookahead |
| `S1` to `S5` | See `config.h` | Stiff-hold gain vector |
| `C1` to `C5` | See `config.h` | Climb gain vector |
| `CLMBZ` / `CLMBV` / `CLMBSTR` | 2.0 / 0.25 / 0.40 | Climb integral bound, reference speed, steering scale |

Start with correct geometry, signs, current, and filter settings. Check saturation flags before attributing drift or oscillation to gains. Change one setting at a time and compare pitch, velocity, position error, and loop timing.

`H,1` selects stiff hold and captures current position. `H,2` selects climb reference tracking. `H,0` returns to normal hold. Arming resets both mode flags, so select a hold mode after arming.

## Input shaping and heading

| Keys | Default | Effect |
| :--- | :--- | :--- |
| `STICKDZ` | 0.10 | Circular deadzone with range rescaling |
| `SNAPIN` / `SNAPOUT` | 10 / 26 degrees | Zero-to-full steering taper away from straight drive |
| `EXPOSTR` / `EXPODRV` | 0.65 / 0.35 | Cubic/linear response blend |
| `LOWDRV` / `LOWSTR` | 0.50 / 0.50 | LOW-mode gamepad authority |
| `BOOTHIGH` | 0 | Speed mode chosen at boot |
| `DRIVEDB` / `STRDB` | 0.03 / 0.05 | Drive-mode and steering thresholds |
| `MAXSTR` / `STRSLEW` | 6000 / 15000 | Differential rate cap and slew, in step-rate units |
| `YAWKP` / `YAWKD` | 1.5 / 0.02 | Encoder-difference heading gains |
| `YAWMAX` | 1200 | Heading-hold correction cap |
| `YAWRELCH` / `YAWTILT` | 2000 counts / 12 degrees | Heading re-latch conditions |

Keep `STICKDZ` above zero and `SNAPOUT` greater than `SNAPIN`. The current code permits values that cause zero denominators. `PADDZ` is retained in the table but the active radial mapping uses `STICKDZ`.

Keep both sticks centered on connection. Center calibration returns before button handling until all axes are within 0.25 of the estimated center. LOW mode only reduces gamepad drive authority; host commands, gestures, and balance recovery are not scaled by it.

## IMU and terrain

| Keys | Default | Purpose |
| :--- | :--- | :--- |
| `IMUCUT` | 50 Hz | Two-stage pitch filter cutoff |
| `MAHKP` / `MAHKI` | 2.0 / 0.005 | Mahony gravity-correction gains |
| `MAGOFFX/Y/Z`, `MAGSGNX/Y/Z` | 0 offsets, +1 signs | Compass correction |
| `TERREN` | 1 | Terrain adaptation enabled |
| `TERRTHR` / `TERRSOFT` | 0.18 / 0.55 | Roughness scale and minimum gain multiplier |
| `TERRTAU` | 0.35 s | Roughness filter time constant |
| `DROPG` / `DROPMS` | 0.55 g / 35 ms | Airborne confirmation |
| `LANDMS` | 350 ms | Softened-gain interval after touchdown |
| `AIRFRZ` | 1 | Suppress airborne integral accumulation |

`C` stops balancing and performs stationary gyro calibration. It does not reset the Mahony quaternion or save bias to NVS. Lowering `IMUCUT` smooths pitch but adds control delay.

Terrain adaptation scales normal K3/K4 and climb C3/C4. Stiff-hold gains are unaffected. Use `P,TERREN,0` for a flat-ground comparison. Airborne detection uses acceleration magnitude, not a wheel contact sensor.

## Driver, ring, and payload tuning

- `SGTHRSL`, `SGTHRSR`, `TCOOL`, `CSSEMIN`, and `CSSEMAX` are applied over driver UART on parameter writes. `CSSEMIN=0` disables CoolStep at runtime.
- Saved driver tuning is not applied by initial driver setup; send `PL` after boot to apply it. Loaded `CURRENT` and `IMUCUT` are used by their initialization paths.
- `LEDBRI`, `LEDFRONT`, and `LEDFPS` control ring brightness, mounting reference, and refresh rate. Use a front index within the physical 16-LED ring.
- Servo travel, home positions, pulse widths, rates, and idle release are runtime parameters. Keep minimum below maximum. Physical pulse output clamps to 0 through 180 degrees.
- `SVRATE` changes how often software writes servo duty; it does not change the fixed 50 Hz PWM carrier.
- `TRCHSTP` and `TRCHMIN` affect torch adjustment. `TRCHBOOT` is exposed but the actual preset still initializes from `TORCH_BOOT_PCT`.

## Auto-tune routines

| Command | Current implementation |
| :--- | :--- |
| `AT,wobble` | Uses 1.5 s pitch-RMS windows; multiplies K3 by 0.93 and K4 by 0.88 when needed; up to eight reductions and a 40% magnitude floor |
| `AT,trim` | Averages pitch in 4 s windows, adjusts TRIM, up to three passes |
| `AT,radius` | Starts a two-metre measurement; completion currently fails because `AT,done` is behind the busy guard |
| `AT,stall` | From idle, increases raw wheel speed by 250 microsteps/s every 120 ms; compares filtered velocity against 70% of expected speed |
| `AT,stop` | Aborts the active routine and restores the saved gain, trim, or speed setting |
| `AT,?` | Prints routine status |

The routine timeout is 45 s. Results remain in RAM until `PS` saves them. Wobble/trim abort if balancing ends or drive input exceeds 0.05.

Stall search runs outside the balance state machine and is not cancelled by `X` or SELECT. Its implemented stop path is `AT,stop`. Radius completion needs a parser fix before it can be used. For now, calibrate radius manually with a measured displacement and `P,WHEELR,value`.

To correct the assumed radius from a known push distance:

$$R_{new}=R_{old}\frac{distance_{actual}}{distance_{reported}}$$

See the [protocol reference](../../firmware/BaseLink/PROTOCOL.md) for complete commands and telemetry fields.
