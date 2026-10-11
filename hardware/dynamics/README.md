# Printed parts and updated LQI calculation

Use the current STLs in `models/`; the older simplified STEP was explicitly
excluded. The two charger parts are not mounted. `beam_D` is mounted in front.

## Re-sliced mass estimates

Bambu Studio 02.08.02.61, H2S 0.4 mm nozzle, 0.20 mm Standard,
eSUN PLA+ (1.25 g/cm³), 15% infill, two wall loops, five top / three bottom
layers, automatic orientation, no supports. Profiles were flattened before
CLI slicing: loading inherited preset JSON directly does **not** populate all
settings (it otherwise used 20% infill and zero filament density).

| Part | Estimated grams |
| :--- | ---: |
| Base back | 25.18 |
| Base front | 24.05 |
| Face | 16.81 |
| Motor left | 41.30 |
| Motor right | 41.31 |
| Shell | 211.61 |
| Front beam | 5.50 |
| **Total** | **365.76** |

These are new slices, not the original H2S/P1S jobs. Brim/purge in the slicer
total, actual orientation, support removal and material variation affect the
result. The operator estimates approximately 400 g for the additions and 1.5 kg
before covers, giving approximately **1.87–1.90 kg all-up**.

## Dynamics and reproduced original gains

Firmware input is wheel **acceleration**, not motor torque or horizontal force.
The inherited linear model uses states `[x, v, theta, omega, integral(x)]` and:

```text
x_dot = v
v_dot = u
theta_dot = omega
omega_dot = (g*theta - u) / L_eff
integral_dot = x
u = -K @ state
L_eff = J_about_axle / (m * h)
```

Mass alone therefore cannot determine a new gain vector. At unchanged geometry,
uniformly adding mass cancels out of this ideal acceleration-input model;
real motor torque and acceleration headroom still depend on mass.

`scripts/lqr_design.py` reproduces all three existing vectors within 0.0001 at
`L_eff = 0.15 m`. The normal weights are **Q = diag(10,4,17.5,0.3,2), R = 0.45**.
The previous normal-weight comment in `config.h` was inconsistent with the gains.
Stiff and climb use the weights already documented in that file.

## Updated estimated design

`lqr_candidate.json` includes CAD centroids/inertias, the calculation, serial
commands, fixed-gain geometry/latency validation and acceleration-limited recovery
simulations. Inputs are estimates rather than a measured identification:

- Baseline all-up mass 1.5 kg, less an estimated 0.16 kg of wheels: body mass 1.34 kg.
- Baseline body COM height 0.06 m, estimated from heavy motors near the axle and
  battery/electronics above it; baseline effective length 0.15 m reproduces the
  original gains.
- Pitch axis parallel to STL X; axle Y = 45 mm, Z = -64.15 mm, inferred from
  enclosure dimensions matching the current STL bounds: 40 mm motor standoffs,
  3 mm bracket, and half of the 42.3 mm NEMA17 body height.
- Printed material distributed uniformly over each STL solid at its re-sliced mass.

This gives body COM height 0.068276 m, pitch inertia 0.017802 kg·m² about the axle,
and `L_eff = 0.152858 m`. Estimated body pitch inertia increases about 47.6%.
The normal and climb pitch/rate weights were retuned because the original
terrain-softened vectors became unstable in the delayed/filtered model.

Updated CARE weights:

- Normal: Q = diag(10,4,52.5,8,2), R = 0.45.
- Stiff: Q = diag(600,40,90,1.5,200), R = 0.30.
- Climb: Q = diag(50,30,160,20,20), R = 0.30.

| Mode | 1 | 2 | 3 | 4 | 5 |
| :--- | ---: | ---: | ---: | ---: | ---: |
| Normal K | -7.3234 | -7.4495 | -42.9506 | -6.6862 | -2.1082 |
| Stiff S | -60.0226 | -31.0364 | -82.0431 | -10.2571 | -25.8199 |
| Climb C | -22.2369 | -20.0745 | -81.3063 | -12.7029 | -8.1650 |

All 420 sampled-loop cases pass with the SAME rounded gains: effective length
0.11–0.20 m, firmware's cascaded 50 Hz pitch / 15 Hz velocity EMAs, actuator lag
0–15 ms, an additional control-cycle delay, normal velocity mode and 0.55 terrain
softening. Stiff hold follows the firmware's unsoftened policy.

All 45 nonlinear recovery scenarios pass with 3.927 m/s² acceleration and
1.669 m/s velocity ceilings, 15 ms lag and one-cycle delay. These include ±2°
initial lean and a 20°/s shove. Traction, lost steps, slopes and unmodelled
wheel/chassis coupling remain outside these model checks. The robot was kept
idle during gain installation; this is model validation, not a physical
balancing trial. Application/NVS readback information is stored in the report.

The compiled defaults and runtime K/S/C parameters use this estimated design.
The generator itself performs no serial writes or motor actions.

```powershell
python scripts/lqr_design.py --output hardware/dynamics/lqr_candidate.json
# After effective-length identification, for example:
python scripts/lqr_design.py --effective-length 0.16
python -m pytest tests/test_lqr_design.py tests/test_tof.py -q
```
