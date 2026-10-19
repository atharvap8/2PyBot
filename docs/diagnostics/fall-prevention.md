# Predictive fall prevention — local revision 3.9.0

Revision 3.10.0 adds [automatic climb](auto-climb.md) and refines climb recovery:
the mild 25% latch no longer drains slope compensation. Immediate high hazard,
velocity saturation and conditional acceleration anti-windup retain priority.
Climb control is now separately stateful and blended into/out of normal control.

The [recorded reversal test](drive-reversal-20261013.md) showed immediate velocity
reference changes coinciding with reduced terrain attitude feedback and occasional
per-wheel speed overruns. `FallPrevention` is a reference governor and steering
allocator around the existing LQR/LQI, evaluated at the 200 Hz control rate.

## Behaviour

- Operator reference acceleration: at most `min(1.20, 0.35 × AMAX)` m/s²;
  virtual reference jerk: 6 m/s³. A reversal clears virtual acceleration pointing
  away from the new goal; arrival at the goal clears it too. Those virtual resets
  do not limit or reset the LQR's physical recovery acceleration or wheel velocity.
- Predicted tilt: `max(abs(pitchF), abs(pitchF + 0.12 × rateF))`. Pitch and rate
  must share the calibrated forward-positive convention. This short projection
  is a warning signal, not a nonlinear model of the bot's future trajectory.
- Tilt warning: `min(14°, 0.35 × MAXTILT)`; critical:
  `min(24°, 0.60 × MAXTILT)`. Speed risk ramps from 85% to 97% of `VMAX`, using
  the larger absolute measured velocity/common wheel-velocity command.
- Recovery latches as risk rises. It clears after 250 ms with pitch below 6°,
  rate below 35°/s and speed below 80% of `VMAX`. Latched risk is at least 25%.
- Effective drive reference moves towards measured velocity as risk grows,
  reducing operator velocity-tracking error. This lets full selected attitude
  feedback dominate instead of asking the wheels to stop during a balance catch.
- Full pitch/rate gains take precedence over terrain softening during reference
  shaping, reversals, recovery, pitch at least 6° or rate at least 40°/s. Grounded
  stiff hold retains its existing stronger gain set; climb/hold references and
  integrals bleed towards the bot during recovery. Calm normal hold retains
  terrain adaptation, and the existing airborne integral-freeze policy remains.
- Yaw is attenuated by `1 − risk`, then allocated only within
  `MAXSPD − abs(common steps/s)`. Steering slew state is retained separately from
  its allocated output. Common balance command has priority over differential yaw.
- A tighter enabled ToF/heartbeat cap clips the accumulated operator reference
  immediately. It does not cap balance wheel motion, reverse escape or recovery
  acceleration. A released stick remains in braking mode until measured motion
  settles, instead of prematurely switching to position hold.

These initial constants are fixed in `firmware/BaseLink/fall_prevention.h`.
There are still 107 NVS parameters; existing gains, `MAXSPD`, `MAXACC`, arming,
startup acknowledgement and fall/rate cutoffs are preserved.

## Diagnostics

Open `http://10.42.0.1:8080/ui/drive-monitor.html` for live references, risk,
selected attitude feedback and wheel targets. `PR,FALLPREVENT,1` identifies the
firmware capability. [The protocol](../../firmware/BaseLink/PROTOCOL.md) documents
the nine appended debug fields and reason bits. The recorder saves them in both
raw JSONL and CSV, with fallback handling for older firmware.

The monitor and recorder are passive. Start a 45-second drive capture only after
the operator's GO. Compare the same manoeuvres with the earlier test: actual
falls, reversal tracking, risk onset, attitude multiplier, saturation and wheel
headroom. No reduced fall count is claimed before that physical test.

## Software verification

```sh
python -m pytest -q
python3 scripts/diagnostics/check_fall_prevention.py \
  --replay /home/radxa/archives/drive-tests/drive-20261013-131741/telemetry.csv
```

The C++ harness compiles the actual sketch's `runController()` with hardware
dependencies stubbed. It checks both reversal directions, bounded reference
slew, predictive risk, recovery hysteresis, stiff/normal hold, climb, airborne
behaviour, stale ToF reference clipping and steering allocation. Recorded-state
replay checked 1,859 balancing samples, including 187 reversal and 785 recovery
evaluations. It validates constraints at the previously observed states; it
does not simulate a closed-loop trajectory or prove real-world stability.
