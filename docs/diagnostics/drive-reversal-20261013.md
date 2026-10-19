# Drive reversal test — 13 October 2026

## Test

Operator scenario: full joystick up, immediately down-right, with the bot's front
facing the operator while moving towards them. Recording started on GO and lasted
45 seconds. The recorder used existing read-only HTTP telemetry; no controller,
arming, gain, guard or speed settings were changed for the test.

Saved data on the Cubie:
`/home/radxa/archives/drive-tests/drive-20261013-131741/`

- 3,795 polls; 3,489 distinct debug samples (77.53 Hz observed).
- 2,180 distinct odometry samples (48.44 Hz observed).
- No HTTP recording errors or stale telemetry samples.
- Worst reported control-loop interval: 5.359 ms.
- ToF guard was disabled throughout this recording.

## Observed events

| Event | Reversal at | Approximate input change | Stop at | Firmware reason |
| ---: | :--- | :--- | :--- | :--- |
| 1 | 4.985 s | +1.00 to −0.41 in 53 ms, steer −0.66 | 5.345 s | FELL |
| 2 | 12.566 s | −0.96 to +0.96 in 51 ms, steer −0.01 | 12.880 s | FELL |
| 3 | 18.863 s | +0.94 to −0.41 in 103 ms, steer +0.67 | 19.720 s | RATE SPIKE |
| 4 | 33.928 s | −0.99 to +0.76 in 93 ms, steer 0.00 | 34.256 s | FELL |

This reproduces the reported reverse-and-turn failure, and also shows failures
during straight reversals in the opposite direction. It is not explained solely
by right steering or by the bot facing the operator.

## Main evidence

The drive speed setting was 0.908784 m/s. The normal velocity-mode controller
changes its desired velocity immediately with the joystick. During reversal,
the wheels were still moving in the old direction and measured speed reached
about 1.2–1.46 m/s in some recovery transients.

At the same time, terrain adaptation reduced the pitch and pitch-rate gains to
55–63% of their nominal values while leaving the velocity term unscaled.
The recorded manoeuvres were classified as rough terrain. `terrain_update()`
includes pitch-rate magnitude in its roughness estimate, so commanded pitching
can contribute to this classification.

Reconstructing the normal drive equation with recorded inputs reproduces the
reported acceleration command closely:

| Time | Balance feedback factor | Recorded acceleration | Reconstructed acceleration | Same state with full pitch/rate gains |
| :--- | ---: | ---: | ---: | ---: |
| 5.063 s | 0.63 | +1.62 m/s² | +1.61 m/s² | −4.78 m/s² before clamping |
| 12.680 s | 0.55 | −0.53 m/s² | −0.52 m/s² | +13.26 m/s² before clamping |
| 18.894 s | 0.71 | +2.55 m/s² | +2.56 m/s² | −0.11 m/s² before clamping |
| 34.024 s | 0.55 | −2.30 m/s² | −2.30 m/s² | +8.57 m/s² before clamping |

These are arithmetic comparisons at the same observed state, not predictions
that a gain change alone will prevent a fall. They show that softening changed
the command sign at important points. Once the tilt grew, recovery repeatedly
hit the approximately ±3.927 m/s² acceleration limit.

## Secondary issue

Steering is added after the common velocity command is limited. Per-wheel targets
can therefore exceed `MAXSPD` even when the common velocity saturation flag is
clear. This test reached 8,522 and 8,597 steps/s against an 8,500 setting.
`setSpeeds()` only clamps to the 20,000 Hz timer limit, not the tuned speed ceiling.

Those small overruns are not sufficient evidence to explain all four falls:
two events had no recorded overrun. Balance-priority steering allocation is still
worth correcting independently.

## Recommended next controlled changes

1. Shape/rate-limit the operator velocity reference, especially reversal, without
   limiting the fast acceleration needed by the balance controller itself.
2. Preserve full pitch/rate recovery feedback during aggressive manoeuvres and
   growing tilt; distinguish intended motion from rough-surface adaptation.
3. Allocate steering from remaining wheel-speed headroom so differential yaw
   does not consume the common balance command's actuator margin.
4. Repeat the same timed test after one change at a time, comparing reversal
   tracking, recovery saturation and actual falls. Keep the existing fall/rate
   cutoffs; raising them would not fix loss of control.

## Measurement limits

`fwd`/`str` are firmware-reported inputs after arbitration/shaping/guard, not raw
Bluetooth ADC values. Left/right targets were reconstructed as `vCmd−steer` and
`vCmd+steer`; they are not independent measurements of ISR-applied motor rate.
Encoder counters and measured/filtered velocity were also saved. The console
was polled, so some 100 Hz debug frames were missed; actual observed rates are
reported above rather than assuming every MCU frame was captured.
