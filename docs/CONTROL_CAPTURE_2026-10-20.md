# Full-stick capture: lost attitude feedback

Capture started 2026-10-20 17:51:10 UTC, unrestricted DRIVEV=1.00 m/s,
predictive reference governor off, manual normal policy, no climb blend.
102 HTTP snapshots cover 9.96 seconds, including the full-stick fall.

The production controller calculated
`u = -(K2*(v-reference) + scale*(K3*pitch_rad + K4*rate_rad_s))`.
Terrain roughness changed `scale`, while leaving the velocity-error term intact.

| Time (s) | Pitch | Rate (deg/s) | Speed (m/s) | Scale | Logged acceleration | Full-gain acceleration |
| --- | --- | --- | --- | --- | --- | --- |
| 5.888 | 5.89° | 10.7 | 0.528 | 0.61 | −0.05 | +2.148 |
| 6.096 | 5.85° | 8.4 | 0.608 | 0.55 | +0.03 | +2.445 |
| 6.329 | 7.03° | 20.6 | 0.681 | 1.00 | +5.29 | +5.297 |
| 6.410 | 9.22° | 36.4 | 0.704 | 1.00 | +6.00 (limited) | +8.954 (before limit) |

The reconstructed old acceleration agrees with the recorded values within
telemetry rounding. Attitude feedback was nearly cancelled or reversed near
six degrees, followed by a discontinuous restoration to full gains as the lean
grew. This is a demonstrated software contribution; it is not an invented lean
setpoint or evidence that the IMU angle was being multiplied.

The fix keeps pitch/rate feedback at full strength throughout normal balancing.
It changes neither DRIVEV=1.00 m/s nor the body gains, current limit, hard fall/rate
cutoffs or operator stops. Native regressions replay both recorded frames.

Later in the fall, current commands reached 2.7 A while reported motor currents
were approximately 0.13–0.18 A at 127–130 RPM. Those are additional actuator
tracking observations, not proof that this software fix alone resolves the fall.
HTTP snapshots carry separate debug and hub timestamps; fine timing between
the two streams must not be inferred from their enclosing HTTP sample time.

No motors were commanded by the capture/analysis tools. After the operator's
SELECT, fresh status confirmed IDLE, zero commanded/measured currents and zero RPM.
