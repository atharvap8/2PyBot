# Automatic climb — local revision 3.10.0

## Revision 3.11 update: exit and downhill operation

The control page now reports **DOWNCLIMB**, **EXITING** and **COOLDOWN**. Uphill
deficit and downhill overspeed/braking lean are evaluated relative to travel
direction; a negative signed velocity error alone does not imply downhill.

Exit confirmation is accumulated evidence, not five uninterrupted perfect
seconds. Acceleration clamp samples do not veto release. Moderate steering is
allowed during hand-back. Quiet neutral input on flat ground can release after
roughly 6–8 seconds; a 1–2 second pause still retains compensation. A staged
normal-controller check can cancel release if load/overspeed returns. Re-entry
is delayed three seconds after successful hand-back to avoid chattering.

Downhill reference speed uses **CLMBV × stick/speed-mode scale** (currently
0.25 m/s full HIGH or 0.125 m/s full LOW). Sustained excess speed progressively
reduces the requested reference toward zero; normal/climb feedback performs the
braking. Deliberate reverse braking is not attenuated. This is a soft reference
limit, not a hard clamp of actual wheel speed: balance catches retain authority,
and inadequate torque/traction can still cause an overrun.

New diagnostic fields are `acKind`, `acExit`, `acExitBlock`, `acCooldown`,
`dsLimit`, `dsExcess`, `dsScale`, `dsActive`, `dsGoal` and `acOverspeed`.
Exit-block bits: load 1, wheel following 2, dynamics 4, minimum dwell/blend 8,
no clear-exit cue 16. `acExit` shows evidence/probe seconds, not a ground-grade
measurement. DOWNCLIMB is inferred assistance, not a measured terrain angle.

Test first on flat ground, then a measured gentle grippy 5–10° ramp in LOW.
Record the session. Check uphill return to flat, neutral flat release, and
forward/reverse gentle descents. Watch actual encoder speed alongside the
descent target and excess-speed fields. SELECT stops the test; do not continue
an uncontrolled or no-progress/LIMITED attempt. A physical descent validation
is still needed after the software/unit checks.

Control page: `http://10.42.0.1:8080/ui/auto-climb.html`.
Charts: `http://10.42.0.1:8080/ui/drive-monitor.html`.

Automatic detection is enabled on firmware boot. It infers sustained load from
control response; body pitch is not a measured terrain slope, and LIMITED is
not a temperature reading. The existing local START arming and SELECT stop remain.

## Detection and switching

- Reference, measured velocity, pitch, signed pitch rate and per-wheel following
  residuals use 0.4 s low-pass filters. Per-wheel velocity is derived from the
  existing common encoder velocity and encoder-differential rate.
- Evaluate intent above 0.15 m/s after normal input shaping/ToF limiting. Require
  1 s of a steady request; reject hard turns (>0.25 steering), airborne/gesture
  activity, immediate hazard ≥0.50, pitch ≥12° or rate ≥120°/s.
- Standard entry: filtered tracking error above `max(0.12 m/s, 35% of reference)`
  and pitch bias ≥2° for another 1.25 s. Filtered mean rate must be below 10°/s.
- Strong-loss fallback: error above `max(0.18 m/s, 65% of reference)` with bias
  ≥1° requires 2.5 s confirmation. This catches severe loss of progress with a
  smaller lean, rather than assuming every loaded surface produces a large tilt.
- Expected pitch starts at ±2° at full normal reference, scaled with speed.
  Healthy normal tracking slowly updates the directional baseline (20 s time
  constant). It is a heuristic low-load baseline, not a ground-angle calibration.
- A following mismatch above `max(0.12 m/s, 30% of wheel command)` for 0.5 s
  sets LIMITED and inhibits new auto entry. It clears after 1 s healthy following.
  If already climbing, retain the current controller rather than suddenly
  weakening it. No progress for 8 s also flags LIMITED; it never auto-arms/stops.
- Entry/exit blends normal and climb controller outputs over 0.75 s. Normal
  reference, C reference, steering scale and feedback handover are bounded.
  Climb state is separately initialized to match the normal acceleration as
  closely as its saved integral bounds allow.
- A brief neutral pause retains automatic climb on the slope. Exit requires
  continued motion intent, healthy tracking/lean/rate, low hazard and 5 s flat
  confirmation after at least 3 s active climb. Isolated acceleration caps
  pause/slow confirmation; they do not trigger an immediate exit.
  Active progress/exit accepts requests above 0.06 m/s, so LOW-mode 0.125 m/s
  climbing can return to normal; automatic entry still requires 0.15 m/s.

Saved `CLMBV` still sets full-stick climb speed (currently 0.25 m/s); gamepad LOW
reduces the request as before. All 107 NVS settings remain intact. The initial
detector thresholds live in `firmware/BaseLink/auto_climb.h`.

## Fix for the pg-to-home failure

Revision 3.9 leaked the climb integral at 0.5/s whenever the mild 25% recovery
latch persisted. At the 0.15 m position-error clamp this restricted steady
compensation to about 0.30 m·s despite a saved 2.0 m·s capacity.

Now `fpHazard` is immediate severity, distinct from latched `fpRisk`. Climb bleed
is zero below hazard 0.35, ramps to full at 0.80, and is full when airborne.
The mild latch retains full balance feedback and steering priority but no longer
erases the C integral or cuts C-reference progress by 25%.
Conditional anti-windup rejects only new integral motion that would drive the
mixed acceleration deeper into its clamp. Velocity saturation still bleeds the
reference/integral. The C branch uses full C3/C4 gains; acceleration/speed limits
and the original fall/rate cutoffs stay active.

## Manual controls

| Action | Result |
| --- | --- |
| Enable auto / `AC,1` / `H,3` | Automatic policy; releases a manual override |
| Disable auto / `AC,0` | Automatic detection off; blended return to normal |
| D-pad DOWN | Select manual climb if not targeted; otherwise manual normal |
| D-pad UP | Toggle manual stiff/normal |
| `H,2` / `H,0` / `H,1` | Force climb / normal / stiff |
| SELECT | Stop motors |

Policy selection persists across disarm/rearm and controller reset during the
same firmware boot. Power cycling returns to auto enabled. None of the mode
controls arms motors. The web page shows both **policy** and **blend**; a manual
normal selection takes up to 0.75 s to fade an existing climb controller out.

## Test procedure

1. **Record first.** Start the Android recorder and label each stage, or request
   a timed passive capture on GO. New debug JSON includes auto state and actual
   C integral/error/reference, so no integral reconstruction is needed.
2. **Flat ground, LOW speed.** Wait for READY; press local START, click Enable
   auto, and drive steadily for 10–15 s. Expect NORMAL and blend 0. Accelerate,
   release, reverse and turn normally; short transients should not latch climb.
3. **Measured gentle ramp, LOW speed.** Use a clear, grippy roughly 5–10° ramp
   with space to recover. Keep a steady straight request; do not hold the wheels
   against a stop. If persistent load cues qualify, expect SUSPECT then CLIMB,
   blend rising to 100%, and an `[AUTOCLIMB] ENTER` event. Standard confirmation
   is about 2.25 s from a new steady request (strong-loss fallback about 3.5 s),
   plus filtering and the 0.75 s handover. Adequate normal tracking may need no switch.
4. **Pause briefly on the ramp.** Neutral for 1–2 s should retain compensation.
   Resume smoothly. A mild recovery latch near 25% should show climb bleed 0;
   the C integral may exceed 0.30 m·s if needed, within saved CLMBZ and anti-windup.
5. **Return to flat.** Continue straight gently for 5–8 s. Once tracking and lean
   return near the baseline, expect `[AUTOCLIMB] EXIT` and blend fading to 0.
6. **Manual override.** Tap D-pad DOWN: verify policy MANUAL CLIMB/normal and
   detector MANUAL. Click Enable auto to restore automatic behaviour. SELECT
   stops the test. A failed/no-progress or LIMITED attempt calls for stopping
   and inspecting load/driver/supply, not holding full input indefinitely.
7. **Warm comparison.** After a successful cold test, repeat the same ramp after
   a longer drive with the same mode/load. Measure motor/driver temperatures
   and battery voltage separately; firmware still has no thermal/supply sensor.

Save the new session, note ramp angle, surface, load, speed mode, cold/warm state,
entry/exit delays and any actual falls. Software replay checks constraints and
detector logic; physical automatic climbing remains to be validated by this test.

## Verification

`python -m pytest -q` checks host/schema/startup/vision compatibility.
`python3 scripts/diagnostics/check_fall_prevention.py --replay <45-second-CSV>`
compiles the actual production controller against hardware stubs, checks mild
integral retention/anti-windup and bumpless bounded transitions, then runs auto
detector cases for flat driving, both directions, overspeed, turns, impacts,
manual precedence, pauses, actuator mismatch and delayed exit.

The application-only flash is preceded by firmware/NVS/settings/calibration
backups and followed by startup-hold, capability, runtime-policy and idle-output
readback. The existing startup ACK gate still holds IMU calibration until BR,1.
