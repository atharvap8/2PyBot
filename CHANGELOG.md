# Platform revisions

For the main milestones and estimated dates for future work, see the
[Revision Table](docs/REVISION_HISTORY.md).

## 3.40.0 — Routes landscape rendering fix

- Preserve the active workspace when switching between the bottom bar and rail,
  including landscape windows narrower than 600 dp.
- Bind WebView lifecycle handling to its actual instance and release it only
  when AndroidView releases that instance; use explicit bounded view sizing.
- Give the embedded route page definite viewport height, a compact landscape
  layout and scrollable fallback for very short windows. Add native Reload/error state.

## 3.30.0 — adaptive Android cockpit

- Rebuild navigation around five primary destinations and a compact landscape rail.
- Redesign Drive/Live/Map around a dominant scene and bounded instrument panes;
  correct live camera framing to 16:9.
- Add adaptive Camera/Setup/Sessions columns and searchable lazy parameter rows.
- Open Routes directly, with a responsive embedded map/teaching workspace.
- Unify surfaces, typography, spacing and insets; preserve the installed controller
  and original-command replay behavior. See [design notes](docs/ANDROID_COCKPIT_DESIGN.md).

## 3.20.0 — post-3.10 development revision

Numbering follows **3.10.0 → 3.20.0 → 3.30.0**. The prototype features below
are consolidated into 3.20.0. Existing local artifacts and recorder metadata
retain their original identifiers for hash verification and rollback.

### Original-command replay and more responsive reference governor

- Record timestamped 20 Hz speed/yaw/odometry samples, preserving reverse,
  pauses and original timing. Remove the replay-only speed/yaw ceiling, host
  ramp, duration/length caps and predictive-recovery mission interruptions.
- Remove the speed selector for recorded replay; old waypoint-only routes need
  re-teaching to obtain timing that was never stored.
- Double reference acceleration allowance, triple jerk allowance, reduce
  prediction horizon and recovery latch, and make mild hazard attenuation quadratic.

### Supervised routes / waypoints

- Teach start-relative wheel-odometry routes, preview them, and explicitly replay
  them from the marked physical start; include a ready-to-create 2 m straight route.
- Default cruise target is 0.6 m/s, including climb assistance; dense taught
  waypoints do not impose a hidden 0.25 m/s limit. Slow for turns and goal approach.
- Keep ToF advisory only: no depth prerequisite, obstacle hold or distance speed
  cap for routes. Remove camera/payload-position and payload-input dependencies.
- Add visible-page supervision, controller watchdog and latched drive-stick takeover.
- Add native navigation leases and separate 10 Hz navigation feedback; retain
  all saved parameters, startup gating and local-only motor arming.
- Expose the route page from Android Auto and the browser console.

### Android latest-build reflash and independent bot worker

- Add Setup firmware controls with compiled version/hash, progress and result.
- Register validated successful builds and publish over SSH; pin each request
  to the displayed build ID.
- Keep camera/API online while an independent unprivileged service flashes
  only the app, verifies rollback and boot/NVS, restores tuning and checks calibration.
- Add maintenance arming/raw-drive lock, durable recovery, duplicate/mutation
  rejection and serial reconnect baseline reset. Updates never auto-arm.
- Document pitch-gimbal and vibration-isolated camera/ToF pod recommendations.

### Reliable hand-back and controlled descent

- Separate uphill deficit from direction-aware downhill/braking cues, including
  sustained uncommanded rolling. Add DOWNCLIMB, EXITING and COOLDOWN states.
- Replace the perfect-sample exit timer with accumulated evidence and monitored
  normal-controller hand-back. Permit quiet flat-ground neutral release while
  retaining short pauses on a slope, and reject failed hand-backs.
- Add a descent-reference governor using saved CLMBV, measured overspeed and
  gradual recovery. Preserve reverse braking and full wheel balance authority.
- Append ten exit/descent diagnostic fields; retain all 107 saved parameters.

## 3.10.0 — automatic load-aware climb and retained slope compensation

- Detect sustained speed-reference error and pitch-baseline bias with 0.4 s
  filters, request confirmation, per-wheel following checks and hysteresis.
- Blend normal/climb control and speed/steering authority over 0.75 s; preserve
  compensation during brief neutral stops and confirm flat recovery for 5 s.
- Give D-pad/H selections a persistent manual override, with AC,1 / H,3 to
  resume auto and AC,0 to disable it. These commands never arm the robot.
- Fix the pg-to-home integral regression: mild 25% recovery no longer drains
  climb compensation; high immediate hazards and velocity saturation still
  bleed state, with conditional acceleration anti-windup.
- Add 14 diagnostic fields, an auto-climb control page and recorder/monitor
  support. Preserve all 107 saved parameters and the startup/calibration gate.

## 3.9.0 — predictive fall-prevention reference governor

- Shape operator speed requests with bounded reference acceleration/jerk and
  predict near-term tilt and wheel-speed margin before the existing fall cutoff.
- Prioritize full selected pitch/rate feedback during manoeuvres/recovery,
  attenuate velocity-tracking error and yaw, and retain stronger stiff-hold gains.
- Allocate steering from per-wheel speed headroom without reducing common balance
  acceleration or changing the 107 saved parameters, fall cutoffs or startup gate.
- Append override/reference/wheel-target debug fields and expose them in the
  read-only drive monitor and recorder. Add production-controller constraint tests
  and recorded-state replay; physical fall reduction requires a repeat drive test.

## 3.8.0 — ONNX objects viewer

- Add a bounded snapshot-based SSD-MobileNetV1 INT8 detector, COCO labels,
  confidence/person filters and temporary class-aware track IDs.
- Add web/Android Objects viewers and inference/frame-age metrics; saved lens
  correction can be used without guessing camera–ToF object distances.
- Provide a fixed checksum-verified model installer and optional ARM64 CPU runtime.

## 3.7.0 — hold firmware initialization until Radxa ACK

- Move IMU/driver/gamepad/payload/LED startup and gyro calibration behind BR,1.
- Keep serial, safe disabled outputs and read-only parameter discovery available
  during the hold; host readiness is independent of IMU telemetry.
- Report WAIT_HOST, INITIALIZING, CALIBRATING, READY and ERROR startup phases.
- ACK does not arm the bot; calibration must finish before a fresh local START.

## 3.6.0 — frame timing and passive depth diagnostics

- Add checksum-bound camera snapshot metadata with V4L2 monotonic capture time,
  publisher/sequence identity and publication timing.
- Record ToF host-read timestamps/uncertainty, sequence, signal/ambient/reflectance
  and bounded depth history; pair snapshots with nearest depth frames.
- Show near-field, inconsistent and rejected-zone reasons plus raw depth grid in
  the calibration GUI. Pairing is time-aware, not claimed hardware synchronization.

## 3.5.0 — guided camera calibration suite

- Add a progressive web/Android wizard with printable ChArUco/checkerboard targets,
  target detection, sharpness/stability checks, coverage guidance and automatic capture.
- Automatically compare pinhole/fisheye fits using held-out views, retain sessions,
  generate raw/corrected previews, and export geometry, captures and PDF reports.
- Use snapshots from the existing camera owner; no competing capture device or
  colour/exposure changes. Actual coefficients require physical target captures.
- Move the live ToF SDK to a managed location before removing the old calibration app.

## 3.4.0 — remove fixed creep limits and prepare vision roadmap

- Remove fixed 0.20/0.45 m/s unknown/partial-depth caps. Fresh unconfirmed depth
  leaves the normal DRIVEV limit; confirmed obstacles still apply braking/stop
  limits and stale/offline depth still holds forward requests.
- Keep unknown depth explicitly labelled as unobserved rather than clear.
- Document camera distortion/intrinsics, camera/base and camera/ToF extrinsics,
  time-aligned distance overlays and a later measured ONNX perception pipeline.

## 3.3.0 — Drive alignment and richer chase world

- Use one forward-first world frame for CAD, encoder movement, trail and depth.
- Read the existing PITCHSGN calibration for display; raw negative forward tilt
  now leans the CAD forward rather than backward.
- Keep grounded CAD yaw aligned with odometry; idle compass changes cannot leave
  a sideways heading offset when forward driving resumes.
- Add a centered rear chase view and richer dark-world reference guidance.
- Expand the world to the full viewport with floating speed/status and collapsible
  map/details; add a fading direction ribbon, range arcs and ground parallax cues.
- Camera follow/explore changes preserve the view; all scene guidance is labelled
  as a reference rather than fabricated sensor-detected terrain or roads.

## 3.2.0 — refined Android Drive world

### Less intrusive collision guard and reference calibration

- Use a 0.20 m close-obstacle clearance, 0.18 s reaction model and narrower wheel
  corridor; partial coverage permits up to 0.45 m/s, fresh unknown depth creep
  up to 0.20 m/s. Confirmed collisions stop requests; stale/offline depth holds.
- Add measured-reference range-bias calibration to Drive, persisted separately
  from robot NVS. Require stable central returns and a bias within ±0.10 m.
- Preserve raw ranges alongside corrected ranges; never fabricate calibration
  from invalid/self-reflection returns. Balance/reverse/steering are unchanged.

### Live world motion and lift evidence

- Move CAD in the same metre-based frame as the encoder trail, with interpolated
  heading/pitch, encoder wheel rotation, camera following and exploration mode.
- Show compass-observed hand-held yaw when wheel odometry cannot observe rotation.
- Show existing firmware airborne/landing events, low-g and handled/tilted cues.
  Lift animation is symbolic; steady hand-held height/contact remain unmeasured.
- Freeze stale pose and preserve short-angle interpolation across 0/360 degrees.

### Dark touch-navigable Drive world

- Replace Android's light painter-style scene with a hardware depth-buffered 3D
  world, plain dark floor, fog, contact shadow and smoothly shaded CAD surfaces.
- One finger orbits, two fingers pan, pinch zooms; Recenter restores the view.
- Add dark instrument cards and a toggleable trail panel. No CAD edge/wireframe
  pass is drawn. Camera gestures change the view only.
- Use full-resolution printed CAD surfaces for the Android world, preserving
  vents/cut-outs rather than decimation artefacts; retain smooth tyre shading.

### Prepared assembled robot launcher icon

- Render a front three-quarter app icon from all seven full-resolution current
  robot STL components, using matching parametric tyres/hubs and app-theme colours.
- Generate 1024px artwork, density-specific launcher/adaptive layers, round icons
  and Android 13 themed monochrome icons with adaptive-mask safe padding.
- Keep the render reproducible with `python scripts/generate_launcher_icon.py`.

## 3.1.7 — measured-depth driving scene and projected guard

- Replace the web ToF grid with a grey driving scene: trail left, current assembled
  STL robot and accepted depth returns centrally, speed/status right. Add matching
  Android Drive destination, preserving existing camera and Live views.
- Use seven current printed CAD components plus parametric tyres/hubs; ship meshes
  locally, without a CDN or the obsolete simplified STEP model.
- Confirm isolated depth returns over successive frames; immediate close returns
  stop forward requests. Apply reaction/braking limits inside a swept corridor.
- A missing zone no longer blocks every request: partial coverage reduces speed;
  stale/wholly unknown depth stays blocked. Reverse, steering and balance recovery
  retain their existing authority. Guard remains opt-in after restart.
- Expose coverage, confidence, measured points and stopping distance; mounting and
  optical projection are explicitly approximate, with a reference floor.

## 3.1.6 — labelled terrain sessions

- Add a Data destination with start/stop, terrain markers and outcome notes.
- Capture ordered raw WebSocket/HTTP events to disk before app decoding, including
  ToF, unknown message types, disconnects, parameter changes and app commands.
- Export persisted sessions as ZIP: JSONL, state/action CSV, live parameter/version
  metadata and pitch/speed/distance/staleness/saturation diagnostics.
- Preserve unassigned reward/terminal fields for offline RL preparation; camera
  video and unobserved sensor internals are not represented as captured data.

## 3.1.5 — controlled three-way rotation

- Support portrait, landscape and reverse landscape with sensor hysteresis.
- Ignore reverse portrait and unknown/flat sensor readings; rotation tests cover
  all permitted windows and the excluded upside-down state.

## 3.1.4 — foreground screen keep-awake

- Keep the display awake while the robot app is visible; clear the window flag
  when leaving the app without requiring wake-lock or background permissions.

## 3.1.3 — complete landscape navigation

- Landscape navigation adapts item height to available screen height and keeps
  all destinations accessible, with scrolling only on unusually short displays.
- Stop remains fixed and accessible above the destination list.

## 3.1.2 — trail reports

- Map exports a rendered trail/robot-position image as PNG or PDF.
- Optional telemetry includes the current pose, controller debug values and
  connection validity; disabling it exports the trail image without the data panel.

## 3.1.1 — robot-log PDF export

- Setup exports a paginated PDF of all buffered robot-log rows with host,
  revision, timestamp and page numbers through Android's document picker.
- Load recent console logs on connect and preserve decoded multiline content.

## 3.1.0 — robot checkpoint

- Captured a complete 107-parameter dump after firmware confirmed loading NVS.
- Captured settings are compiled defaults, including user-adjusted trim, drive
  speed, steering limit and LED front offset.
- Checkpoints current balancing firmware, camera/ISP profile, host readiness,
  ToF integration and CAD assets before Android feature work.
