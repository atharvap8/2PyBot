# Platform revisions

For the main milestones and estimated dates for future work, see the
[Revision Table](docs/REVISION_HISTORY.md).

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
