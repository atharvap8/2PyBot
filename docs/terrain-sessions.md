# Terrain sessions and driving scene (3.1.6–3.4.0)

## Drive alignment and chase scene (3.3.0)

The 3D world uses one forward-first convention: X right, Y up, and -Z forward
at zero heading. Radxa odometry X/Y maps to world -Z/-X respectively. CAD nose,
trail, optical points, ground references and camera following use that same frame.
During ground driving the displayed yaw follows odometry; idle compass updates
cannot retain a sideways offset when driving resumes. Hand-held/unloaded yaw can
still follow compass observations while not wheel-referenced.

The IMU's raw pitch is converted with the existing **PITCHSGN** parameter (the
captured bot uses -1). A negative raw forward tilt therefore tilts the model
forward. This is display conversion only: calibration, gains and NVS are not changed.

Drive defaults to a centered, look-ahead rear chase camera and a full-width scene
with floating speed/status controls. The blue direction ribbon, faded shoulders,
range arcs and subtle ground markers are **display references**, not detected
lanes, a planned path or a guarantee of traversable ground. Reverse velocity puts
the reference ribbon behind the bot. Measured depth remains separate and can be
unavailable. Map and instrument Details panels are expandable; reference guides
can be hidden in Details. Camera gestures and follow/explore transitions retain
their view without moving or commanding the robot.

## Android 3D Drive world (3.2.0)

Drive uses a dark, plain depth-buffered 3D world with studio-shaded CAD surfaces,
a soft contact shadow and distance fog. The CAD has no edge or wireframe pass.
Its pose and the ground trail share the encoder's metre-based coordinate frame.

- One finger: orbit the camera. Two fingers: pan around the world. Pinch: zoom.
- **Following** keeps the camera on the moving bot; panning switches to **Explore**.
  **Recenter** restores the camera. Touch gestures never issue driving commands.
- Heading, pitch and encoder wheel rotation interpolate between telemetry samples.
  Compass yaw supplies observed hand-held rotation when the encoders cannot see it.
- Firmware terrain state 2 is airborne/free-fall, state 3 is touchdown. Low-g and
  stationary handled/tilted states are shown separately from confirmed events.
  The raised visual cue is symbolic, not an estimated/measured lift height.
- A steady hand-held bot still reads about 1 g; contact/height are unobserved in
  that case. Roll is used if reported in debug telemetry; absent roll is not
  fabricated. Moving the bot by hand does not produce reliable encoder position.
- Stale/disconnected pose freezes. The floor is a coordinate reference; measured
  ToF points are not a complete reconstructed terrain surface.

## Recording

Open **Data**, enter a terrain label, and start a session. Use **Mark terrain**
when the surface/outcome changes, then **Stop and save** and export the session ZIP.
Captures persist in the app's private `files/sessions` directory across screen
changes and can be exported again after reopening the app. Clearing app storage
or uninstalling deletes these private copies, so export datasets before doing so.

- `events.jsonl`: every received WebSocket message before decoding, HTTP requests
  and responses, disconnects, annotations and session boundaries. Original JSON
  fields and unknown message types are retained. Rows include sequence, wall time,
  monotonic time, elapsed time and current human terrain label.
- `telemetry.csv`: telemetry state and mirrored drive/steer commands; all debug
  fields also remain in `debug_json`. Empty reward/terminal columns intentionally
  need offline definitions. `v_cmd_steps_s` is the firmware's **steps/s** command.
- `metadata.json`: schema, app version, robot host and initial live parameter and
  derived-value snapshots. Parameters re-read on start also appear in raw events.
- `diagnostics.json`: pitch RMS/peak, peak absolute speed, encoder distance,
  saturation/stale/balancing sample counts and event counts. Present only after a
  normal stop. Interrupted captures still have flushed raw JSONL and CSV.

This is reception-level telemetry, at the console's 10 Hz push rates. It does not
capture encoded camera video, unseen firmware sensor internals or packets that
never reached the app. Human labels and encoder pose are not ground-truth terrain
or SLAM. Training/reward construction belongs in offline dataset preparation.

## Guard and visualization decisions

The guard still limits forward **references**, not wheel actuation. The existing
ESP32 heartbeat/reverse/steering/balance-recovery behaviour is preserved.

- Valid zone: ST target status 5 or 9, one or more targets, 50–4000 mm and sigma
  0–20 mm. Invalid/raw zones remain in the wire payload and recordings.
- Confirmation: a compatible neighbouring zone or a repeat within 10%/100 mm;
  close returns at/below 0.20 m are acted on immediately.
- Corridor: 0.12 m half-width plus angular zone footprint. Steering expands a
  symmetric swept envelope because sensor mounting handedness is uncalibrated.
- Reaction/braking model: 0.18 s reaction, 0.7 m/s² assumed deceleration, 0.20 m
  clearance. The requested cap solves `v*t + v²/(2*a) <= available clearance`.
  These are model assumptions, not a measured physical stopping guarantee.
- From 3.4.0 there are no fixed unknown/partial-coverage creep caps. Fresh data
  without a confirmed corridor obstacle leaves the normal `DRIVEV` limit;
  confirmed returns still impose the clearance-based braking/stop cap.
  Stale/offline depth caps at zero. Unknown range does not prove a clear path.
- Guard is OFF after console restart. Firmware support and fresh telemetry are
  exposed separately from sensor confidence.

The scene shows accepted optical points and the current assembled STL components.
Tyres/hubs are parametric visual approximations; charger parts are excluded.
The grey floor is a coordinate reference, not reconstructed terrain. The initial
45° per-axis projection and 0.20 m sensor height need mounting calibration;
`BOT_TOF_HEIGHT_M` can adjust the displayed mount height. No floor returns are
discarded based on this uncalibrated height.

Regenerate matching web/Android meshes using `python scripts/generate_driving_model.py`
with `numpy`, `trimesh` and `fast-simplification` installed.
The full-resolution Android world uses `robot-world-model.json`, generated by
`python scripts/generate_world_model.py` with `numpy` and `trimesh`.

## Measured-reference range calibration (3.2.0)

In Drive, expand **Range calibration**, face a flat target and enter its measured
distance **from the ToF sensor face**, in metres. At least three of the central
four zones must be valid for three consecutive frames with no more than 8 cm
spread. Reference range is 0.20–3.00 m; allowable bias is ±0.10 m.

**Save bias** persists the measured offset in `/var/lib/2pybot/tof_calibration.json`
(`BOT_TOF_CALIBRATION` overrides the path). Raw ranges remain unchanged in the API
and recorder; corrected ranges feed collision limits and displayed optical points.
Range-bias calibration does not calibrate sensor height, field of view or mounting
handedness. Invalid near-window returns cannot establish a useful calibration.
No guessed offset is installed automatically; this calibration does not modify NVS.
