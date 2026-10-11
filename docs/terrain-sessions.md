# Terrain sessions and driving scene (3.1.6–3.1.7)

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
  close returns at/below 0.35 m are acted on immediately.
- Corridor: 0.14 m half-width plus angular zone footprint. Steering expands a
  symmetric swept envelope because sensor mounting handedness is uncalibrated.
- Reaction/braking model: 0.25 s reaction, 0.7 m/s² assumed deceleration, 0.35 m
  clearance. The requested cap solves `v*t + v²/(2*a) <= available clearance`.
  These are model assumptions, not a measured physical stopping guarantee.
- Partial central coverage below 50% caps requests at 0.25 m/s; unconfirmed/no
  corridor evidence is capped at 0.15 m/s. Stale/wholly unknown depth caps at zero.
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
