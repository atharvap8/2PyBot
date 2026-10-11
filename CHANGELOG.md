# Platform revisions

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
