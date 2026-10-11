# Revision Table

Complete recorded history from repository inception, feature-level breakdowns,
and planned work. Columns follow the requested standard:
**Sr. No. / Feature Name/Change / Version / Date**.

## Reading the table

- Dates use `YYYY-MM-DD`. Historical commit rows use Git author dates.
- Rows 1–75 cover every unique recorded commit reachable from branches, remote
  branches and tags at the `55f0883` baseline. Stash/index snapshots are not
  feature revisions. Commit IDs make untagged history traceable.
- **Untagged** means no version was assigned to that change; version numbers
  have not been invented retroactively. Branch experiments are explicitly marked
  and must not be mistaken for functionality merged into the current robot.
- Later rows unpack large historical feature bundles and document the local
  3.2.0–3.4.0 work consolidated into the 3.4.0 release candidate. Their
  `2026-10-12 (recorded)` date is the documentation date, not a fabricated earlier
  release/commit date. 3.2.0 and 3.3.0 were local APK revisions, not Git tags.
- **Planned** rows are estimates as of 2026-10-12, dependent on calibration
  captures, hardware access and measured results. Target versions are indicative;
  dates are not completion claims. The implementation checklist is in the
  [vision roadmap](VISION_ROADMAP.md).

| Sr. No. | Feature Name/Change | Version | Date |
| ---: | :--- | :--- | :--- |
| 1 | Repository inception: firmware/hardware/software/docs scaffold, initial balancing firmware, joystick transmitter and tuning tools | Untagged (`e6e090e`) | 2026-04-17 |
| 2 | Complete README structure and technical overview | Untagged (`bcf8b74`) | 2026-04-17 |
| 3 | README doubledash formatting cleanup | Untagged (`678cb08`) | 2026-04-17 |
| 4 | Project media/assets folder and empty-directory placeholders | Untagged (`eebb63c`) | 2026-04-17 |
| 5 | IMU model and component-list documentation update | Untagged (`68a51c6`) | 2026-04-18 |
| 6 | Controller Mermaid diagram label fix | Untagged (`0ef8989`) | 2026-04-18 |
| 7 | Move desktop robot controller GUI into `gui/` | Untagged (`1d16a6c`) | 2026-04-18 |
| 8 | Update README paths after GUI relocation | Untagged (`5397749`) | 2026-04-18 |
| 9 | Synchronize README directory tree with repository layout | Untagged (`682755d`) | 2026-04-18 |
| 10 | Ignore local virtual environments and build output | Untagged (`890f485`) | 2026-04-18 |
| 11 | Firmware compilation, Python lint and documentation CI workflows | Untagged (`5646731`) | 2026-04-18 |
| 12 | Remove obsolete lychee documentation-check flag | Untagged (`2434ecc`) | 2026-04-18 |
| 13 | CI workflow-trigger maintenance | Untagged (`d28019b`) | 2026-04-18 |
| 14 | Direct arduino-cli build handling for sketch names and lint-rule adjustment | Untagged (`26d7a6b`) | 2026-04-18 |
| 15 | E701 lint corrections and firmware CI invocation fixes | Untagged (`b0cf333`) | 2026-04-18 |
| 16 | Rename Arduino sketches and correct CI paths | Untagged (`2a4829e`) | 2026-04-18 |
| 17 | Huge APP partition build fix, release automation and formatter corrections | Untagged (`4b6de87`) | 2026-04-18 |
| 18 | CI verification-trigger content updates | Untagged (`dae5caf`) | 2026-04-18 |
| 19 | First stable tagged BaseLink/Controller firmware release, notes and release badge | 1.0.0 (`a53bf9a`) | 2026-04-18 |
| 20 | Automated firmware build and release publishing workflow | Untagged (`1b23484`) | 2026-04-18 |
| 21 | Project title and structure documentation revision | Untagged (`39a0dd5`) | 2026-04-18 |
| 22 | Encoder feedback integration and associated firmware/GUI updates | Untagged (`1c1c39d`) | 2026-05-08 |
| 23 | Codebase maintenance/fixes | Untagged (`6bd9122`) | 2026-05-09 |
| 24 | Synchronize main through recorded merge | Untagged (`a705ad0`) | 2026-05-09 |
| 25 | Core telemetry/SerialBT buffering corrections | Untagged (`08ddb4c`) | 2026-05-10 |
| 26 | Stability-logic refactor and telemetry GUI cleanup | Untagged (`5cc3b15`) | 2026-05-10 |
| 27 | LED feedback deadband and GUI stability improvements | Untagged (`ec138e2`) | 2026-05-11 |
| 28 | Add FastLED dependency to firmware compilation CI | Untagged (`820787f`) | 2026-05-11 |
| 29 | Interactive desktop 3D robot viewer, UI polish and CAD STEP model | Untagged (`d183078`) | 2026-07-12 |
| 30 | EvoFox One S Bluepad32-to-ESP-NOW gamepad bridge | Untagged (`3ea041f`) | 2026-07-12 |
| 31 | NEMA17-style motors added to STEP model | Untagged (`0efda5f`) | 2026-07-12 |
| 32 | ROS 2 `pybot_bridge` package — separate development branch | Untagged experiment (`acb9668`) | 2026-07-12 |
| 33 | STEP model detailing from concept render | Untagged (`a0178bb`) | 2026-07-13 |
| 34 | Predictive fall protection/input constraining — separate safety experiment | Untagged experiment (`829215d`) | 2026-07-13 |
| 35 | ESP32 core 2.x API compatibility for Bluepad32 bridge | Untagged (`16e3794`) | 2026-07-13 |
| 36 | Merge EvoFox bridge feature into main | Untagged (`6979531`) | 2026-07-13 |
| 37 | Radxa ROS teleoperation package, Docker stack and micro-ROS agent — separate branch | Untagged experiment (`4d533ca`) | 2026-07-17 |
| 38 | Lidar SLAM package placeholder — not implemented SLAM/current main | Untagged experiment (`ea4d786`) | 2026-07-17 |
| 39 | Docker install-tree persistence fix — ROS branch | Untagged experiment (`0f05f2d`) | 2026-07-17 |
| 40 | ESP32 micro-ROS client bridge — separate ROS branch | Untagged experiment (`3b3de74`) | 2026-07-17 |
| 41 | USB serial micro-ROS transport to Radxa — separate ROS branch | Untagged experiment (`14b1af9`) | 2026-07-17 |
| 42 | Radxa operations scripts/runbook and micro-ROS enablement — ROS branch | Untagged experiment (`02a8715`) | 2026-07-17 |
| 43 | Arduino `sketch.yaml` Huge APP defaults — ROS branch | Untagged experiment (`3d166b1`) | 2026-07-17 |
| 44 | 115200-baud flash workaround and official micro-ROS agent image — ROS branch | Untagged experiment (`215ef9b`) | 2026-07-18 |
| 45 | Operations runbook/script updates and ESP32-balance checkpoint — separate branch | `checkpoint-esp32-balance` (`8c3716f`) | 2026-07-19 |
| 46 | ROS-hosted balance / ESP32 micro-ROS slave architecture — separate experiment | Untagged experiment (`9f2eeb3`) | 2026-07-19 |
| 47 | Replace PID architecture with LQR/LQI acceleration-input control | Untagged (`0f65b24`) | 2026-08-20 |
| 48 | Direct EVOFOX Bluetooth pairing, remove active ESP-NOW path, RMT WS2812 expressions/gesture feedback | Untagged (`7d14314`) | 2026-08-20 |
| 49 | Legacy Radxa vision-controller runtime: person following and distance guard | Untagged (`45352da`) | 2026-08-20 |
| 50 | Physics simulation and controller testing lab | Untagged (`f1ed067`) | 2026-08-23 |
| 51 | Climbing controller/reference-tracking additions | Untagged (`ce432dc`) | 2026-08-27 |
| 52 | Move browser tuner into software runtime layout | Untagged (`ed61be3`) | 2026-08-27 |
| 53 | TMC2226 driver migration | Untagged (`2a83fec`) | 2026-08-27 |
| 54 | Remove obsolete EvoFox bridge sketch from active control stack | Untagged (`f63264b`) | 2026-08-27 |
| 55 | Relocate tuner to `tools/` | Untagged (`121223d`) | 2026-08-27 |
| 56 | Documentation preparation for LQR branch merge | Untagged (`22f1824`) | 2026-08-27 |
| 57 | Control-stack documentation update | Untagged (`018e09a`) | 2026-08-27 |
| 58 | README stale-reference corrections | Untagged (`0c62d47`) | 2026-08-27 |
| 59 | Merge/release LQR/LQI, TMC2226 and direct Bluetooth control stack | 2.0.0 (`7bbd71f`) | 2026-08-27 |
| 60 | Camera payload pan/zoom/torch controls and hardened motion I/O | Untagged (`cbdee62`) | 2026-09-09 |
| 61 | LQR gains and wheel-diameter update | Untagged (`e0c6554`) | 2026-09-30 |
| 62 | Runtime parameter/NVS tuning, auto-calibration and terrain-aware control bundle | Untagged (`a8427f0`) | 2026-09-30 |
| 63 | Radxa console/API, native camera, Android client, deployment files and rev5 CAD bundle | Untagged (`95adca3`) | 2026-09-30 |
| 64 | Commit-aware Radxa handler deployment and selective restarts | Untagged (`27666d5`) | 2026-09-30 |
| 65 | Read bot checkout as its owner during deployment | Untagged (`08f7ed1`) | 2026-09-30 |
| 66 | SDK preparation and versioned firmware output publishing fixes | Untagged (`db07a74`) | 2026-09-30 |
| 67 | Merge/release integrated firmware, Radxa, Android, CAD and deployment stack | 3.0.0 (`70df0c3`) | 2026-10-01 |
| 68 | Confirmed 107-parameter NVS and platform checkpoint bundle | 3.1.0 (`148717c`) | 2026-10-11 |
| 69 | Android paginated robot-log PDF export | 3.1.1 (`6b006d1`) | 2026-10-11 |
| 70 | Trail PNG/PDF export with optional telemetry | 3.1.2 (`1eeaed8`) | 2026-10-11 |
| 71 | Landscape navigation exposes all destinations | 3.1.3 (`7f2d4cd`) | 2026-10-11 |
| 72 | Foreground screen keep-awake | 3.1.4 (`1d54e19`) | 2026-10-11 |
| 73 | Portrait/both-landscape orientation policy, reverse portrait excluded | 3.1.5 (`28e2c29`) | 2026-10-11 |
| 74 | Labelled terrain sessions, raw JSONL/CSV, annotations, diagnostics and ZIP export | 3.1.6 (`5697a0b`) | 2026-10-11 |
| 75 | Projected-depth guard and assembled-CAD driving scene release | 3.1.7 (`55f0883`) | 2026-10-11 |
| 76 | Initial timer-driven step generation, sensor estimation and PID-era balance | Initial bundle (`e6e090e`) | 2026-04-17 |
| 77 | Initial analog joystick ESP-NOW transmitter and robot receiver | Initial bundle (`e6e090e`) | 2026-04-17 |
| 78 | Initial desktop telemetry/tuning GUI and browser serial tuner | Initial bundle (`e6e090e`) | 2026-04-17 |
| 79 | Runtime bounded parameter metadata and NVS save/load/defaults | 3.0.0 integration (`a8427f0`) | 2026-09-30 |
| 80 | Wobble, trim, radius and stall auto-tune routines | 3.0.0 integration (`a8427f0`) | 2026-09-30 |
| 81 | Terrain roughness, airborne/landing state and pitch-gain softening | 3.0.0 integration (`a8427f0`) | 2026-09-30 |
| 82 | Explicit START/SELECT handling and removal of arming via host V transitions | 3.0.0 integration (`a8427f0`) | 2026-09-30 |
| 83 | Expanded acceleration/roughness/softening/terrain/drop debug telemetry | 3.0.0 integration (`a8427f0`) | 2026-09-30 |
| 84 | Radxa FastAPI/WebSocket console, dynamic tuning and serial ownership | 3.0.0 integration (`95adca3`) | 2026-09-30 |
| 85 | Cedar camera publication, MediaMTX/WHEP and service/device-rule packaging | 3.0.0 integration (`95adca3`) | 2026-09-30 |
| 86 | Kotlin/Compose Android Live, Map, Tune, Auto, Camera and Setup screens | 3.0.0 integration (`95adca3`) | 2026-09-30 |
| 87 | Android CI, Gradle wrapper and debug/unsigned-release build support | 3.0.0 integration (`95adca3`) | 2026-09-30 |
| 88 | Revision-5 OpenSCAD enclosure and STL exports | 3.0.0 integration (`95adca3`) | 2026-09-30 |
| 89 | Compile captured user NVS settings as defaults, retaining live saved settings | 3.1.0 | 2026-10-11 |
| 90 | Native capture-owner camera-control IPC, exposure-unit/gain fixes and persistence | 3.1.0 | 2026-10-11 |
| 91 | Raspberry Pi-derived IMX708 hardware CCM/gamma/white-point profile | 3.1.0 | 2026-10-11 |
| 92 | VL53L5CX isolated 8×8 acquisition, quality/raw telemetry and optional TG guard | 3.1.0 | 2026-10-11 |
| 93 | BOOTWAIT readiness handshake and USB reset gate until host/camera/hotspot ready | 3.1.0 | 2026-10-11 |
| 94 | Dashboard ribbon sizing and reusable live-video session lifecycle | 3.1.0 | 2026-10-11 |
| 95 | Current printed STL assembly and charger assets checkpoint | 3.1.0 | 2026-10-11 |
| 96 | Printed-mass estimates, sampled LQR validation and candidate gain documentation | 3.1.0 | 2026-10-11 |
| 97 | Root VERSION/CHANGELOG and Android version-code derivation | 3.1.0 | 2026-10-11 |
| 98 | Confidence, temporal confirmation and projected stopping-corridor guard | 3.1.7 | 2026-10-11 |
| 99 | Web/Android CAD driving view with trail and accepted depth returns | 3.1.7 | 2026-10-11 |
| 100 | Locally shipped current-STL meshes plus parametric tyre/hub geometry | 3.1.7 | 2026-10-11 |
| 101 | Tagged Android APK publication and APK checksums alongside firmware releases | 3.1.7 | 2026-10-11 |
| 102 | Dark depth-buffered Android 3D world, shaded CAD without wireframe edges | 3.2.0 local revision | 2026-10-12 (recorded) |
| 103 | One-finger orbit, two-finger pan, pinch zoom and camera recenter controls | 3.2.0 local revision | 2026-10-12 (recorded) |
| 104 | Full-resolution printed CAD preserving cut-outs/vents and smooth tyre shading | 3.2.0 local revision | 2026-10-12 (recorded) |
| 105 | Metre-based live pose, heading/pitch and encoder wheel animation | 3.2.0 local revision | 2026-10-12 (recorded) |
| 106 | Follow/explore camera and stale-pose freeze | 3.2.0 local revision | 2026-10-12 (recorded) |
| 107 | IMU/firmware airborne, landing, low-g and handled/tilted indications | 3.2.0 local revision | 2026-10-12 (recorded) |
| 108 | Measured-reference ToF range-bias calibration, persisted separately from NVS | 3.2.0 local revision | 2026-10-12 (recorded) |
| 109 | Reduced close-obstacle clearance and earlier partial/unknown creep policy | 3.2.0 local revision; superseded by 3.4.0 | 2026-10-12 (recorded) |
| 110 | Assembled-CAD launcher artwork, density variants, round/adaptive/themed icons | 3.2.0 local revision; mirror correction pending | 2026-10-12 (recorded) |
| 111 | Shared forward-first frame for CAD, odometry, trail and depth | 3.3.0 local revision | 2026-10-12 (recorded) |
| 112 | Correct front/back tilt with existing PITCHSGN calibration | 3.3.0 local revision | 2026-10-12 (recorded) |
| 113 | Grounded yaw uses odometry, avoiding idle-compass sideways offsets | 3.3.0 local revision | 2026-10-12 (recorded) |
| 114 | Centered look-ahead rear chase camera and non-jumping view-mode transitions | 3.3.0 local revision | 2026-10-12 (recorded) |
| 115 | Full-width world with floating speed/status and collapsible map/details | 3.3.0 local revision | 2026-10-12 (recorded) |
| 116 | Reference direction ribbon, range arcs, shoulders and ground-parallax cues | 3.3.0 local revision | 2026-10-12 (recorded) |
| 117 | Regression coverage for cardinal forward alignment, pitch and camera behaviour | 3.3.0 local revision | 2026-10-12 (recorded) |
| 118 | Remove fixed fresh unknown/partial-depth creep caps; retain measured braking/stop and stale handling | 3.4.0 candidate | 2026-10-12 (recorded) |
| 119 | Calibration, sensor-fusion, replay and ONNX implementation roadmap | 3.4.0 candidate | 2026-10-12 (recorded) |
| 120 | Complete standard revision table and estimated future milestones | 3.4.0 candidate | 2026-10-12 (recorded) |
| 121 | Consolidated Drive/icon/guard work on a feature branch, main merge and prerelease candidate publication | 3.4.0-rc.1 | 2026-10-12 (target publication) |
| 122 | Planned: mirrored launcher correction and real-bot forward/tilt regression fixtures | Target 3.5.0 | 2026-10-16 (estimate) |
| 123 | Planned: measured ChArUco/checkerboard capture tool and image-mode metadata | Target 3.5.0 | 2026-10-20 (estimate) |
| 124 | Planned: ToF near-window/invalid-return diagnosis and measured range validation | Target 3.6.0 | 2026-10-23 (estimate) |
| 125 | Planned: camera intrinsics, distortion-model selection and held-out geometric validation | Target 3.5.0 | 2026-10-27 (estimate) |
| 126 | Planned: undistorted raw/corrected preview and rectification performance integration | Target 3.5.0 | 2026-11-03 (estimate) |
| 127 | Planned: camera/ToF acquisition timestamps, frame IDs and synchronized servo/IMU recording | Target 3.6.0 | 2026-11-06 (estimate) |
| 128 | Planned: camera-to-base and ToF-to-base extrinsic calibration at a repeatable home pose | Target 3.7.0 | 2026-11-10 (estimate) |
| 129 | Planned: camera/ToF relative transform, 8×8 ray orientation and overlap calibration | Target 3.7.0 | 2026-11-17 (estimate) |
| 130 | Planned: moving camera-payload kinematics, pose telemetry and transform validation | Target 3.7.0 | 2026-11-24 (estimate) |
| 131 | Planned: camera-feed ToF zone/range/confidence/age overlay with occlusion handling | Target 3.8.0 | 2026-11-30 (estimate) |
| 132 | Planned: synchronized replay, calibration-aware datasets and perception error/latency dashboard | Target 3.9.0 | 2026-12-08 (estimate) |
| 133 | Planned: ARM64 ONNX object-detector baseline and actual-board performance benchmark | Target 4.0.0 | 2026-12-15 (estimate) |
| 134 | Planned: stable object tracks and foreground-aware measured distance association | Target 4.0.0 | 2026-12-22 (estimate) |
| 135 | Planned: inference quantization/runtime acceleration evaluation and difficult-scene validation | Target 4.0.0 | 2026-12-29 (estimate) |
| 136 | Planned: confidence/age-aware local obstacle map and floor/free-space perception evaluation | Target 4.1.0 | 2027-01-05 (estimate) |
| 137 | Planned: visual odometry/pose fusion and hand-carried/slip uncertainty handling | Target 4.1.0 | 2027-01-12 (estimate) |
| 138 | Planned: evaluated obstacle-aware assistance or constrained waypoint behaviour | Target 4.2.0 | 2027-01-26 (estimate) |

## Source and maintenance

Historical coverage was reconstructed from:

```text
git log --reverse --date=short --format="%h|%ad|%s" --branches --remotes --tags
git show --stat <commit>
git merge-base --is-ancestor <commit> main
```

Feature bundles were cross-checked against [project history](BaseLink/Project_History.md),
[platform changelog](../CHANGELOG.md), [terrain/Drive notes](terrain-sessions.md),
and source changes. Published historical tags are preserved; the candidate does
not retag old development commits. Add a row when a feature/fix lands, and replace
an estimate with a verified completion date once future work is completed.
