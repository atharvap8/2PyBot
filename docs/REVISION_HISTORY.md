# Revision Table

The main milestones in 2PyBot's development, from the first balancing firmware
to the current Android app and vision work. Small fixes and routine maintenance
have been grouped together.

## Project history

A dash means the change did not have a separate release version.

| Sr. No. | Feature Name/Change | Version | Date |
| ---: | :--- | :--- | :--- |
| 1 | Started the project with PID balancing, a joystick controller and tuning tools. | — | 17 Apr 2026 |
| 2 | Released the first firmware version and added automated builds and releases. | 1.0.0 | 18 Apr 2026 |
| 3 | Added wheel encoder feedback to the firmware and desktop dashboard. | — | 08 May 2026 |
| 4 | Added a 3D robot viewer to the desktop app and created the first CAD model. | — | 12 Jul 2026 |
| 5 | Added support for the EvoFox gamepad through a Bluetooth-to-ESP-NOW bridge. | — | 12 Jul 2026 |
| 6 | Explored ROS 2 and micro-ROS for the Radxa in a separate prototype. | Prototype | 19 Jul 2026 |
| 7 | Added a physics simulator for testing the balance controller. | — | 23 Aug 2026 |
| 8 | Switched to LQR/LQI balancing, direct Bluetooth control and TMC2226 drivers. Added climb mode and LED feedback. | 2.0.0 | 27 Aug 2026 |
| 9 | Added camera pan/zoom controls and a controllable torch. | — | 09 Sep 2026 |
| 10 | Added live tuning, saved parameters, auto-tuning and terrain-aware balance control. | 3.0.0 | 01 Oct 2026 |
| 11 | Added the Radxa console, Android app, camera streaming and updated enclosure CAD. | 3.0.0 | 01 Oct 2026 |
| 12 | Saved the bot's tuned settings as a checkpoint and improved startup coordination. | 3.1.0 | 11 Oct 2026 |
| 13 | Fixed camera controls and exposure handling, and improved the camera's colours. | 3.1.0 | 11 Oct 2026 |
| 14 | Added ToF depth readings and an optional obstacle guard. | 3.1.0 | 11 Oct 2026 |
| 15 | Added Android report exports, terrain recording and better screen handling. | 3.1.1–3.1.6 | 11 Oct 2026 |
| 16 | Added the first Drive view with the assembled robot model, trail and depth readings. | 3.1.7 | 11 Oct 2026 |
| 17 | Added the shaded 3D world, touch controls and rear chase view. Corrected the model's movement and tilt. | 3.4.0 | 12 Oct 2026 |
| 18 | Removed fixed creep limits and added measured-reference ToF range calibration. | 3.4.0 | 12 Oct 2026 |
| 19 | Added a guided camera calibration wizard with automatic capture, lens fitting, corrected previews and reports. | 3.5.0 (local) | 12 Oct 2026 |
| 20 | Added camera/depth frame timing, paired capture records and ToF health diagnostics. | 3.6.0 (local) | 12 Oct 2026 |
| 21 | Held firmware startup/calibration until Radxa acknowledgement, while keeping discovery and safe disabled outputs available. | 3.7.0 (local) | 12 Oct 2026 |
| 22 | Added ONNX object detection with a corrected viewer, filters and temporary tracks. Validated real fisheye lens calibration. | 3.8.0 (local) | 12 Oct 2026 |
| 23 | Recorded reversal failures and added a predictive reference governor, full recovery feedback and wheel-headroom steering allocation. | 3.9.0 (local) | 13 Oct 2026 |

Current development numbering advances **3.10.0 → 3.20.0 → 3.30.0**.
Post-3.10 local prototype changes are consolidated into 3.20.0; historical
artifact identifiers stay intact for traceability. See [version policy](VERSIONING.md).

The 3.2.0 and 3.3.0 Android builds were local revisions. Their changes were
included in **3.4.0**, following the **3.4.0-rc.1** release candidate.
The latest stable release is **3.4.0**.
Revisions 3.5.0–3.9.0 are local work, not published GitHub releases. Real camera
target captures produced a saved fisheye solution; full-rate corrected video and
sensor extrinsics remain pending. Fall prevention requires a repeat drive test.

## Planned work

These are estimated completion dates. They may change as we collect calibration
data and test on the bot. The [vision roadmap](VISION_ROADMAP.md) has the detailed
task list.

| Sr. No. | Feature Name/Change | Version | Date |
| ---: | :--- | :--- | :--- |
| 24 | Integrate full-rate corrected video using the saved lens calibration. | Target: 3.30.0 | 03 Nov 2026 (estimate) |
| 25 | Validate measured ToF ranges and timing. | Target: 3.30.0 | 06 Nov 2026 (estimate) |
| 26 | Calibrate the camera and ToF positions relative to the bot and to each other. | Target: 3.30.0 | 24 Nov 2026 (estimate) |
| 27 | Show ToF distance readings over the camera feed, with confidence and frame age. | Target: 3.40.0 | 30 Nov 2026 (estimate) |
| 28 | Increase detection throughput and integrate tracked overlays, with distance labels where depth is available. | Target: 4.0.0 | 22 Dec 2026 (estimate) |
| 29 | Build a local obstacle map and improve position tracking with camera and IMU data. | Target: 4.1.0 | 12 Jan 2027 (estimate) |
| 30 | Use the calibrated perception system for obstacle-aware assistance and waypoint driving. | Target: 4.2.0 | 26 Jan 2027 (estimate) |

For smaller changes, see the [changelog](../CHANGELOG.md). Earlier designs and
experiments are described in [project history](BaseLink/Project_History.md).
