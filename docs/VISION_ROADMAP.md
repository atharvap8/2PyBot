# 2PyBot vision roadmap

Status: **planned work**, recorded while preparing revision **3.4.0** locally.
The tasks below have not been implemented or physically calibrated yet.

Estimated milestone dates and indicative target versions are recorded in the
[Revision Table](REVISION_HISTORY.md). These are planning estimates, not promises
or completion claims.

## Direction

Build a calibrated mobile perception platform: see an object, localize it in the
image, attach a trustworthy distance when depth supports it, and place the
observation in the robot's frame. Later, turn those observations into a local
map and useful assistance. Preserve the working ESP32 balance controller and
keep vision/inference asynchronous on the Radxa.

The next implementation I would choose is **camera intrinsic calibration plus
an undistorted preview**, with a repeatable capture/validation tool. Straight
lines appearing curved are a lens-geometry problem; the existing colour/ISP
profile and brightness auto-calibration do not correct that geometry.

## Terms

- **Intrinsics:** the camera's internal geometry: focal lengths `fx/fy`, principal
  point `cx/cy`, image size and lens-distortion coefficients. These are what allow
  undistortion and conversion between image pixels and optical rays.
- **Extrinsics:** position and orientation of one sensor/frame relative to another,
  represented by rotation `R` and translation `t` (or a 4×4 transform).
- **Range calibration:** correcting measured ToF distance bias. This does not
  establish its mounting pose, field of view or alignment to camera pixels.
- **Time alignment:** relating a depth frame to the camera frame captured at the
  corresponding time, rather than whichever video frame happens to be displayed.

## Ordered work list

### 1. Camera optics, intrinsics and straight-line correction — first milestone

- [ ] Record the actual IMX708 lens, focus setting, active capture mode, image size,
  crop/binning, output resize and saved ISP profile. Keep focus/mechanics fixed.
- [ ] Create a measured, flat ChArUco or checkerboard target and a capture workflow
  through the existing capture owner; do not open a competing V4L2 reader.
- [ ] Collect roughly 30–60 sharp, diverse views spanning corners, edges, distances
  and target angles. Save target dimensions and source-frame metadata.
- [ ] Fit intrinsics/distortion using OpenCV. Compare a standard radial/tangential
  model with a fisheye model if the observed lens warrants it; select using
  held-out reprojection and straight-line residuals, not the name “wide angle”.
- [ ] Export versioned calibration, validation plots, per-view errors and
  coverage maps. Include independent test views, not only fitting images.
- [ ] Add an undistorted preview with raw/corrected comparison and correct ROI.
- [ ] Benchmark rectification latency/FPS before integrating it into live video.
  Cache remap tables; preserve the approved colour/exposure pipeline.

**Deliverable:** camera calibration and an undistorted image whose held-out
straight edges remain straight, including near the frame borders. Report actual
residual errors; aim for approximately pixel-level reprojection where feasible.

An intrinsic calibration is mode-specific. A pure resize permits corresponding
scaling of `K`; a crop changes the principal point. A different sensor readout or
optical setting must not silently reuse an incompatible calibration.

### 2. ToF health, range validation and synchronized recording — before fusion

- [ ] Investigate the recent zero-valid-zone / near-window return pattern using
  raw status, target count, sigma and distance logs. Check mounting, optical
  obstruction/crosstalk and sensor configuration against measured targets.
- [ ] Validate useful ranges over several known distances, materials, ambient
  light levels and torch states. Do not loosen quality rules just to fill a grid.
- [ ] Validate or extend the existing measured-reference bias calibration; use
  manufacturer crosstalk calibration if the diagnosed hardware setup requires it.
- [ ] Capture frame sequences and monotonic acquisition timestamps for camera and
  ToF. Record receive time separately from acquisition time and measure delay.
- [ ] Add synchronized image/depth/IMU/encoder/servo metadata to sessions, with
  calibration IDs and raw values retained for offline replay.

**Deliverable:** repeatable depth quality and time-aligned replay samples with
measured range errors. At present, range bias alone does not solve unusable
returns or unmeasured sensor geometry.

### 3. Camera-to-robot and sensor extrinsics — physical geometry

- [ ] Define the base, IMU, camera optical and ToF frames, handedness and units.
  Use metres internally and explicitly convert sensor millimetres.
- [ ] Establish camera-to-base and ToF-to-base transforms from measured mounting
  geometry, then refine/validate them with calibration observations.
- [ ] Start at a repeatable centered payload pose and record that servo state.
- [ ] Determine what the payload's “yaw” and “zoom” mechanisms actually change.
  If they move the camera, camera extrinsics change; if optics/crop change,
  intrinsics may also change.
- [ ] Calibrate moving payload kinematics or restrict overlays to the calibrated
  home pose initially. Servo commands are not measured joint angles: account for
  backlash/repeatability and add pose telemetry or feedback where needed.

**Deliverable:** documented transform conventions and a validated sensor pose
at known payload positions. A single fixed camera/ToF transform is not generally
valid while a camera-only pan mechanism moves.

### 4. Camera/ToF relative calibration and image overlay — first fusion milestone

- [ ] Verify the 8×8 zone orientation, rotations/flips and optical ray/footprint
  model. Replace the current approximate FOV/height projection with measured data.
- [ ] Observe a camera-visible calibration target or flat plane at multiple
  positions, distances and tilts, with synchronized ToF samples.
- [ ] Fit and validate `T_camera_from_tof`, including held-out poses and edges
  of the shared field of view. Use camera target poses and ToF plane/ray geometry.
- [ ] Project each valid zone's footprint into the matching camera frame:
  `p_camera = R_camera_from_tof * p_tof + t_camera_from_tof`.
- [ ] For raw images, use `K` and the distortion model; for rectified images,
  use the rectified camera matrix. Do not distort or undistort twice.
- [ ] Draw measured ranges, confidence, frame age and zone footprints over live
  video. Handle occlusion, mixed/background returns and camera-pose changes.
- [ ] Display unknown distance outside ToF coverage or without a usable match.
  An 8×8 sensor does not provide a dense depth value for every RGB pixel.

**Deliverable:** camera overlays whose measured distance corresponds to the
same real target across the image and through tested payload poses. Preserve
separation between measured range and any later model-estimated depth.

### 5. Replay, evaluation and local geometric perception

- [ ] Build an offline replay viewer for raw/corrected camera, projected ToF,
  telemetry, timestamps and calibration versions.
- [ ] Add a measured validation set: different distances, clutter, dark/light
  scenes, motion, partial coverage and foreground/background boundaries.
- [ ] Log overlay error, range error, timestamp skew, stale observations and
  end-to-end perception latency.
- [ ] Accumulate calibrated observations into a small confidence/age-aware local
  obstacle map using encoder/IMU pose. Treat hand-carried pose and wheel slip as
  uncertain; the current decorative/reference floor is not an observed map.

**Deliverable:** perception that can be evaluated and replayed before it changes
driving behaviour.

### 6. ONNX detection and useful semantics — after geometry is trustworthy

- [ ] Select one concrete initial use case: tracked person/obstacle detection with
  measured distance labels. Whole-image classification alone cannot localize
  an object for depth association.
- [ ] Benchmark a small exported detector (for example a nano detector or mobile
  SSD) with ONNX Runtime on ARM64, validating preprocessing, class labels,
  output format and postprocessing.
- [ ] Run inference on downscaled frames in a separate bounded worker. Measure
  latency, memory/CPU load and video performance on the actual board.
- [ ] Add object tracking and stable IDs to avoid flickering labels/ranges.
- [ ] Associate projected ToF footprints with detections using confidence-aware
  foreground checks; do not automatically assign a background range to a small
  object just because both fall inside the same coarse zone.
- [ ] Show class, confidence, track and supported range/unknown state in Android.
- [ ] Collect difficult examples and fine-tune off-board if needed. Evaluate
  low light, glare, occlusion and domain-specific objects before integration.
- [ ] Investigate quantization and board acceleration only after the CPU baseline
  works. Verify available runtime/compiler support instead of assuming a usable
  NPU execution provider from the hardware specification.

**Deliverable:** useful detections and stable distance-labelled tracks, with
measured performance. Possible later applications include person finding,
door/landmark recognition and semantic room mapping.

### 7. Assisted driving and mapping — later

- [ ] Evaluate calibrated perception against measured obstacles and recorded
  driving scenarios; relate errors/latency to reference stopping behaviour.
- [ ] Explore local free-space/obstacle mapping, floor segmentation and visual
  odometry once sensor transforms and timing are reliable.
- [ ] Consider constrained obstacle-aware assistance or waypoint behaviour based
  on that evaluated map. Keep the existing balance loop independent of ML.

## Additional backlog

- [ ] Fix the mirrored launcher artwork separately; keep CAD/icon handedness and
  the now-tested Drive coordinate convention explicit.
- [ ] Validate Drive forward/turn/tilt behaviour on the real bot and retain signed
  telemetry samples as regression fixtures.
- [ ] Improve observability of roll, actual payload joint pose and ground contact
  if future behaviour needs them. A steady hand-held height is not observable
  from the current scalar acceleration/pitch telemetry alone.

## Calibration and dataset assets to create

Proposed deliverables, not populated calibration files:

- `hardware/calibration/`: versioned camera intrinsics, lens model, image-mode
  metadata, sensor transforms, payload kinematics and validation summaries.
- Local runtime copies under `/var/lib/2pybot/`, with explicit calibration IDs.
- Large capture/replay datasets outside normal Git history; commit schemas,
  small reproducible fixtures and results rather than raw sessions or videos.
- Offline tools under `scripts/vision/` once implementation starts.

## Current guard policy, distinct from future perception

Revision 3.4.0 removes fixed unknown/partial-depth creep caps. With fresh data and
no confirmed corridor obstacle, the guard leaves normal `DRIVEV`; confirmed
nearby obstacles still set braking/stop limits, and stale/offline depth holds
forward references. Unknown depth remains unknown. The current guard is not yet
camera-calibrated collision perception.
