# Object detection first, SLAM later

## Objects viewer (local revision 3.8.0)

The Objects tab and `/ui/ml.html` provide ONNX object detection and temporary
track IDs. Android Camera can open the same viewer. The first model is the
ONNX Model Zoo's MIT-licensed SSD-MobileNetV1 INT8 detector (COCO classes).

- The native camera remains the capture owner. A bounded worker processes its
  snapshots, currently about 1 Hz, rather than opening another camera reader.
- Input is RGB NHWC uint8 at 300×300. Normalized model boxes are mapped back to
  the exact annotated preview frame. Confidence and people-only filters are exposed.
- When valid saved intrinsics match the image size, the preview can use lens
  correction before inference. Main WebRTC video is still a separate stream.
- Track IDs match class/box overlap over a short interval. They are not persistent
  person identities and are not a substitute for robust motion tracking.
- Object distances stay unknown until camera–ToF extrinsics and association are
  calibrated. Detection does not send motor, arming or steering commands.

Install the optional `requirements-ml.txt` in the console environment, then run
`python scripts/vision/install_detector.py` from the repository root. The fixed
model URL and SHA-256 are checked before installation; weights live under
`/var/lib/2pybot/models/`, outside Git. The viewer defaults off after console
restart; Start detection enables it. Inference is limited to one CPU thread.

APIs: `GET /api/ml/status`, `POST /api/ml/configure`, `GET /api/ml/preview.jpg`.
Reported latency is measured inference time; camera snapshot age is separate.
Benchmark on the actual Cubie before increasing sampling rates.

## What SLAM would add

Object detection answers **“what is in the image?”**. SLAM answers **“where am I,
and what does the surrounding map look like?”** while the robot moves.

A first SLAM demo is manageable. Reliable mapping on this balancing platform is
considerably harder because pitch motion, wheel slip, blur, texture-poor scenes
and time offsets affect pose tracking. Camera-only monocular SLAM also has a
metric-scale problem. The 8×8 ToF is sparse and does not replace a lidar scan or
a dense RGB-D camera.

The practical sequence is:

1. Keep the camera at a calibrated fixed pose for initial motion tests.
2. Export timestamped raw gyro/accelerometer axes, not only filtered pitch and
   scalar acceleration magnitude, if using visual–inertial odometry.
3. Establish camera/base/IMU transforms and clock alignment. Camera pan adds a
   changing transform, so either model the joint pose or lock it initially.
4. Test encoder/IMU odometry and visual odometry with recorded sequences first.
5. Run an existing visual–inertial or SLAM implementation on the laptop for replay
   and evaluation before attempting a limited onboard deployment.
6. If robust indoor mapping is the goal, a supported 2D lidar with encoder/IMU
   odometry is usually a simpler route than monocular visual SLAM alone.

A demo may take days after the inputs are ready; reliable results typically need
weeks of integration and testing, and are not guaranteed by installing a package.
The ESP32 balance loop should remain independent of perception/SLAM workload.
