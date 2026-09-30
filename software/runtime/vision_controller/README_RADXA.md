# Radxa Vision Controller

`radxa_brain.py` provides a YOLOv4-tiny collision guard, person following, ArUco mode switching, and a browser camera stream. It communicates with BaseLink through USB `O` odometry and `V` drive messages.

The current Cubie console package is documented in [software/radxa/console](../../radxa/console/README.md). Both applications own the camera and serial port; run one at a time.

## Match the current firmware first

The checked-in brain still uses:

- 115200 baud in `open_port()`.
- `COUNTS_PER_M = 4096 / (2*pi*0.0350)`.
- `ENC_GUARD_SIGN=+1` for forward velocity.

Current BaseLink uses **460800 baud** and a compiled wheel radius of **0.050 m**. Match the brain's baud and geometry to the actual firmware/NVS settings before running it. Read `P?` from BaseLink to check `WHEELR` and `COUNTSM`; the brain does not request parameters automatically.

The brain's `V,...,1` messages request drive authority but do not arm current BaseLink. Arm with gamepad START or serial `E`. See the [firmware protocol](../../../firmware/BaseLink/PROTOCOL.md).

## Install on the Radxa

Copy the files from this directory:

```bash
scp radxa_brain.py get_models.sh radxa-brain.service radxa@<CUBIE_IP>:~/robot/
```

On the Radxa:

```bash
cd ~/robot
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-serial
bash get_models.sh
python3 radxa_brain.py --make-cards
```

OpenCV must include `cv2.aruco`. The model downloader fetches `yolov4-tiny.cfg`, `yolov4-tiny.weights`, and `coco.names` into the working directory. Card generation writes FOLLOW-ME marker 7 and STOP-FOLLOWING marker 8 from `DICT_4X4_50`.

## Run

```bash
python3 radxa_brain.py /dev/ttyUSB0
```

Open `http://<cubie-ip>:8080` for annotated video and FOLLOW/GUARD buttons. Camera discovery tries V4L2 devices 0 through 3 with MJPG, 640x480, and a small buffer. Serial is opened with an exclusive lock and DTR/RTS disabled.

The process must own its camera and serial port. To inspect existing users:

```bash
sudo fuser -v /dev/video0
sudo lsof /dev/ttyUSB0
```

Terminal commands are `f`, `g`, or `q` followed by Enter. Marker 7 selects FOLLOW; marker 8 selects GUARD. `--show` opens a local OpenCV window and requires a display. Without it, use the browser stream.

JPEGs are encoded only when a stream client is connected. Detection continues without a viewer.

## GUARD mode

GUARD starts silent so the gamepad can drive. It sends zero forward/steering commands when:

- A non-person detection in the center corridor is closer than `STOP_CM=70`, or an unknown object's box is wider than 45% of the frame.
- Encoder-derived forward velocity exceeds 0.05 m/s.
- Odometry is less than 0.5 s old.

It holds the zero command for at least one second and releases after the corridor has been clear for one second. Person detections are collected separately and do not trigger this guard-distance test.

Zero `V` commands stop the drive request while leaving balance armed. BaseLink's red hazard display appears only while balancing, with near-zero commanded motion and measured speed above 0.10 m/s. It need not remain red for the entire guard hold.

## FOLLOW mode

The brain selects the widest detected person box. It steers from horizontal center error and drives from width-fraction error relative to `TARGET_W_FRAC=0.34`.

- Forward request is limited to ±0.40 and steering to ±0.55.
- Commands are smoothed with coefficient 0.35.
- A forward obstacle or person wider than 55% of the frame sets the forward target to zero; command smoothing still applies.
- During a brief target loss, forward request is zero and the last smoothed steering request is halved. After 0.7 s without a person, transmission becomes silent.

BaseLink retains host authority until its freshness window expires, default 600 ms after the last `V` line. At the default `DRIVEV=0.70`, a 0.40 forward request corresponds to a 0.28 m/s velocity target. This is separate from the balancer's recovery speed ceiling.

## Wire behavior

An I/O thread sends active commands at 50 Hz. If the vision command becomes older than 0.5 s while transmission remains active, it sends `V,0,0,1`. It reads `O` records for average encoder position and filtered velocity; other serial records are ignored.

Switching to GUARD or quitting makes transmission silent. The GUARD/STOP button is a mode switch, not the firmware's `X` balancing stop.

## Settings

| Setting | Default | Purpose |
| :--- | :--- | :--- |
| `STOP_CM` | 70 | Known-object stop-distance threshold |
| `CORRIDOR` | 0.25 to 0.75 | Horizontal corridor by box-center fraction |
| `FOCAL_PX` | 600 | Width-based distance estimate |
| `TARGET_W_FRAC` | 0.34 | Desired apparent person width |
| `FWD_KP` / `FWD_MAX` | 2.2 / 0.40 | Following speed gain and request cap |
| `STEER_KP` / `STEER_MAX` | 1.4 / 0.55 | Following steering gain and request cap |
| `FOLLOW_STEER_SIGN` | +1 | Direction convention |
| `LOST_SILENT_S` | 0.7 | Target-loss timeout |

Distance estimates depend on assumed object widths and camera calibration. Person following uses apparent width, not measured range.

## Service

`radxa-brain.service` runs from `/home/radxa/robot` as user `radxa`, uses `/dev/ttyUSB0`, and restarts on failure. Adjust those values for the target device, then install:

```bash
sudo cp radxa-brain.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now radxa-brain
journalctl -u radxa-brain -f
```

The working directory must contain the three model files. Under systemd, switch modes through the browser or ArUco cards.
