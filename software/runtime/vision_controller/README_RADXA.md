# Radxa upload pack — brain v2 (guard + leg-follower + WEB UI)

`radxa_brain.py` SUPERSEDES `vision_guard.py` AND the old 2pybot
camera/control server — it now streams the live annotated camera view and
takes FOLLOW/GUARD commands from any browser at `http://<cubie-ip>:8080`.
The old server MUST be stopped first: it is holding the camera (that is why
you saw the GStreamer pipeline warnings and neither mode did anything), and
if it also writes to the serial port, its V-lines would interleave with the
brain's. Find and stop it:

```bash
sudo fuser -v /dev/video0            # who has the camera
sudo lsof /dev/ttyUSB0               # who has the serial port
systemctl list-units | grep -i -e py -e bot -e stream   # find the service
sudo systemctl disable --now <that-service>
```

v2 also opens the camera with the V4L2 backend explicitly (scanning
/dev/video0–3, MJPG), and exits with a diagnosis instead of silently
spinning when no frames arrive. The serial port is opened with an exclusive
lock so a lingering old server fails loudly instead of corrupting commands.

## Upload (from your PC)

```bash
scp radxa_brain.py get_models.sh radxa-brain.service radxa@<CUBIE_IP>:~/robot/
```

## One-time setup on the Radxa

```bash
cd ~/robot
sudo apt update && sudo apt install -y python3-opencv python3-numpy python3-serial
bash get_models.sh                      # 24 MB, pretrained, no training
python3 radxa_brain.py --make-cards     # prints card_FOLLOW-ME_7.png etc.
```
Print the two ArUco cards ~10 cm wide and tape them to cardboard — they are
your headless remote control for the brain.

## Run

```bash
python3 radxa_brain.py /dev/ttyUSB0
```
Then open **http://<cubie-ip>:8080** on your phone or PC: live annotated
view (green person boxes, orange/red obstacle boxes with distances, the gray
corridor lines, mode/velocity/command bar) plus big FOLLOW and GUARD/STOP
buttons. This replaces the old app. The JPEG stream is only encoded while a
browser is actually connected, so it costs nothing when nobody is watching.

Seeing what OpenCV sees: use the web UI over SSH/headless. `--show` opens a
local cv2 window instead, but ONLY works with a monitor attached to the
Cubie (or `ssh -X`, which is painfully slow) — headless Linux cannot draw
windows, which is the other reason "nothing seemed to happen."

Keys in the terminal: `f` follow · `g` guard · `q` quit. ArUco cards still
work: **card 7 = follow me**, **card 8 = stop following** — and they are the
mode switch when running under systemd.

## How each mode behaves

**GUARD (default):** totally silent — the EVOFOX drives the robot directly
over Bluetooth. Only when something sits closer than `STOP_CM` (70) in the
center corridor AND the robot is actually rolling forward does it stream
`V,0,0,1`, which instantly outranks the pad (your firmware's arbitration),
brakes on balance, holds, then releases when clear. The ring shows the red
hazard pulse for exactly as long as the guard has authority.

**FOLLOW ("legs mode"):** locks onto the widest (= nearest) detected person
and drives to keep them centered and at a constant apparent size. Because it
regulates the bbox **width fraction** instead of absolute distance, it works
when the low-mounted camera only ever sees legs — no leg-specific model
needed, YOLO's person class detects partial bodies fine. Obstacle stop stays
armed the whole time; a person suddenly filling >55 % of the frame also
forces a stop (someone stepped right in front). Lose the person for 0.7 s
and it goes silent — the gamepad has full control again within 600 ms, no
buttons pressed. Reacquire and it resumes.

## Knobs you will actually touch (top of radxa_brain.py)

`TARGET_W_FRAC` 0.34 — bigger = heels closer to you. `FWD_MAX` 0.40 — speed
cap as a fraction of the robot's 0.55 m/s drive limit; raise only after it
follows smoothly. `FOLLOW_STEER_SIGN` — flip to -1 if it turns AWAY from you
on first test. `STEER_KP`/`FWD_KP` — lower if it oscillates, the vision loop
is only ~4-6 FPS so gentle gains feel best. `FOCAL_PX` — paste your value
from the earlier calibration so guard distances are true.

## First test, in this order

1. Wheels off the ground, mode GUARD, drive with the pad — untouched feel.
2. Walk a hand-held "obstacle" (a chair) in front while wheels spin: guard
   stop + red ring, release after it clears.
3. Floor, press `f`, stand 1.5 m ahead, walk slowly. Check steer sign first!
4. Show card 8 mid-follow — it must drop to guard and hand you the pad.
