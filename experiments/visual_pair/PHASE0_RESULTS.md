# Phase 0 results: measured on the robot camera

Captured through the robot's own camera (1920x1080 snapshot): 12 frames of the
EVOFOX pad held in front of the lens at a range of angles, and 6 background frames
of the same room with no pad.

## ORB feature matching (the DESIGN.md MVP candidate): rejected

Reference cropped tightly to the controller. RANSAC inlier counts:

- Pad frames: only the exact reference frame scored high (134). Every other pad
  frame scored 0 to 14.
- Room frames: 0 to 7.

No usable separation. The controller is matte black with little texture, which is
ORB's known weak spot, so this approach is out.

## COCO detector (SSD-MobileNetV1, already on the bot): chosen

The pad, with its phone clamp, reads as COCO class **"remote"**.

- Room frames: **0** "remote" or "cell phone" detections. Zero false positives.
- Pad frames, face-on: "remote" at **0.46 to 0.89**, centred (cx about 0.52,
  cy about 0.55), area about 0.10 to 0.14 of the frame.
- Pad frames at steep angles or held too close to fill the frame: missed.

Precision is excellent; per-frame recall is moderate and strongly favours a face-on,
whole-pad-visible pose, which is exactly how someone trying to pair will hold it.

## Design decisions taken from this

- Signal: COCO "remote" (or "cell phone"), score >= 0.40, centred, area 0.03 to 0.55.
- Hysteresis over the 1 fps snapshots: require 3 qualifying frames spanning >= 2 s
  within a 6 s window, newest within 1.8 s, so a single frame cannot fire and short
  dropouts are tolerated. In practice this is a 3 to 5 second steady hold.
- Gate: only while enabled, camera on, controller connected and disarmed, firmware
  supports re-pair, and no gamepad connected. The action is only `BT,FORGET`.
- Cooldown 90 s after a trigger (covers the controller restart and re-pair).
- Guidance to the user: hold the whole pad face-on, about 30 to 50 cm, not so close
  that it fills the frame.

Implemented in `software/radxa/console/console/visual_pair.py`.

## Known limitations / follow-ups

- A real TV remote or a phone held up could also read as "remote"/"cell phone".
  Acceptable: it only fires when no pad is connected and the only effect is a
  re-pair. Narrow later with a small reference-embedding check if needed.
- Recall depends on pose and lighting; the console shows a live progress banner so
  the user can adjust. On-robot LED ring feedback is Phase 2.
- Sample size here is small (12 + 6, one room, one lighting). Treat thresholds as a
  starting point and tune from live use; the feature can be disabled via
  `POST /api/visual_pair {"enabled": false}` or `BOT_VISUAL_PAIR=0`.
