# Visual re-pair: show the gamepad to the camera to clear pairings

Status: experiment, not wired into the console. Branch `experiment/visual-repair`.

## The idea

Hold the gamepad in front of the robot's camera for about 3 seconds. The robot
recognises it, clears every Bluetooth pairing and restarts into pairing mode. No
app, no terminal, no SSH.

## Is it feasible

Yes. Two reasons it is a good fit:

1. **The action is safe.** The trigger only runs `BT,FORGET` (clear pairings and
   restart). It never moves a wheel and only works while the robot is disarmed. A
   false trigger costs a re-pair, not a fall. So the recogniser does not need to be
   perfect, which is what makes this realistic rather than a research project.
2. **The premise matches the state.** You reach for this exactly when the pad will
   not connect. The pad does not need to be powered to be shown to the camera, and
   the robot is already disarmed and resting on its bumper, so holding the pad in
   front of the face camera is natural.

What it cannot do: it cannot make the gamepad itself advertise. After the robot
clears its keys and restarts, you still put the pad into its HOME + A pairing mode.
So the feature automates the robot half (clear keys, accept new connections); the
pad half is one button hold.

## What we already have

- IMX708 camera, `native_camera`/`stream`, and a JPEG snapshot written about once a
  second to `SNAPSHOT_PATH`.
- `console/object_detection.py`: a 2D detector with a model, confidence threshold
  and a run loop. Good as a coarse gate and as ML plumbing to reuse.
- `console/power.py` and the repair path: `POST /api/gamepad/repair` sends
  `BT,FORGET`, which since 4.7.8 erases `BTstack` and `bp32` and restarts. The visual
  trigger just calls the same code path.
- OpenCV is available in the console environment (`cv2` 4.x), so classical feature
  matching needs no new dependency and no model file.

## Recognising the gamepad: options

| Approach | How | Needs training | Robustness | Verdict |
| --- | --- | --- | --- | --- |
| COCO detector class | Look for the "remote"/"cell phone" box, large and centred | no | low specificity (any dark remote-like object) | coarse gate only |
| ORB feature match | Match live frame to reference photos of the EVOFOX, keep RANSAC inliers | no | good near frontal, weaker at steep angles | **MVP** |
| Embedding similarity | Frozen small CNN (MobileNet/SqueezeNet ONNX), cosine similarity to reference embeddings | no (few-shot) | good, more viewpoint tolerant | phase 3 if ORB recall is low |
| Custom trained detector | Fine-tune a detector on gamepad photos | yes (dataset + training) | highest | overkill for a safe convenience trigger |
| Printed marker/QR on the pad | Detect a sticker | no | bulletproof | fallback if vision proves flaky |

Recommendation: start with **ORB feature matching** against a few reference photos
of the actual pad (we already have `docs/manual/img/photo_gamepad.jpg` and its
cutout as a first reference). Zero training, zero model files, deterministic and
tunable. Use the COCO detector only as an optional coarse gate to find the region.
If recall at angles is poor, add the embedding path, which reuses the ML plumbing.

`match_prototype.py` in this folder implements the ORB path and runs entirely
offline on a folder of images, so feasibility can be measured before any robot or
console change.

## Trigger logic (once integrated)

A new `console/visual_pair.py` task:

1. Only runs when: the feature is enabled (a setting, default OFF at first), the
   camera is on (not powered down), the controller is connected and **disarmed**
   (not balancing or self-righting), and **no gamepad is currently connected**
   (if one is connected the feature is pointless and must not drop a good link).
2. Reads the latest frame (snapshot file, or a few fps during an active window),
   scores it with the matcher on a centre region of interest.
3. Keeps a rolling "seen" timer. The pad must score above threshold continuously
   for `HOLD_S` (3 s) with small gaps allowed (hysteresis), so a single frame
   cannot trigger it.
4. On success: call the existing repair path (same as `POST /api/gamepad/repair`),
   then enter a cooldown (for example 60 s) so the restart and re-pair are not
   retriggered.

## Feedback (what the user sees)

Without this, the 3 second hold is invisible and feels broken. Minimum viable
feedback, in order of effort:

- Web console / system map: a "Gamepad seen... 2.1 s" overlay and a toast when it
  fires. No firmware change.
- Status/ring: a progress cue on the robot itself. The LED ring can fill over the
  3 seconds and flash on trigger. This needs a small host to firmware channel (a
  new line such as `VP,<0..100>` that draws a progress ring while disarmed) or
  reuse of the start-up loader animation. Phase 2.

## Honest risks and mitigations

- **False positive drops a working pairing.** Mitigated by requiring no connected
  pad and disarmed state, plus the 3 second hold. Worst case is a re-pair.
- **Specificity.** COCO "remote" alone misfires on phones and remotes. That is why
  the MVP keys off reference photos of the actual pad, not a generic class.
- **Viewpoint and lighting.** ORB is viewpoint sensitive. Mitigate with several
  reference photos (front and both angles), and the 3 second window lets the user
  adjust. Embedding similarity is the fallback for more tolerance.
- **Camera focus/exposure on a close object.** Hold at roughly 20 to 40 cm; verify
  the IMX708 autofocus and exposure cope. Tune the ROI size accordingly.
- **Compute alongside SLAM and the stream.** ORB on 1 to 3 fps centre crops is
  cheap. Keep it off the hot path and rate limited.

## Phased plan

- **Phase 0 (this branch): offline feasibility.** Collect reference and test photos
  (pad at several angles and lighting, plus negatives: hand, phone, face, empty).
  Run `match_prototype.py` to measure the score separation and pick a threshold.
  Decide ORB vs embeddings on the numbers. No robot, no risk.
- **Phase 1: console integration behind a default-off setting.** `visual_pair.py`
  task with the gating above, calling the existing repair path. Web overlay
  feedback. Test on the robot with the feature explicitly enabled.
- **Phase 2: on-robot feedback.** Progress ring during the hold, flash on trigger.
- **Phase 3: robustness.** More references or the embedding matcher, ROI from the
  detector, sensitivity setting, telemetry on near-misses to tune the threshold.

## Decision needed before Phase 1

- Default state of the setting (recommend OFF until the false-positive rate is
  measured on the real robot).
- Whether to require the robot to be in a specific state (for example only when a
  "pad lost" error is showing) to narrow when the detector is even allowed to fire.
