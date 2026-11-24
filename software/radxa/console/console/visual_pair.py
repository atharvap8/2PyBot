"""Visual re-pair: show the gamepad to the camera to clear Bluetooth pairings.

Hold the gamepad face-on in front of the camera. When the on-board COCO detector
reports a "remote" (which is how the EVOFOX pad reads), centred and held for a few
seconds, the controller is told to clear every pairing and restart into pairing
mode. See experiments/visual_pair/DESIGN.md for the feasibility work.

Why this is safe to automate: the only action is `BT,FORGET` (clear pairings and
restart). It never moves a wheel, it only runs while the robot is disarmed, and it
only runs while no gamepad is connected (when one is connected the gesture is
pointless). A false trigger therefore costs a re-pair, not a fall.

The detector runs about once a second (the camera snapshot refresh rate) and only
while the gate below is open, so the idle cost is zero.
"""
import asyncio
import os
import time
from pathlib import Path

from .config import SNAPSHOT_PATH

# COCO ids a gamepad plausibly registers as. "remote" is the reliable one in
# testing; "cell phone" is accepted too (the pad's phone clamp reads that way).
CANDIDATE_IDS = (75, 77)          # remote, cell phone
SCORE_MIN = 0.40                  # zero false positives on background in testing
AREA_MIN, AREA_MAX = 0.03, 0.55   # ignore tiny specks and the too-close full-frame miss
CENTRE_X = (0.28, 0.72)
CENTRE_Y = (0.22, 0.80)
WINDOW_S = 6.0                    # rolling window for the hold
HITS_NEEDED = 3                  # qualifying snapshots within the window
SPAN_NEEDED = 2.0                # oldest-to-newest hit span, so one lucky frame cannot fire
RECENT_S = 1.8                   # newest hit must be this fresh
COOLDOWN_S = 90.0                # covers the controller restart and re-pair
POLL_S = 0.6


class VisualPair:
    def __init__(self, link, power, model_path=None, log=None):
        self.link = link
        self.power = power
        self.log = log or (lambda *_: None)
        self.model_path = Path(model_path or os.environ.get("BOT_ML_MODEL",
                                                            "/var/lib/2pybot/models/ssd-mobilenet-v1-int8.onnx"))
        self.enabled = os.environ.get("BOT_VISUAL_PAIR", "1") != "0"
        self.session = None
        self._decode = None
        self.hits = []                  # monotonic timestamps of qualifying snapshots
        self.cooldown_until = 0.0
        self.last_seen_score = 0.0
        self.last_frame_mtime = None
        self.state = "idle"             # idle | watching | seen | triggered | cooldown | unavailable
        self.message = ""
        self.last_tick = 0.0

    # -------------------------------------------------- gating
    def gate(self):
        """Return (open, reason). Open only when a visual re-pair is both safe and useful."""
        if not self.enabled:
            return False, "disabled"
        if getattr(self.power, "asleep", False):
            return False, "powered down"
        link = self.link
        if not link.connected or link.tel.get("stale", True):
            return False, "no controller link"
        if link.derived.get("PADREPAIR") not in (1, 2):
            return False, "controller firmware too old"
        if link.tel.get("balancing") or link.tel.get("self_righting"):
            return False, "robot is armed"
        if link.tel.get("startup_state") not in (None, "READY"):
            return False, "controller starting up"
        nav = getattr(link, "nav", {}) or {}
        if nav.get("pad"):
            return False, "a gamepad is already connected"
        return True, ""

    # -------------------------------------------------- perception
    def _ensure_session(self):
        if self.session is not None:
            return True
        if not self.model_path.is_file():
            return False
        try:
            import onnxruntime as ort
            from .object_detection import decode_outputs
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 1
            opts.inter_op_num_threads = 1
            self.session = ort.InferenceSession(str(self.model_path), sess_options=opts,
                                                providers=["CPUExecutionProvider"])
            self._decode = decode_outputs
            return True
        except Exception as exc:  # noqa: BLE001 -- report instead of killing the loop
            self.log("[visual-pair] model load failed: %r" % exc)
            self.message = "model load failed: " + str(exc)[-160:]
            return False

    def _detect(self):
        """Best qualifying candidate score on the current snapshot, or 0.0."""
        import cv2
        import numpy as np
        try:
            mtime = os.stat(SNAPSHOT_PATH).st_mtime
        except OSError:
            return 0.0, "no camera frame"
        if time.time() - mtime > 3.0:
            return 0.0, "camera frame stale"
        self.last_frame_mtime = mtime
        image = cv2.imread(SNAPSHOT_PATH, cv2.IMREAD_COLOR)
        if image is None:
            return 0.0, "camera frame unreadable"
        tensor = cv2.cvtColor(cv2.resize(image, (300, 300), interpolation=cv2.INTER_AREA),
                              cv2.COLOR_BGR2RGB)[None].astype(np.uint8)
        names = [o.name for o in self.session.get_outputs()]
        values = self.session.run(names, {self.session.get_inputs()[0].name: tensor})
        best = 0.0
        for d in self._decode(dict(zip(names, values)), threshold=SCORE_MIN):
            if d["class_id"] not in CANDIDATE_IDS:
                continue
            x1, y1, x2, y2 = d["box"]
            area = (x2 - x1) * (y2 - y1)
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            if (AREA_MIN <= area <= AREA_MAX and CENTRE_X[0] <= cx <= CENTRE_X[1]
                    and CENTRE_Y[0] <= cy <= CENTRE_Y[1]):
                best = max(best, d["score"])
        return best, ""

    # -------------------------------------------------- action
    def _trigger(self):
        # The gate already guaranteed disarmed, supported and no pad connected.
        if self.link.send("BT,FORGET"):
            self.cooldown_until = time.monotonic() + COOLDOWN_S
            self.state = "triggered"
            self.message = "Gamepad recognised: clearing pairings and restarting. Hold HOME + A on the pad."
            self.log("[visual-pair] gamepad recognised; sent BT,FORGET (clear pairings + restart)")
            self.hits.clear()
            self.link._publish({"type": "visual_pair", "visual_pair": self.status()})
            return True
        return False

    # -------------------------------------------------- status
    def status(self):
        held = (self.hits[-1] - self.hits[0]) if len(self.hits) >= 2 else 0.0
        progress = 0.0 if not self.hits else min(1.0, max(len(self.hits) / HITS_NEEDED, held / SPAN_NEEDED))
        return {"enabled": self.enabled, "state": self.state, "message": self.message,
                "progress": round(progress, 2), "hits": len(self.hits),
                "last_score": round(self.last_seen_score, 2),
                "model_installed": self.model_path.is_file(),
                "cooldown_s": round(max(0.0, self.cooldown_until - time.monotonic()), 1)}

    def set_enabled(self, value):
        self.enabled = bool(value)
        if not self.enabled:
            self.hits.clear()
            self.state = "idle"
            self.message = ""
        return self.status()

    # -------------------------------------------------- loop
    async def run(self):
        loop = asyncio.get_running_loop()
        while True:
            await asyncio.sleep(POLL_S)
            try:
                await self._tick(loop)
            except Exception as exc:  # noqa: BLE001 -- the loop must never die silently
                self.state = "unavailable"
                self.message = str(exc)[-200:]
                self.log("[visual-pair] loop error: %r" % exc)

    async def _tick(self, loop):
        now = time.monotonic()
        if now < self.cooldown_until:
            self.state = "cooldown"
            return
        open_, reason = self.gate()
        if not open_:
            if self.hits:
                self.hits.clear()
            self.state = "idle"
            self.message = reason
            return
        if not self._ensure_session():
            self.state = "unavailable"
            return
        score, note = await loop.run_in_executor(None, self._detect)
        self.last_seen_score = score
        self.hits = [t for t in self.hits if now - t <= WINDOW_S]
        if score >= SCORE_MIN:
            self.hits.append(now)
            self.state = "seen"
            self.message = note or f"gamepad in view ({score:.2f})"
        else:
            self.state = "watching"
            self.message = note or "show the gamepad face-on, whole pad in view"
        span = (self.hits[-1] - self.hits[0]) if len(self.hits) >= 2 else 0.0
        ready = (len(self.hits) >= HITS_NEEDED and span >= SPAN_NEEDED
                 and now - self.hits[-1] <= RECENT_S)
        self.link._publish({"type": "visual_pair", "visual_pair": self.status()})
        if ready:
            self._trigger()
