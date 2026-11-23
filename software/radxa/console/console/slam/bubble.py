"""Lidar safety bubble around the robot body.

Footprint measured from the C1 lidar centre (owner): right 10 cm, left 17 cm,
front 6 cm, back 7 cm. The lidar sits 31 mm right of the centreline on the
axle line, so in the robot base frame (x forward, y left, origin = axle centre)
the body is the rectangle

    x in [-0.07, +0.06],  y in [-0.131, +0.139]

Its corners are the triangulated diagonal clearances from the lidar centre:
front-left 0.180 m, front-right 0.117 m, back-left 0.184 m, back-right 0.122 m.

Every scan gives
  * forward / reverse speed caps (braking model) for the straight corridor,
  * whether turning left / right in place would sweep a corner into something,
  * free straight-line travel for 24 headings (used by roaming / planning).
Points inside the body (+ a small self tolerance) are the robot itself.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

LIDAR_Y = -0.031
EXTENT = {"front": 0.06, "back": 0.07, "right": 0.10, "left": 0.17}   # from the lidar centre


def footprint(lidar_y=LIDAR_Y, ext=EXTENT):
    """(xmin, xmax, ymin, ymax) of the body in the base frame."""
    return (-ext["back"], ext["front"], lidar_y - ext["right"], lidar_y + ext["left"])


def diagonals(ext=EXTENT):
    return {"front_left": math.hypot(ext["front"], ext["left"]), "front_right": math.hypot(ext["front"], ext["right"]),
            "back_left": math.hypot(ext["back"], ext["left"]), "back_right": math.hypot(ext["back"], ext["right"])}


DEFAULTS = {
    "self_tol": 0.015,      # returns this close to the body are the robot itself (fence, cables)
    "stop_margin": 0.06,    # never let the body get closer than this in the travel direction
    "side_margin": 0.03,    # corridor is the body width plus this on each side
    "turn_margin": 0.025,   # rotating in place must keep corners this far from returns
    "decel": 0.60,          # m/s^2 the balancing controller can reliably brake with
    "reaction": 0.25,       # s: scan age + host loop + firmware lease latency
    "max_cap": 3.0,         # "no limit" value sent to the firmware
    "stale_s": 0.40,        # no fresh scan for this long -> everything capped to 0
    "headings": 24,
}


def braking_cap(clearance, decel, reaction):
    """Highest speed that still stops within `clearance` metres."""
    clearance = max(0.0, clearance)
    return math.sqrt((decel * reaction) ** 2 + 2 * decel * clearance) - decel * reaction


class Bubble:
    def __init__(self, settings=None):
        self.cfg = dict(DEFAULTS, **(settings or {}))
        self.box = footprint()
        self.lock = threading.Lock()
        self.t = 0.0
        self.result = None

    # ---------------------------------------------------------------- geometry
    def _inside(self, pts, grow):
        x0, x1, y0, y1 = self.box
        return ((pts[:, 0] >= x0 - grow) & (pts[:, 0] <= x1 + grow) &
                (pts[:, 1] >= y0 - grow) & (pts[:, 1] <= y1 + grow))

    def _corridor(self, pts):
        """Free distance ahead of the front face and behind the back face."""
        x0, x1, y0, y1 = self.box
        m = self.cfg["side_margin"]
        band = (pts[:, 1] >= y0 - m) & (pts[:, 1] <= y1 + m)
        ahead = pts[band & (pts[:, 0] > x1)]
        behind = pts[band & (pts[:, 0] < x0)]
        front = float(ahead[:, 0].min() - x1) if len(ahead) else math.inf
        back = float(x0 - behind[:, 0].max()) if len(behind) else math.inf
        return front, back

    def _turn_clear(self, pts, sign):
        grow = self.cfg["turn_margin"]
        for ang in (0.12, 0.25, 0.40, 0.60):
            th = -sign * ang                          # rotate the world opposite to the body
            c, s = math.cos(th), math.sin(th)
            rot = np.c_[c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1]]
            if self._inside(rot, grow).any():
                return False
        return True

    def free_by_heading(self, pts):
        """Straight-line free travel (m) if the robot first turned to each heading."""
        n = self.cfg["headings"]
        out = []
        for k in range(n):
            th = 2 * math.pi * k / n
            c, s = math.cos(-th), math.sin(-th)
            rot = np.c_[c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1]] if len(pts) else pts
            front, _ = self._corridor(rot) if len(pts) else (math.inf, math.inf)
            out.append(front)
        return out

    # ---------------------------------------------------------------- update
    def update(self, pts, t):
        """pts: (N, 2) floor-clipped returns in the base frame; t: monotonic time."""
        pts = np.asarray(pts, float).reshape(-1, 2)
        c = self.cfg
        if len(pts):
            pts = pts[~self._inside(pts, c["self_tol"])]
        front, back = self._corridor(pts) if len(pts) else (math.inf, math.inf)
        fwd = braking_cap(front - c["stop_margin"], c["decel"], c["reaction"]) if math.isfinite(front) else c["max_cap"]
        rev = braking_cap(back - c["stop_margin"], c["decel"], c["reaction"]) if math.isfinite(back) else c["max_cap"]
        x0, x1, y0, y1 = self.box
        if len(pts):
            dx = np.maximum(np.maximum(x0 - pts[:, 0], pts[:, 0] - x1), 0)
            dy = np.maximum(np.maximum(y0 - pts[:, 1], pts[:, 1] - y1), 0)
            nearest = float(np.hypot(dx, dy).min())
        else:
            nearest = math.inf
        res = {
            "front_m": front, "back_m": back, "nearest_m": nearest,
            "forward_cap": min(c["max_cap"], fwd), "reverse_cap": min(c["max_cap"], rev),
            "turn_left_ok": self._turn_clear(pts, +1) if len(pts) else True,
            "turn_right_ok": self._turn_clear(pts, -1) if len(pts) else True,
            "free_by_heading": self.free_by_heading(pts),
            "points": int(len(pts)),
        }
        with self.lock:
            self.t = t
            self.result = res

    # ---------------------------------------------------------------- output
    def status(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            res, t = self.result, self.t
        stale = res is None or now - t > self.cfg["stale_s"]
        if stale:
            res = {"front_m": None, "back_m": None, "nearest_m": None, "forward_cap": 0.0, "reverse_cap": 0.0,
                   "turn_left_ok": False, "turn_right_ok": False, "free_by_heading": [], "points": 0}
        out = dict(res, stale=stale, age_s=None if res is None or t == 0 else round(now - t, 3),
                   footprint=list(self.box), diagonals=diagonals(), stop_margin=self.cfg["stop_margin"])
        for k in ("front_m", "back_m", "nearest_m"):
            v = out.get(k)
            out[k] = None if v is None or not math.isfinite(v) else round(v, 3)
        out["free_by_heading"] = [None if not math.isfinite(v) else round(v, 3) for v in out["free_by_heading"]]
        out["forward_cap"] = round(out["forward_cap"], 3)
        out["reverse_cap"] = round(out["reverse_cap"], 3)
        if stale:
            out["state"] = "stale"
        elif out["forward_cap"] <= 0.02 and out["reverse_cap"] <= 0.02:
            out["state"] = "boxed_in"
        elif out["forward_cap"] <= 0.02:
            out["state"] = "front_blocked"
        elif out["reverse_cap"] <= 0.02:
            out["state"] = "rear_blocked"
        elif min(out["forward_cap"], out["reverse_cap"]) < 0.4:
            out["state"] = "limiting"
        else:
            out["state"] = "clear"
        return out
