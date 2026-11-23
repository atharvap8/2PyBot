"""Lidar safety bubble around the robot body (v2).

Footprint from the C1 lidar centre (owner-measured) plus 1 cm all round:
right 11 cm, left 18 cm, front 7 cm, back 8 cm. The lidar sits 31 mm right of
the centreline on the axle line, so in the base frame (x forward, y left,
origin = axle centre) the body is the rectangle

    x in [-0.08, +0.07],  y in [-0.141, +0.149]

Each scan produces:
  * front_m / back_m: free travel to the nearest return inside the corridor the
    body sweeps along the CURRENT path (straight, or an arc from the measured
    yaw rate and speed), sent to the controller as BG at scan rate;
  * turn flags: turning in place left/right would swing a corner into something;
  * free travel for 24 headings (roaming / planning).

The controller turns distances into speed limits every control cycle, using its
own wheel odometry to subtract travel since the scan (see bubble_guard.h).
The same braking model is evaluated here for the UI and the host missions.

Noise: a return only counts if another return lies within 6 cm in the same
scan, or a return was within 8 cm of it in the previous scan. One stray point
can no longer stop the robot; a chair leg seen in two scans still does.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

LIDAR_Y = -0.031
GROW = 0.01                                       # owner: +1 cm on every side
EXTENT = {"front": 0.06 + GROW, "back": 0.07 + GROW, "right": 0.10 + GROW, "left": 0.17 + GROW}


def footprint(lidar_y=LIDAR_Y, ext=EXTENT):
    """(xmin, xmax, ymin, ymax) of the body in the base frame."""
    return (-ext["back"], ext["front"], lidar_y - ext["right"], lidar_y + ext["left"])


def diagonals(ext=EXTENT):
    return {"front_left": math.hypot(ext["front"], ext["left"]), "front_right": math.hypot(ext["front"], ext["right"]),
            "back_left": math.hypot(ext["back"], ext["left"]), "back_right": math.hypot(ext["back"], ext["right"])}


DEFAULTS = {
    "self_tol": 0.015,      # returns this close to the body are the robot itself
    "side_margin": 0.03,    # corridor = body width + this on each side
    "turn_margin": 0.025,   # rotating in place must keep corners this far from returns
    "support_scan": 0.06,   # a return needs a neighbour this close in the same scan...
    "support_prev": 0.08,   # ...or one this close in the previous scan
    "max_range": 4.0,       # corridor look-ahead (m)
    "stale_s": 0.40,
    "headings": 24,
    # braking model mirrored from the firmware parameters (BUBDEC/BUBLAT/BUBMRG)
    "decel": 0.50, "latency": 0.30, "margin": 0.05,
}


def braking_cap(clearance, margin, decel, latency):
    c = clearance - margin
    if c <= 0 or decel <= 0:
        return 0.0
    dl = decel * latency
    return math.sqrt(dl * dl + 2 * decel * c) - dl


class Bubble:
    def __init__(self, settings=None):
        self.cfg = dict(DEFAULTS, **(settings or {}))
        self.box = footprint()
        self.lock = threading.Lock()
        self.t = 0.0
        self.result = None
        self._prev = np.zeros((0, 2))

    # ---------------------------------------------------------------- geometry
    def _inside(self, pts, grow):
        x0, x1, y0, y1 = self.box
        return ((pts[:, 0] >= x0 - grow) & (pts[:, 0] <= x1 + grow) &
                (pts[:, 1] >= y0 - grow) & (pts[:, 1] <= y1 + grow))

    def _straight(self, pts):
        x0, x1, y0, y1 = self.box
        m = self.cfg["side_margin"]
        band = (pts[:, 1] >= y0 - m) & (pts[:, 1] <= y1 + m)
        ahead = pts[band & (pts[:, 0] > x1)]
        behind = pts[band & (pts[:, 0] < x0)]
        front = float(ahead[:, 0].min() - x1) if len(ahead) else math.inf
        back = float(x0 - behind[:, 0].max()) if len(behind) else math.inf
        return front, back

    def _arc(self, pts, kappa):
        """Free arc length along a circle of curvature kappa (1/m, + = left)
        before the swept body (annulus around the turn centre) meets a return."""
        R = 1.0 / kappa
        c = np.array([0.0, R])
        x0, x1, y0, y1 = self.box
        m = self.cfg["side_margin"]
        corners = np.array([[x0, y0], [x0, y1], [x1, y0], [x1, y1]])
        # nearest body point to the centre (centre lies outside the body for |R| > half width)
        near = np.array([np.clip(0.0, x0, x1), np.clip(R, y0, y1)])
        r_in = max(0.0, float(np.hypot(*(near - c))) - m)
        r_out = float(np.hypot(corners[:, 0] - c[0], corners[:, 1] - c[1]).max()) + m
        rel = pts - c
        rp = np.hypot(rel[:, 0], rel[:, 1])
        band = (rp >= r_in) & (rp <= r_out)
        if not band.any():
            return math.inf, math.inf
        a0 = math.atan2(-R, 0.0)                         # robot origin seen from the centre
        ap = np.arctan2(rel[band, 1], rel[band, 0])
        s = np.sign(R)
        fwd = np.mod((ap - a0) * s, 2 * np.pi)          # angle travelled going forward
        bwd = np.mod((a0 - ap) * s, 2 * np.pi)          # ... going backward
        L = abs(R)
        f_ok = fwd < np.pi
        b_ok = bwd < np.pi
        front = float((L * fwd[f_ok]).min() - x1) if f_ok.any() else math.inf
        back = float((L * bwd[b_ok]).min() + x0) if b_ok.any() else math.inf
        return max(0.0, front), max(0.0, back)

    def _turn_clear(self, pts, sign):
        grow = self.cfg["turn_margin"]
        for ang in (0.12, 0.25, 0.40, 0.60):
            th = -sign * ang
            c, s = math.cos(th), math.sin(th)
            rot = np.c_[c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1]]
            if self._inside(rot, grow).any():
                return False
        return True

    def free_by_heading(self, pts):
        n = self.cfg["headings"]
        out = []
        for k in range(n):
            th = 2 * math.pi * k / n
            c, s = math.cos(-th), math.sin(-th)
            rot = np.c_[c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1]] if len(pts) else pts
            out.append(self._straight(rot)[0] if len(pts) else math.inf)
        return out

    def _supported(self, pts):
        """Reject isolated single returns (noise); keep anything seen twice."""
        if len(pts) < 2:
            ok = np.zeros(len(pts), bool)
        else:
            d = np.hypot(pts[:, None, 0] - pts[None, :, 0], pts[:, None, 1] - pts[None, :, 1])
            np.fill_diagonal(d, np.inf)
            ok = d.min(axis=1) <= self.cfg["support_scan"]
        if len(self._prev) and len(pts):
            dp = np.hypot(pts[:, None, 0] - self._prev[None, :, 0], pts[:, None, 1] - self._prev[None, :, 1])
            ok |= dp.min(axis=1) <= self.cfg["support_prev"]
        return ok

    # ---------------------------------------------------------------- update
    def update(self, pts, t, v=0.0, w=0.0):
        """pts: (N, 2) floor-clipped returns in the base frame; t: scan time
        (monotonic, oldest point); v (m/s), w (rad/s): current motion."""
        pts = np.asarray(pts, float).reshape(-1, 2)
        c = self.cfg
        if len(pts):
            pts = pts[~self._inside(pts, c["self_tol"])]
            pts = pts[np.hypot(pts[:, 0], pts[:, 1]) <= c["max_range"] + 0.5]
        raw = pts
        sup = self._supported(raw) if len(raw) else np.zeros(0, bool)
        pts = raw[sup] if len(raw) else raw
        self._prev = raw
        kappa = 0.0
        if abs(v) > 0.08:
            kappa = max(-2.5, min(2.5, w / v))
        if len(pts):
            front, back = self._arc(pts, kappa) if abs(kappa) > 0.2 else self._straight(pts)
        else:
            front = back = math.inf
        x0, x1, y0, y1 = self.box
        if len(pts):
            dx = np.maximum(np.maximum(x0 - pts[:, 0], pts[:, 0] - x1), 0)
            dy = np.maximum(np.maximum(y0 - pts[:, 1], pts[:, 1] - y1), 0)
            nearest = float(np.hypot(dx, dy).min())
        else:
            nearest = math.inf
        # returns within 25 cm of the body, for diagnosis / drawing (base frame, rounded)
        near_mask = self._inside(raw, 0.25) if len(raw) else np.zeros(0, bool)
        near_raw, near_ok = raw[near_mask], sup[near_mask]
        res = {
            "near": [[round(float(x), 3), round(float(y), 3), int(k)] for (x, y), k in zip(near_raw[:80], near_ok[:80])],
            "front_m": front, "back_m": back, "nearest_m": nearest, "kappa": kappa,
            "turn_left_ok": self._turn_clear(pts, +1) if len(pts) else True,
            "turn_right_ok": self._turn_clear(pts, -1) if len(pts) else True,
            "free_by_heading": self.free_by_heading(pts),
            "points": int(len(pts)), "rejected": int(len(raw) - len(pts)),
        }
        with self.lock:
            self.t = t
            self.result = res

    def diag_near(self, pts):
        """Returns within 25 cm of the body while scans are gated out (robot
        tilted / resting). Diagnostic only: finds self-detection (fence, cables)."""
        pts = np.asarray(pts, float).reshape(-1, 2)
        if len(pts):
            pts = pts[self._inside(pts, 0.25)]
        with self.lock:
            self.near_diag = [[round(float(x), 3), round(float(y), 3)] for x, y in pts[:80]]
            self.near_diag_t = time.monotonic()

    # ---------------------------------------------------------------- output
    def status(self, now=None, decel=None, latency=None, margin=None):
        now = time.monotonic() if now is None else now
        c = self.cfg
        decel = c["decel"] if decel is None else decel
        latency = c["latency"] if latency is None else latency
        margin = c["margin"] if margin is None else margin
        with self.lock:
            res, t = self.result, self.t
        stale = res is None or now - t > c["stale_s"]
        if stale:
            res = {"near": [], "front_m": 0.0, "back_m": 0.0, "nearest_m": None, "kappa": 0.0, "turn_left_ok": False,
                   "turn_right_ok": False, "free_by_heading": [], "points": 0, "rejected": 0}
        with self.lock:
            nd = list(getattr(self, "near_diag", []))
            ndt = getattr(self, "near_diag_t", 0.0)
        out = dict(res, near_diag=nd if now - ndt < 1.0 else [],
                   stale=stale, age_s=None if res is None or t == 0 else round(now - t, 3),
                   footprint=list(self.box), diagonals=diagonals(), stop_margin=margin)
        f, b = res["front_m"], res["back_m"]
        out["forward_cap"] = round(min(3.0, braking_cap(f, margin, decel, latency)) if math.isfinite(f) else 3.0, 3)
        out["reverse_cap"] = round(min(3.0, braking_cap(b, margin, decel, latency)) if math.isfinite(b) else 3.0, 3)
        for k in ("front_m", "back_m", "nearest_m"):
            v = out.get(k)
            out[k] = None if v is None or not math.isfinite(v) else round(v, 3)
        out["free_by_heading"] = [None if not math.isfinite(v) else round(v, 3) for v in out["free_by_heading"]]
        out["flags"] = (0 if out["turn_left_ok"] else 1) | (0 if out["turn_right_ok"] else 2)
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
