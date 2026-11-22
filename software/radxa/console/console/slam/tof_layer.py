"""VL53L5CX 8x8 depth -> 3D points -> obstacle and drop-off (cliff) layers.

The lidar sees one horizontal plane at 0.265 m. The ToF adds what that plane
misses: low objects and edges below it, and floor that suddenly falls away.
This layer is kept apart from the lidar map so scan matching is unaffected.

Sensor frame (as in console/tof.py): zone row 0 = top, column 0..7 = left to
right, 45 deg square field of view. Robot frame: x forward, y left, z up.
The sensor's own down-tilt relative to the balanced robot is estimated from
floor returns in the bottom rows while the robot balances steadily.
"""
from __future__ import annotations

import math

import numpy as np

FOV_DEG = 45.0
_cols = (np.arange(64) % 8 - 3.5) * FOV_DEG / 8
_rows = (3.5 - np.arange(64) // 8) * FOV_DEG / 8
TAN_H = np.tan(np.radians(_cols))          # positive = right of the optical axis
TAN_V = np.tan(np.radians(_rows))          # positive = above the optical axis
BOTTOM = np.arange(64) // 8 >= 5           # rows that normally see the floor


class ToFLayer:
    def __init__(self, height=0.21, x_offset=0.0, floor_band=0.04, cliff_drop=0.06,
                 max_obstacle_z=0.50, size_m=40.0, res=0.05, decay_s=3.0):
        self.h, self.x_off = height, x_offset
        self.floor_band, self.cliff_drop, self.max_z = floor_band, cliff_drop, max_obstacle_z
        self.res, self.n, self.origin = res, int(round(size_m / res)), -size_m / 2
        self.obstacle = np.zeros((self.n, self.n), np.float32)   # decaying hit evidence
        self.decay_s = decay_s
        self.tilt_deg = 0.0                # mount down-tilt vs. the balanced robot
        self.tilt_samples = 0
        self.last_seq = None
        self.last_t = None
        self.points = np.zeros((0, 3))     # latest frame, map frame, for display
        self.cliffs = np.zeros((0, 2))
        self.counts = {"frames": 0, "obstacles": 0, "cliffs": 0, "floor": 0}

    # ------------------------------------------------------------------ geometry
    def _robot_points(self, d, tau_deg):
        """d: (64,) metres along the optical axis. tau: total down-tilt (deg)."""
        f, left, up = d, -d * TAN_H, d * TAN_V
        c, s = math.cos(math.radians(tau_deg)), math.sin(math.radians(tau_deg))
        fx = f * c + up * s
        uz = up * c - f * s
        return np.c_[self.x_off + fx, left, self.h + uz]

    def _estimate_tilt(self, d, valid, pitch_dev):
        sel = valid & BOTTOM
        if sel.sum() < 6:
            return
        taus = np.arange(-10.0, 30.5, 0.5)
        dd, tv = d[sel], TAN_V[sel]
        rad = np.radians(taus)[:, None]
        z = self.h + dd * tv * np.cos(rad) - dd * np.sin(rad)
        err = np.median(np.abs(z), axis=1)
        k = int(np.argmin(err))
        if err[k] > 0.02:
            return                          # bottom rows are not seeing open floor
        mount = taus[k] - pitch_dev
        a = 0.2 if self.tilt_samples < 20 else 0.03
        self.tilt_deg += a * (mount - self.tilt_deg)
        self.tilt_samples += 1

    # ------------------------------------------------------------------ update
    def update(self, frame, pose, pitch_dev, steady, t):
        seq = frame.get("sequence")
        if seq is None or seq == self.last_seq or frame.get("state") != "ranging":
            return False
        self.last_seq = seq
        if self.last_t is not None:
            self.obstacle *= math.exp(-max(0.0, t - self.last_t) / self.decay_s)
        self.last_t = t
        d = np.asarray(frame["distance_mm"], float) / 1000.0
        valid = np.asarray(frame.get("valid", [False] * 64), bool)
        if steady:
            self._estimate_tilt(d, valid, pitch_dev)
        pts = self._robot_points(d, pitch_dev + self.tilt_deg)[valid]
        floor = np.abs(pts[:, 2]) <= self.floor_band
        obstacle = (pts[:, 2] > self.floor_band) & (pts[:, 2] <= self.max_z)
        cliff = pts[:, 2] < -self.cliff_drop
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        world = np.c_[x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1], pts[:, 2]]
        if obstacle.any():
            r = np.floor((world[obstacle, 1] - self.origin) / self.res).astype(int)
            col = np.floor((world[obstacle, 0] - self.origin) / self.res).astype(int)
            ok = (r >= 0) & (r < self.n) & (col >= 0) & (col < self.n)
            np.add.at(self.obstacle, (r[ok], col[ok]), 1.0)
        self.points = world[obstacle]
        self.cliffs = world[cliff, :2]
        self.counts["frames"] += 1
        self.counts["obstacles"] = int(obstacle.sum())
        self.counts["cliffs"] = int(cliff.sum())
        self.counts["floor"] = int(floor.sum())
        return True

    def state(self):
        return {"height_m": self.h, "tilt_deg": round(self.tilt_deg, 2),
                "tilt_confident": self.tilt_samples >= 20, **self.counts,
                "obstacles": self.points[:, :3].round(3).tolist()[:64],
                "cliffs": self.cliffs.round(3).tolist()[:64]}
