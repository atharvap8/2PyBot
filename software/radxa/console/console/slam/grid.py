"""Occupancy grid + correlative scan matcher (numpy only).

Frames: map x/y in metres, yaw CCW radians (REP-103). Grid row = y, col = x.
"""
from __future__ import annotations

import math
import struct
import zlib

import numpy as np

L_OCC, L_FREE, L_MIN, L_MAX = 0.85, -0.40, -4.0, 6.0
MATCH_ANG = math.radians(8)


def _box_blur(a, r):
    """Separable box blur, radius r cells (cheap stand-in for a Gaussian)."""
    if r <= 0:
        return a
    k = 2 * r + 1
    p = np.pad(a, r, mode="edge")
    c = np.cumsum(p, axis=0)
    c = np.vstack([np.zeros((1, c.shape[1]), c.dtype), c])
    a = (c[k:] - c[:-k]) / k
    c = np.cumsum(a, axis=1)
    c = np.hstack([np.zeros((c.shape[0], 1), c.dtype), c])
    return (c[:, k:] - c[:, :-k]) / k


class OccupancyGrid:
    def __init__(self, size_m=40.0, res=0.05):
        self.res = res
        self.n = int(round(size_m / res))
        self.origin = -size_m / 2.0                    # world coord of cell (0,0) corner, both axes
        self.logodds = np.zeros((self.n, self.n), np.float32)
        self.touched = np.zeros((self.n, self.n), bool)
        self._field = None
        self.version = 0

    def copy(self):
        """Independent snapshot (arrays copied) for export off the hot locks."""
        g = OccupancyGrid.__new__(OccupancyGrid)
        g.res, g.n, g.origin, g.version = self.res, self.n, self.origin, self.version
        g.logodds = self.logodds.copy()
        g.touched = self.touched.copy()
        g._field = None
        return g

    # --------------------------------------------------------- coordinates
    def to_cell(self, x, y):
        return (np.floor((y - self.origin) / self.res).astype(np.int32),
                np.floor((x - self.origin) / self.res).astype(np.int32))

    def inside(self, r, c):
        return (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)

    # --------------------------------------------------------- update
    def integrate(self, pose, pts_robot, sensor=(0.0, 0.0)):
        """Clear cells from the sensor to each hit, then mark the hit cells.
        pts_robot: (N,2) hit points in the robot frame; sensor: lidar position in the robot frame."""
        x, y, th = pose
        cs, sn = math.cos(th), math.sin(th)
        px = x + cs * pts_robot[:, 0] - sn * pts_robot[:, 1]
        py = y + sn * pts_robot[:, 0] + cs * pts_robot[:, 1]
        sx = x + cs * sensor[0] - sn * sensor[1]
        sy = y + sn * sensor[0] + cs * sensor[1]
        d = np.hypot(px - sx, py - sy)
        steps = np.maximum(1, (d / self.res).astype(np.int32))
        kmax = int(steps.max()) if len(steps) else 0
        if kmax:
            t = np.arange(kmax, dtype=np.float32)[None, :] / steps[:, None]
            mask = t < 1.0 - 1e-6
            fx = (sx + (px - sx)[:, None] * t)[mask]
            fy = (sy + (py - sy)[:, None] * t)[mask]
            r, c = self.to_cell(fx, fy)
            ok = self.inside(r, c)
            np.add.at(self.logodds, (r[ok], c[ok]), L_FREE)
            self.touched[r[ok], c[ok]] = True
        r, c = self.to_cell(px, py)
        ok = self.inside(r, c)
        np.add.at(self.logodds, (r[ok], c[ok]), L_OCC - L_FREE)   # undo the free step on the hit cell
        self.touched[r[ok], c[ok]] = True
        np.clip(self.logodds, L_MIN, L_MAX, out=self.logodds)
        self._field = None
        self.version += 1

    # --------------------------------------------------------- matching
    def field(self):
        """Likelihood field: occupied probability, blurred so the matcher has a slope."""
        if self._field is None:
            occ = (self.logodds > 0.6).astype(np.float32)
            self._field = 0.6 * _box_blur(occ, 1) + 0.4 * _box_blur(occ, 3)
        return self._field

    SCORE_CHUNK = 2048                                  # poses per batch: bounds peak memory

    def score(self, pts_robot, poses):
        """Score many candidate poses at once. poses: (K,3). Returns (K,).
        Large K (whole-map relocalization) is scored in batches so memory stays
        ~tens of MB on the A7A instead of growing with the map area."""
        if poses.shape[0] > self.SCORE_CHUNK:
            return np.concatenate([self._score(pts_robot, poses[i:i + self.SCORE_CHUNK])
                                   for i in range(0, poses.shape[0], self.SCORE_CHUNK)])
        return self._score(pts_robot, poses)

    def _score(self, pts_robot, poses):
        f = self.field()
        cs, sn = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        px = poses[:, 0:1] + cs * pts_robot[None, :, 0] - sn * pts_robot[None, :, 1]
        py = poses[:, 1:2] + sn * pts_robot[None, :, 0] + cs * pts_robot[None, :, 1]
        r, c = self.to_cell(px, py)
        ok = self.inside(r, c)
        vals = np.where(ok, f[np.clip(r, 0, self.n - 1), np.clip(c, 0, self.n - 1)], 0.0)
        return vals.sum(axis=1) / max(1, pts_robot.shape[0])

    def match(self, pts_robot, guess, lin=0.20, ang=MATCH_ANG, with_cov=False):
        """Coarse-to-fine correlative search around `guess`. Returns (pose, score)
        or (pose, score, cov) where cov comes from the middle-level score surface."""
        best = np.array(guess, float)
        cov = None
        for level, (lin_w, ang_w, n_lin, n_ang) in enumerate(
                ((lin, ang, 9, 9), (lin / 4, ang / 4, 7, 7), (lin / 16, ang / 16, 5, 5))):
            dx = np.linspace(-lin_w, lin_w, n_lin)
            da = np.linspace(-ang_w, ang_w, n_ang)
            gx, gy, ga = np.meshgrid(dx, dx, da, indexing="ij")
            cand = np.stack([best[0] + gx.ravel(), best[1] + gy.ravel(), best[2] + ga.ravel()], axis=1)
            s = self.score(pts_robot, cand)
            # small prior towards the guess keeps featureless corridors from drifting
            pen = 0.15 * (np.hypot(cand[:, 0] - guess[0], cand[:, 1] - guess[1]) / max(lin, 1e-6)
                          + np.abs(cand[:, 2] - guess[2]) / max(ang, 1e-6))
            k = int(np.argmax(s - pen * s.max()))
            if with_cov and level == 1:
                cov = self._surface_cov(cand, s, cand[k], lin_w, ang_w)
            best = cand[k]
            best_score = float(s[k])
        if with_cov:
            return best, best_score, cov
        return best, best_score

    def _surface_cov(self, cand, s, peak, lin_w, ang_w):
        """Weighted second moment of near-peak candidates (Olson-style)."""
        smax = float(s.max())
        if smax <= 0:
            return np.diag([lin_w ** 2, lin_w ** 2, ang_w ** 2])
        w = np.exp((s - smax) / max(0.01, 0.08 * smax))
        w /= w.sum()
        d = cand - peak
        d[:, 2] = np.arctan2(np.sin(d[:, 2]), np.cos(d[:, 2]))
        cov = (d * w[:, None]).T @ d
        floor = np.diag([(self.res / 2) ** 2, (self.res / 2) ** 2, math.radians(0.5) ** 2])
        return cov + floor

    # --------------------------------------------------------- export
    def known_bbox(self, pad=10):
        rows, cols = np.nonzero(self.touched)
        if not len(rows):
            return None
        return (max(0, rows.min() - pad), min(self.n, rows.max() + pad + 1),
                max(0, cols.min() - pad), min(self.n, cols.max() + pad + 1))

    def image(self, bbox=None):
        """uint8 image, row 0 = top (max y): 0 occupied ... 255 free, 128 unknown."""
        r0, r1, c0, c1 = bbox or (0, self.n, 0, self.n)
        lo = self.logodds[r0:r1, c0:c1]
        p = 1.0 / (1.0 + np.exp(lo))                 # free probability
        img = (p * 255).astype(np.uint8)
        img[~self.touched[r0:r1, c0:c1]] = 128
        return img[::-1].copy()

    def png(self, bbox=None):
        img = self.image(bbox)
        h, w = img.shape
        raw = b"".join(b"\x00" + img[i].tobytes() for i in range(h))

        def chunk(tag, data):
            return (struct.pack(">I", len(data)) + tag + data
                    + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))
        return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
