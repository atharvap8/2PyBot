"""Pose filter: wheel+gyro odometry prediction, lidar scan-match correction.

State [x, y, yaw] in the map frame with covariance P. Odometry increments are
predicted with noise that grows with distance, rotation and detected wheel slip
(gyro yaw rate disagreeing with the wheels). Scan-match poses arrive with a
covariance estimated from the matcher's score surface and are rejected by a
Mahalanobis gate when they disagree with the prediction.
"""
from __future__ import annotations

import math

import numpy as np

CHI2_3DOF_99 = 11.34


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PoseFilter:
    def __init__(self, k_dist=0.03, k_turn=0.02, k_slip=0.5, base_xy=0.002, base_th=math.radians(0.2)):
        self.k_dist, self.k_turn, self.k_slip = k_dist, k_turn, k_slip
        self.base_xy, self.base_th = base_xy, base_th
        self.reset(np.zeros(3))
        self.stats = {"updates": 0, "rejected": 0, "skipped": 0, "slip": 0.0, "last_nis": 0.0}

    def reset(self, pose, sigma_xy=0.05, sigma_th=math.radians(3)):
        self.mu = np.array(pose, float)
        self.P = np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_th ** 2])

    def predict(self, dx, dy, dth, slip=0.0):
        """dx, dy, dth: motion since the last step, in the previous robot frame."""
        th = self.mu[2]
        c, s = math.cos(th), math.sin(th)
        self.mu[0] += c * dx - s * dy
        self.mu[1] += s * dx + c * dy
        self.mu[2] = wrap(th + dth)
        F = np.array([[1, 0, -s * dx - c * dy], [0, 1, c * dx - s * dy], [0, 0, 1]])
        d = math.hypot(dx, dy)
        sxy = self.base_xy + self.k_dist * d + self.k_slip * slip * max(d, 0.02)
        sth = self.base_th + self.k_turn * abs(dth) + 0.5 * slip
        self.P = F @ self.P @ F.T + np.diag([sxy ** 2, sxy ** 2, sth ** 2])
        self.stats["slip"] = slip

    def update(self, z, R):
        """Fuse a scan-match pose. Returns True if accepted by the gate."""
        y = np.array(z, float) - self.mu
        y[2] = wrap(y[2])
        S = self.P + R
        try:
            Si = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.stats["skipped"] += 1
            return False
        nis = float(y @ Si @ y)
        self.stats["last_nis"] = nis
        if nis > CHI2_3DOF_99:
            self.stats["rejected"] += 1
            return False
        K = self.P @ Si
        self.mu = self.mu + K @ y
        self.mu[2] = wrap(self.mu[2])
        I_K = np.eye(3) - K
        self.P = I_K @ self.P @ I_K.T + K @ R @ K.T      # Joseph form: stays symmetric
        self.stats["updates"] += 1
        return True

    def sigma(self):
        return {"x": math.sqrt(max(0.0, self.P[0, 0])), "y": math.sqrt(max(0.0, self.P[1, 1])),
                "yaw_deg": math.degrees(math.sqrt(max(0.0, self.P[2, 2])))}
