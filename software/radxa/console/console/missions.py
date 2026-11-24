"""Lidar-based autonomous missions driven through the firmware navigation lease.

* RoamPlanner    - vacuum-style roaming: drive straight, stop at the safety bubble,
                   turn toward open and less-visited space, back out when boxed in.
* GotoPlanner    - map-based navigation: A* on the saved/live map to a point or an
                   area label, pure-pursuit tracking on the SLAM pose, re-plans
                   around new obstacles seen by the lidar.
* ExplorePlanner - autonomous mapping: repeatedly drives to the nearest frontier
                   (edge of mapped space) while mapping, until none remain.

Planners never talk to the serial link. Each step returns (v m/s, w rad/s) or a
finish reason; Autonomy owns the lease, the supervisor heartbeat and the stops.
All commands are already limited by the bubble here; the firmware applies the
same bubble caps again (BG) to every drive source.
"""
from __future__ import annotations

import math
import random
import threading

import numpy as np

from .slam import planner as pl


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip(v, lo, hi):
    return min(hi, max(lo, v))


MAX_W = 0.85            # rad/s for spins and tracking (firmware allows ~4 rad/s)


class Step:
    __slots__ = ("v", "w", "done", "reason", "info")

    def __init__(self, v=0.0, w=0.0, done=False, reason="", info=None):
        self.v, self.w, self.done, self.reason, self.info = v, w, done, reason, info or {}


def _spin(err, bubble, prefer=None):
    """In-place turn toward heading error `err` if the bubble allows that direction."""
    sign = 1 if err > 0 else -1
    if prefer is not None:
        sign = prefer
    ok = bubble.get("turn_left_ok") if sign > 0 else bubble.get("turn_right_ok")
    if not ok:
        return None
    return sign * min(MAX_W, 0.25 + 1.4 * abs(err))


# ------------------------------------------------------------------ roaming
class RoamPlanner:
    kind = "roam"

    def __init__(self, speed=0.25, duration_s=600.0, seed=None):
        self.speed = clip(float(speed), 0.08, 0.6)
        self.duration = clip(float(duration_s), 10.0, 7200.0)
        self.rng = random.Random(seed)
        self.state = "cruise"
        self.target = None
        self.since = 0.0
        self.elapsed = 0.0
        self.visited = set()
        self.stuck = 0
        self.slow_since = None
        self.turns = 0
        self.distance = 0.0
        self._last_xy = None

    def _cell(self, x, y):
        return int(math.floor(x / 0.3)), int(math.floor(y / 0.3))

    def _choose(self, pose, bubble):
        free = bubble.get("free_by_heading") or []
        n = len(free)
        best, best_s = None, -math.inf
        for k in range(1, n):
            ang = wrap(2 * math.pi * k / n)
            f = free[k]
            f = 4.0 if f is None else f
            if f < 0.45:
                continue
            fx = pose[0] + math.cos(pose[2] + ang) * min(f, 1.0)
            fy = pose[1] + math.sin(pose[2] + ang) * min(f, 1.0)
            novelty = 0.0 if self._cell(fx, fy) in self.visited else 0.9
            score = min(f, 2.0) + novelty + self.rng.uniform(0, 0.5) - 0.15 * abs(ang)
            if score > best_s:
                best, best_s = ang, score
        return best

    def step(self, now, dt, pose, bubble, depth_cap, measured_v):
        self.elapsed += dt
        if self._last_xy is not None:
            self.distance += math.hypot(pose[0] - self._last_xy[0], pose[1] - self._last_xy[1])
        self._last_xy = (pose[0], pose[1])
        self.visited.add(self._cell(pose[0], pose[1]))
        info = {"roam_state": self.state, "turns": self.turns, "visited_cells": len(self.visited),
                "distance_m": round(self.distance, 2), "remaining_s": round(self.duration - self.elapsed)}
        if self.elapsed >= self.duration:
            return Step(done=True, reason="roam time finished", info=info)
        fwd_cap = min(bubble["forward_cap"], depth_cap)
        if self.state == "cruise":
            front = bubble.get("front_m")
            if fwd_cap < min(self.speed, 0.14) or (front is not None and front < 0.28):
                self.state, self.target, self.since = "choose", None, now
                return Step(info=info)
            if self.target is None:
                self.target = pose[2]
            v = min(self.speed, fwd_cap)
            w = clip(1.5 * wrap(self.target - pose[2]), -0.5, 0.5)
            # Commanding motion but not moving (wheel stuck on a low object the
            # lidar cannot see): treat it like an obstacle.
            if v > 0.1 and abs(measured_v) < 0.03:
                self.slow_since = self.slow_since or now
                if now - self.slow_since > 3.0:
                    self.slow_since = None
                    self.state, self.since = "backup", now
            else:
                self.slow_since = None
            return Step(v, w, info=info)
        if self.state == "choose":
            ang = self._choose(pose, bubble)
            if ang is None:
                self.state, self.since = "backup", now
                return Step(info=info)
            self.target = wrap(pose[2] + ang)
            self.turns += 1
            self.state, self.since = "turn", now
            return Step(info=info)
        if self.state == "turn":
            err = wrap(self.target - pose[2])
            if abs(err) < 0.12:
                self.state, self.stuck = "cruise", 0
                return Step(info=info)
            w = _spin(err, bubble)
            if w is None:
                w = _spin(err, bubble, prefer=-1 if err > 0 else 1)   # the long way round
            if w is None or now - self.since > 8.0:
                self.state, self.since = "backup", now
                return Step(info=info)
            return Step(0.0, w, info=info)
        if self.state == "backup":
            rev = bubble["reverse_cap"]
            if rev < 0.03:
                self.stuck += 1
                if self.stuck > 60:                # ~3 s boxed in on all sides
                    return Step(done=True, reason="boxed in: no room to turn or back out", info=info)
                return Step(info=info)
            if now - self.since > 0.9:
                self.state = "choose"
                return Step(info=info)
            return Step(-min(0.12, rev), 0.0, info=info)
        self.state = "cruise"
        return Step(info=info)


# ------------------------------------------------------------------ map navigation
class GotoPlanner:
    kind = "goto"
    REACHED = 0.18
    LOOKAHEAD = 0.35

    def __init__(self, engine, goal, speed=0.30):
        self.engine = engine
        self.goal = dict(goal)
        self.speed = clip(float(speed), 0.08, 0.6)
        self.path = []
        self.index = 0
        self.replans = 0
        self.blocked_since = None
        self.lost_since = None
        self._job = None
        self._result = None
        self.error = ""
        self.final_heading = goal.get("heading")

    # planning runs in a worker thread so the 20 Hz lease stream never stalls
    def _start_plan(self, pose, obstacles=False):
        grid, _, _, _ = self.engine.snapshot()
        extra = None
        if obstacles:
            with self.engine.lock:
                pts = self.engine.bubble_pts.copy()
            c, s = math.cos(pose[2]), math.sin(pose[2])
            near = pts[np.hypot(pts[:, 0], pts[:, 1]) < 2.5]
            extra = np.c_[pose[0] + c * near[:, 0] - s * near[:, 1], pose[1] + s * near[:, 0] + c * near[:, 1]]
        start, goal = (pose[0], pose[1]), (self.goal["x"], self.goal["y"])

        def work():
            try:
                self._result = ("ok", pl.plan(grid, start, goal, extra))
            except pl.PlanError as e:
                self._result = ("error", str(e))
            except Exception as e:  # noqa: BLE001 - report, never kill the mission loop
                self._result = ("error", "planner failed: " + str(e))
        self._result = None
        self._job = threading.Thread(target=work, name="planner", daemon=True)
        self._job.start()

    def publish(self):
        with self.engine.lock:
            self.engine.plan = [list(p) for p in self.path[self.index:]]
            self.engine.goal = {k: self.goal[k] for k in ("name", "x", "y") if k in self.goal}

    def step(self, now, dt, pose, bubble, depth_cap, measured_v, localized=True):
        gx, gy = self.goal["x"], self.goal["y"]
        dist = math.hypot(gx - pose[0], gy - pose[1])
        info = {"goal": self.goal.get("name"), "distance_m": round(dist, 2), "replans": self.replans,
                "path_points": max(0, len(self.path) - self.index)}
        if not localized:
            self.lost_since = self.lost_since or now
            if now - self.lost_since > 6.0:
                return Step(done=True, reason="lost localization on the map", info=info)
            return Step(info=dict(info, waiting="localizing"))
        self.lost_since = None
        if dist < self.REACHED:
            if self.final_heading is not None:
                err = wrap(self.final_heading - pose[2])
                if abs(err) > 0.15:
                    w = _spin(err, bubble)
                    return Step(0.0, w or 0.0, info=info)
            return Step(done=True, reason="goal reached", info=info)
        if self._job is not None:
            if self._job.is_alive():
                return Step(info=dict(info, waiting="planning"))
            self._job = None
            status, value = self._result
            if status != "ok":
                if self.replans < 4 and self.path:
                    self.replans += 1
                    self._start_plan(pose, obstacles=False)
                    return Step(info=info)
                return Step(done=True, reason=value, info=info)
            self.path, self.index = value, 0
            self.publish()
        if not self.path:
            self._start_plan(pose)
            return Step(info=dict(info, waiting="planning"))
        # advance along the path; pick the lookahead point
        best_i, best_d = self.index, math.inf
        for i in range(self.index, min(len(self.path), self.index + 25)):
            d = math.hypot(self.path[i][0] - pose[0], self.path[i][1] - pose[1])
            if d < best_d:
                best_i, best_d = i, d
        if best_d > 0.6:                              # pushed off the path: plan again
            self._start_plan(pose)
            return Step(info=dict(info, waiting="planning"))
        self.index = best_i
        la = self.index
        while la < len(self.path) - 1 and math.hypot(self.path[la][0] - pose[0], self.path[la][1] - pose[1]) < self.LOOKAHEAD:
            la += 1
        tx, ty = self.path[la]
        alpha = wrap(math.atan2(ty - pose[1], tx - pose[0]) - pose[2])
        self.publish()
        if abs(alpha) > 0.7:
            w = _spin(alpha, bubble)
            if w is None:
                return self._blocked(now, pose, info, "turn blocked")
            self.blocked_since = None
            return Step(0.0, w, info=info)
        brake = math.sqrt(0.09 + 2 * 0.3 * max(0.0, dist - 0.1)) - 0.3
        v = min(self.speed, brake, depth_cap) * max(0.3, math.cos(alpha))
        cap = bubble["forward_cap"]
        if cap < 0.05:
            return self._blocked(now, pose, info, "obstacle ahead")
        self.blocked_since = None
        v = min(v, cap)
        L = max(0.15, math.hypot(tx - pose[0], ty - pose[1]))
        w = clip(2.0 * max(v, 0.1) * math.sin(alpha) / L, -MAX_W, MAX_W)
        return Step(max(0.0, v), w, info=info)

    def _blocked(self, now, pose, info, why):
        self.blocked_since = self.blocked_since or now
        if now - self.blocked_since > 2.0:
            self.blocked_since = None
            self.replans += 1
            if self.replans > 6:
                return Step(done=True, reason="path blocked: " + why, info=info)
            self._start_plan(pose, obstacles=True)       # route around what the lidar sees now
        return Step(info=dict(info, waiting=why))


class ExplorePlanner:
    """Autonomous mapping: drive to frontier after frontier while SLAM maps."""
    kind = "explore"

    def __init__(self, engine, speed=0.25, duration_s=1200.0):
        self.engine = engine
        self.speed = clip(float(speed), 0.08, 0.5)
        self.duration = clip(float(duration_s), 30.0, 7200.0)
        self.elapsed = 0.0
        self.goto = None
        self.visited_goals = []
        self.failures = 0
        self.reached = 0
        self._job = None
        self._goals = None

    def _frontiers(self, pose):
        grid, _, _, _ = self.engine.snapshot()

        def work():
            try:
                self._goals = pl.frontier_goals(grid, (pose[0], pose[1]))
            except Exception:  # noqa: BLE001
                self._goals = []
        self._goals = None
        self._job = threading.Thread(target=work, name="frontiers", daemon=True)
        self._job.start()

    def step(self, now, dt, pose, bubble, depth_cap, measured_v, localized=True):
        self.elapsed += dt
        info = {"frontiers_reached": self.reached, "failures": self.failures,
                "remaining_s": round(self.duration - self.elapsed)}
        if self.elapsed > self.duration:
            return Step(done=True, reason="explore time finished", info=info)
        if self.goto is None:
            if self._job is None:
                self._frontiers(pose)
            if self._job.is_alive():
                return Step(info=dict(info, waiting="finding frontiers"))
            self._job = None
            goals = [g for g in (self._goals or []) if all(math.hypot(g[0] - a, g[1] - b) > 0.5 for a, b in self.visited_goals)]
            if not goals:
                return Step(done=True, reason="exploration complete: no reachable unmapped edges", info=info)
            g = goals[0]
            self.visited_goals.append(g)
            self.goto = GotoPlanner(self.engine, {"name": "frontier", "x": g[0], "y": g[1]}, self.speed)
            return Step(info=info)
        st = self.goto.step(now, dt, pose, bubble, depth_cap, measured_v, localized)
        if st.done:
            if st.reason == "goal reached":
                self.reached += 1
            else:
                self.failures += 1
                if self.failures > 8:
                    return Step(done=True, reason="exploration stopped: too many unreachable frontiers", info=info)
            self.goto = None
            return Step(info=info)
        st.info = dict(info, **st.info)
        return st
