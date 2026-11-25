"""Lidar-based autonomous missions driven through the firmware navigation lease.

* RoamPlanner    - vacuum-style roaming: drive straight, stop at the safety bubble,
                   turn toward open and less-visited space, back out when boxed in.
* GotoPlanner    - map-based navigation: A* on the saved/live map to a point or an
                   area label, pure-pursuit tracking on the SLAM pose. Watches the
                   route ahead and plans round new obstacles as soon as they show.
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
    """Drive to a point on the map, and go round whatever turns up on the way.

    The route is planned on the saved map plus what the lidar and the depth sensor see
    right now. While driving, the stretch of route ahead is checked against the live
    scan several times a second; as soon as something sits on it a new route is planned,
    without first driving up to the obstacle and waiting. If the robot is stopped by
    something it cannot see in the scan (too low for the lidar), it marks the spot,
    backs off a little and plans round it. It gives up only after staying stuck in one
    place for PATIENCE seconds.
    """
    kind = "goto"
    REACHED = 0.18
    LOOKAHEAD = 0.35
    REPLAN_GAP = 1.5         # s between route requests
    CHECK_EVERY = 0.25       # s between checks of the route ahead against the live scan
    CHECK_AHEAD = 1.6        # m of route checked
    CLEARANCE = 0.11         # m: a live return this close to the route line blocks it (the robot is 0.105 m to each side)
    MEMORY = 6.0             # s something seen keeps blocking after it was last seen
    MARK_MEMORY = 20.0       # s a spot the robot was stopped at (but could not see) stays blocked
    PATIENCE = 40.0          # s stuck in one place before giving up
    BACK_OFF = 0.20          # m reversed to get clear of an obstacle

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
        self._job_obstacles = False
        self.error = ""
        self.final_heading = goal.get("heading")
        self.seen = {}               # 10 cm cell -> time until which it blocks the route
        self.plan_after = -math.inf  # earliest time of the next route request
        self.check_at = -math.inf
        self.hits = 0                # consecutive checks that found the route blocked
        self.map_failures = 0
        self.anchor = None           # where the robot last made real progress, and when
        self.moved_at = None
        self.backing = None          # {"from": (x, y), "until": t} while reversing clear
        self.backed = False
        self.last_step = -math.inf

    # ---------------------------------------------------------- what is in the way
    def _live(self, pose):
        """Lidar returns near the robot and depth-sensor obstacles, in the map frame."""
        with self.engine.lock:
            pts = np.asarray(self.engine.bubble_pts, float).reshape(-1, 2).copy()
        c, s = math.cos(pose[2]), math.sin(pose[2])
        near = pts[np.hypot(pts[:, 0], pts[:, 1]) < 2.5]
        out = [np.c_[pose[0] + c * near[:, 0] - s * near[:, 1], pose[1] + s * near[:, 0] + c * near[:, 1]]]
        layer = getattr(self.engine, "tof_layer", None)       # low things the lidar plane passes over, and drop-offs
        for name in ("points", "cliffs"):
            extra = getattr(layer, name, None)
            if extra is not None and len(extra):
                out.append(np.asarray(extra, float)[:, :2])
        return np.vstack(out)

    def _remember(self, live, now, keep=None):
        until = now + (self.MEMORY if keep is None else keep)
        for ix, iy in set(map(tuple, np.round(live / 0.1).astype(int).tolist())):
            if self.seen.get((ix, iy), 0.0) < until:
                self.seen[(ix, iy)] = until
        for k in [k for k, t in self.seen.items() if t < now]:
            del self.seen[k]

    def _obstacles(self, pose, now):
        live = self._live(pose)
        self._remember(live, now)
        cells = np.array(list(self.seen), float).reshape(-1, 2) * 0.1
        return np.vstack([live, cells])

    def _route_blocked(self, pose, live):
        """Does anything in view sit on the next stretch of the route?"""
        if not len(live):
            return False
        pts, travelled, prev = [], 0.0, (pose[0], pose[1])
        for q in self.path[self.index:]:
            travelled += math.hypot(q[0] - prev[0], q[1] - prev[1])
            prev = q
            if travelled > self.CHECK_AHEAD:
                break
            # the first steps are the bubble's job; the goal itself may be right beside furniture
            if travelled > 0.2 and math.hypot(q[0] - self.goal["x"], q[1] - self.goal["y"]) > 0.3:
                pts.append(q)
        if not pts:
            return False
        d2 = ((np.asarray(pts, float)[:, None, :] - live[None, :, :]) ** 2).sum(axis=2)
        return bool((d2 < self.CLEARANCE ** 2).any())

    # ---------------------------------------------------------- planning
    # planning runs in a worker thread so the 20 Hz lease stream never stalls
    def _start_plan(self, pose, obstacles=False, now=0.0):
        grid, _, _, _ = self.engine.snapshot()
        extra = self._obstacles(pose, now) if obstacles else None
        start, goal = (pose[0], pose[1]), (self.goal["x"], self.goal["y"])

        def work():
            try:
                self._result = ("ok", pl.plan(grid, start, goal, extra))
            except pl.PlanError as e:
                self._result = ("error", str(e))
            except Exception as e:  # noqa: BLE001 - report, never kill the mission loop
                self._result = ("error", "planner failed: " + str(e))
        self._result = None
        self._job_obstacles = bool(obstacles)
        self._job = threading.Thread(target=work, name="planner", daemon=True)
        self._job.start()

    def _request(self, now, pose, obstacles=True, force=False):
        """Ask for a new route unless one is being worked out or was asked for a moment ago."""
        if self._job is not None or (now < self.plan_after and not force):
            return False
        self.plan_after = now + self.REPLAN_GAP
        if self.path:
            self.replans += 1
        self._start_plan(pose, obstacles, now)
        return True

    def publish(self):
        with self.engine.lock:
            self.engine.plan = [list(p) for p in self.path[self.index:]]
            self.engine.goal = {k: self.goal[k] for k in ("name", "x", "y") if k in self.goal}

    def _collect(self, now, pose, info):
        """Take a finished route. Returns a Step only when the mission has to end."""
        if self._job is None or self._job.is_alive():
            return None
        self._job = None
        status, value = self._result
        if status == "ok":
            self.path, self.index, self.error, self.hits, self.map_failures = value, 0, "", 0, 0
            self.publish()
            return None
        self.error = value
        if self._job_obstacles:
            # No way round what is in the way right now. With no route at all, fall back to the
            # map alone; otherwise keep the route we have and look again shortly.
            if not self.path:
                self._start_plan(pose, False, now)
            return None
        self.map_failures += 1
        if not self.path or self.map_failures > 4:
            return Step(done=True, reason=value, info=info)       # the map itself has no route
        return None

    # ---------------------------------------------------------- driving
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
        if now - self.last_step > 1.0:                # first step, or back from a pause: start the stuck clock afresh
            self.anchor, self.blocked_since = None, None
        self.last_step = now
        if self.anchor is None or math.hypot(pose[0] - self.anchor[0], pose[1] - self.anchor[1]) > 0.25:
            self.anchor, self.moved_at = (pose[0], pose[1]), now
        if now - self.moved_at > self.PATIENCE:
            return Step(done=True, reason="path blocked: no way round" + (" (" + self.error + ")" if self.error else ""), info=info)
        ended = self._collect(now, pose, info)
        if ended is not None:
            return ended
        if self.backing is not None:                  # reversing clear of something before going round it
            gone = math.hypot(pose[0] - self.backing["from"][0], pose[1] - self.backing["from"][1])
            rev = bubble["reverse_cap"]
            if gone >= self.BACK_OFF or now > self.backing["until"] or rev < 0.03:
                self.backing = None
                self._request(now, pose, force=True)
            else:
                return Step(-min(0.12, rev), 0.0, info=dict(info, waiting="replanning"))
        if not self.path:
            self._request(now, pose, force=self._job is None and not self.error)
            return Step(info=dict(info, waiting="planning"))
        # advance along the path; pick the lookahead point
        best_i, best_d = self.index, math.inf
        for i in range(self.index, min(len(self.path), self.index + 25)):
            d = math.hypot(self.path[i][0] - pose[0], self.path[i][1] - pose[1])
            if d < best_d:
                best_i, best_d = i, d
        if best_d > 0.6:                              # pushed off the path: plan again
            self.path, self.index = [], 0
            self._request(now, pose, force=True)
            return Step(info=dict(info, waiting="planning"))
        self.index = best_i
        if now - self.check_at >= self.CHECK_EVERY:   # is the way ahead still open?
            self.check_at = now
            live = self._live(pose)
            self._remember(live, now)
            self.hits = self.hits + 1 if self._route_blocked(pose, live) else 0
            if self.hits >= 2:
                self._request(now, pose)
        if self.hits >= 2 and self.error:             # no way round it yet: wait here, not nose-on to it
            self._request(now, pose)
            return Step(info=dict(info, waiting="replanning", blocked="no way round yet"))
        la = self.index
        while la < len(self.path) - 1 and math.hypot(self.path[la][0] - pose[0], self.path[la][1] - pose[1]) < self.LOOKAHEAD:
            la += 1
        tx, ty = self.path[la]
        alpha = wrap(math.atan2(ty - pose[1], tx - pose[0]) - pose[2])
        self.publish()
        if abs(alpha) > 0.7:
            w = _spin(alpha, bubble)
            if w is None:
                w = _spin(alpha, bubble, prefer=-1 if alpha > 0 else 1)      # the long way round
            if w is None:
                return self._blocked(now, pose, bubble, info, "turn blocked")
            self.blocked_since = None
            return Step(0.0, w, info=info)
        brake = math.sqrt(0.09 + 2 * 0.3 * max(0.0, dist - 0.1)) - 0.3
        v = min(self.speed, brake, depth_cap) * max(0.3, math.cos(alpha))
        cap = min(bubble["forward_cap"], depth_cap)
        if cap < 0.05:
            return self._blocked(now, pose, bubble, info, "obstacle ahead")
        self.blocked_since = None
        self.backed = False
        v = min(v, cap)
        L = max(0.15, math.hypot(tx - pose[0], ty - pose[1]))
        w = clip(2.0 * max(v, 0.1) * math.sin(alpha) / L, -MAX_W, MAX_W)
        return Step(max(0.0, v), w, info=info)

    def _blocked(self, now, pose, bubble, info, why):
        """Stopped by the safety bubble: look for a way round at once, do not wait it out."""
        if self.blocked_since is None:
            self.blocked_since = now
        if now - self.blocked_since > 2.0 and not self.backed and self._job is None:
            # A new route did not free us. Either we are nose-on to it, or it is something
            # the lidar cannot see. Remember the spot, reverse a little, plan again.
            self.backed = True
            c, s = math.cos(pose[2]), math.sin(pose[2])
            live = self._live(pose)
            ahead = np.array([[pose[0] + c * 0.30 - s * y, pose[1] + s * 0.30 + c * y] for y in (-0.15, -0.075, 0.0, 0.075, 0.15)])
            unseen = not len(live) or float(np.hypot(live[:, 0] - ahead[2, 0], live[:, 1] - ahead[2, 1]).min()) > 0.30
            if unseen:
                self._remember(ahead, now, keep=self.MARK_MEMORY)
            if bubble["reverse_cap"] >= 0.05:
                self.backing = {"from": (pose[0], pose[1]), "until": now + 3.0}
            else:
                self._request(now, pose, force=True)
        else:
            self._request(now, pose)
        return Step(info=dict(info, waiting="replanning", blocked=why))


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
