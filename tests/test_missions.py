"""Go-to missions on a simulated floor: the planner has to go round things, not wait for them."""
import importlib
import math
import sys
import threading
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software/radxa/console"))
missions = importlib.import_module("console.missions")
OccupancyGrid = importlib.import_module("console.slam.grid").OccupancyGrid

DT = 0.05


def room(walls=()):
    """A mapped 6 m x 4 m room centred on the origin, with optional mapped walls [(x0, y0, x1, y1)]."""
    g = OccupancyGrid(size_m=12.0, res=0.05)

    def box(x0, y0, x1, y1, value):
        (r0, c0), (r1, c1) = g.to_cell(np.float64(x0), np.float64(y0)), g.to_cell(np.float64(x1), np.float64(y1))
        g.logodds[r0:r1 + 1, c0:c1 + 1] = value
        g.touched[r0:r1 + 1, c0:c1 + 1] = True
    box(-3.1, -2.1, 3.1, 2.1, 2.0)
    box(-3.0, -2.0, 3.0, 2.0, -2.0)
    for wall in walls:
        box(*wall, 2.0)
    g.version = 1
    return g


class Engine:
    def __init__(self, grid):
        self.grid, self.lock, self.bubble_pts = grid, threading.Lock(), np.zeros((0, 2))
        self.plan, self.goal, self.pose, self.mode = [], None, np.zeros(3), "localizing"

    def snapshot(self):
        return self.grid.copy(), self.pose.copy(), self.mode, True


def disc(x, y, r=0.12, n=16):
    return [(x + r * math.cos(2 * math.pi * k / n), y + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


class World:
    """Kinematic robot plus things on the floor. `seen` are in the lidar's view, `low` are not."""

    def __init__(self, planner, engine, pose=(-2.0, 0.0, 0.0)):
        self.planner, self.engine, self.pose, self.t = planner, engine, list(pose), 0.0
        self.seen, self.low, self.steps, self.done = [], [], [], None

    def robot_frame(self, pts):
        if not pts:
            return np.zeros((0, 2))
        p = np.asarray(pts, float) - self.pose[:2]
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        return np.c_[c * p[:, 0] + s * p[:, 1], -s * p[:, 0] + c * p[:, 1]]

    def bubble(self):
        local = self.robot_frame(self.seen + self.low)
        ahead = (local[:, 0] > 0.0) & (local[:, 0] < 0.30) & (np.abs(local[:, 1]) < 0.13)
        behind = (local[:, 0] < 0.0) & (local[:, 0] > -0.28) & (np.abs(local[:, 1]) < 0.13)
        return {"forward_cap": 0.0 if ahead.any() else 0.5, "reverse_cap": 0.0 if behind.any() else 0.3,
                "turn_left_ok": True, "turn_right_ok": True, "stale": False}

    def run(self, seconds, until=None):
        for _ in range(int(seconds / DT)):
            self.t += DT
            with self.engine.lock:
                self.engine.bubble_pts = self.robot_frame(self.seen)
            step = self.planner.step(self.t, DT, tuple(self.pose), self.bubble(), 9.0, 0.0)
            if self.planner._job is not None:
                self.planner._job.join()                  # planning is a thread on the robot; here it is instant
            self.steps.append(step)
            if step.done:
                self.done = step.reason
                return
            self.pose[2] = missions.wrap(self.pose[2] + step.w * DT)
            self.pose[0] += step.v * math.cos(self.pose[2]) * DT
            self.pose[1] += step.v * math.sin(self.pose[2]) * DT
            if until and until(self):
                return

    def nearest(self, pts):
        return float(np.hypot(*(np.asarray(pts, float) - self.pose[:2]).T).min())


def make(walls=(), goal=(2.0, 0.0)):
    engine = Engine(room(walls))
    planner = missions.GotoPlanner(engine, {"name": "spot", "x": goal[0], "y": goal[1]}, speed=0.3)
    return World(planner, engine)


def test_it_drives_to_the_goal_on_a_clear_floor():
    w = make()
    w.run(40)
    assert w.done == "goal reached"
    assert w.planner.replans == 0


def test_something_put_on_the_route_is_driven_round_without_stopping_at_it():
    w = make()
    w.run(40, until=lambda w: w.pose[0] > -1.0)           # under way
    box = disc(0.2, 0.0)
    w.seen = box
    closest = []
    start = len(w.steps)
    w.run(60, until=lambda w: closest.append(w.nearest(box)) or False)
    assert w.done == "goal reached"
    assert w.planner.replans >= 1
    assert min(closest) > 0.13, "it must pass clear of the thing, not brush it"
    assert not any("blocked" in s.info for s in w.steps[start:]), "it must turn away before the safety bubble has to stop it"


def test_a_blocked_doorway_ends_the_mission_instead_of_waiting_forever():
    # a mapped wall across the room with one 0.8 m doorway, then something left in the doorway
    w = make(walls=[(0.0, -2.0, 0.1, -0.4), (0.0, 0.4, 0.1, 2.0)])
    w.seen = [(0.05, y) for y in np.arange(-0.4, 0.41, 0.05)]
    w.run(240)
    assert w.done and w.done.startswith("path blocked")
    assert w.pose[0] < 0.0, "it must not push through"
    assert w.t < 120, "it gives up after a bounded time"


def test_it_goes_through_when_the_doorway_clears():
    w = make(walls=[(0.0, -2.0, 0.1, -0.4), (0.0, 0.4, 0.1, 2.0)])
    w.seen = [(0.05, y) for y in np.arange(-0.4, 0.41, 0.05)]
    w.run(15)
    assert not w.done
    w.seen = []                                           # whoever stood there walked off
    w.run(60)
    assert w.done == "goal reached"


def test_stopped_by_something_the_lidar_cannot_see_it_backs_off_and_goes_round():
    w = make()
    w.low = disc(0.0, 0.0, r=0.08)                        # under the scan plane: only the bubble's depth guard stops for it
    w.run(90)
    assert w.done == "goal reached"
    assert any(s.v < 0 for s in w.steps), "it reverses to get clear"
    assert any(s.info.get("blocked") for s in w.steps)


def test_a_long_pause_does_not_count_as_being_stuck():
    w = make()
    w.run(3)
    w.t += 600                                            # paused for ten minutes, then resumed
    w.run(40)
    assert w.done == "goal reached"


def test_no_route_on_the_map_is_reported_at_once():
    w = make(walls=[(0.0, -2.0, 0.1, 2.0)])               # a wall with no doorway
    w.run(10)
    assert w.done and "path" in w.done.lower()
    assert w.t < 2
