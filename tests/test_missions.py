"""Go-to missions on a simulated floor: the planner has to go round things, not wait for them."""
import importlib
import math
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

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
        return {"forward_cap": 0.0 if ahead.any() else 1.5, "reverse_cap": 0.0 if behind.any() else 0.3,
                "turn_left_ok": True, "turn_right_ok": True, "stale": False}

    def run(self, seconds, until=None):
        for _ in range(int(seconds / DT)):
            self.t += DT
            with self.engine.lock:
                self.engine.bubble_pts = self.robot_frame(self.seen)
            step = self.planner.step(self.t, DT, tuple(self.pose), self.bubble(), 9.0, 0.0)
            job = getattr(self.planner, '_job', None) or getattr(getattr(self.planner, 'goto', None), '_job', None)
            if job is not None:
                job.join()                                # planning is a thread on the robot; here it is instant
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


# ---- 5.4.0: several stops, in the order given ----
def route(stops, walls=()):
    engine = Engine(room(walls))
    planner = missions.RoutePlanner(engine, [{"name": n, "x": x, "y": y} for n, x, y in stops], speed=0.3)
    return World(planner, engine)


def test_a_route_visits_its_stops_in_the_order_given():
    w = route([("a", 0.0, 1.2), ("b", 2.0, -1.0), ("c", -1.0, -1.2)])
    visited = []

    def watch(w):
        for name, x, y in (("a", 0.0, 1.2), ("b", 2.0, -1.0), ("c", -1.0, -1.2)):
            if name not in visited and math.hypot(w.pose[0] - x, w.pose[1] - y) < 0.25:
                visited.append(name)
        return False
    w.run(120, until=watch)
    assert w.done == "goal reached" and visited == ["a", "b", "c"]
    info = w.steps[-1].info
    assert info["legs"] == 3 and all(s["done"] for s in info["stops"])


def test_a_route_reports_which_leg_it_is_on():
    w = route([("a", 0.0, 1.2), ("b", 2.0, -1.0)])
    w.run(2)
    info = w.steps[-1].info
    assert (info["leg"], info["legs"], info["goal"]) == (1, 2, "a")
    assert [s["done"] for s in info["stops"]] == [False, False]
    w.run(60, until=lambda w: w.steps[-1].info.get("leg") == 2)
    assert w.steps[-1].info["stops"][0]["done"] and not w.done


def test_a_stop_that_cannot_be_reached_ends_the_route_and_names_it():
    w = route([("a", -1.0, 1.0), ("behind the wall", 2.0, 0.0), ("c", -1.0, -1.0)], walls=[(0.5, -2.0, 0.6, 2.0)])
    w.run(120)
    assert w.done.startswith("stop 2 (behind the wall):") and w.pose[0] < 0.5


def test_a_route_needs_a_stop():
    with pytest.raises(ValueError):
        missions.RoutePlanner(Engine(room()), [])


# ---- 5.5.0: RT on the gamepad is the autopilot's speed boost ----
def test_boost_is_one_step_up_the_speed_ladder():
    assert missions.boosted(0.25, 0.0) == 0.25, "released: the speed chosen for the mission"
    assert missions.boosted(0.15, 1.0) == pytest.approx(0.25), "slow -> normal"
    assert missions.boosted(0.25, 1.0) == pytest.approx(0.40), "normal -> fast"
    assert missions.boosted(0.40, 1.0) == pytest.approx(0.90), "fast -> the robot's top speed"
    assert missions.boosted(0.40, 1.0, top=0.7) == pytest.approx(0.70), "or the controller's own limit if that is lower"
    assert missions.boosted(0.25, 0.5) == pytest.approx(0.325), "proportional in between"
    assert missions.boosted(0.40, 5.0) == pytest.approx(0.90) and missions.boosted(0.40, -1.0) == 0.40
    assert missions.boosted(0.60, 1.0, top=0.5) == 0.60, "never slower than what was chosen"


def test_a_boosted_goto_arrives_sooner_and_still_arrives():
    plain = make()
    plain.run(60)
    fast = make()
    fast.planner.speed = missions.boosted(fast.planner.base_speed, 1.0)
    assert fast.planner.speed == pytest.approx(0.4) and fast.planner.base_speed == pytest.approx(0.3)
    fast.planner.speed = 0.9
    fast.run(60)
    assert plain.done == fast.done == "goal reached"
    assert fast.t < plain.t * 0.75, "it must actually drive faster"
    assert max(s.v for s in fast.steps) > 0.6


def test_it_slows_for_corners_so_a_boosted_robot_stays_on_the_route():
    w = make(walls=[(0.0, -2.0, 0.1, 0.6)], goal=(2.0, -1.2))       # round the end of a wall: a real corner
    w.planner.speed = 0.9
    worst = []

    def off_route(w):
        p = w.planner
        if p.path:
            worst.append(min(math.hypot(q[0] - w.pose[0], q[1] - w.pose[1]) for q in p.path))
        return False
    w.run(90, until=off_route)
    assert w.done == "goal reached"
    assert max(worst) < 0.35, "it must not run wide of its own route"
    assert max(s.v for s in w.steps) > 0.7, "boosted on the straights"
    corner = [s.v for s in w.steps if s.v > 0.05 and abs(s.w) > 0.5]
    assert corner and min(corner) < 0.8, "and slower through the corner"
    assert max(abs(s.w) for s in w.steps) <= missions.MAX_W + 1e-9


def test_a_route_passes_its_boosted_speed_to_every_leg():
    w = route([("a", 0.0, 1.2), ("b", 2.0, -1.0)])
    w.planner.speed = 0.8
    w.run(60)
    assert w.done == "goal reached" and max(s.v for s in w.steps) > 0.55
    assert w.planner.base_speed == pytest.approx(0.3)
