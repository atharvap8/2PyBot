"""Localization on a loaded map must not throw the robot into another room on one look."""
import importlib
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software/radxa/console"))
engine_mod = importlib.import_module("console.slam.engine")
SlamEngine = engine_mod.SlamEngine

COV = np.diag([0.01 ** 2, 0.01 ** 2, math.radians(0.5) ** 2])


class FakeGrid:
    """Stands in for the occupancy grid: the test decides what each search finds."""
    version, res, origin, n = 7, 0.05, -20.0, 800

    def __init__(self):
        self.local = lambda guess: (np.array(guess, float), 0.0)        # (pose, score) around a guess
        self.calls = []

    def match(self, pts, guess, lin=0.20, ang=0.14, with_cov=False):
        self.calls.append(lin)
        pose, score = self.local(guess)
        return (pose, score, COV) if with_cov else (pose, score)


def lost_engine(tmp_path, here=(1.0, 1.0, 0.0)):
    e = SlamEngine(str(tmp_path))
    e.grid, e.mode = FakeGrid(), "localizing"
    e.scan_pts = np.c_[np.cos(np.linspace(0, 6.28, 90)), np.sin(np.linspace(0, 6.28, 90))] * 2.0
    e.pose = np.array(here, float)
    e.filter.reset(e.pose)
    e.searches = []
    return e


def scan(n=90, dist_mm=2000):
    now = time.monotonic()
    return {"angle_deg": list(np.linspace(0, 360, n, endpoint=False)), "dist_mm": [dist_mm] * n, "quality": [50] * n,
            "t0": now - 0.05, "t1": now}


def test_a_look_alike_room_is_not_believed_on_one_search(tmp_path):
    e = lost_engine(tmp_path)
    e._global_search = lambda pts: (np.array([6.0, -3.0, 1.5]), 0.55)   # another room scores well
    e._recover()
    assert np.allclose(e.pose, [1.0, 1.0, 0.0]) and e.jumps == 0, "one good score elsewhere must not move the robot"
    assert e.state({})["relocating"] is True
    e._recover()                                                         # it wins again, same place: now believe it
    assert np.allclose(e.pose, [6.0, -3.0, 1.5]) and e.jumps == 1
    assert e.state({})["relocating"] is False


def test_two_rooms_taking_turns_never_move_the_robot(tmp_path):
    e = lost_engine(tmp_path)
    places = [np.array([6.0, -3.0, 1.5]), np.array([-4.0, 5.0, -2.0])]
    for i in range(8):                                                   # the search flips between two look-alikes
        e._global_search = lambda pts, i=i: (places[i % 2], 0.5)
        e._recover()
    assert np.allclose(e.pose, [1.0, 1.0, 0.0]) and e.jumps == 0, "this is the corner-to-corner jumping"


def test_it_looks_around_before_it_looks_everywhere(tmp_path):
    e = lost_engine(tmp_path)
    e.grid.local = lambda guess: (np.array([1.3, 1.1, 0.1]), 0.5)       # 0.3 m off: found by the nearby search
    e._global_search = lambda pts: pytest.fail("the whole-map search must not run when the robot is found nearby")
    e._recover()
    assert np.allclose(e.pose, [1.3, 1.1, 0.1])
    assert e.jumps == 0, "a small correction is not a jump: the view glides to it"
    assert e.grid.calls == [0.60]


def test_a_weak_best_guess_is_ignored(tmp_path):
    e = lost_engine(tmp_path)
    e.grid.local = lambda guess: (np.array(guess, float), 0.22)          # nearby: poor
    e._global_search = lambda pts: (np.array([6.0, -3.0, 1.5]), 0.26)   # elsewhere: barely better, under the bar
    for _ in range(4):
        e._recover()
    assert np.allclose(e.pose, [1.0, 1.0, 0.0]) and e.jumps == 0 and e._candidate is None


def test_the_second_vote_allows_for_the_robot_having_moved(tmp_path):
    e = lost_engine(tmp_path)
    t = time.monotonic()
    e._scan_t = t
    e.odom_hist.append((t - 1.0, 0.0, 0.0, 0.0))
    e._global_search = lambda pts: (np.array([6.0, -3.0, 0.0]), 0.6)
    e._recover()
    e.odom_hist.append((t, 0.8, 0.0, 0.0))                              # drove 0.8 m forward meanwhile
    e._scan_t = t
    e._global_search = lambda pts: (np.array([6.8, -3.0, 0.0]), 0.6)    # the same place, 0.8 m on
    e._recover()
    assert np.allclose(e.pose, [6.8, -3.0, 0.0]) and e.jumps == 1


def test_find_me_and_loading_move_at_once(tmp_path):
    e = lost_engine(tmp_path)
    e._global_search = lambda pts: (np.array([6.0, -3.0, 1.5]), 0.4)
    assert e.relocalize() == pytest.approx(0.4)                          # the operator asked: no second vote
    assert np.allclose(e.pose, [6.0, -3.0, 1.5]) and e.jumps == 1


def test_a_confident_match_the_filter_refuses_is_not_being_lost(tmp_path):
    e = lost_engine(tmp_path, here=(0.0, 0.0, 0.0))
    e.filter.reset(e.pose, sigma_xy=0.002, sigma_th=math.radians(0.1))  # far too sure of itself
    e.grid.local = lambda guess: (np.array([0.15, 0.0, 0.0]), 0.5)      # the scan clearly says 15 cm further on
    e._recover = lambda: pytest.fail("a confident nearby match must never trigger a re-find")
    for _ in range(30):
        e._process(scan())
    assert e.match["lost"] == 0 and e.match["ok"]
    assert e.pose[0] == pytest.approx(0.15, abs=0.02) and e.jumps == 0
    assert "Localized" in e.message


def test_matching_nothing_for_a_second_starts_a_re_find(tmp_path):
    e = lost_engine(tmp_path)
    looked = []
    e._recover = lambda: looked.append(1)
    for _ in range(25):
        e._process(scan())                                               # FakeGrid scores 0: nothing matches
    assert e.match["lost"] >= 20 and len(looked) == 1, "one search, then wait: not one per scan"


def test_live_pose_is_the_last_match_carried_forward_by_odometry(tmp_path):
    e = lost_engine(tmp_path)
    e.pose = np.array([1.0, 2.0, math.pi / 2])
    e._anchor = (e.pose.copy(), np.array([5.0, 5.0, 0.0]))              # matched when odometry read (5, 5, 0)
    e.odom.x, e.odom.y, e.odom.th = 5.5, 5.0, 0.2                        # since then: 0.5 m forward, 0.2 rad left
    pose, jumps = e.live_pose()
    assert pose == pytest.approx([1.0, 2.5, math.pi / 2 + 0.2]) and jumps == 0
    e.mode = "idle"                                                      # no map: plain odometry
    assert e.live_pose()[0] == pytest.approx([5.5, 5.0, 0.2])


def test_unload_goes_back_to_idle_and_keeps_the_saved_files(tmp_path):
    e = SlamEngine(str(tmp_path))
    e.start_mapping()
    e.grid.integrate(np.zeros(3), np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]]))
    e.save("flat")
    e.load("flat")
    first = e.grid.version
    e.add_label("desk", 0.2, 0.2)
    e.load("flat")
    assert e.grid.version != first, "loading again (or another map) must count as a new map for the viewers"
    jumps = e.jumps
    e.unload()
    s = e.state({})
    assert s["mode"] == "idle" and s["map_name"] == "" and s["map_version"] == 0 and s["labels"] == []
    assert e.jumps == jumps + 1 and e.map_png() == (None, None)
    assert (tmp_path / "flat.npz").is_file() and {m["name"] for m in e.list_maps()} == {"flat"}
    e.load("flat")
    assert e.state({})["labels"][0]["name"] == "desk"
