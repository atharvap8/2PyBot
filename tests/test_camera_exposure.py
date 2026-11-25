"""The camera re-exposes by itself when the picture stays too dark or too bright."""
import asyncio
import importlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software/radxa/console"))
camera = importlib.import_module("console.camera")


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def watch(tmp_path, **kw):
    clock = Clock()
    logs = []
    w = camera.ExposureWatch(log=logs.append, settings=str(tmp_path / "auto_exposure.json"), clock=clock, **kw)
    return w, clock, logs


def test_a_good_picture_is_left_alone(tmp_path):
    w, clock, _ = watch(tmp_path)
    assert not any(w.due(luma, clock()) for luma in (60, 110, 180, 110, 110, 110))


def test_it_waits_for_the_picture_to_stay_bad(tmp_path):
    w, clock, _ = watch(tmp_path)
    assert [w.due(30, clock()), w.due(30, clock()), w.due(30, clock())] == [False, False, True]
    w.bad = 0
    assert [w.due(30, clock()), w.due(120, clock()), w.due(30, clock()), w.due(30, clock())] == [False] * 4, \
        "one good reading in between starts the count again (a light flickering past is not a reason)"


def test_too_bright_triggers_as_well(tmp_path):
    w, clock, _ = watch(tmp_path)
    assert [w.due(230, clock()) for _ in range(3)] == [False, False, True]


def test_it_rests_after_a_walk_and_longer_at_the_sensor_limit(tmp_path):
    w, clock, logs = watch(tmp_path)
    w.finished({"ok": True, "converged": True, "luma": 108.0}, 30, clock())
    assert "too dark" in logs[-1] and "now 108" in logs[-1]
    clock.t += w.REST - 1
    assert not any(w.due(30, clock()) for _ in range(5)), "still resting"
    clock.t += 2
    assert [w.due(30, clock()) for _ in range(3)][-1]
    w.finished({"ok": True, "converged": False}, 20, clock())       # a dark room: nothing more the sensor can do
    clock.t += w.REST + 5
    assert not any(w.due(20, clock()) for _ in range(5)), "it must not keep walking the exposure every minute in the dark"
    clock.t += w.REST_AT_LIMIT
    assert [w.due(20, clock()) for _ in range(3)][-1]


def test_out_of_range_in_the_dark_then_the_light_comes_on(tmp_path):
    w, clock, _ = watch(tmp_path)
    w.finished({"ok": True, "converged": False}, 1, clock())        # lens covered: exposure and gain end up at maximum
    clock.t += w.REST + 1
    assert not any(w.due(3, clock()) for _ in range(5)), "still dark: keep resting"
    assert [w.due(250, clock()) for _ in range(3)] == [False, False, True], \
        "uncovered and now washed out: act after the usual three readings, not after five minutes"


def test_no_picture_means_no_action(tmp_path):
    w, clock, _ = watch(tmp_path)
    assert not any(w.due(None, clock()) for _ in range(6))           # asleep, or the stream is down
    assert camera.ExposureWatch.fresh_luma(str(tmp_path / "missing.jpg")) is None


def test_switching_it_off_sticks_across_restarts(tmp_path):
    w, clock, _ = watch(tmp_path)
    assert w.enabled and w.status()["enabled"]
    w.set_enabled(False)
    assert json.loads((tmp_path / "auto_exposure.json").read_text())["enabled"] is False
    assert not any(w.due(10, clock()) for _ in range(5))
    again, _, _ = watch(tmp_path)
    assert again.enabled is False


def test_the_watch_loop_runs_the_walk_once_and_reports_it(tmp_path):
    readings = [20, 20, 20, 110, 110, 110, 110, 110]
    walks = []

    def measure():
        return readings.pop(0) if readings else 110

    def calibrate():
        walks.append(1)
        return {"ok": True, "converged": True, "luma": 111.0}

    w, clock, logs = watch(tmp_path, measure=measure, calibrate=calibrate)
    w.PERIOD = 0.01

    async def go():
        task = asyncio.create_task(w.run())
        await asyncio.sleep(0.4)
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
    asyncio.run(go())
    assert len(walks) == 1 and "re-exposed" in w.status()["action"] and logs


def test_only_one_exposure_walk_runs_at_a_time():
    assert camera._CALIBRATING.acquire(blocking=False)
    try:
        assert camera.autocalibrate()["error"] == "a calibration is already running"
    finally:
        camera._CALIBRATING.release()
