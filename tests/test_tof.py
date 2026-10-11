"""Depth validity, stale-data handling and optional guard transport (no hardware)."""
import asyncio
import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

SOURCE = Path(__file__).resolve().parents[1] / "software/radxa/console/console/tof.py"
spec = importlib.util.spec_from_file_location("bot_tof", SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def frame(distance=1200):
    return {"state": "ranging", "distance_mm": [distance] * 64, "status": [5] * 64,
            "sigma_mm": [2] * 64, "targets": [1] * 64}


def link(supported=True):
    return SimpleNamespace(connected=True, tel={"stale": False},
                           derived={"TOFGUARD": int(supported)},
                           params={"DRIVEV": SimpleNamespace(value=0.7)})


def test_restart_is_opt_out_and_unknown_is_not_clear():
    tof = module.ToF()
    info = tof.status(link())
    assert not info["guard_enabled"]
    assert info["stale"] and info["nearest_m"] is None
    assert info["forward_limit_mps"] is None


@pytest.mark.parametrize("distance,cap", [(1200, 0.7), (575, 0.35), (350, 0), (100, 0)])
def test_clear_slow_and_blocked(distance, cap):
    tof = module.ToF()
    tof.accept(frame(distance))
    tof.guard_enabled = True
    assert tof.status(link())["forward_limit_mps"] == cap


@pytest.mark.parametrize("field,value", [("status", 9), ("sigma_mm", 21),
                                        ("targets", 0), ("distance_mm", 12)])
def test_single_unknown_central_zone_blocks_forward(field, value):
    tof = module.ToF()
    data = frame()
    data[field][3 * 8 + 3] = value
    tof.accept(data)
    tof.guard_enabled = True
    assert tof.status(link())["forward_limit_mps"] == 0


def test_stale_and_worker_error_invalidate_last_clear_frame():
    tof = module.ToF()
    tof.accept(frame())
    tof.guard_enabled = True
    tof.received = time.monotonic() - 0.6
    assert tof.status(link())["forward_limit_mps"] == 0
    tof.accept(frame())
    tof.accept({"state": "error", "message": "I2C disconnected"})
    assert tof.status(link())["stale"]
    assert tof.status(link())["forward_limit_mps"] == 0


def test_malformed_frame_does_not_replace_last_good_frame():
    tof = module.ToF()
    tof.accept(frame())
    data = frame()
    data["distance_mm"] = [float("nan")] * 64
    with pytest.raises(ValueError):
        tof.accept(data)
    assert tof.frame["distance_mm"][0] == 1200


def test_guard_does_not_report_active_without_firmware_support():
    tof = module.ToF()
    tof.guard_enabled = True
    assert not tof.status(link(False))["guard_active"]


def test_disabled_transport_releases_once_and_never_steals_v_authority():
    async def scenario():
        tof = module.ToF()
        sent = []
        robot = link()
        robot.send = sent.append
        robot._publish = lambda message: None
        task = asyncio.create_task(tof.publish(robot))
        await asyncio.sleep(0.22)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert sent == ["TG,0,0"]
    asyncio.run(scenario())
