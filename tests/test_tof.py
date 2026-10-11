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


@pytest.fixture(autouse=True)
def isolated_calibration(monkeypatch, tmp_path):
    monkeypatch.setenv("BOT_TOF_CALIBRATION", str(tmp_path / "tof-calibration.json"))


def test_restart_is_opt_out_and_unknown_is_not_clear():
    tof = module.ToF()
    info = tof.status(link())
    assert not info["guard_enabled"]
    assert info["stale"] and info["nearest_m"] is None
    assert info["forward_limit_mps"] is None


@pytest.mark.parametrize("distance,cap", [(1200, 0.7), (200, 0), (100, 0)])
def test_clear_slow_and_blocked(distance, cap):
    tof = module.ToF()
    tof.accept(frame(distance))
    tof.guard_enabled = True
    assert tof.status(link())["forward_limit_mps"] == cap


@pytest.mark.parametrize("field,value", [("status", 4), ("sigma_mm", 21),
                                        ("targets", 0), ("distance_mm", 12)])
def test_single_unknown_central_zone_does_not_block_observed_corridor(field, value):
    tof = module.ToF()
    data = frame()
    data[field][3 * 8 + 3] = value
    tof.accept(data)
    tof.guard_enabled = True
    assert tof.status(link())["forward_limit_mps"] == .7


def test_sparse_far_returns_do_not_force_a_fixed_creep_limit():
    tof = module.ToF()
    data = frame()
    data["status"] = [255] * 64
    data["status"][27] = data["status"][28] = 5
    tof.accept(data)
    tof.guard_enabled = True
    info = tof.status(link())
    assert info["forward_limit_mps"] == .7
    assert "partial coverage" in info["guard_reason"]
    assert len(info["points"]) == 2


def test_braking_cap_respects_reaction_and_deceleration():
    tof = module.ToF()
    tof.accept(frame(575)); tof.guard_enabled = True
    info = tof.status(link())
    cap = info["forward_limit_mps"]
    assert .2 < cap < .7
    assert cap * tof.reaction_s + cap * cap / (2 * tof.deceleration_mps2) <= .575 - tof.stop_m


def test_side_return_outside_corridor_does_not_stop_forward_request():
    tof = module.ToF()
    data = frame(2000)
    data["distance_mm"][24] = 800
    tof.accept(data); tof.guard_enabled = True
    assert tof.status(link())["forward_limit_mps"] == .7


def test_isolated_noncritical_outlier_needs_repeated_frame():
    tof = module.ToF()
    data = frame(1800)
    data["distance_mm"][27] = 500
    tof.accept(data); tof.guard_enabled = True
    assert tof.status(link())["forward_limit_mps"] == .7
    tof.accept(data)
    assert tof.status(link())["forward_limit_mps"] < .7


def test_close_single_zone_blocks_immediately_and_status9_is_valid():
    tof = module.ToF()
    data = frame()
    data["status"][27] = 9
    data["distance_mm"][27] = 200
    tof.accept(data); tof.guard_enabled = True
    assert tof.status(link())["forward_limit_mps"] == 0


def test_speed_increases_projected_stopping_distance():
    tof = module.ToF()
    tof.accept(frame())
    robot = link()
    still = tof.status(robot)["stopping_distance_m"]
    robot.tel["v_ms"] = .7
    assert tof.status(robot)["stopping_distance_m"] > still + .4


def test_close_lower_zone_is_not_discarded_as_assumed_floor():
    tof = module.ToF()
    data = frame()
    data["distance_mm"][59] = 150
    tof.accept(data); tof.guard_enabled = True
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


def test_fresh_unknown_ranges_do_not_force_creep_or_claim_clear():
    tof = module.ToF()
    data = frame(17)  # current near-window/self-return pattern, outside useful range
    tof.accept(data); tof.guard_enabled = True
    info = tof.status(link())
    assert info["valid_count"] == 0 and info["nearest_m"] is None
    assert info["forward_limit_mps"] == .7
    assert "unknown" in info["guard_reason"]
    assert "clear" not in info["guard_reason"]
    assert "unknown_speed_mps" not in info and "partial_speed_mps" not in info


def test_partial_near_obstacle_still_sets_a_physical_braking_limit():
    tof = module.ToF()
    data = frame(575)
    data["status"] = [255] * 64
    data["status"][27] = data["status"][28] = 5
    tof.accept(data); tof.guard_enabled = True
    info = tof.status(link())
    cap = info["forward_limit_mps"]
    assert .45 < cap < .7
    assert "slowing" in info["guard_reason"]
    assert cap * tof.reaction_s + cap * cap / (2 * tof.deceleration_mps2) <= .575 - tof.stop_m


def test_bias_from_real_reference_persists_and_keeps_raw_distances():
    tof = module.ToF()
    for _ in range(3):
        tof.accept(frame(540))
    tof.calibrate(.50)
    assert tof.range_offset_m == -.04
    restored = module.ToF()
    restored.accept(frame(240)); restored.guard_enabled = True
    info = restored.status(link())
    assert info["range_calibrated"]
    assert info["distance_mm"][27] == 240
    assert info["corrected_distance_mm"][27] == 200
    assert info["forward_limit_mps"] == 0


@pytest.mark.parametrize("reference", [float("nan"), .1, 4.0])
def test_invalid_reference_cannot_modify_calibration(reference):
    tof = module.ToF()
    with pytest.raises(ValueError):
        tof.calibrate(reference)
    assert not tof.range_calibrated


def test_calibration_rejects_unstable_unknown_or_implausibly_large_bias():
    tof = module.ToF()
    tof.accept(frame(500))
    with pytest.raises(ValueError, match="stable"):
        tof.calibrate(.50)
    for _ in range(3):
        tof.accept(frame(17))
    with pytest.raises(ValueError, match="stable"):
        tof.calibrate(.50)
    for _ in range(3):
        tof.accept(frame(500))
    with pytest.raises(ValueError, match="Bias"):
        tof.calibrate(.80)


def test_stale_calibration_target_is_rejected():
    tof = module.ToF()
    for _ in range(3):
        tof.accept(frame(500))
    tof.received = time.monotonic() - .6
    with pytest.raises(ValueError, match="fresh"):
        tof.calibrate(.50)


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
