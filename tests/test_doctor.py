"""Unit tests for the pure doctor rule engine (no I/O)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "software", "radxa", "console"))

from console.doctor import diagnose  # noqa: E402


def ids(signals):
    return {c.id for c in diagnose(signals)}


def test_empty_signals_no_crash_no_cases():
    assert diagnose({}) == []


def test_deploy_deferring_is_critical():
    cases = diagnose({"deploy": {"phase": "waiting_idle", "defer_count": 3}})
    match = [c for c in cases if c.id == "deploy_deferring"]
    assert match and match[0].severity == "critical"


def test_deploy_single_defer_not_flagged():
    assert "deploy_deferring" not in ids({"deploy": {"phase": "waiting_idle", "defer_count": 1}})


def test_deploy_success_not_flagged():
    assert "deploy_deferring" not in ids({"deploy": {"phase": "success", "defer_count": 0}})


def test_console_crash_on_failed():
    assert "console_crash" in ids({"services": {"2pybot-console": "failed"}})


def test_console_crash_on_restarts():
    assert "console_crash" in ids({"service_restarts": {"2pybot-console": 4}})


def test_serial_stale():
    assert "serial_stale" in ids({"serial": {"connected": True, "stale": True}})
    assert "serial_stale" not in ids({"serial": {"connected": True, "stale": False}})


def test_camera_down_only_when_explicitly_off():
    assert "camera_down" in ids({"camera": {"running": False}})
    assert "camera_down" not in ids({"camera": {}})                       # no data, no false alarm
    assert "camera_down" not in ids({"camera": {"running": False}, "tel": {"asleep": True}})
    assert "camera_down" not in ids({"camera": {"running": False, "paused": True}})


def test_onnxruntime_only_when_feature_enabled():
    assert "onnxruntime_missing" in ids({"visual_pair": {"enabled": True}, "deps": {"onnxruntime": False}})
    assert "onnxruntime_missing" not in ids({"visual_pair": {"enabled": False}, "deps": {"onnxruntime": False}})
    assert "onnxruntime_missing" not in ids({"visual_pair": {"enabled": True}, "deps": {"onnxruntime": True}})


def test_motor_fault_and_rs485():
    assert "motor_fault_l" in ids({"hub": {"fault_l": 0x08}})
    assert "rs485_r" in ids({"hub": {}, "hub_stats": {"RIGHT": {"timeout": 200, "crc": 1}}})


def test_powered_down_is_info():
    cases = diagnose({"tel": {"asleep": True}})
    match = [c for c in cases if c.id == "powered_down"]
    assert match and match[0].severity == "info"


def test_disk_and_cpu():
    assert "disk_full" in ids({"system": {"disk_pct": 95}})
    assert "disk_full" not in ids({"system": {"disk_pct": 60}})
    assert "cpu_hot" in ids({"system": {"cpu_temp_c": 90}})
    assert "cpu_hot" not in ids({"system": {"cpu_temp_c": 55}})


def test_critical_ranks_first():
    cases = diagnose({"serial": {"connected": True, "stale": True}, "tel": {"asleep": True, "stale": True}})
    assert cases[0].severity == "critical"


def test_recoveries_have_valid_safety():
    cases = diagnose({"deploy": {"phase": "waiting_idle", "defer_count": 5},
                      "services": {"2pybot-console": "failed"},
                      "visual_pair": {"enabled": True}, "deps": {"onnxruntime": False}})
    assert cases
    for c in cases:
        assert c.severity in ("critical", "warn", "info")
        for r in c.recoveries:
            assert r.safety in ("auto", "confirm", "manual")


def test_to_dict_serialisable():
    import json
    for c in diagnose({"tel": {"asleep": True}}):
        json.dumps(c.to_dict())
