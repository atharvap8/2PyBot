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


def _recovery_actions(cases, case_id):
    for c in cases:
        if c.id == case_id:
            return {(r.action, r.safety) for r in c.recoveries}
    return set()


def test_hotspot_stuck_but_ap_up_is_warn_and_auto_restart():
    cases = diagnose({"services": {"2pybot-hotspot": "activating"},
                      "services_enabled": {"2pybot-hotspot": "enabled"},
                      "network": {"hotspot_ap_up": True}})
    match = [c for c in cases if c.id == "hotspot_down"]
    assert match and match[0].severity == "warn"
    assert ("restart_hotspot", "auto") in _recovery_actions(cases, "hotspot_down")


def test_hotspot_down_is_critical():
    cases = diagnose({"services": {"2pybot-hotspot": "failed"},
                      "network": {"hotspot_ap_up": False}})
    match = [c for c in cases if c.id == "hotspot_down"]
    assert match and match[0].severity == "critical"


def test_hotspot_active_or_disabled_not_flagged():
    assert "hotspot_down" not in ids({"services": {"2pybot-hotspot": "active"}})
    assert "hotspot_down" not in ids({"services": {"2pybot-hotspot": "failed"},
                                      "services_enabled": {"2pybot-hotspot": "disabled"}})


def test_service_rule_emits_a_case_per_unhealthy_service():
    cases = diagnose({"services": {"2pybot-firmware": "failed", "2pybot-mediamtx": "inactive"}})
    got = {c.id for c in cases}
    assert "svc_2pybot-firmware" in got and "svc_2pybot-mediamtx" in got
    assert ("restart_firmware", "auto") in _recovery_actions(cases, "svc_2pybot-firmware")


def test_imx708_oneshot_inactive_is_not_flagged_but_failed_is():
    assert "svc_2pybot-imx708" not in ids({"services": {"2pybot-imx708": "inactive"}})
    assert "svc_2pybot-imx708" in ids({"services": {"2pybot-imx708": "failed"}})


def test_numpy2_auto_downgrade():
    cases = diagnose({"deps": {"numpy_major": 2}})
    assert "numpy2" in {c.id for c in cases}
    assert ("pin_numpy", "auto") in _recovery_actions(cases, "numpy2")
    assert "numpy2" not in ids({"deps": {"numpy_major": 1}})


def test_deploy_failed_and_disabled():
    assert "deploy_failed" in ids({"deploy": {"phase": "failed"}})
    assert "deploy_disabled" in ids({"deploy_units": {"timer": "inactive", "path": "inactive"}})
    assert "deploy_disabled" not in ids({"deploy_units": {"timer": "active", "path": "active"}})


def test_mem_pressure():
    assert "mem_pressure" in ids({"system": {"mem_pct": 96}})
    assert "mem_pressure" not in ids({"system": {"mem_pct": 70}})


def test_to_dict_serialisable():
    import json
    for c in diagnose({"tel": {"asleep": True}}):
        json.dumps(c.to_dict())


# ---- 5.3.0: the doctor must not freeze the console or poke the system while the robot balances ----
def test_service_probes_slow_down_while_balancing(monkeypatch):
    import console.doctor as doctor

    calls = []
    monkeypatch.setattr(doctor, "_batch", lambda verb, names: calls.append(verb) or {n: "active" for n in names})
    monkeypatch.setattr(doctor, "_hotspot_ap_up", lambda: True)

    class Link:
        connected = True
        tel = {"balancing": False, "stale": False}
        events = {}

    d = doctor.Doctor(Link())
    d.collect(); d.collect()
    parked = len(calls)
    assert parked == 4, "parked: every pass looks at the services (one is-active and one is-enabled call each)"
    Link.tel = {"balancing": True, "stale": False}
    for _ in range(5):
        signals = d.collect()
    assert len(calls) == parked, "balancing: the last answer is reused, nothing is started"
    assert signals["services"]["2pybot-console"] == "active" and signals["network"]["hotspot_ap_up"] is True


def test_probes_start_without_copying_the_console():
    import console.doctor as doctor
    import inspect

    src = inspect.getsource(doctor._probe)
    assert "close_fds=False" in src and "shutil.which" in src, "this is what lets Python use posix_spawn instead of fork"
    assert doctor._probe(["definitely-not-a-command-2pybot"]) == ""
    run = inspect.getsource(doctor.Doctor.run)
    assert "run_in_executor" in run, "the checks wait on other processes: never on the event loop"


# ---- 5.4.0: a stuck gamepad Bluetooth ----
def test_stuck_gamepad_bluetooth_is_restarted_only_when_the_robot_is_idle():
    idle = diagnose({"gamepad": {"stuck": True, "why": "Failed to set device information client: 0xc"}, "tel": {"balancing": False}})
    case = [c for c in idle if c.id == "gamepad_bt_stuck"][0]
    assert case.recoveries[0].action == "restart_controller_bt" and case.recoveries[0].safety == "auto"
    assert "idle" in case.cause
    busy = [c for c in diagnose({"gamepad": {"stuck": True}, "tel": {"balancing": True}}) if c.id == "gamepad_bt_stuck"][0]
    assert "STOP" in busy.cause
    assert "gamepad_bt_stuck" not in ids({"gamepad": {"stuck": False}})

    import console.doctor as doctor
    assert "restart_controller_bt" in doctor._SERVICE_RESTART_ACTIONS, "never while the robot is balancing"

    class Link:
        connected, maintenance, sent = True, False, []
        tel = {"balancing": True, "stale": False}
        derived = {"PADRESTART": 1}
        pad_stuck = {"why": "x"}

        def send(self, line):
            self.sent.append(line)
            return True

    link = Link()
    d = doctor.Doctor(link)
    assert d._restart_controller_bt()[0] is False and not Link.sent, "balancing: hands off"
    Link.tel = {"balancing": False, "stale": False}
    Link.derived = {}
    assert d._restart_controller_bt()[0] is False and not Link.sent, "older firmware has no such command"
    Link.derived = {"PADRESTART": 1}
    assert d._restart_controller_bt()[0] is True and Link.sent == ["BT,RESTART"] and link.pad_stuck == {}
