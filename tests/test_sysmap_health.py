"""System map health rules: no hardware, no FastAPI, standard library only."""
import importlib.util
import time
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1] / "software/radxa/console/console/sysmap_health.py"
spec = importlib.util.spec_from_file_location("sysmap_health", SOURCE)
health = importlib.util.module_from_spec(spec)
spec.loader.exec_module(health)


def healthy():
    return {
        "serial": {"connected": True, "stale": False, "rx_hz": 50, "last_rx_age": 0.02,
                   "port": "/dev/ttyUSB0", "baud": 460800, "dump_complete": True},
        "tel": {"pitch": 0.3, "yaw": 182.0, "balancing": True, "startup_state": "READY", "host_ready": True},
        "debug": {"loopMax": 3100, "w": 1.2},
        "hub": {"current_cmd_l": .6, "current_cmd_r": .6, "current_l": .6, "current_r": .6,
                "rpm_l": 40, "rpm_r": 40, "temp_l": 44, "temp_r": 43, "fault_l": 0, "fault_r": 0,
                "saturated": 0, "link_ok": 1, "mode_ok": 1},
        "hub_age": 0.05,
        "nav": {},
        "tof": {"state": "ranging", "stale": False, "age_ms": 60, "valid_count": 58,
                "bus": 7, "address": "0x29", "ranging_hz": 10, "nearest_mm": 640},
        "stream": {"running": True, "fps": "30", "frame": "100", "capture_to_publish_ms": "60"},
        "host": {"ready": True, "camera": True, "hotspot": True},
        "system": {"cpu_temp_c": 52, "load1": .4, "cores": 8},
        "clients": {"console": 1, "sysmap": 1},
        "events": {"pad": True, "pad_name": "EVOFOX", "hub_probe": {"LEFT": "OK", "RIGHT": "OK"},
                   "hub_stats": {}, "yaw_unchanged_s": 0},
    }


def status(out, name):
    return out["nodes"][name]["status"]


def test_healthy_robot_has_no_problems():
    out = health.evaluate(healthy())
    assert out["summary"] == {"errors": [], "warnings": [], "state": "ok"}
    assert status(out, "lights") == "nofb" and status(out, "payload") == "nofb"


def test_dead_serial_link_is_the_only_red_block():
    inp = healthy()
    inp["serial"].update(stale=True, rx_hz=0, last_rx_age=4.2)
    out = health.evaluate(inp)
    assert out["summary"]["errors"] == ["serial"]
    for child in ("esp32", "imu", "compass", "gamepad", "wheel_l", "wheel_r", "payload", "lights"):
        assert status(out, child) == "blocked", child
    # Radxa-side blocks are not behind the serial link.
    assert status(out, "tof") == "ok" and status(out, "camera") == "ok"


def test_closed_port_reports_the_os_error():
    inp = healthy()
    inp["serial"].update(connected=False, error="could not open port /dev/ttyUSB0")
    node = health.evaluate(inp)["nodes"]["serial"]
    assert node["status"] == "error" and "/dev/ttyUSB0" in node["reason"]


def test_firmware_maintenance_blocks_children_without_an_error():
    inp = healthy()
    inp["serial"]["maintenance"] = True
    out = health.evaluate(inp)
    assert status(out, "serial") == "warn" and status(out, "esp32") == "blocked"
    assert out["summary"]["errors"] == []


def test_blocked_node_keeps_its_own_status_for_later():
    inp = healthy()
    inp["serial"]["stale"] = True
    inp["hub"]["fault_l"] = 0x10
    node = health.evaluate(inp)["nodes"]["wheel_l"]
    assert node["status"] == "blocked" and node["own_status"] == "error"


def test_m0601_fault_bits_are_named():
    inp = healthy()
    inp["hub"]["fault_r"] = 0x02 | 0x08
    node = health.evaluate(inp)["nodes"]["wheel_r"]
    assert node["status"] == "error"
    assert "bus overcurrent" in node["reason"] and "stall" in node["reason"]
    assert health.evaluate(inp)["nodes"]["wheel_l"]["status"] == "ok"


def test_wheel_temperature_thresholds_and_unknown_temperature():
    for temp, expected in ((69, "ok"), (72, "warn"), (80, "error"), (-1, "ok")):
        inp = healthy()
        inp["hub"]["temp_l"] = temp
        assert status(health.evaluate(inp), "wheel_l") == expected, temp


def test_stale_motor_feedback_is_an_error():
    inp = healthy()
    inp["hub_age"] = 3.0
    assert status(health.evaluate(inp), "wheel_l") == "error"
    inp["hub_age"] = None
    assert status(health.evaluate(inp), "wheel_l") == "nodata"


def test_shared_link_flag_is_blamed_using_the_boot_probe():
    inp = healthy()
    inp["hub"]["link_ok"] = 0
    inp["events"]["hub_probe"] = {"LEFT": "NO REPLY", "RIGHT": "OK"}
    out = health.evaluate(inp)
    assert status(out, "wheel_l") == "error" and status(out, "wheel_r") == "warn"


def test_tof_states():
    for change, expected in (({"state": "starting"}, "warn"), ({"state": "error", "error": "No VL53L5CX"}, "error"),
                             ({"stale": True, "age_ms": 2400}, "error"), ({"valid_count": 0}, "warn")):
        inp = healthy()
        inp["tof"].update(change)
        assert status(health.evaluate(inp), "tof") == expected, change
    inp = healthy()
    inp["tof"].update(stale=True, age_ms=2400)
    assert "2.4 s" in health.evaluate(inp)["nodes"]["tof"]["reason"]


def test_camera_and_hotspot():
    inp = healthy()
    inp["stream"]["running"] = False
    assert status(health.evaluate(inp), "camera") == "error"
    inp = healthy()
    inp["host"]["camera"] = False
    assert status(health.evaluate(inp), "camera") == "error"
    inp = healthy()
    inp["host"]["hotspot"] = False
    assert status(health.evaluate(inp), "clients") == "error"


def test_control_loop_overrun():
    inp = healthy()
    inp["debug"]["loopMax"] = 25000
    assert status(health.evaluate(inp), "esp32") == "error"
    inp["debug"]["loopMax"] = 8000
    assert status(health.evaluate(inp), "esp32") == "warn"


def test_compass_freeze_is_only_a_warning_while_moving():
    inp = healthy()
    inp["events"]["yaw_unchanged_s"] = 45
    assert status(health.evaluate(inp), "compass") == "warn"
    inp["tel"]["balancing"] = False
    inp["hub"].update(rpm_l=0, rpm_r=0)
    assert status(health.evaluate(inp), "compass") == "ok"


def test_non_finite_values_do_not_crash():
    inp = healthy()
    inp["tel"]["pitch"] = float("nan")
    inp["hub"]["temp_l"] = float("inf")
    out = health.evaluate(inp)
    assert status(out, "imu") == "nodata"


def log(*lines):
    return [{"t": time.time(), "text": text} for text in lines]


def test_tracker_follows_firmware_log_lines():
    tracker = health.EventTracker()
    entries = log("[BOOT] Waiting for Radxa ACK; IMU calibration and device startup are on HOLD",
                  "[IMU] ERROR: begin() failed — check wiring / address",
                  "[HUB] RIGHT id2 OK; LEFT id1 NO REPLY",
                  "[HUB] LEFT id1 mode1 rpm0 cmd0.00A measured0.00A fault0 age12ms CRC3 timeout41",
                  "[PAD] Connected: EVOFOX One S", "[PAY] Pan  -> 150 deg", "[PAY] Torch ON (35%)")
    state = tracker.update(entries, 180.0, True)
    assert state["imu_error"].startswith("ERROR: begin()")
    assert state["hub_probe"] == {"RIGHT": "OK", "LEFT": "NO REPLY"}
    assert state["hub_stats"]["LEFT"] == {"crc": 3, "timeout": 41}
    assert state["pad"] is True and state["pad_name"] == "EVOFOX One S"
    assert state["pan"] == 150 and state["torch"] == 35


def test_tracker_is_incremental_and_resets_on_boot():
    tracker = health.EventTracker()
    entries = log("[PAD] Connected: EVOFOX", "[IMU] ERROR: Enable_X failed")
    tracker.update(entries, 0, True)
    entries += log("[PAD] Disconnected — inputs zeroed, balance continues", "[PAY] Torch off")
    state = tracker.update(entries, 0, True)
    assert state["pad"] is False and state["torch"] == 0
    entries += log("[BOOT] Waiting for Radxa ACK; IMU calibration and device startup are on HOLD",
                   "[IMU] Initialisation OK")
    state = tracker.update(entries, 0, True)
    assert state["pad"] is None and state["imu_error"] == ""


def test_tracker_measures_how_long_heading_is_frozen():
    tracker = health.EventTracker()
    tracker.update([], 90.0, True)
    tracker._yaw_since -= 40
    assert tracker.update([], 90.0, True)["yaw_unchanged_s"] >= 40
    assert tracker.update([], 91.0, True)["yaw_unchanged_s"] < 1
