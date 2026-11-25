"""Gamepad lines from the controller: autopilot keys and the signs of a stuck Bluetooth stack."""
import importlib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software/radxa/console"))
serial_link = importlib.import_module("console.serial_link")
bubble = importlib.import_module("console.slam.bubble")


def link():
    return serial_link.SerialLink()


def test_autopilot_keys_reach_their_listeners():
    l, keys = link(), []
    l.add_key_listener(keys.append)
    for line in ("PK,GO", "PK,PAUSE", "PK,END", "PK,ROAM"):
        l._handle(line)
    assert keys == ["GO", "PAUSE", "END", "ROAM"]


def test_a_refused_connection_or_a_dead_link_marks_bluetooth_as_stuck():
    l = link()
    assert not l.pad_stuck
    l._handle("Failed to set device information client: 0xc")
    l._handle("Failed to set device information client: 0xc")
    assert l.pad_stuck["count"] == 2 and "device information" in l.pad_stuck["why"]
    l._handle("[PAD] link check: 0 input reports in the first 3 s")
    assert l.pad_stuck, "a silent link is not proof of life"
    l._handle("[PAD] link check: 180 input reports in the first 3 s")
    assert not l.pad_stuck
    l._handle("[PAD] DEAD LINK: connected but no input in 3 s; dropping the connection")
    assert l.pad_stuck
    l._handle("[PAD] btn=0x0001 misc=0x0000 dpad=0x00")
    assert not l.pad_stuck, "a button press is proof of life"
    l._handle("[PAD] DEAD LINK: connected but no input in 3 s; dropping the connection")
    l._handle("[BOOT] Reset reason: SOFTWARE")
    assert not l.pad_stuck, "a restart clears it"


def test_text_listeners_see_plain_log_lines_only():
    l, seen = link(), []
    l.add_text_listener(seen.append)
    l._handle("[PAY] Torch ON (25%)")
    l._handle("PK,GO")
    l._handle("NR,1,0,0,0,0,1,0,5000")
    assert seen == ["[PAY] Torch ON (25%)"]
    assert l.nav["idle_ms"] == 5000


def test_the_safety_bubble_is_the_measured_body_with_nothing_added():
    assert bubble.footprint() == (-0.07, 0.06, bubble.LIDAR_Y - 0.10, bubble.LIDAR_Y + 0.17)
    b = bubble.Bubble()
    # what counts as the robot itself has not moved: the body plus 2.5 cm, as when the box was 1 cm larger
    assert abs((0.06 + b.cfg["self_tol"]) - (0.07 + 0.015)) < 1e-9


def test_the_rt_pull_arrives_with_the_navigation_feedback():
    l = link()
    l._handle("NR,1,0,0,0,0,1,0,5000,73")
    assert l.nav["boost"] == 73 and l.nav["idle_ms"] == 5000
    l._handle("NR,1,0,0,0,0,1,0,5000")            # 5.3.0 / 5.4.0 firmware: no boost field
    assert l.nav["boost"] == 0
    l._handle("NR,1,0,0,0,0,1,0")                 # older still
    assert l.nav["boost"] == 0 and l.nav["idle_ms"] is None
