"""The flashlight looks after itself: on in the dark, its own brightness, off when it is not needed."""
import importlib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software/radxa/console"))
torch = importlib.import_module("console.torch")
camera = importlib.import_module("console.camera")


class Clock:
    def __init__(self):
        self.t = 500.0

    def __call__(self):
        return self.t


class Link:
    def __init__(self):
        self.connected, self.sent, self.params = True, [], {}
        self.tel = {"stale": False, "asleep": False, "balancing": True, "heading": 0.0}
        self.derived = {"TORCHSET": 1}

    def send(self, line):
        self.sent.append(line)
        return True


def make(tmp_path, front=None):
    link, clock, logs = Link(), Clock(), []
    t = torch.AutoTorch(link, measure=lambda: None, front_m=lambda: front, log=logs.append,
                        settings=str(tmp_path / "auto_torch.json"), clock=clock)
    return t, link, clock, logs


def feed(t, clock, values, dt=2.0):
    for luma in values:
        clock.t += dt
        t.step(luma, clock())


def test_dark_for_a_few_seconds_switches_it_on_low(tmp_path):
    t, link, clock, logs = make(tmp_path)
    feed(t, clock, [20, 20])
    assert not link.sent, "two dark readings are not enough (a shadow passing is not night)"
    feed(t, clock, [20])
    assert link.sent == ["TL,1,25"] and t.on and "dark" in logs[-1]


def test_it_finds_its_own_brightness(tmp_path):
    t, link, clock, _ = make(tmp_path)
    feed(t, clock, [20, 20, 20])                       # on at 25
    feed(t, clock, [50, 60, 70])                       # still dim: 35, 45, 55
    assert t.pct == 55
    feed(t, clock, [110, 120])                         # in the band: leave it
    assert t.pct == 55 and link.sent[-1] == "TL,1,55"
    feed(t, clock, [170])                              # too bright: a step down
    assert t.pct == 45
    feed(t, clock, [230])                              # washed out: a big step down
    assert t.pct == 15 and t.on


def test_it_never_asks_for_more_than_the_cap_or_less_than_the_floor(tmp_path):
    t, link, clock, _ = make(tmp_path)
    feed(t, clock, [20, 20, 20] + [30] * 12)
    assert t.pct == t.CEILING == 90 and all(int(s.split(",")[2]) <= 90 for s in link.sent)
    feed(t, clock, [180] * 9)
    assert t.pct == t.FLOOR and t.on


def test_enough_light_without_it_switches_it_off_and_it_stays_off_a_while(tmp_path):
    t, link, clock, logs = make(tmp_path)
    feed(t, clock, [20, 20, 20])
    feed(t, clock, [180, 180])                         # down to the floor (25 -> 15 -> 10)
    assert t.on and t.pct == 10
    feed(t, clock, [180, 180, 180])                    # at the floor and still bright: the room lights are on
    assert not t.on and link.sent[-1].startswith("TL,0,") and "enough light" in logs[-1]
    feed(t, clock, [20, 20, 20, 20])                   # 8 s later it is dark again: not yet (20 s minimum off)
    assert not t.on
    feed(t, clock, [20] * 8)
    assert t.on


def test_it_waits_for_a_picture_taken_with_the_new_light(tmp_path):
    t, link, clock, _ = make(tmp_path)
    feed(t, clock, [20, 20, 20])
    n = len(link.sent)
    feed(t, clock, [30, 30], dt=0.5)                   # the snapshot is still from before the change
    assert len(link.sent) == n


def test_parked_robot_gets_its_light_switched_off_and_is_otherwise_left_alone(tmp_path):
    t, link, clock, _ = make(tmp_path)
    feed(t, clock, [20, 20, 20])
    link.tel["balancing"] = False
    feed(t, clock, [20])
    assert not t.on and link.sent[-1].startswith("TL,0,")
    n = len(link.sent)
    feed(t, clock, [5] * 30)                           # a dark room at night with the robot parked
    assert len(link.sent) == n, "it must not light the room while parked"


def test_using_the_flashlight_from_the_gamepad_makes_the_bot_keep_off(tmp_path):
    t, link, clock, _ = make(tmp_path)
    feed(t, clock, [20, 20, 20])
    t.on_line("[PAY] Torch ON (25%)")                  # the echo of the bot's own command: not the operator
    assert t.on and t.status()["holding_s"] == 0
    clock.t += 5
    t.on_line("[PAY] Torch off")                       # RB on the gamepad
    assert not t.on and t.status()["holding_s"] > 250
    n = len(link.sent)
    feed(t, clock, [10] * 20)
    assert len(link.sent) == n, "the operator switched it off: it stays off"
    clock.t += t.MANUAL_HOLD_S
    feed(t, clock, [10] * 3)
    assert t.on


def test_old_firmware_or_a_sleeping_robot_is_not_commanded(tmp_path):
    t, link, clock, _ = make(tmp_path)
    link.derived = {}
    feed(t, clock, [5] * 6)
    assert not link.sent and not t.status()["available"]
    link.derived = {"TORCHSET": 1}
    link.tel["asleep"] = True
    feed(t, clock, [5] * 6)
    assert not link.sent


def test_the_switch_sticks_and_switching_off_turns_the_light_off(tmp_path):
    t, link, clock, _ = make(tmp_path)
    feed(t, clock, [20, 20, 20])
    t.set(enabled=False)
    assert not t.on and link.sent[-1].startswith("TL,0,")
    again, _, _, _ = make(tmp_path)
    assert again.enabled is False


def test_beam_leads_into_turns_and_tightens_when_the_way_is_open(tmp_path):
    t, link, clock, _ = make(tmp_path, front=None)     # nothing ahead within range
    feed(t, clock, [20, 20, 20])
    t.aim(clock())                                     # first reading of the heading
    clock.t += 1.0
    link.tel["heading"] = 30.0                         # turning left at about 0.52 rad/s
    t.aim(clock())
    pan = [s for s in link.sent if s.startswith("P=")][-1]
    zoom = [s for s in link.sent if s.startswith("Z=")][-1]
    assert int(pan[2:]) < 165, "a left turn pans the beam left (a smaller angle than home)"
    assert zoom == "Z=140", "open ahead: tight beam"
    assert 120 <= int(pan[2:]) <= 190


def test_beam_goes_wide_when_something_is_close(tmp_path):
    t, link, clock, _ = make(tmp_path, front=0.5)
    feed(t, clock, [20, 20, 20])
    t.aim(clock())
    assert [s for s in link.sent if s.startswith("Z=")][-1] == "Z=80"
    n = len(link.sent)
    clock.t += 1.0
    t.aim(clock())
    assert len(link.sent) == n, "nothing changed: the servos are left alone"


def test_no_aiming_while_the_light_is_off(tmp_path):
    t, link, clock, _ = make(tmp_path)
    for _ in range(5):
        clock.t += 1
        t.aim(clock())
    assert not link.sent


# ---- the camera's re-exposure steps back while the light can act ----
def watch(tmp_path, defer):
    clock = Clock()
    w = camera.ExposureWatch(log=lambda *_: None, settings=str(tmp_path / "ae.json"), clock=clock, defer=defer)
    return w, clock


def test_exposure_is_left_alone_while_the_flashlight_deals_with_the_dark(tmp_path):
    t, link, clock, _ = make(tmp_path)
    w, wclock = watch(tmp_path, t.handles)
    assert not any(w.due(20, wclock()) for _ in range(6)), "dark, light available: the light's job"
    feed(t, clock, [20, 20, 20] + [30] * 12)           # the light is flat out and it is still dark
    assert t.pct == 90
    assert [w.due(20, wclock()) for _ in range(3)] == [False, False, True], "now the exposure may help"


def test_exposure_watch_is_unchanged_when_the_flashlight_is_off_duty(tmp_path):
    t, link, clock, _ = make(tmp_path)
    link.tel["balancing"] = False                      # parked: the light does nothing, so exposure handles it
    w, wclock = watch(tmp_path, t.handles)
    assert [w.due(20, wclock()) for _ in range(3)] == [False, False, True]


def test_a_walk_that_could_not_run_says_so_and_is_tried_again_soon(tmp_path):
    w, wclock = watch(tmp_path, None)
    w.finished({"ok": False, "error": "CSI publisher control channel is not ready"}, 30, wclock())
    assert "could not re-expose" in w.status()["action"] and "re-exposed," not in w.status()["action"]
    assert w.status()["resting_s"] <= w.REST
