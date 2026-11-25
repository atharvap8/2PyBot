"""The flashlight: full brightness when the picture is fully dark, off when the darkness is gone, nothing else."""
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


def make(tmp_path):
    link, clock, logs = Link(), Clock(), []
    t = torch.AutoTorch(link, measure=lambda: None, front_m=lambda: None, log=logs.append,
                        settings=str(tmp_path / "auto_torch.json"), clock=clock)
    return t, link, clock, logs


def feed(t, clock, values, dt=0.5):
    for luma in values:
        clock.t += dt
        t.step(luma, clock())


def light(link):
    return [s for s in link.sent if s.startswith("TL,")]


def switched_on(tmp_path, lit=60):
    """In the dark, light on and settled, the lit picture measured."""
    t, link, clock, logs = make(tmp_path)
    feed(t, clock, [10] * 4)
    assert t.on
    feed(t, clock, [lit] * 6)
    return t, link, clock, logs


def test_fully_dark_for_a_second_and_a_half_switches_it_on_at_full_brightness(tmp_path):
    t, link, clock, logs = make(tmp_path)
    feed(t, clock, [10, 10, 10])                       # dark for 1.0 s so far
    assert not link.sent, "a shadow passing is not night"
    feed(t, clock, [10])
    assert light(link) == ["TL,1,90"] and t.on and "dark" in logs[-1]


def test_dim_is_not_dark(tmp_path):
    t, link, clock, _ = make(tmp_path)
    feed(t, clock, [30, 35, 28, 40] * 5)               # a dim room: it only reacts to a fully dark picture
    assert not link.sent


def test_a_brighter_moment_starts_the_dark_count_again(tmp_path):
    t, link, clock, _ = make(tmp_path)
    feed(t, clock, [10, 10, 90, 10, 10, 10])
    assert not link.sent
    feed(t, clock, [10])
    assert t.on


def test_it_takes_one_fixed_position_and_then_leaves_the_head_alone(tmp_path):
    t, link, clock, _ = switched_on(tmp_path)
    assert link.sent == ["TL,1,90", "P=165", "Z=140"], "centred, and the wide end of the zoom (the other end is a dot)"
    link.tel["heading"] = 90.0                         # turning, things close, things far: nothing moves the head
    feed(t, clock, [60] * 40)
    assert link.sent == ["TL,1,90", "P=165", "Z=140"]


def test_no_brightness_changes_while_it_is_on(tmp_path):
    t, link, clock, _ = switched_on(tmp_path)
    feed(t, clock, [40, 80, 30, 90, 55, 70] * 6)       # the lit picture wanders as the robot drives
    assert light(link) == ["TL,1,90"], "one command: on, full. No stepping up and down"


def test_when_the_darkness_is_gone_it_switches_off_and_lets_go(tmp_path):
    t, link, clock, logs = switched_on(tmp_path, lit=60)
    feed(t, clock, [170] * 12)                         # the room lights came on
    assert not t.on and light(link)[-1] == "TL,0,90" and "gamepad" in logs[-1]
    n = len(link.sent)
    feed(t, clock, [120] * 30)                         # a lit room: the bot does nothing more
    assert len(link.sent) == n


def test_a_bright_flash_does_not_switch_it_off(tmp_path):
    t, link, clock, _ = switched_on(tmp_path, lit=60)
    feed(t, clock, [170, 170, 60, 170, 60] * 4)
    assert t.on


def test_its_own_light_on_something_close_is_not_daylight(tmp_path):
    t, link, clock, _ = switched_on(tmp_path, lit=40)
    feed(t, clock, [160] * 12)                         # drove up to a white wall: bright, but only because of the light
    assert not t.on
    feed(t, clock, [5])                                # dark again at once: it was our own light
    assert t.on, "back on straight away, not after another 1.5 s"
    feed(t, clock, [60] * 6 + [160] * 40)              # the same wall again: this time it is not fooled
    assert t.on and light(link) == ["TL,1,90", "TL,0,90", "TL,1,90"]
    feed(t, clock, [245] * 12)                         # real daylight is still believed
    assert not t.on


def test_parked_robot_gets_the_light_switched_off_and_is_otherwise_left_alone(tmp_path):
    t, link, clock, _ = switched_on(tmp_path)
    link.tel["balancing"] = False
    feed(t, clock, [60])
    assert not t.on and light(link)[-1] == "TL,0,90"
    n = len(link.sent)
    feed(t, clock, [2] * 60)                           # a dark room at night with the robot parked
    assert len(link.sent) == n, "it must not light the room while parked"


def test_the_gamepad_takes_the_flashlight_at_once(tmp_path):
    t, link, clock, _ = switched_on(tmp_path)
    t.on_line("[PAY] Torch ON (90%)")                  # would be the echo of the bot's own command if it came in time
    clock.t += 5
    t.on_line("[PAY] Torch 60%")                       # D-pad left, three times
    assert t.on and 0 < t.status()["holding_s"] <= 1.5
    n = len(link.sent)
    feed(t, clock, [60] * 30)
    assert len(link.sent) == n, "the bot does not put the brightness back"
    feed(t, clock, [200] * 12)                         # daylight: it still switches off a light that is on
    assert not t.on


def test_switching_it_off_from_the_gamepad_holds_for_a_moment_then_the_dark_wins(tmp_path):
    t, link, clock, _ = switched_on(tmp_path)
    clock.t += 5
    t.on_line("[PAY] Torch off")                       # RB on the gamepad
    assert not t.on
    n = len(link.sent)
    feed(t, clock, [5, 5])
    assert len(link.sent) == n
    feed(t, clock, [5] * 6)
    assert t.on


def test_the_controllers_boot_banner_is_not_the_operator(tmp_path):
    t, link, clock, _ = make(tmp_path)
    t.on_line("[PAY] Torch GPIO2 off, 25% preset, HARD CAP 90% duty @ 20 kHz")     # printed at every controller start
    t.on_line("[PAY] Servos re-centred")
    assert t.status()["holding_s"] == 0 and not t.on


def test_old_firmware_or_a_sleeping_robot_is_not_commanded(tmp_path):
    t, link, clock, _ = make(tmp_path)
    link.derived = {}
    feed(t, clock, [5] * 10)
    assert not link.sent and not t.status()["available"]
    link.derived = {"TORCHSET": 1}
    link.tel["asleep"] = True
    feed(t, clock, [5] * 10)
    assert not link.sent


def test_the_switch_sticks_and_switching_off_turns_the_light_off(tmp_path):
    t, link, clock, _ = switched_on(tmp_path)
    t.set(enabled=False)
    assert not t.on and light(link)[-1] == "TL,0,90"
    again, _, _, _ = make(tmp_path)
    assert again.enabled is False


# ---- the camera's re-exposure steps back while darkness is the light's job ----
def watch(tmp_path, defer):
    clock = Clock()
    w = camera.ExposureWatch(log=lambda *_: None, settings=str(tmp_path / "ae.json"), clock=clock, defer=defer)
    return w, clock


def test_exposure_is_left_alone_while_the_flashlight_deals_with_the_dark(tmp_path):
    t, link, clock, _ = make(tmp_path)
    w, wclock = watch(tmp_path, t.handles)
    assert not any(w.due(20, wclock()) for _ in range(6)), "dark, light about to come on: the light's job"
    feed(t, clock, [10] * 4)
    assert t.on
    assert [w.due(40, wclock()) for _ in range(3)] == [False, False, True], "light flat out and still dim: now the exposure may help"


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
