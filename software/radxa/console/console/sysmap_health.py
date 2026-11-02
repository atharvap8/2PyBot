"""sysmap_health.py — turn raw console state into per-block health for the
system map (port 6969).

Pure functions, standard library only, so the rules can be unit-tested
without FastAPI, a serial port or a robot.

Status vocabulary (one per node and per wire):

    ok       data is fresh and inside limits
    warn     working, but something needs attention
    error    this block is the most likely culprit
    blocked  cannot be judged because a block upstream has failed
    nodata   no signal has arrived yet (console just started, etc.)
    nofb     output-only hardware: the firmware commands it, nothing reports back

Root-cause rule: a node only turns red when its own evidence says so. When a
parent (for example the USB serial link) fails, everything that is only known
*through* that parent becomes `blocked`, not `error`, so exactly one block
lights up red and points at the culprit.
"""
from __future__ import annotations

import math
import re
import time

OK, WARN, ERROR, BLOCKED, NODATA, NOFB = "ok", "warn", "error", "blocked", "nodata", "nofb"
RANK = {ERROR: 5, WARN: 4, BLOCKED: 3, NODATA: 2, NOFB: 1, OK: 0}

# M0601 fault byte (DDT M0601C-111 manual / firmware hub_drive.cpp)
FAULT_BITS = ((0x01, "sensor fault"), (0x02, "bus overcurrent"), (0x04, "phase overcurrent"),
              (0x08, "stall"), (0x10, "overheat"))

# Which node each node is observed through. Used for root-cause suppression.
PARENT = {
    "radxa": None, "camera": "radxa", "tof": "radxa", "clients": "radxa",
    "serial": "radxa", "esp32": "serial",
    "imu": "esp32", "compass": "esp32", "gamepad": "esp32", "lights": "esp32",
    "wheel_l": "esp32", "wheel_r": "esp32", "payload": "esp32",
}

PARENT_NAME = {"radxa": "the Radxa", "serial": "the USB serial link", "esp32": "the ESP32"}

HM_STALE_S = 1.5          # HM is 10 Hz
LOOP_WARN_US = 6000       # 200 Hz loop nominal 5000 us
LOOP_ERROR_US = 20000     # firmware disarms above 20 ms
TEMP_WARN_C = 70          # M0601 overheat trips at 80 °C
TEMP_ERROR_C = 80
CPU_WARN_C, CPU_ERROR_C = 75, 85
CAM_LATENCY_WARN_MS = 150
COMPASS_FROZEN_S = 30


def _num(value, default=None):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def _fmt(value, unit="", digits=1):
    v = _num(value)
    if v is None:
        return "–"
    return f"{v:.{digits}f}{(' ' + unit) if unit else ''}"


def node(status, reason="", metrics=None, **extra):
    out = {"status": status, "reason": reason, "metrics": metrics or []}
    out.update(extra)
    return out


def worst(*statuses):
    return max(statuses, key=lambda s: RANK.get(s, 0)) if statuses else OK


def decode_faults(mask):
    mask = int(_num(mask, 0) or 0)
    names = [name for bit, name in FAULT_BITS if mask & bit]
    if mask & 0xE0:
        names.append(f"reserved bits 0x{mask & 0xE0:02X}")
    return names


# ---------------------------------------------------------------- Radxa side
def eval_radxa(system):
    temp = _num(system.get("cpu_temp_c"))
    load = _num(system.get("load1"))
    cores = max(1, int(_num(system.get("cores"), 1)))
    mem = _num(system.get("mem_used_pct"))
    metrics = [["CPU temp", _fmt(temp, "°C", 0)], ["Load", _fmt(load, "", 2)],
               ["Memory", _fmt(mem, "%", 0)], ["Uptime", system.get("uptime", "–")]]
    if temp is not None and temp >= CPU_ERROR_C:
        return node(ERROR, f"CPU at {temp:.0f} °C; the board will throttle", metrics)
    if temp is not None and temp >= CPU_WARN_C:
        return node(WARN, f"CPU running hot at {temp:.0f} °C", metrics)
    if load is not None and load > cores * 1.5:
        return node(WARN, f"Load {load:.1f} on {cores} cores; video or vision may lag", metrics)
    return node(OK, "Console, video and ToF services running", metrics)


def eval_camera(stream, host):
    fps = _num(stream.get("fps"))
    latency = _num(stream.get("capture_to_publish_ms"))
    metrics = [["Frame rate", _fmt(fps, "fps", 0)], ["Capture to publish", _fmt(latency, "ms", 0)],
               ["Encoder", stream.get("encoder") or "–"], ["Frames", stream.get("frame") or "–"]]
    if not stream.get("running"):
        return node(ERROR, stream.get("error") or "Video encoder is not running", metrics)
    if not host.get("camera"):
        return node(ERROR, "Encoder is running but no fresh frame in the last 5 s", metrics)
    if latency is not None and latency > CAM_LATENCY_WARN_MS:
        return node(WARN, f"Capture to publish is {latency:.0f} ms", metrics)
    return node(OK, "Streaming", metrics)


def eval_tof(tof):
    age = _num(tof.get("age_ms"))
    valid = int(_num(tof.get("valid_count"), 0) or 0)
    nearest = _num(tof.get("nearest_mm"))
    metrics = [["Nearest", _fmt(nearest / 1000 if nearest else None, "m", 2)],
               ["Valid zones", f"{valid}/64"], ["Last frame", (_fmt(age / 1000, "s ago", 1) if age and age >= 1000 else _fmt(age, "ms ago", 0))],
               ["Bus", f"I2C-{tof.get('bus', '?')} {tof.get('address', '')}"],
               ["Rate", _fmt(tof.get("ranging_hz"), "Hz", 0)]]
    state = tof.get("state", "starting")
    if state == "starting":
        return node(WARN, "ToF worker starting", metrics)
    if state != "ranging":
        return node(ERROR, tof.get("error") or f"ToF worker state: {state}", metrics)
    if tof.get("stale"):
        secs = f"{age / 1000:.1f} s" if age is not None else "a while"
        return node(ERROR, f"No frame for {secs}", metrics)
    if valid == 0:
        return node(WARN, "Ranging, but every zone fails its status/sigma checks", metrics)
    return node(OK, "Ranging", metrics)


def eval_clients(host, clients):
    metrics = [["Cockpit clients", str(clients.get("console", 0))],
               ["Map viewers", str(clients.get("sysmap", 0))],
               ["Hotspot", "up" if host.get("hotspot") else "down"]]
    if not host.get("hotspot"):
        return node(ERROR, "Hotspot address is not up; phones cannot reach the bot", metrics)
    if not clients.get("console"):
        return node(OK, "Hotspot up, no cockpit connected", metrics)
    return node(OK, "Connected", metrics)


def eval_serial(serial):
    rate = _num(serial.get("rx_hz"), 0)
    age = _num(serial.get("last_rx_age"))
    metrics = [["Odometry rate", _fmt(rate, "Hz", 0)], ["Port", serial.get("port", "–")],
               ["Baud", str(serial.get("baud", "–"))],
               ["Parameters", "loaded" if serial.get("dump_complete") else "loading"]]
    if serial.get("maintenance"):
        return node(WARN, "Firmware maintenance holds the port", metrics)
    if not serial.get("connected"):
        return node(ERROR, serial.get("error") or "Serial port is not open", metrics)
    if serial.get("stale"):
        secs = f"{age:.1f} s" if age is not None and age < 1e6 else "ever"
        return node(ERROR, f"Port open but no telemetry for {secs}", metrics)
    if rate and rate < 25:
        return node(WARN, f"Odometry at {rate:.0f} Hz, expected 50", metrics)
    return node(OK, "Telemetry flowing", metrics)


# ---------------------------------------------------------------- ESP32 side
def eval_esp32(tel, debug):
    loop = _num(debug.get("loopMax"))
    state = tel.get("startup_state")
    metrics = [["State", "Balancing" if tel.get("balancing") else (state or "–").title()],
               ["Loop max", _fmt(loop / 1000 if loop is not None else None, "ms", 1)],
               ["Host ready", "yes" if tel.get("host_ready") else "no"]]
    if state == "ERROR":
        return node(ERROR, "Firmware reported a startup error", metrics)
    if loop is not None and loop > LOOP_ERROR_US:
        return node(ERROR, f"Control interval {loop / 1000:.1f} ms; above 20 ms disarms", metrics)
    if state in ("WAIT_HOST", "INITIALIZING", "CALIBRATING"):
        return node(WARN, f"Startup: {state.replace('_', ' ').lower()}", metrics)
    if loop is not None and loop > LOOP_WARN_US:
        return node(WARN, f"Control interval peaked at {loop / 1000:.1f} ms", metrics)
    return node(OK, "Control loop healthy", metrics)


def eval_imu(tel, debug, events):
    pitch = _num(tel.get("pitch"))
    metrics = [["Pitch", _fmt(pitch, "°", 1)], ["Rate", _fmt(debug.get("w"), "°/s", 1)],
               ["Accel", _fmt(debug.get("accelMag"), "g", 2)]]
    if events.get("imu_error"):
        return node(ERROR, events["imu_error"], metrics)
    if pitch is None:
        return node(NODATA, "No attitude yet", metrics)
    return node(OK, "Attitude updating", metrics)


def eval_compass(tel, events, moving):
    metrics = [["Heading", _fmt(tel.get("yaw"), "°", 0)],
               ["Unchanged for", _fmt(events.get("yaw_unchanged_s"), "s", 0)]]
    frozen = _num(events.get("yaw_unchanged_s"), 0)
    if moving and frozen >= COMPASS_FROZEN_S:
        return node(WARN, f"Heading frozen for {frozen:.0f} s while the wheels turn", metrics,
                    note="Firmware has no direct compass health flag; this is inferred.")
    return node(OK, "Heading updating", metrics,
                note="Firmware has no direct compass health flag; this is inferred.")


def eval_gamepad(events, nav, balancing):
    pad = events.get("pad")
    if nav.get("pad") is not None and nav.get("fresh"):
        pad = bool(nav["pad"])
    metrics = [["Controller", events.get("pad_name") or "–"],
               ["Speed mode", events.get("speed_mode") or "–"],
               ["Robot", "balancing" if balancing else "idle"]]
    if pad is None:
        return node(NODATA, "No connect or disconnect seen since the console started", metrics)
    if not pad:
        return node(WARN, "Disconnected; balance continues but manual drive is gone", metrics)
    return node(OK, "Connected over Bluetooth", metrics)


def expected_status_led(tel, events):
    if events.get("imu_error") or tel.get("startup_state") == "ERROR":
        return "fast blink (error)"
    if tel.get("balancing"):
        return "breathing (balancing)"
    state = tel.get("startup_state")
    if state in ("WAIT_HOST", "INITIALIZING", "CALIBRATING"):
        return "slow blink (startup)"
    return "steady on (ready)"


def eval_lights(tel, events):
    return node(NOFB, "Output only: firmware drives both, nothing reports back",
                [["Blue LED should be", expected_status_led(tel, events)],
                 ["Ring", "firmware animations"]])


def eval_payload(events):
    return node(NOFB, "Output only: these are the last commanded values",
                [["Pan", _fmt(events.get("pan"), "°", 0)], ["Zoom", _fmt(events.get("zoom"), "°", 0)],
                 ["Torch", _fmt(events.get("torch"), "%", 0)]])


def eval_wheel(side, hub, hub_age, balancing, events):
    s = "l" if side == "LEFT" else "r"
    fault = int(_num(hub.get(f"fault_{s}"), 0) or 0)
    temp = _num(hub.get(f"temp_{s}"))
    temp = None if temp is not None and temp < 0 else temp
    stats = events.get("hub_stats", {}).get(side, {})
    metrics = [["Speed", _fmt(hub.get(f"rpm_{s}"), "rpm", 0)],
               ["Current", f"{_fmt(hub.get(f'current_{s}'), '', 2)} / {_fmt(hub.get(f'current_cmd_{s}'), 'A', 2)}"],
               ["Temp", _fmt(temp, "°C", 0) if temp is not None else "idle query only"],
               ["Fault byte", f"0x{fault:02X}"],
               ["CRC errors", str(stats.get("crc", "–"))], ["Timeouts", str(stats.get("timeout", "–"))]]
    if hub_age is None:
        return node(NODATA, "No HM motor records yet", metrics)
    if hub_age > HM_STALE_S:
        return node(ERROR, f"No HM motor record for {hub_age:.1f} s", metrics)
    if fault:
        return node(ERROR, "M0601 fault: " + ", ".join(decode_faults(fault)), metrics)
    probe = events.get("hub_probe", {}).get(side)
    if not hub.get("link_ok"):
        if probe == "OK":
            return node(WARN, "Shared link flag down; this wheel answered at boot", metrics)
        return node(ERROR, "RS485 reply timeout", metrics)
    if temp is not None and temp >= TEMP_ERROR_C:
        return node(ERROR, f"Winding at {temp:.0f} °C; overheat protection trips at 80", metrics)
    if not hub.get("mode_ok"):
        return node(WARN, "Motor has not confirmed current mode", metrics)
    if temp is not None and temp >= TEMP_WARN_C:
        return node(WARN, f"Winding at {temp:.0f} °C", metrics)
    if balancing and hub.get("saturated"):
        return node(WARN, "Current limit reached", metrics)
    return node(OK, "Replying, no faults", metrics)


# ---------------------------------------------------------------- top level
def evaluate(inp):
    serial, tel = inp.get("serial", {}), inp.get("tel", {})
    debug, hub = inp.get("debug", {}), inp.get("hub", {})
    events, host = inp.get("events", {}), inp.get("host", {})
    balancing = bool(tel.get("balancing"))
    hub_age = _num(inp.get("hub_age"))
    moving = balancing or any(abs(_num(hub.get(k), 0) or 0) > 5 for k in ("rpm_l", "rpm_r"))

    nodes = {
        "radxa": eval_radxa(inp.get("system", {})),
        "camera": eval_camera(inp.get("stream", {}), host),
        "tof": eval_tof(inp.get("tof", {})),
        "clients": eval_clients(host, inp.get("clients", {})),
        "serial": eval_serial(serial),
        "esp32": eval_esp32(tel, debug),
        "imu": eval_imu(tel, debug, events),
        "compass": eval_compass(tel, events, moving),
        "gamepad": eval_gamepad(events, inp.get("nav", {}), balancing),
        "lights": eval_lights(tel, events),
        "wheel_l": eval_wheel("LEFT", hub, hub_age, balancing, events),
        "wheel_r": eval_wheel("RIGHT", hub, hub_age, balancing, events),
        "payload": eval_payload(events),
    }

    # Root-cause suppression, walked top-down so chains collapse correctly.
    for name in ("serial", "esp32", "imu", "compass", "gamepad", "lights",
                 "wheel_l", "wheel_r", "payload", "camera", "tof", "clients"):
        parent = PARENT.get(name)
        if not parent:
            continue
        p = nodes[parent]["status"]
        if p in (ERROR, BLOCKED) or (parent == "serial" and p == WARN):
            nodes[name] = dict(nodes[name], status=BLOCKED,
                               reason=f"Unknown until {PARENT_NAME.get(parent, parent)} recovers",
                               own_status=nodes[name]["status"])

    errors = [k for k, v in nodes.items() if v["status"] == ERROR]
    warns = [k for k, v in nodes.items() if v["status"] == WARN]
    return {"nodes": nodes,
            "summary": {"errors": errors, "warnings": warns,
                        "state": ERROR if errors else WARN if warns else OK}}


# ------------------------------------------------------------ log tracker
class EventTracker:
    """Folds firmware log lines into the few facts that have no telemetry
    field of their own: IMU init errors, gamepad connect state, the boot-time
    motor probe, per-wheel CRC/timeout counters and payload commands."""

    PAD_ON = re.compile(r"^\[PAD\] Connected: ?(.*)$")
    HUB_PROBE = re.compile(r"^\[HUB\] RIGHT id\d+ (OK|NO REPLY); LEFT id\d+ (OK|NO REPLY)")
    HUB_STAT = re.compile(r"^\[HUB\] (LEFT|RIGHT) id\d+ .*CRC(\d+) timeout(\d+)")
    PAN = re.compile(r"^\[PAY\] Pan\s+-> (-?[\d.]+)")
    ZOOM = re.compile(r"^\[PAY\] Zoom -> (-?[\d.]+)")
    TORCH = re.compile(r"^\[PAY\] Torch\D*?(\d+)%")

    def __init__(self):
        self._last = None
        self.state = {"imu_error": "", "pad": None, "pad_name": "", "speed_mode": "",
                      "hub_probe": {}, "hub_stats": {}, "pan": None, "zoom": None, "torch": None}
        self._yaw = None
        self._yaw_since = time.monotonic()

    def _apply(self, text):
        s = self.state
        if text.startswith(("[BOOT] Waiting for Radxa", "[BOOT] BOOTWAIT")):
            s.update(imu_error="", pad=None, pad_name="", hub_probe={}, hub_stats={})
        elif text.startswith(("[IMU] ERROR", "[MAIN] FATAL: IMU")):
            s["imu_error"] = text.split("] ", 1)[-1]
        elif text.startswith("[IMU] Initialisation OK"):
            s["imu_error"] = ""
        elif m := self.PAD_ON.match(text):
            s.update(pad=True, pad_name=m.group(1).strip())
        elif text.startswith(("[PAD] Disconnected", "[PAD] Bluepad32 ready")):
            s["pad"] = False
        elif text.startswith("[PAD] SPEED "):
            s["speed_mode"] = text.rsplit(" ", 1)[-1].lower()
        elif m := self.HUB_PROBE.match(text):
            s["hub_probe"] = {"RIGHT": m.group(1), "LEFT": m.group(2)}
        elif m := self.HUB_STAT.match(text):
            s["hub_stats"][m.group(1)] = {"crc": int(m.group(2)), "timeout": int(m.group(3))}
        elif m := self.PAN.match(text):
            s["pan"] = float(m.group(1))
        elif m := self.ZOOM.match(text):
            s["zoom"] = float(m.group(1))
        elif text.startswith("[PAY] Torch"):
            m = self.TORCH.match(text)
            off = text.rstrip().endswith(("off", "(off)"))
            s["torch"] = 0.0 if off or not m else float(m.group(1))

    def update(self, log, yaw, fresh):
        entries = list(log)
        start = 0
        if self._last is not None:
            for i in range(len(entries) - 1, -1, -1):
                if entries[i] is self._last:
                    start = i + 1
                    break
        for entry in entries[start:]:
            self._apply(str(entry.get("text", "")))
        if entries:
            self._last = entries[-1]
        now = time.monotonic()
        if not fresh or yaw is None or yaw != self._yaw:
            self._yaw, self._yaw_since = yaw, now
        return dict(self.state, yaw_unchanged_s=round(now - self._yaw_since, 1))
