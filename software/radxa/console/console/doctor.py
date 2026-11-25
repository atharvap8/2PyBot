"""Doctor: root-cause analysis and bounded self-healing.

Two layers, kept apart on purpose:

  * a PURE rule engine (`diagnose`) that turns a dict of collected signals into
    ranked `Case`s, each with recoveries tagged ``auto`` | ``confirm`` | ``manual``.
    No I/O, so every rule is unit-testable (see tests/test_doctor.py).

  * a runtime `Doctor` that collects the signals, exposes the cases over the API,
    and applies only the *auto* recoveries, with a global attempt cap and a
    per-action cooldown so the healer can never become its own loop (the exact
    failure mode that started this: a retry with no backoff).

Safety rules (hard): the doctor never arms, drives, or unlocks motion. Auto
recoveries are limited to things the non-root console can safely do and that are
idempotent (restart the camera stream, install a missing Python dependency).
Anything that needs sudo or touches the controller (service restart, deploy
timer, re-pair, reflash) is ``confirm``/``manual`` and surfaced to the operator
in the CLI/TUI, which can elevate. Design notes: docs/DOCTOR_SELF_HEALER.md.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

SEVERITY_RANK = {"critical": 3, "warn": 2, "info": 1}


@dataclass
class Recovery:
    label: str
    safety: str              # "auto" | "confirm" | "manual"
    action: str = ""         # key the runtime maps to a callable; empty = pure manual
    detail: str = ""

    def to_dict(self):
        return {"label": self.label, "safety": self.safety, "action": self.action, "detail": self.detail}


@dataclass
class Case:
    id: str
    title: str
    cause: str
    severity: str = "warn"
    evidence: dict = field(default_factory=dict)
    recoveries: list = field(default_factory=list)

    def to_dict(self):
        return {"id": self.id, "title": self.title, "cause": self.cause, "severity": self.severity,
                "evidence": self.evidence, "recoveries": [r.to_dict() for r in self.recoveries]}


# ---------------------------------------------------------------- pure rules
# Each rule takes the signals dict and returns a Case or None. Keep them pure.

def rule_deploy_deferring(s):
    d = s.get("deploy") or {}
    if d.get("phase") == "waiting_idle" and (d.get("defer_count") or 0) >= 2:
        return Case(
            "deploy_deferring", "Deployment keeps deferring",
            "The deploy cannot take the controller maintenance lock and has deferred "
            f"{d.get('defer_count')} times. Powered-down or not-ready controllers cause this.",
            "critical", {"phase": d.get("phase"), "defer_count": d.get("defer_count"), "message": d.get("message")},
            [Recovery("Wake the robot", "confirm", "wake"),
             Recovery("Stop the deploy retries", "confirm", "stop_deploy_timer"),
             Recovery("Stop with SELECT, wait for READY, then redeploy", "manual")])
    return None


def rule_console_crash(s):
    svc = s.get("services") or {}
    restarts = (s.get("service_restarts") or {}).get("2pybot-console", 0)
    if svc.get("2pybot-console") == "failed" or restarts >= 3:
        return Case(
            "console_crash", "Console is restart-looping",
            "The console service is failing to stay up" + (f" ({restarts} restarts)" if restarts else "") + ".",
            "critical", {"state": svc.get("2pybot-console"), "restarts": restarts},
            [Recovery("Restart the console once", "confirm", "restart_console"),
             Recovery("Roll back to the previous release", "confirm", "rollback_release"),
             Recovery("Read the last traceback in the service log", "manual")])
    return None


def rule_serial_stale(s):
    ser = s.get("serial") or {}
    if ser.get("connected") and ser.get("stale"):
        return Case(
            "serial_stale", "No telemetry from the controller",
            "The serial port is open but no telemetry is arriving; the ESP32 may be held in reset or flashing.",
            "critical", {"connected": True, "stale": True},
            [Recovery("Check the boot gate and USB cable", "manual"),
             Recovery("Restart the console", "confirm", "restart_console")])
    return None


def rule_camera_down(s):
    cam = s.get("camera") or {}
    if cam.get("running") is False and not (s.get("tel") or {}).get("asleep") and not cam.get("paused"):
        return Case(
            "camera_down", "Camera stream is down",
            cam.get("error") or "The video encoder is not running.",
            "warn", {"running": False, "error": cam.get("error")},
            [Recovery("Restart the camera stream", "auto", "restart_stream")])
    return None


def rule_onnxruntime_missing(s):
    deps = s.get("deps") or {}
    vp = s.get("visual_pair") or {}
    if vp.get("enabled") and deps.get("onnxruntime") is False:
        return Case(
            "onnxruntime_missing", "Camera re-pair detector unavailable",
            "onnxruntime is not installed in the console environment, so showing the gamepad to the camera cannot work.",
            "warn", {"onnxruntime": False},
            [Recovery("Install onnxruntime into the console venv", "auto", "pip_onnxruntime")])
    return None


def rule_rs485(s):
    hub = s.get("hub") or {}
    for side, key in (("left", "l"), ("right", "r")):
        stats = (s.get("hub_stats") or {}).get(side.upper(), {})
        if hub.get(f"fault_{key}"):
            return Case(
                f"motor_fault_{key}", f"{side.title()} motor fault",
                f"The {side} M0601 reports a fault byte 0x{int(hub.get(f'fault_{key}')):02X}.",
                "warn", {"fault": hub.get(f"fault_{key}")},
                [Recovery(f"Let the {side} motor cool; check for a jammed wheel", "manual")])
        if stats.get("timeout", 0) and stats.get("timeout", 0) > stats.get("crc", 0) + 50:
            return Case(
                f"rs485_{key}", f"{side.title()} RS485 link dropping replies",
                f"The {side} motor link is losing replies (timeouts {stats.get('timeout')}).",
                "warn", {"timeouts": stats.get("timeout")},
                [Recovery(f"Inspect the {side} motor cable, connector and transceiver DE line", "manual")])
    return None


def rule_powered_down(s):
    if (s.get("tel") or {}).get("asleep"):
        return Case(
            "powered_down", "Robot is powered down",
            "Sleep mode is on: camera, lidar, depth sensor, ring, servos and the gamepad are off, and it cannot arm.",
            "info", {"asleep": True},
            [Recovery("Wake the robot", "confirm", "wake")])
    return None


def rule_disk_full(s):
    pct = (s.get("system") or {}).get("disk_pct")
    if isinstance(pct, (int, float)) and pct >= 90:
        return Case(
            "disk_full", "Disk almost full",
            f"The root filesystem is {pct:.0f}% full; logs and maps may fail to write.",
            "warn", {"disk_pct": pct},
            [Recovery("Vacuum the journal to 200M", "confirm", "vacuum_journal")])
    return None


def rule_cpu_hot(s):
    t = (s.get("system") or {}).get("cpu_temp_c")
    if isinstance(t, (int, float)) and t >= 85:
        return Case(
            "cpu_hot", "Radxa is overheating",
            f"CPU at {t:.0f} °C; the board will throttle video and SLAM.",
            "warn", {"cpu_temp_c": t},
            [Recovery("Improve airflow; consider power-down when idle", "manual")])
    return None


RULES = [rule_deploy_deferring, rule_console_crash, rule_serial_stale, rule_camera_down,
         rule_onnxruntime_missing, rule_rs485, rule_cpu_hot, rule_disk_full, rule_powered_down]


def diagnose(signals):
    """Pure: signals -> ranked list of Cases (most severe first)."""
    cases = []
    for rule in RULES:
        try:
            case = rule(signals)
        except Exception:  # noqa: BLE001 -- a broken rule must not break diagnosis
            case = None
        if case:
            cases.append(case)
    cases.sort(key=lambda c: SEVERITY_RANK.get(c.severity, 0), reverse=True)
    return cases


# ---------------------------------------------------------------- runtime
import asyncio  # noqa: E402
import importlib.util  # noqa: E402
import os  # noqa: E402
import shutil  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402

AUTO_CAP = 3          # max auto-heals of one action within the window
AUTO_WINDOW = 900     # seconds
COOLDOWN = 120        # min seconds between auto attempts of the same action


def _dep_installed(name):
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def _service_state(name):
    try:
        return subprocess.run(["systemctl", "is-active", name], capture_output=True, text=True, timeout=4).stdout.strip()
    except Exception:  # noqa: BLE001
        return "unknown"


def _system_info():
    info = {}
    try:
        usage = shutil.disk_usage("/")
        info["disk_pct"] = round(usage.used / usage.total * 100, 1)
    except OSError:
        pass
    try:
        p = "/sys/class/thermal/thermal_zone0/temp"
        if os.path.exists(p):
            info["cpu_temp_c"] = int(open(p).read().strip()) / 1000.0
    except (OSError, ValueError):
        pass
    return info


class Doctor:
    """Collects signals, exposes cases, and applies only the auto-safe recoveries."""

    def __init__(self, link, power=None, streamer=None, visual_pair=None, log=None):
        self.link, self.power, self.streamer, self.visual_pair = link, power, streamer, visual_pair
        self.log = log or (lambda *_: None)
        self.cases = []
        self.history = []
        self._attempts = {}
        self.actions = {                      # only non-sudo, idempotent actions run here
            "restart_stream": self._restart_stream,
            "pip_onnxruntime": self._pip_onnxruntime,
            "wake": self._wake,
        }

    def collect(self):
        link = self.link
        tel = getattr(link, "tel", {}) or {}
        stream = self.streamer.status() if self.streamer else {}
        deploy = {}
        try:
            from .deployment_status import STATE_DIR, read as _read
            deploy = _read(STATE_DIR / "progress.json", {})
        except Exception:  # noqa: BLE001
            pass
        vp = self.visual_pair.status() if self.visual_pair else {}
        events = getattr(link, "events", {}) or {}
        return {
            "tel": {"asleep": tel.get("asleep"), "stale": tel.get("stale"), "balancing": tel.get("balancing")},
            "serial": {"connected": getattr(link, "connected", False), "stale": tel.get("stale", True)},
            "camera": {"running": stream.get("running"), "paused": stream.get("paused"), "error": stream.get("error")},
            "hub": tel.get("hub", {}) or {},
            "hub_stats": events.get("hub_stats", {}) if isinstance(events, dict) else {},
            "deploy": {"phase": deploy.get("phase"), "defer_count": deploy.get("defer_count"), "message": deploy.get("message")},
            "services": {n: _service_state(n) for n in ("2pybot-console", "2pybot-mediamtx")},
            "deps": {"onnxruntime": _dep_installed("onnxruntime")},
            "visual_pair": {"enabled": vp.get("enabled")},
            "system": _system_info(),
        }

    def refresh(self):
        self.cases = diagnose(self.collect())
        return self.cases

    def status(self):
        return {"cases": [c.to_dict() for c in self.cases], "history": self.history[-20:]}

    def _allowed(self, action):
        now = time.monotonic()
        win = [t for t in self._attempts.get(action, []) if now - t < AUTO_WINDOW]
        self._attempts[action] = win
        if win and now - win[-1] < COOLDOWN:
            return False
        return len(win) < AUTO_CAP

    def _record(self, action, ok, note=""):
        self._attempts.setdefault(action, []).append(time.monotonic())
        self.history.append({"t": time.time(), "action": action, "ok": ok, "note": note})
        self.log("[doctor] %s: %s%s" % (action, "ok" if ok else "FAILED", (" " + note) if note else ""))

    # ---- auto actions: no sudo, idempotent ----
    def _restart_stream(self):
        if not self.streamer:
            return False, "no streamer"
        ok, msg = self.streamer.start()
        return ok, msg

    def _pip_onnxruntime(self):
        try:
            # keep numpy < 2 so SLAM keeps working (onnxruntime would pull numpy 2 otherwise)
            subprocess.run([sys.executable, "-m", "pip", "install", "--no-input", "numpy>=1.21.6,<2", "onnxruntime"],
                           check=True, capture_output=True, timeout=600)
            return True, "installed"
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)[-160:]

    async def _wake(self):
        if self.power:
            await self.power.wake()
            return True, "awake"
        return False, "no power controller"

    async def heal(self):
        for case in self.cases:
            for rec in case.recoveries:
                if rec.safety != "auto" or rec.action not in self.actions:
                    continue
                if not self._allowed(rec.action):
                    continue
                fn = self.actions[rec.action]
                try:
                    result = await fn() if asyncio.iscoroutinefunction(fn) else fn()
                    ok, note = result if isinstance(result, tuple) else (bool(result), "")
                except Exception as exc:  # noqa: BLE001
                    ok, note = False, str(exc)[-160:]
                self._record(rec.action, ok, note)

    async def run(self, period=5.0):
        while True:
            await asyncio.sleep(period)
            try:
                self.refresh()
                await self.heal()
            except Exception as exc:  # noqa: BLE001 -- the monitor must never die
                self.log("[doctor] loop error: %r" % exc)
