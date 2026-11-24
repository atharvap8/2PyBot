"""Shared data collection and human-readable formatting for `bot` and `2pybot`.

Everything here is read-only: it gathers the console API, a few system facts
and the persistent controller log, and turns them into labelled values with a
tone (ok / warn / err / info / dim). The CLI prints them with ANSI colours, the
TUI draws them in panels. Python 3.9, standard library only.
"""
import json
import os
import re
import shutil
import subprocess
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

SERVICES = [
    ("console", "2pybot-console.service", "Web console, telemetry, SLAM, autonomy"),
    ("camera", "2pybot-mediamtx.service", "Video server (WebRTC)"),
    ("firmware", "2pybot-firmware.service", "Controller firmware worker"),
    ("hotspot", "2pybot-hotspot.service", "Robot Wi-Fi hotspot"),
    ("imx708", "2pybot-imx708.service", "Camera power-up"),
    ("boot", "2pybot-boot-gate.service", "Holds the controller until the host is ready"),
    ("deploy", "2pybot-deploy.service", "Software deployment"),
]
CONTROLLER_LOG = Path(os.environ.get("BOT_LOG_DIR", "/var/lib/2pybot/logs")) / "controller.log"
DEPLOY_CONFIG = Path("/etc/2pybot-deploy.json")


class Api:
    def __init__(self, base="http://127.0.0.1:8080"):
        self.base = base.rstrip("/")

    def call(self, path, body=None, timeout=5):
        request = Request(self.base + path, data=None if body is None else json.dumps(body).encode(),
                          headers={"Content-Type": "application/json"})
        try:
            with urlopen(request, timeout=timeout) as response:
                return json.load(response)
        except HTTPError as error:
            text = error.read().decode(errors="replace")
            try:
                text = json.loads(text).get("detail") or text
            except ValueError:
                pass
            raise RuntimeError(str(text)) from error
        except (URLError, OSError) as error:
            raise RuntimeError("console is not answering (" + str(getattr(error, "reason", error)) + ")") from error

    def get(self, path, timeout=3):
        try:
            return self.call(path, timeout=timeout)
        except (RuntimeError, ValueError):
            return None


def sh(command, timeout=4):
    try:
        out = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        return out.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


# ------------------------------------------------------------------ collection
def system_info():
    info = {}
    try:
        up = float(Path("/proc/uptime").read_text().split()[0])
        info["uptime"] = "{}d {}h {}m".format(int(up // 86400), int(up % 86400 // 3600), int(up % 3600 // 60))
        info["load"] = Path("/proc/loadavg").read_text().split()[0]
        mem = {l.split(":")[0]: int(l.split()[1]) for l in Path("/proc/meminfo").read_text().splitlines() if l.split()[1:]}
        info["mem_used"] = (mem["MemTotal"] - mem["MemAvailable"]) / 1048576
        info["mem_total"] = mem["MemTotal"] / 1048576
    except (OSError, KeyError, ValueError, IndexError):
        pass
    try:
        du = shutil.disk_usage("/")
        info["disk_used"], info["disk_total"] = du.used / 1e9, du.total / 1e9
    except OSError:
        pass
    temps = []
    for zone in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            temps.append(int(zone.read_text()) / 1000)
        except (OSError, ValueError):
            pass
    if temps:
        info["temp"] = max(temps)
    return info


def network_info():
    net = {"ips": {}, "devices": []}
    for line in sh(["ip", "-4", "-br", "addr"]).splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] not in ("lo",) and not parts[0].startswith(("docker", "br-")):
            net["ips"][parts[0]] = parts[2].split("/")[0]
    for line in sh(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"]).splitlines():
        f = line.replace("\\:", "\x00").split(":")
        if len(f) == 4 and f[1] in ("wifi", "ethernet"):
            net["devices"].append([x.replace("\x00", ":") for x in f])
    stations = sh(["iw", "dev", "wlan1", "station", "dump"])
    net["hotspot_clients"] = stations.count("Station ")
    return net


def service_states():
    names = [s[1] for s in SERVICES]
    active = sh(["systemctl", "is-active"] + names).splitlines()
    enabled = sh(["systemctl", "is-enabled"] + names).splitlines()
    out = []
    for i, (short, unit, desc) in enumerate(SERVICES):
        out.append({"name": short, "unit": unit, "desc": desc,
                    "active": active[i] if i < len(active) else "?", "enabled": enabled[i] if i < len(enabled) else "?"})
    return out


def log_tail(lines=400, path=CONTROLLER_LOG):
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - lines * 220))
            data = f.read().decode("utf-8", "replace").splitlines()
    except OSError:
        return []
    out = []
    for line in data[-lines:]:
        if line.count("\ufffd") > 4:                     # ESP32 boot ROM noise
            tail = line[line.rfind("\ufffd") + 1:]
            line = line[:24] + tail[tail.find("["):] if "[" in tail else line[:24] + "(boot noise)"
        out.append(line)
    return out


EVENT_PATTERNS = [
    ("falls", re.compile(r"FELL|RATE SPIKE")),
    ("motor faults", re.compile(r"\[HUB\] \w+ fault 0x(?!00)")),
    ("ignored corrupt frames", re.compile(r"ignored unconfirmed fault")),
    ("controller resets", re.compile(r"\[BOOT\] Reset reason")),
    ("link losses", re.compile(r"telemetry LOST")),
    ("mission stops", re.compile(r"\[AUTONOMY\].*(aborted|paused)")),
]


def event_summary(hours=24):
    since = time.time() - hours * 3600
    counts = {name: 0 for name, _ in EVENT_PATTERNS}
    last = {}
    for line in log_tail(6000):
        try:
            t = time.mktime(time.strptime(line[:19], "%Y-%m-%d %H:%M:%S"))
        except ValueError:
            continue
        if t < since:
            continue
        for name, rx in EVENT_PATTERNS:
            if rx.search(line):
                counts[name] += 1
                last[name] = line[11:19] + "  " + line[24:].strip()[:70]
    return counts, last


class Collector:
    """Snapshot of everything the dashboard shows. Slow parts are cached."""

    def __init__(self, api):
        self.api = api
        self.cache = {}
        self.stamp = {}

    def _every(self, key, seconds, fn):
        now = time.monotonic()
        if key not in self.cache or now - self.stamp.get(key, 0) >= seconds:
            try:
                self.cache[key] = fn()
            except Exception:  # noqa: BLE001 - never break the display
                self.cache[key] = None
            self.stamp[key] = now
        return self.cache[key]

    def snapshot(self):
        a = self.api
        return {
            "status": a.get("/api/status"),
            "bubble": self._every("bubble", 1, lambda: a.get("/api/bubble")),
            "auto": self._every("auto", 2, lambda: a.get("/api/autonomy/mission")),
            "slam": self._every("slam", 2, lambda: a.get("/slam/api/state", timeout=4)),
            "fw": self._every("fw", 5, lambda: a.get("/api/firmware/status")),
            "deploy": self._every("deploy", 5, lambda: a.get("/api/deployment/status")),
            "sys": self._every("sys", 3, system_info),
            "net": self._every("net", 10, network_info),
            "services": self._every("services", 5, service_states),
            "config": self._every("config", 30, lambda: read_json(DEPLOY_CONFIG, {})),
            "time": time.strftime("%H:%M:%S"),
        }


# ------------------------------------------------------------------ formatting
def f2(v, d=2, unit=""):
    try:
        return "{:.{}f}{}".format(float(v), d, unit)
    except (TypeError, ValueError):
        return "-"


def robot_state(s):
    st = s.get("status")
    if not st:
        return "CONSOLE OFFLINE", "err"
    tel = st.get("telemetry", {})
    if not st.get("connected") or tel.get("stale", True):
        return "NO CONTROLLER LINK", "err"
    if tel.get("firmware_lock"):
        return "FIRMWARE UPDATE", "warn"
    if not tel.get("startup_ready"):
        return "STARTING (" + str(tel.get("startup_state", "")).lower() + ")", "warn"
    if tel.get("self_righting"):
        return "SELF-UPRIGHTING", "info"
    if (st.get("host") or {}).get("asleep"):
        return "POWERED DOWN (wake from console)", "warn"
    m = (s.get("auto") or {}).get("mission") or {}
    if tel.get("balancing"):
        return ("AUTOPILOT: " + m.get("name", "")) if m.get("state") == "running" else "BALANCING", "info"
    return "READY, disarmed", "ok"


def short(commit):
    return (commit or "")[:8] or "-"


def dashboard(s):
    """[(section, [(label, value, tone)])]"""
    st = s.get("status") or {}
    tel, dbg, hub = st.get("telemetry", {}), st.get("debug", {}), (st.get("telemetry", {}) or {}).get("hub") or {}
    state, tone = robot_state(s)
    sections = []
    pitch = dbg.get("pF", tel.get("pitch"))
    rows = [("State", state, tone),
            ("Pitch / speed", "{} / {}".format(f2(pitch, 1, "°"), f2(tel.get("v_ms"), 2, " m/s")), None),
            ("Controller link", f2(tel.get("rx_hz"), 0, " Hz") if st else "-", "ok" if tel.get("rx_hz", 0) > 30 else "warn")]
    m = (s.get("auto") or {}).get("mission")
    if m:
        rows.append(("Mission", "{} · {}{}".format(m.get("name"), m.get("state"), (" (" + m["reason"] + ")") if m.get("reason") else ""),
                     "info" if m.get("state") == "running" else "dim"))
    sections.append(("Robot", rows))
    if hub:
        def wheel(side):
            fault = int(hub.get("fault_" + side, 0) or 0)
            return ("{} A cmd {}  {:.0f}°C".format(f2(hub.get("current_" + side), 2), f2(hub.get("current_cmd_" + side), 2),
                                                  hub.get("temp_" + side, 0)) + ("  FAULT 0x%02X" % fault if fault else ""),
                    "err" if fault else ("warn" if hub.get("temp_" + side, 0) > 60 else "ok"))
        lv, lt = wheel("l")
        rv, rt = wheel("r")
        sections.append(("Motors", [("Left", lv, lt), ("Right", rv, rt),
                                    ("Link / mode", ("ok" if hub.get("link_ok") else "LOST") + " / " + ("current" if hub.get("mode_ok") else "NOT READY"),
                                     "ok" if hub.get("link_ok") and hub.get("mode_ok") else "err")]))
    b = s.get("bubble") or {}
    if b:
        names = {"clear": "clear", "limiting": "slowing", "front_blocked": "blocked ahead", "rear_blocked": "blocked behind",
                 "boxed_in": "boxed in", "stale": "no lidar data"}
        btone = {"clear": "ok", "limiting": "warn", "stale": "dim"}.get(b.get("state"), "err")
        sections.append(("Safety bubble", [
            ("State", names.get(b.get("state"), b.get("state", "-")), btone),
            ("Free ahead / behind", "-" if b.get("stale") else "{} / {}".format(f2(b.get("front_m"), 2, " m") if b.get("front_m") is not None else "clear",
                                                     f2(b.get("back_m"), 2, " m") if b.get("back_m") is not None else "clear"), None),
            ("Limits gamepad", "yes" if b.get("enabled") else "no (missions only)", "ok" if b.get("enabled") else "dim")]))
    sl = s.get("slam") or {}
    tof = st.get("tof") or {}
    stream = st.get("stream") or {}
    sens = []
    if sl:
        hz = (sl.get("rates") or {}).get("scan", 0)
        sens.append(("Lidar", "{:.0f} Hz · {}{}".format(hz, {"idle": "not mapping", "mapping": "mapping", "localizing": "on map"}.get(sl.get("mode"), sl.get("mode")),
                                                         (" '" + sl["map_name"] + "'") if sl.get("map_name") else ""), "ok" if hz > 15 else "warn"))
    if tof:
        sens.append(("Depth sensor", tof.get("state", "-") + (" · " + tof["error"] if tof.get("error") else ""), "ok" if tof.get("state") == "ranging" else "warn"))
    if stream:
        fps = (stream.get("metrics") or {}).get("fps")
        sens.append(("Camera", "{}×{} · {} fps".format((stream.get("mode") or {}).get("w", "?"), (stream.get("mode") or {}).get("h", "?"), fps or "-")
                     if stream.get("running") else "stopped " + str(stream.get("error", "")), "ok" if stream.get("running") else "err"))
    if sens:
        sections.append(("Sensors", sens))
    fw, dep, cfg = s.get("fw") or {}, s.get("deploy") or {}, s.get("config") or {}
    inst, latest = (fw.get("installed") or {}), (fw.get("latest") or {})
    prog = dep.get("progress") or {}
    soft = [("Firmware", inst.get("version", "-") + ("" if inst.get("build_id") == latest.get("build_id") else "  (newer build " + latest.get("version", "?") + " ready)"),
             "ok" if inst.get("build_id") and inst.get("build_id") == latest.get("build_id") else "warn"),
            ("Branch", cfg.get("branch", "-"), None),
            ("Running commit", short((dep.get("applied") or {}).get("commit")), None),
            ("Last deployment", "{} · {}".format(prog.get("phase", "-"), (prog.get("message") or "")[:46]),
             {"success": "ok", "failed": "err"}.get(prog.get("phase"), "warn"))]
    sections.append(("Software", soft))
    sysi, net = s.get("sys") or {}, s.get("net") or {}
    host = st.get("host") or {}
    hrow = [("Host ready", "yes" if host.get("ready") else "no ({})".format(", ".join(k for k in ("camera", "hotspot") if not host.get(k)) or "?"),
             "ok" if host.get("ready") else "warn")]
    wifi = [d for d in net.get("devices", []) if d[0] == "wlan0"]
    if wifi:
        hrow.append(("Wi-Fi", "{} · {}".format(wifi[0][3] or wifi[0][2], net.get("ips", {}).get("wlan0", "no IP")),
                     "ok" if wifi[0][2] == "connected" else "warn"))
    hrow.append(("Hotspot", "{} · {} client(s)".format(net.get("ips", {}).get("wlan1", "off"), net.get("hotspot_clients", 0)),
                 "ok" if "wlan1" in net.get("ips", {}) else "warn"))
    if "temp" in sysi:
        hrow.append(("CPU temp / load", "{:.0f}°C / {}".format(sysi["temp"], sysi.get("load", "-")), "warn" if sysi["temp"] > 75 else None))
    if "disk_used" in sysi:
        hrow.append(("Disk / memory", "{:.0f}/{:.0f} GB  ·  {:.1f}/{:.1f} GB".format(sysi["disk_used"], sysi["disk_total"],
                                                                                   sysi.get("mem_used", 0), sysi.get("mem_total", 0)),
                     "warn" if sysi["disk_used"] / max(1, sysi["disk_total"]) > 0.85 else None))
    hrow.append(("Uptime", sysi.get("uptime", "-"), None))
    sections.append(("Host", hrow))
    svc = s.get("services") or []
    down = [x["name"] for x in svc if x["active"] != "active" and x["name"] not in ("deploy", "boot", "imx708")]
    sections.append(("Services", [("2PyBot services", "all running" if not down else "down: " + ", ".join(down), "ok" if not down else "err")]))
    return sections


def log_tone(line):
    if re.search(r"FELL|RATE SPIKE|fault 0x(?!00)|Motors OFF|LOST|ERROR|FATAL|failed|PE,", line):
        return "err"
    if re.search(r"ignored|WARN|paused|aborted|unavailable|Invalid", line):
        return "warn"
    if re.search(r"\[AUTONOMY\]|\[NAV\]|BALANCING|READY|restored", line):
        return "info"
    if re.search(r"\[PAD\] btn=|\[console\] opened", line):
        return "dim"
    return None


# ------------------------------------------------------------------ ANSI output for the CLI
ANSI = {"ok": "\033[32m", "warn": "\033[33m", "err": "\033[31m", "info": "\033[36m", "dim": "\033[90m",
        "acc": "\033[38;5;208m", "bold": "\033[1m", "": "\033[0m"}


def colour(text, tone, enabled):
    return (ANSI.get(tone, "") + text + ANSI[""]) if enabled and tone else text


def print_sections(sections, use_colour=True, out=print):
    width = max((len(lbl) for _, rows in sections for lbl, _, _ in rows), default=10) + 2
    for title, rows in sections:
        out(colour(title.upper(), "acc", use_colour))
        for label, value, tone in rows:
            out("  " + label.ljust(width) + colour(str(value), tone, use_colour))
        out("")


def table(headers, rows, use_colour=True, tones=None, out=print):
    widths = [max(len(str(h)), *(len(str(r[i])) for r in rows)) if rows else len(str(h)) for i, h in enumerate(headers)]
    out(colour("  ".join(str(h).ljust(w) for h, w in zip(headers, widths)), "dim", use_colour))
    for n, r in enumerate(rows):
        line = "  ".join(str(c).ljust(w) for c, w in zip(r, widths))
        out(colour(line, (tones or {}).get(n), use_colour))
