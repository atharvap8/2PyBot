"""sysmap.py — the live system map, served on its own port (6969 by default)
and also mounted on the console at /sysmap/.

It runs inside the console process and reads the console's objects directly.
It never opens the serial port and never sends anything to the robot except
the operator stop, which uses exactly the same path as the console's E-STOP.

    GET  /                 the map page
    GET  /static/...       page assets
    GET  /api/snapshot     one health snapshot (JSON)
    WS   /ws               a snapshot every 200 ms
    POST /api/estop        operator stop (never arms)
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import time

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from .sysmap_health import EventTracker, evaluate

UI_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ui", "sysmap")
PUSH_PERIOD_S = 0.2
LOG_TAIL = 120


# ------------------------------------------------------------ host stats
def system_stats():
    out = {"cores": os.cpu_count() or 1}
    try:
        out["load1"] = os.getloadavg()[0]
    except OSError:
        pass
    temps = []
    base = "/sys/class/thermal"
    try:
        for zone in os.listdir(base):
            try:
                with open(os.path.join(base, zone, "temp")) as f:
                    temps.append(int(f.read().strip()) / 1000)
            except (OSError, ValueError):
                continue
    except OSError:
        pass
    if temps:
        out["cpu_temp_c"] = max(temps)
    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                key, _, rest = line.partition(":")
                info[key] = int(rest.split()[0])
        out["mem_used_pct"] = 100 * (1 - info["MemAvailable"] / info["MemTotal"])
    except (OSError, KeyError, ValueError, ZeroDivisionError):
        pass
    try:
        with open("/proc/uptime") as f:
            secs = int(float(f.read().split()[0]))
        out["uptime"] = f"{secs // 3600} h {secs % 3600 // 60} min"
    except (OSError, ValueError):
        pass
    return out


def _nearest_mm(tof):
    best = None
    for d, ok in zip(tof.get("distance_mm") or [], tof.get("valid") or []):
        if ok and d and (best is None or d < best):
            best = d
    return best


# ------------------------------------------------------------ app factory
def create_app(link, tof, streamer, host_ready, estop):
    """`link`, `tof` and `streamer` are the console's live objects.
    `host_ready()` and `estop()` are the console's own functions."""
    app = FastAPI(title="2PyBot system map", version="1.0")
    tracker = EventTracker()
    viewers = set()

    def collect():
        tel = link.tel
        now = time.time()
        fresh = link.connected and not tel.get("stale", True)
        tof_status = tof.status(link)
        stream_status = streamer.status()
        metrics = stream_status.get("metrics") or {}
        hub = dict(getattr(link, "hub", {}) or {})
        rx_mono = hub.pop("received_monotonic", None)
        nav = dict(getattr(link, "nav", {}) or {})
        nav["fresh"] = bool(nav.get("received_monotonic")) and time.monotonic() - nav["received_monotonic"] < 2
        events = tracker.update(link.log, tel.get("yaw"), fresh)
        last_rx = tel.get("last_rx") or 0
        serial_error = ""
        for entry in reversed(list(link.log)[-60:]):
            text = str(entry.get("text", ""))
            if text.startswith("[console] opened"):
                break
            if text.startswith(("[console] serial unavailable:", "[console] serial error:")):
                serial_error = text.split(": ", 1)[-1]
                break
        return {
            "serial": {"connected": link.connected, "maintenance": link.maintenance,
                       "stale": tel.get("stale", True), "rx_hz": tel.get("rx_hz", 0),
                       "last_rx_age": (now - last_rx) if last_rx else None,
                       "port": getattr(link.ser, "port", None) or os.environ.get("BOT_SERIAL", "/dev/ttyUSB0"),
                       "baud": getattr(link.ser, "baudrate", None) or int(os.environ.get("BOT_BAUD", "460800")),
                       "dump_complete": link.dump_complete, "error": serial_error},
            "tel": {k: tel.get(k) for k in ("pitch", "yaw", "balancing", "startup_state", "host_ready",
                                             "v_ms", "x_m", "y_m", "heading")},
            "debug": dict(link.debug),
            "hub": hub,
            "hub_age": (time.monotonic() - rx_mono) if rx_mono else None,
            "nav": nav,
            "tof": dict({k: tof_status.get(k) for k in ("state", "error", "stale", "age_ms", "valid_count",
                                                         "bus", "address", "ranging_hz")},
                        nearest_mm=_nearest_mm(tof_status)),
            "tof_grid": {"distance_mm": tof_status.get("distance_mm") or [],
                         "valid": tof_status.get("valid") or []},
            "stream": {"running": stream_status.get("running"), "error": stream_status.get("error"),
                       "encoder": stream_status.get("encoder"), "fps": metrics.get("fps"),
                       "frame": metrics.get("frame"),
                       "capture_to_publish_ms": metrics.get("capture_to_publish_ms")},
            "host": host_ready(),
            "system": system_stats(),
            "clients": {"console": len(getattr(link, "_subs", ())), "sysmap": len(viewers)},
            "events": events,
        }

    def snapshot():
        inp = collect()
        out = evaluate(inp)
        out.update(t=time.time(),
                   live={"pitch": inp["tel"].get("pitch"), "yaw": inp["tel"].get("yaw"),
                         "balancing": inp["tel"].get("balancing"),
                         "startup_state": inp["tel"].get("startup_state"),
                         "v_ms": inp["tel"].get("v_ms"), "rx_hz": inp["serial"].get("rx_hz"),
                         "port": inp["serial"]["port"], "baud": inp["serial"]["baud"]},
                   tof_grid=inp["tof_grid"],
                   log=[{"t": e.get("t"), "text": e.get("text")} for e in list(link.log)[-LOG_TAIL:]])
        return out

    def _safe(value):
        # NaN/inf would make the browser's JSON.parse fail.
        if isinstance(value, float) and not math.isfinite(value):
            return None
        if isinstance(value, dict):
            return {k: _safe(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [_safe(v) for v in value]
        return value

    @app.get("/")
    def index():
        return FileResponse(os.path.join(UI_DIR, "index.html"))

    if os.path.isdir(UI_DIR):
        app.mount("/static", StaticFiles(directory=UI_DIR), name="sysmap-static")

    @app.get("/api/snapshot")
    def api_snapshot():
        return JSONResponse(_safe(snapshot()))

    @app.post("/api/estop")
    def api_estop():
        estop()
        return {"ok": True}

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        viewers.add(sock)
        try:
            while True:
                await sock.send_text(json.dumps(_safe(snapshot()), default=str))
                await asyncio.sleep(PUSH_PERIOD_S)
        except (WebSocketDisconnect, RuntimeError, OSError):
            pass
        finally:
            viewers.discard(sock)

    return app
