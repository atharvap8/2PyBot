"""Sends the lidar safety bubble to the controller (BG) on every scan (~20 Hz).

v2 protocol (firmware 4.7.0, PR,BUBBLE,2):
    BG,<en>,<front_m>,<back_m>,<flags>,<age_ms>
front/back are free distances inside the swept corridor of the current path;
flags bit0/bit1 block turning left/right; age_ms is the scan age at send time.
The controller converts distances to speed limits every control cycle and
subtracts its own wheel travel since the scan. An enabled bubble fails closed
in the firmware (no fresh BG within BUBSTALE ms -> zero clearance). A stale
lidar is sent as zero clearance.

The setting is persisted (default ON) and only engages after the lidar has
produced scans in this session, so a robot without a lidar still drives.
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import time
from pathlib import Path

STATE = Path(os.environ.get("BOT_STATE_DIR", "/var/lib/2pybot")) / "bubble.json"
CLEAR = 9.99                                     # "nothing in the corridor" (m)


class BubbleGuard:
    def __init__(self, engine, link, path=STATE):
        self.engine = engine
        self.link = link
        self.path = Path(path)
        self.enabled = True
        self._sent_enabled = False
        self._was_connected = False
        self._last_t = None
        self._last_send = 0.0
        self.last_sent = ""
        try:
            self.enabled = bool(json.loads(self.path.read_text()).get("enabled", True))
        except (OSError, ValueError):
            pass

    def set_enabled(self, value):
        self.enabled = bool(value)
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"enabled": self.enabled}))
            os.replace(tmp, self.path)
        except OSError:
            pass
        self.link._log("[console] lidar safety bubble " + ("ENABLED" if self.enabled else "DISABLED by operator"))
        return self.status()

    def supported(self):
        return self.link.derived.get("BUBBLE", 0) >= 2

    def lidar_seen(self):
        return self.engine.last_scan > 0

    def _param(self, key, default):
        try:
            return float(self.link.params[key].value)
        except (KeyError, AttributeError, TypeError, ValueError):
            return default

    def status(self):
        info = self.engine.bubble.status(decel=self._param("BUBDEC", 0.5), latency=self._param("BUBLAT", 0.3),
                                         margin=self._param("BUBMRG", 0.05))
        active = bool(self.enabled and self.supported() and self.link.connected and self.lidar_seen())
        return dict(info, enabled=self.enabled, firmware_supported=self.supported(), active=active,
                    lidar_seen=self.lidar_seen(), last_sent=self.last_sent, controller=dict(self.link.bubble or {}))

    def _line(self):
        b = self.engine.bubble
        with b.lock:
            res, t = b.result, b.t
        now = time.monotonic()
        if res is None or now - t > b.cfg["stale_s"]:
            return "BG,1,0.000,0.000,3,0", t
        f = res["front_m"] if math.isfinite(res["front_m"]) else CLEAR
        k = res["back_m"] if math.isfinite(res["back_m"]) else CLEAR
        flags = (0 if res["turn_left_ok"] else 1) | (0 if res["turn_right_ok"] else 2)
        age = int(min(999, max(0, (now - t) * 1000)))
        return f"BG,1,{min(f, CLEAR):.3f},{min(k, CLEAR):.3f},{flags},{age}", t

    async def publish(self):
        while True:
            try:
                connected = self.link.connected
                if connected and self.enabled and self.supported() and self.lidar_seen():
                    line, t = self._line()
                    now = time.monotonic()
                    # send each new scan at once; repeat at 10 Hz if scans stop (stays fail-closed)
                    if t != self._last_t or now - self._last_send >= 0.1:
                        self.link.send(line)
                        self.last_sent, self._last_t, self._last_send = line, t, now
                        self._sent_enabled = True
                elif connected and (self._sent_enabled or not self._was_connected):
                    if self.supported():
                        self.link.send("BG,0,0,0,0,0")
                        self.last_sent = "BG,0,0,0,0,0"
                    self._sent_enabled = False
                self._was_connected = connected
            except (OSError, RuntimeError, ValueError):
                pass
            await asyncio.sleep(0.01)
