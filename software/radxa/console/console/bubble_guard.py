"""Sends the lidar safety bubble to the controller (BG) at 10 Hz.

BG,<en>,<forward cap m/s>,<reverse cap m/s> limits the drive reference of every
source (gamepad, gestures, host drive and the navigation lease). The firmware
fails closed: an enabled bubble without a fresh BG within 500 ms caps both
directions to zero, and a stale lidar is sent as 0/0. Steering and balance
recovery are never limited.

The setting is persisted (default ON). It only engages after the lidar has
produced scans in this session, so a robot without a lidar still drives.
"""
from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path

STATE = Path(os.environ.get("BOT_STATE_DIR", "/var/lib/2pybot")) / "bubble.json"


class BubbleGuard:
    def __init__(self, engine, link, path=STATE):
        self.engine = engine
        self.link = link
        self.path = Path(path)
        self.enabled = True
        self._sent_enabled = False
        self._was_connected = False
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
        return self.link.derived.get("BUBBLE", 0) == 1

    def lidar_seen(self):
        return self.engine.last_scan > 0

    def status(self):
        info = self.engine.bubble.status()
        active = bool(self.enabled and self.supported() and self.link.connected and self.lidar_seen())
        return dict(info, enabled=self.enabled, firmware_supported=self.supported(), active=active,
                    lidar_seen=self.lidar_seen(), last_sent=self.last_sent)

    async def publish(self):
        while True:
            try:
                info = self.status()
                if self.link.connected:
                    if info["active"]:
                        line = f"BG,1,{info['forward_cap']:.3f},{info['reverse_cap']:.3f}"
                        self.link.send(line)
                        self.last_sent = line
                        self._sent_enabled = True
                    elif self._sent_enabled or not self._was_connected:
                        if self.supported():
                            self.link.send("BG,0,0,0")
                            self.last_sent = "BG,0,0,0"
                        self._sent_enabled = False
                self._was_connected = self.link.connected
            except (OSError, RuntimeError, ValueError):
                pass
            await asyncio.sleep(0.1)
