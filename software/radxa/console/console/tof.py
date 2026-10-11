"""Asynchronous depth telemetry and an opt-in forward-reference guard."""
import asyncio
import json
import math
import os
import sys
import time
from pathlib import Path


class ToF:
    def __init__(self):
        self.process = None
        self.frame = {}
        self.received = None
        self.error = ""
        self.state = "starting"
        self.guard_enabled = False  # never persisted; explicit opt-in each startup
        self.stop_m = 0.35
        self.slow_m = 0.80
        self._was_connected = False
        self._sent_enabled = False

    def accept(self, frame):
        if frame.get("state") != "ranging":
            self.state = frame.get("state", "error")
            self.error = str(frame.get("message", ""))
            return
        fields = ("distance_mm", "status", "sigma_mm", "targets")
        if any(len(frame.get(k, [])) != 64 for k in fields):
            raise ValueError("Depth frame must contain 64 zones")
        if any(not isinstance(v, (int, float)) or not math.isfinite(v)
               for k in fields for v in frame[k]):
            raise ValueError("Invalid depth value")
        frame["valid"] = [
            frame["status"][i] == 5 and frame["targets"][i] >= 1
            and 50 <= frame["distance_mm"][i] <= 4000
            and frame["sigma_mm"][i] <= 20 for i in range(64)
        ]
        self.frame = frame
        self.received = time.monotonic()
        self.state, self.error = "ranging", ""

    def status(self, link=None):
        age = None if self.received is None else max(0, time.monotonic() - self.received)
        stale = self.state != "ranging" or age is None or age > 0.5
        valid = self.frame.get("valid", [False] * 64)
        distance = self.frame.get("distance_mm", [0] * 64)
        # Native central 4x6 ROI; symmetric until mounting orientation is measured.
        roi = [r * 8 + c for r in range(2, 6) for c in range(1, 7)]
        usable = [distance[i] / 1000 for i in roi if valid[i]]
        nearest = min(usable) if usable and not stale else None
        # Missing central zones are unknown, not clear; enabled guard fails closed.
        observed = not stale and all(valid[i] for i in roi)
        if not observed:
            fraction, reason = 0.0, "stale or unknown depth"
        else:
            fraction = max(0.0, min(1.0, (nearest - self.stop_m) / (self.slow_m - self.stop_m)))
            reason = "blocked" if fraction == 0 else ("slowing" if fraction < 1 else "clear")
        supported = bool(link and link.derived.get("TOFGUARD", 0) == 1)
        active = bool(self.guard_enabled and supported and link.connected and not link.tel.get("stale", True))
        max_speed = link.params["DRIVEV"].value if link and "DRIVEV" in link.params else 0.7
        cap = round(fraction * max_speed, 3)
        return {
            "state": self.state, "error": self.error,
            "bus": int(os.environ.get("BOT_TOF_BUS", "7")),
            "address": hex(int(os.environ.get("BOT_TOF_ADDRESS", "0x29"), 0)),
            "ranging_hz": int(os.environ.get("BOT_TOF_HZ", "10")),
            "integration_ms": int(os.environ.get("BOT_TOF_INTEGRATION_MS", "20")),
            "resolution": 8, "stale": stale,
            "age_ms": round(age * 1000) if age is not None else None,
            "distance_mm": distance, "valid": valid,
            "status": self.frame.get("status", []),
            "valid_count": sum(valid) if not stale else 0,
            "quality_note": ("All zones currently fail status/range/sigma checks; raw distances are shown. "
                             "Enabled guard treats this as unknown depth."
                             if not stale and not any(valid) else ""),
            "sigma_mm": self.frame.get("sigma_mm", []),
            "nearest_m": nearest, "guard_enabled": self.guard_enabled,
            "guard_supported": supported, "guard_active": active,
            "guard_reason": reason if self.guard_enabled else "off",
            "forward_limit_mps": cap if self.guard_enabled else None,
            "stop_m": self.stop_m, "slow_m": self.slow_m,
        }

    async def acquire(self):
        worker = str(Path(__file__).with_name("tof_worker.py"))
        try:
            while True:
                try:
                    self.state = "starting"
                    self.process = await asyncio.create_subprocess_exec(
                        sys.executable, "-u", worker, stdout=asyncio.subprocess.PIPE,
                        stderr=asyncio.subprocess.STDOUT, limit=16384)
                    while True:
                        timeout = 50 if self.state in ("starting", "initializing") else 4
                        raw = await asyncio.wait_for(self.process.stdout.readline(), timeout)
                        if not raw:
                            raise RuntimeError(f"Depth worker exited ({await self.process.wait()})")
                        text = raw.decode(errors="replace").strip()
                        if text.startswith("{"):
                            self.accept(json.loads(text))
                            if self.state == "error":
                                raise RuntimeError(self.error)
                        else:
                            self.error = text[-300:]
                except (OSError, RuntimeError, ValueError, asyncio.TimeoutError) as exc:  # noqa: UP041 (Python 3.9)
                    self.state, self.error = "error", str(exc) or "Depth worker timed out"
                finally:
                    await self.stop()
                await asyncio.sleep(5)
        finally:
            await self.stop()

    async def stop(self):
        proc, self.process = self.process, None
        if proc and proc.returncode is None:
            proc.terminate()
            try:
                await asyncio.wait_for(proc.wait(), 3)
            except asyncio.TimeoutError:  # noqa: UP041 (Python 3.9)
                proc.kill()
                await proc.wait()

    async def publish(self, link):
        while True:
            info = self.status(link)
            if link.connected:
                if self.guard_enabled and info["guard_supported"]:
                    link.send(f"TG,1,{info['forward_limit_mps']:.3f}")
                    self._sent_enabled = True
                elif self._sent_enabled or not self._was_connected:
                    # Reset a previous session's guard on reconnect/startup.
                    link.send("TG,0,0")
                    self._sent_enabled = False
            self._was_connected = link.connected
            link._publish({"type": "tof", "tof": info})
            await asyncio.sleep(0.1)


tof = ToF()
