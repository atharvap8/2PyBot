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
        self._hits = [0] * 64
        self._previous = [None] * 64
        self.half_width_m = 0.14  # shell/wheels plus uncertainty margin
        self.reaction_s = 0.25
        self.deceleration_mps2 = 0.7
        self.partial_speed_mps = 0.25
        self.sensor_height_m = float(os.environ.get("BOT_TOF_HEIGHT_M", "0.20"))

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
            frame["status"][i] in (5, 9) and frame["targets"][i] >= 1
            and 50 <= frame["distance_mm"][i] <= 4000
            and 0 <= frame["sigma_mm"][i] <= 20 for i in range(64)
        ]
        continuous = self.state == "ranging" and self.received is not None and time.monotonic() - self.received < 0.5
        for i in range(64):
            distance = frame["distance_mm"][i]
            repeat = continuous and self._previous[i] is not None and abs(distance - self._previous[i]) <= max(100, distance * .1)
            self._hits[i] = min(5, self._hits[i] + 1) if frame["valid"][i] and repeat else int(frame["valid"][i])
            self._previous[i] = distance if frame["valid"][i] else None
        self.frame = frame
        self.received = time.monotonic()
        self.state, self.error = "ranging", ""

    def status(self, link=None):
        age = None if self.received is None else max(0, time.monotonic() - self.received)
        stale = self.state != "ranging" or age is None or age > 0.5
        valid = self.frame.get("valid", [False] * 64)
        distance = self.frame.get("distance_mm", [0] * 64)
        # Confidence/coverage is measured independently of collision geometry.
        roi = [r * 8 + c for r in range(2, 6) for c in range(1, 7)]
        coverage = sum(valid[i] for i in roi) / len(roi) if not stale else 0.0
        points, candidates = [], []
        speed = max(0.0, float(link.tel.get("v_ms", 0))) if link else 0.0
        debug = getattr(link, "debug", {}) if link else {}
        steps_m = getattr(link, "derived", {}).get("STEPSM", 0) if link else 0
        yaw_rate = 2 * float(debug.get("steer", 0)) / steps_m / .155 if steps_m > 0 else 0.0
        # Mounting handedness is uncalibrated: use a symmetric swept-turn envelope.
        curvature = min(2.0, abs(yaw_rate) / max(.15, speed))
        pitch = math.radians(float(link.tel.get("pitch", 0))) if link else 0.0
        if not stale:
            for i in range(64):
                if not valid[i]:
                    continue
                r, c = divmod(i, 8)
                d = distance[i] / 1000
                horizontal = math.radians((c - 3.5) * 45 / 8)
                vertical = math.radians((3.5 - r) * 45 / 8)
                x = d * math.tan(horizontal)
                y = d * math.cos(pitch) - d * math.tan(vertical) * math.sin(pitch)
                z = self.sensor_height_m + d * math.sin(pitch) + d * math.tan(vertical) * math.cos(pitch)
                adjacent = any(valid[j] and abs(distance[j] - distance[i]) <= max(100, distance[i] * .1)
                               for j in (i - 8, i + 8, i - 1, i + 1)
                               if 0 <= j < 64 and abs(j % 8 - c) <= 1)
                confirmed = adjacent or self._hits[i] >= 2 or d <= self.stop_m
                confidence = min(1.0, .4 + .15 * self._hits[i] + .2 * int(adjacent))
                if self.frame["status"][i] == 9:
                    confidence *= .8
                width = self.half_width_m + min(.30, .5 * curvature * y * y)
                # Include each zone's angular footprint instead of only its centre.
                in_path = y > 0 and abs(x) <= width + d * math.tan(math.radians(45 / 16))
                points.append({"zone": i, "x_m": round(x, 3), "y_m": round(y, 3),
                               "z_m": round(z, 3), "confidence": round(confidence, 2),
                               "confirmed": confirmed, "in_path": in_path})
                if in_path and confirmed:
                    candidates.append(y)
        nearest = min(candidates) if candidates else None
        max_speed = link.params["DRIVEV"].value if link and "DRIVEV" in link.params else 0.7
        stopping_distance = self.stop_m + speed * self.reaction_s + speed * speed / (2 * self.deceleration_mps2)
        if stale or not points:
            cap, reason = 0.0, "stale or unknown depth"
        elif nearest is not None and nearest <= self.stop_m:
            cap, reason = 0.0, "obstacle in stopping corridor"
        else:
            # v*t + v²/(2a) <= measured clearance; balance recovery is independent.
            clearance = max(0.0, nearest - self.stop_m) if nearest is not None else 0.0
            braking_cap = math.sqrt((self.deceleration_mps2 * self.reaction_s) ** 2 +
                                    2 * self.deceleration_mps2 * clearance) - self.deceleration_mps2 * self.reaction_s
            cap = min(max_speed, braking_cap) if nearest is not None else max_speed
            if coverage < .5 or nearest is None:
                cap = min(cap, self.partial_speed_mps if candidates else .15)
                reason = "partial coverage — reduced speed"
            else:
                reason = "slowing for stopping clearance" if cap < max_speed else "clear observed corridor"
        supported = bool(link and link.derived.get("TOFGUARD", 0) == 1)
        active = bool(self.guard_enabled and supported and link.connected and not link.tel.get("stale", True))
        cap = math.floor(max(0.0, cap) * 1000) / 1000
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
            "targets": self.frame.get("targets", []),
            "points": points, "coverage": round(coverage, 3),
            "corridor_half_width_m": self.half_width_m,
            "turn_envelope_curvature": round(curvature, 3),
            "stopping_distance_m": round(stopping_distance, 3),
            "reaction_s": self.reaction_s, "deceleration_mps2": self.deceleration_mps2,
            "projection_calibrated": False,
            "projection_note": "Approximate 45° optics / 0.20 m mount; symmetric steering envelope until mounting calibration",
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
