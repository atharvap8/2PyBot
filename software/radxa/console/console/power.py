"""Console power-down mode: switch off what heats the robot while it is not used.

Asleep:
  * camera pipeline stopped (sensor, ISP and hardware encoder idle)
  * lidar scanning stopped and its motor spun down
  * depth sensor ranging stopped
  * object detection off, autonomy stopped
  * controller (4.7.7+ firmware, capability SLEEP): arming refused, LED ring
    reduced to one faint breathing LED, camera servos unpowered, torch off,
    gamepad disconnected and new Bluetooth connections refused

The Radxa itself stays up with the console and Wi-Fi running: it is the only
way to wake the robot again (from the web console, the app or `2pybot`). A
Linux suspend would take Wi-Fi down with it and this board has no wake-on-LAN.

The state is deliberately not persisted: a console restart, a deployment or a
reboot comes back awake, so update health checks always see the full system.
"""
import asyncio
import time


class Power:
    def __init__(self, link, streamer, lidar, tof, autonomy, detector, log):
        self.link, self.streamer, self.lidar, self.tof = link, streamer, lidar, tof
        self.autonomy, self.detector, self.log = autonomy, detector, log
        self.asleep = False
        self.since = None
        self.busy = False
        self.detector_was_on = False
        self.detector_cfg = (.50, False, True)
        self.lidar_enabled = True          # BOT_LIDAR=0 bench mode keeps the lidar off when waking
        self._last_sync = 0.0

    def controller_supported(self):
        return self.link.derived.get("SLEEP") == 1

    def status(self):
        return {"asleep": self.asleep, "busy": self.busy,
                "since": self.since, "asleep_s": round(time.time() - self.since) if self.asleep and self.since else 0,
                "controller_supported": self.controller_supported(),
                "controller_asleep": bool(self.link.tel.get("asleep")),
                "camera": "off" if self.streamer.paused else ("on" if self.streamer.running else "starting"),
                "lidar": self.lidar.status.get("state"),
                "tof": self.tof.state}

    async def sleep(self):
        if self.asleep:
            return self.status()
        if self.link.tel.get("balancing") or self.link.tel.get("self_righting"):
            raise ValueError("Press SELECT to stop the motors before powering down")
        self.busy = True
        try:
            self.asleep, self.since = True, time.time()
            self.log("[console] POWER DOWN: camera, lidar, depth sensor and controller outputs off")
            try:
                self.autonomy.stop("power_down")
            except Exception:  # noqa: BLE001 -- nothing may block the power-down
                pass
            self.detector_was_on = bool(getattr(self.detector, "enabled", False))
            if self.detector_was_on:
                d = self.detector
                self.detector_cfg = (d.threshold, d.people_only, d.corrected)
                try:
                    self.detector.configure(False)
                except Exception:  # noqa: BLE001
                    pass
            if self.controller_supported():
                self.link.send("SL,1")
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, self.streamer.pause, True)
            await loop.run_in_executor(None, self.lidar.stop)
            await self.tof.pause(True)
        finally:
            self.busy = False
        return self.status()

    async def wake(self):
        if not self.asleep:
            return self.status()
        self.busy = True
        try:
            self.asleep, self.since = False, None
            self.log("[console] WAKE: camera, lidar, depth sensor and controller back on")
            if self.controller_supported():
                self.link.send("SL,0")
            self.streamer.pause(False)            # the supervisor restarts the camera within a second
            if self.lidar_enabled:
                self.lidar.start()
            await self.tof.pause(False)
            if self.detector_was_on:
                try:
                    self.detector.configure(True, *self.detector_cfg)
                except Exception:  # noqa: BLE001 -- detection is optional
                    pass
                self.detector_was_on = False
        finally:
            self.busy = False
        return self.status()

    async def run(self):
        """Keep the controller in step: it boots awake after a reset, and a console
        restart while it slept must wake it."""
        while True:
            await asyncio.sleep(1.0)
            link = self.link
            if self.busy or not link.connected or not self.controller_supported():
                continue
            if link.tel.get("startup_state") not in (None, "READY"):
                continue
            want, have = self.asleep, bool(link.tel.get("asleep"))
            if want != have and time.monotonic() - self._last_sync > 3:
                self._last_sync = time.monotonic()
                link.send("SL,1" if want else "SL,0")
