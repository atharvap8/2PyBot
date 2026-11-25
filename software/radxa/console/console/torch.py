"""The flashlight comes on by itself in the dark. One simple rule, nothing clever.

While the robot is up (balancing), the console looks at the brightness of the camera
picture twice a second:

  * fully dark for a second and a half -> flashlight on at full brightness, head
    centred, beam at its widest. It is then left exactly like that.
  * the darkness is gone               -> flashlight off, and it is the gamepad's again.

"The darkness is gone" has to be judged with the light on, when the picture is no longer
dark. So when the light has settled the bot notes how bright the picture is with it, and
switches off when the picture becomes clearly brighter than that (the room lights came
on, or it drove into daylight). If it then finds itself in the dark again straight
away, that brightness was its own light on something close: it switches back on and,
for the next two minutes, only believes a washed-out picture.

There is no brightness regulation and no aiming: both made the light restless. The
firmware's clamp and its 90 % hard cap apply to every command (TL / P= / Z=). Touch the
flashlight from the gamepad and the bot takes what you set as the new state and waits a
second and a half after your last touch. Parked or powered down, the light goes off.

The camera's automatic re-exposure (camera.ExposureWatch) is told when darkness is the
light's job, so the two do not fight.
"""
from __future__ import annotations

import asyncio
import json
import os
import time

from .config import CAM_PROFILES


class AutoTorch:
    PERIOD = 0.5             # s between looks at the picture
    DARK = 25.0              # mean luma (0 to 255) below this is fully dark
    ON_DELAY = 1.5           # s of darkness before switching on
    FULL = 90                # percent: full brightness (the firmware's hard cap)
    SETTLE = 2.0             # s after switching on before the lit picture is measured (snapshots are 1 Hz)
    MIN_ON_S = 6.0           # s the light stays on once the bot switched it on
    BRIGHTER = 1.6           # the darkness is gone when the picture is this many times brighter than lit ...
    OFF_FLOOR = 150.0        # ... and at least this bright
    WASHED = 235.0           # a picture this bright always ends it
    OFF_DELAY = 1.5          # s of that before switching off
    RELAPSE_S = 4.0          # dark again this soon after switching off: it was our own light
    STRICT_S = 120.0         # then only WASHED counts, for this long
    MANUAL_HOLD_S = 1.5      # s after the operator's last touch before the bot carries on

    def __init__(self, link, measure, front_m=None, log=print, settings=None, clock=time.monotonic):
        self.link, self.measure, self.log, self.clock = link, measure, log, clock
        self.settings = settings or os.path.join(os.path.dirname(CAM_PROFILES), "auto_torch.json")
        self.enabled = True
        try:
            with open(self.settings) as f:
                self.enabled = bool(json.load(f).get("enabled", True))
        except (OSError, ValueError, AttributeError):
            pass
        self.on, self.pct = False, self.FULL            # `on`: the light is on and the bot looks after it
        self.dark_since = self.bright_since = None
        self.lit = None                                 # how bright the picture is with the light on
        self.on_at = self.off_at = -1e9
        self.bright_off_at = -1e9                       # when the bot last switched off because it got bright
        self.strict_until = 0.0
        self.hold_until = 0.0
        self._sent_at = -1e9                            # the bot's last command, to tell its echo from the gamepad
        self.last = {"luma": None, "action": "", "at": None}

    # ------------------------------------------------------------ state of the robot
    def _param(self, key, default):
        p = getattr(self.link, "params", {}).get(key)
        return float(p.value) if p is not None else float(default)

    def available(self):
        tel = self.link.tel
        return bool(self.link.connected and not tel.get("stale", True) and not tel.get("asleep")
                    and self.link.derived.get("TORCHSET") == 1)

    def active(self):
        """The light only matters, and is only touched, while the robot is up."""
        return self.available() and bool(self.link.tel.get("balancing"))

    def handles(self, side):
        """For the exposure watch: is this the light's problem right now?"""
        if not (self.enabled and self.active()) or self.clock() < self.hold_until:
            return False
        return (not self.on) if side == "dark" else self.on      # flat out and still dim: the exposure may help

    # ------------------------------------------------------------ commands
    def _switch(self, on, why):
        now = self.clock()
        if not self.link.send(f"TL,{1 if on else 0},{self.FULL}"):
            return False
        self._sent_at = now
        self.on, self.pct = bool(on), self.FULL
        self.dark_since = self.bright_since = None
        if on:
            self.on_at, self.lit = now, None
            # one fixed position: head centred, beam at its widest (the high end of the zoom servo)
            self.link.send(f"P={self._param('SVYAWHOME', 165):.0f}")
            self.link.send(f"Z={self._param('SVZMAX', 140):.0f}")
        else:
            self.off_at = now
        text = f"Flashlight on, full: {why}" if on else f"Flashlight off: {why}"
        self.last.update(action=text, at=time.time())
        self.log("[torch] " + text)
        return True

    def on_line(self, line):
        """A flashlight line from the controller that the bot did not cause is the operator on
        the gamepad. Take what they set as the state and wait for them to finish."""
        # "[PAY] Torch ON (25%)", "[PAY] Torch off", "[PAY] Torch 35%": a change of state.
        # Not the boot banner ("[PAY] Torch GPIO2 off, 25% preset, ...").
        if not line.startswith(("[PAY] Torch ON", "[PAY] Torch off")) and not (
                line.startswith("[PAY] Torch ") and line[12:13].isdigit()):
            return
        now = self.clock()
        if now - self._sent_at < 1.5:
            return                                                 # the echo of our own command
        self.on = not line.startswith("[PAY] Torch off") and not line.rstrip().endswith("(off)")
        self.dark_since = self.bright_since = None
        if self.on:
            self.on_at, self.lit = now, None
        self.hold_until = now + self.MANUAL_HOLD_S
        self.last.update(action="You set the flashlight from the gamepad", at=time.time())

    # ------------------------------------------------------------ the decision, twice a second
    def step(self, luma, now):
        self.last["luma"] = None if luma is None else round(luma, 1)
        if not self.enabled or now < self.hold_until:
            return
        if not self.active():
            if self.on:
                self._switch(False, "the robot is parked")
            self.dark_since = self.bright_since = None
            return
        if luma is None:
            return
        if not self.on:
            if luma >= self.DARK:
                self.dark_since = None
            elif self.dark_since is None:
                self.dark_since = now
            if self.dark_since is None:
                return
            relapse = now - self.bright_off_at < self.RELAPSE_S     # it was our own light that made it bright
            if relapse or now - self.dark_since >= self.ON_DELAY:
                if relapse:
                    self.strict_until = now + self.STRICT_S
                self._switch(True, f"the picture is dark ({luma:.0f})")
            return
        if now - self.on_at < self.SETTLE:
            return
        if self.lit is None:
            self.lit = luma                                         # the picture with the light on: the reference
        level = self.WASHED if now < self.strict_until else min(self.WASHED, max(self.OFF_FLOOR, self.BRIGHTER * self.lit))
        if luma < level:
            self.bright_since = None
        elif self.bright_since is None:
            self.bright_since = now
        elif now - self.bright_since >= self.OFF_DELAY and now - self.on_at >= self.MIN_ON_S:
            if self._switch(False, f"the darkness is gone ({luma:.0f}); the flashlight is the gamepad's again"):
                self.bright_off_at = now

    # ------------------------------------------------------------ plumbing
    def set(self, enabled=None, aim=None):
        if enabled is not None:
            self.enabled = bool(enabled)
            if not self.enabled and self.on:
                self._switch(False, "automatic flashlight switched off")
            self.hold_until = 0.0
        try:
            os.makedirs(os.path.dirname(self.settings), exist_ok=True)
            with open(self.settings, "w") as f:
                json.dump({"enabled": self.enabled}, f)
        except OSError:
            pass                                                    # still applies until the console restarts
        return self.status()

    def status(self):
        now = self.clock()
        return {"enabled": self.enabled, "available": self.available(), "active": self.active(),
                "on": self.on, "percent": self.pct if self.on else 0, "dark_below": self.DARK, "lit": self.lit,
                "holding_s": max(0.0, round(self.hold_until - now, 1)), **self.last}

    async def run(self):
        loop = asyncio.get_running_loop()
        while True:
            await asyncio.sleep(self.PERIOD)
            try:
                wanted = self.enabled and (self.active() or self.on)
                luma = await loop.run_in_executor(None, self.measure) if wanted else None
                self.step(luma, self.clock())
            except asyncio.CancelledError:
                raise
            except Exception as error:  # noqa: BLE001 - a camera hiccup must not end the watch
                self.last.update(action=f"Flashlight check failed: {error}", at=time.time())
