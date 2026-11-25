"""The flashlight looks after itself.

While the robot is up (balancing), the console watches the brightness of the camera
picture once a second:

  * dark for three seconds          -> flashlight on, at a low brightness
  * still dim with the light on     -> brighter, a step at a time, up to the hard cap
  * too bright with the light on    -> dimmer; at the lowest setting and still bright
                                       (the room lights came on) -> off
  * washed out                      -> a big step down at once

and it points the beam where it is useful. It leads into turns like a car's cornering
light, and it is tight (thrown far) when the way ahead is open and wide when something
is close.

Every command goes through the firmware's own clamp and hard cap (TL / P= / Z=). The bot
only ever switches off a light it switched on itself: touch the flashlight from the
gamepad and the bot leaves it alone for five minutes. When the robot is parked or
powered down, a light the bot switched on goes off.

It shares the job with the camera's automatic re-exposure (camera.ExposureWatch) so the
two do not fight: while the light can still act, darkness is the light's problem and
the exposure is left where it is.
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import time

from .config import CAM_PROFILES

def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class AutoTorch:
    PERIOD = 1.0             # s between looks at the picture
    SETTLE = 1.6             # s after changing the light before the picture is trusted (snapshots are 1 Hz)
    DARK = 40.0              # mean luma below this is "dark" (0 to 255)
    LOW, HIGH = 75.0, 150.0  # with the light on, keep the picture in this band
    WASHED = 200.0           # far too bright: step down hard
    NEED = 3                 # readings in a row before switching on or off
    START, STEP, BIG_STEP = 25, 10, 30
    FLOOR, CEILING = 10, 90  # percent; the firmware caps at 90 whatever is asked
    MIN_OFF_S = 20.0         # s the light stays off after the bot switched it off
    MANUAL_HOLD_S = 300.0    # s the bot keeps its hands off after the operator used the flashlight
    PAN_PER_RADS = 25.0      # degrees of beam lead per rad/s of turning
    NEAR_M, FAR_M = 0.8, 2.5 # free distance ahead: wide beam at NEAR or closer, tight at FAR or beyond

    def __init__(self, link, measure, front_m=lambda: None, log=print, settings=None, clock=time.monotonic):
        self.link, self.measure, self.front_m, self.log, self.clock = link, measure, front_m, log, clock
        self.settings = settings or os.path.join(os.path.dirname(CAM_PROFILES), "auto_torch.json")
        self.enabled, self.aim_enabled = True, True
        try:
            with open(self.settings) as f:
                saved = json.load(f)
            self.enabled = bool(saved.get("enabled", True))
            self.aim_enabled = bool(saved.get("aim", True))
        except (OSError, ValueError, AttributeError):
            pass
        self.on, self.pct = False, self.START          # `on`: the bot switched the light on and still owns it
        self.dark_n = self.bright_n = 0
        self.changed_at = self.off_at = self.aimed_at = -1e9
        self.hold_until = 0.0
        self._sent = None                              # (on, pct, when) of the bot's last command, to tell it from the pad
        self._pan = self._zoom = None
        self._heading = None                           # (degrees, when) for the turn rate
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
        if side == "dark":
            return not (self.on and self.pct >= self.CEILING)     # until the light is flat out
        return self.on                                            # too bright with our light on: dim that first

    # ------------------------------------------------------------ commands
    def _set(self, on, pct, why):
        now = self.clock()
        pct = int(max(self.FLOOR, min(self.CEILING, pct)))
        if not self.link.send(f"TL,{1 if on else 0},{pct}"):
            return False
        was = self.on
        self.on, self.pct, self.changed_at = bool(on), pct, now
        self._sent = (bool(on), pct, now)
        self.dark_n = self.bright_n = 0
        if not on:
            self.off_at = now
            if was and self._pan is not None:
                self.link.send("N")                                # head back to centre
            self._pan = self._zoom = None
        if on != was or why:
            text = (f"Flashlight on at {pct}%: {why}" if on and not was else
                    f"Flashlight {pct}%: {why}" if on else f"Flashlight off: {why}")
            self.last.update(action=text, at=time.time())
            if on != was:
                self.log("[torch] " + text)
        return True

    def on_line(self, line):
        """A flashlight line from the controller that the bot did not cause means the operator
        used the gamepad: the light is theirs for a while."""
        # "[PAY] Torch ON (25%)", "[PAY] Torch off", "[PAY] Torch 35%": a change of state.
        # Not the boot banner ("[PAY] Torch GPIO2 off, 25% preset, ...").
        if not line.startswith(("[PAY] Torch ON", "[PAY] Torch off")) and not (
                line.startswith("[PAY] Torch ") and line[12:13].isdigit()):
            return
        now = self.clock()
        if self._sent and now - self._sent[2] < 1.5:
            return                                                 # the echo of our own command
        if self.on or self.enabled:
            self.on = False
            self._pan = self._zoom = None
            self.hold_until = now + self.MANUAL_HOLD_S
            self.last.update(action="You used the flashlight: the bot leaves it alone for 5 minutes", at=time.time())

    # ------------------------------------------------------------ the decision, once a second
    def step(self, luma, now):
        self.last["luma"] = None if luma is None else round(luma, 1)
        if not self.enabled or now < self.hold_until:
            return
        if not self.active():
            if self.on:
                self._set(False, self.pct, "the robot is parked")
            self.dark_n = self.bright_n = 0
            return
        if luma is None or now - self.changed_at < self.SETTLE:
            return
        if not self.on:
            self.dark_n = self.dark_n + 1 if luma < self.DARK else 0
            if self.dark_n >= self.NEED and now - self.off_at >= self.MIN_OFF_S:
                self._set(True, self.START, f"the picture is dark ({luma:.0f})")
            return
        if luma >= self.WASHED and self.pct > self.FLOOR:
            self._set(True, self.pct - self.BIG_STEP, f"washed out ({luma:.0f})")
        elif luma > self.HIGH:
            if self.pct > self.FLOOR:
                self._set(True, self.pct - self.STEP, f"too bright ({luma:.0f})")
            else:
                self.bright_n += 1
                if self.bright_n >= self.NEED:
                    self._set(False, self.pct, f"there is enough light without it ({luma:.0f})")
        elif luma < self.LOW and self.pct < self.CEILING:
            self._set(True, self.pct + self.STEP, f"still dim ({luma:.0f})")
        else:
            self.bright_n = 0

    # ------------------------------------------------------------ where the beam points
    def turn_rate(self, now):
        """rad/s, left positive, from the change of odometry heading."""
        heading = self.link.tel.get("heading")
        if not isinstance(heading, (int, float)):
            return 0.0
        prev, self._heading = self._heading, (float(heading), now)
        if prev is None or not 0.05 < now - prev[1] < 3.0:
            return 0.0
        return math.radians(_wrap_deg(heading - prev[0])) / (now - prev[1])

    def aim(self, now):
        """Lead into the turn; tight beam when the way ahead is open, wide when something is close.
        Larger pan angle is to the right and larger zoom angle is tighter, as on the gamepad's
        right stick. Sent only when it changes enough to be worth moving a servo for."""
        rate = self.turn_rate(now)
        if not (self.on and self.aim_enabled) or now - self.aimed_at < 0.7:
            return
        lo, hi, home = self._param("SVYAWMIN", 120), self._param("SVYAWMAX", 190), self._param("SVYAWHOME", 165)
        pan = max(lo, min(hi, home - self.PAN_PER_RADS * (rate if abs(rate) > 0.15 else 0.0)))
        zlo, zhi = self._param("SVZMIN", 80), self._param("SVZMAX", 140)
        front = self.front_m()
        open_ahead = 1.0 if not isinstance(front, (int, float)) or not math.isfinite(front) else (
            max(0.0, min(1.0, (front - self.NEAR_M) / (self.FAR_M - self.NEAR_M))))
        zoom = zlo + (zhi - zlo) * open_ahead
        if self._pan is None or abs(pan - self._pan) >= 4.0:
            if self.link.send(f"P={pan:.0f}"):
                self._pan, self.aimed_at = pan, now
        if self._zoom is None or abs(zoom - self._zoom) >= 8.0:
            if self.link.send(f"Z={zoom:.0f}"):
                self._zoom, self.aimed_at = zoom, now

    # ------------------------------------------------------------ plumbing
    def set(self, enabled=None, aim=None):
        if enabled is not None:
            self.enabled = bool(enabled)
            if not self.enabled and self.on:
                self._set(False, self.pct, "automatic flashlight switched off")
            self.hold_until = 0.0
        if aim is not None:
            self.aim_enabled = bool(aim)
        try:
            os.makedirs(os.path.dirname(self.settings), exist_ok=True)
            with open(self.settings, "w") as f:
                json.dump({"enabled": self.enabled, "aim": self.aim_enabled}, f)
        except OSError:
            pass                                                    # still applies until the console restarts
        return self.status()

    def status(self):
        now = self.clock()
        return {"enabled": self.enabled, "aim": self.aim_enabled, "available": self.available(), "active": self.active(),
                "on": self.on, "percent": self.pct if self.on else 0, "dark_below": self.DARK, "band": [self.LOW, self.HIGH],
                "holding_s": max(0.0, round(self.hold_until - now, 1)), "pan": self._pan, "zoom": self._zoom, **self.last}

    async def run(self):
        loop = asyncio.get_running_loop()
        while True:
            await asyncio.sleep(self.PERIOD)
            try:
                wanted = self.enabled and (self.active() or self.on)
                luma = await loop.run_in_executor(None, self.measure) if wanted else None
                now = self.clock()
                self.step(luma, now)
                self.aim(now)
            except asyncio.CancelledError:
                raise
            except Exception as error:  # noqa: BLE001 - a camera hiccup must not end the watch
                self.last.update(action=f"Flashlight check failed: {error}", at=time.time())
