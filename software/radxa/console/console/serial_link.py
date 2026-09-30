"""
serial_link.py — the only thing in the system that touches /dev/ttyUSB0.

Parses the v2 protocol (see firmware PROTOCOL.md):
    O,<ms>,<encL>,<encR>,<pitch>,<yaw>      50 Hz odometry
    D,...                                    debug stream (toggle with L)
    P#,<n> / P,... / PR,... / P.             parameter dump
    P!,<key>,<val> / PE,<reason>             set ack / error
    anything else                            log line

Everything else in the console asks THIS for data. One owner, no races.
"""
import asyncio, math, time, collections
from typing import Dict, Optional

import serial

from .config import SERIAL_PORT, SERIAL_BAUD, ENCODER_CPR


class Param:
    __slots__ = ("key", "value", "lo", "hi", "group", "desc", "dirty")

    def __init__(self, key, value, lo, hi, group, desc):
        self.key, self.value, self.lo, self.hi = key, value, lo, hi
        self.group, self.desc, self.dirty = group, desc, False

    def as_dict(self):
        return {"key": self.key, "value": self.value, "lo": self.lo, "hi": self.hi,
                "group": self.group, "desc": self.desc, "dirty": self.dirty}


class SerialLink:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.connected = False
        self.params: Dict[str, Param] = {}
        self.derived: Dict[str, float] = {}
        self.dump_complete = False
        self.log = collections.deque(maxlen=400)

        # live telemetry
        self.tel = {
            "ms": 0, "encL": 0, "encR": 0, "pitch": 0.0, "yaw": 0.0,
            "x_m": 0.0, "v_ms": 0.0, "heading": 0.0,
            "armed": False, "balancing": False, "rx_hz": 0.0, "last_rx": 0.0,
            "stale": True,
        }
        self.debug = {}                       # latest D-line fields
        self.path = collections.deque(maxlen=3000)   # (x, y) for the map
        self._pose = [0.0, 0.0, 0.0]          # x, y, theta (rad)
        self._prev_enc = None
        self._rx_times = collections.deque(maxlen=50)
        self._pending = collections.deque()   # lines to send
        self._subs = set()                    # asyncio.Queue for websockets

    # ---------------- lifecycle ----------------
    def open(self):
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = SERIAL_PORT, SERIAL_BAUD, 0
        s.dtr = False          # never reset the ESP32 by connecting
        s.rts = False
        s.exclusive = True
        s.open()
        self.ser = s
        self.connected = True
        self._log(f"[console] opened {SERIAL_PORT} @ {SERIAL_BAUD}")

    def close(self):
        if self.ser:
            self.ser.close()
        self.connected = False

    # ---------------- outbound ----------------
    def send(self, line: str):
        if not self.ser:
            return False
        if not line.endswith("\n"):
            line += "\n"
        self.ser.write(line.encode())
        return True

    def request_dump(self):
        self.dump_complete = False
        self.params.clear()
        self.send("P?")

    def set_param(self, key: str, value: float):
        p = self.params.get(key.upper())
        if p and not (p.lo <= value <= p.hi):
            return False, f"{key} out of range [{p.lo:g}, {p.hi:g}]"
        self.send(f"P,{key},{value:.6g}")
        return True, "sent"

    # ---------------- inbound ----------------
    def _log(self, text):
        self.log.append({"t": time.time(), "text": text})

    def _publish(self, msg):
        for q in list(self._subs):
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                pass

    def subscribe(self) -> asyncio.Queue:
        q = asyncio.Queue(maxsize=64)
        self._subs.add(q)
        return q

    def unsubscribe(self, q):
        self._subs.discard(q)

    def _odometry(self, encL, encR, pitch, yaw, ms):
        """Differential-drive dead reckoning for the map view."""
        from .config import TRACK_WIDTH_M
        cpm = self.derived.get("COUNTSM") or (ENCODER_CPR / (2 * math.pi * 0.05))
        if self._prev_enc is not None:
            dL = (encL - self._prev_enc[0]) / cpm
            dR = (encR - self._prev_enc[1]) / cpm
            d = (dL + dR) / 2.0
            dth = (dR - dL) / TRACK_WIDTH_M
            self._pose[2] += dth
            self._pose[0] += d * math.cos(self._pose[2])
            self._pose[1] += d * math.sin(self._pose[2])
            dt = max(1e-3, (ms - self.tel["ms"]) / 1000.0)
            v = d / dt
            self.tel["v_ms"] += 0.35 * (v - self.tel["v_ms"])
            if len(self.path) == 0 or (abs(self._pose[0] - self.path[-1][0]) > 0.01 or
                                       abs(self._pose[1] - self.path[-1][1]) > 0.01):
                self.path.append((round(self._pose[0], 3), round(self._pose[1], 3)))
        self._prev_enc = (encL, encR)
        self.tel.update(ms=ms, encL=encL, encR=encR, pitch=pitch, yaw=yaw,
                        x_m=round(self._pose[0], 3), y_m=round(self._pose[1], 3),
                        heading=round(math.degrees(self._pose[2]) % 360, 1))

    def reset_pose(self):
        self._pose = [0.0, 0.0, 0.0]
        self._prev_enc = None
        self.path.clear()

    def _handle(self, line: str):
        if not line:
            return
        try:
            if line.startswith("O,"):
                p = line.split(",")
                if len(p) >= 6:
                    self._odometry(int(p[2]), int(p[3]), float(p[4]), float(p[5]), int(p[1]))
                    now = time.time()
                    self._rx_times.append(now)
                    self.tel["last_rx"] = now
                    if len(self._rx_times) > 5:
                        span = self._rx_times[-1] - self._rx_times[0]
                        self.tel["rx_hz"] = round((len(self._rx_times) - 1) / span, 1) if span else 0.0
                return

            if line.startswith("D,"):
                f = line.split(",")[1:]
                names = ["ms", "pF", "w", "u", "v", "ex", "vCmd", "steer",
                         "satV", "satA", "loopMax", "spdHi", "bal", "fwd", "str"]
                self.debug = {n: float(x) for n, x in zip(names, f) if x not in ("", None)}
                self.tel["balancing"] = bool(self.debug.get("bal", 0))
                return

            if line.startswith("P#,"):
                self.dump_complete = False
                self.params.clear()
                return
            if line == "P.":
                self.dump_complete = True
                self._publish({"type": "params", "params": self.params_list(),
                               "derived": self.derived})
                return
            if line.startswith("PR,"):
                _, k, v = line.split(",", 2)
                self.derived[k] = float(v)
                return
            if line.startswith("P,"):
                parts = line.split(",", 6)
                if len(parts) == 7:
                    _, key, val, lo, hi, grp, desc = parts
                    self.params[key] = Param(key, float(val), float(lo), float(hi), grp, desc)
                return
            if line.startswith("P!,"):
                _, key, val = line.split(",", 2)
                if key in self.params:
                    self.params[key].value = float(val)
                    self.params[key].dirty = True
                self._publish({"type": "param_ack", "key": key, "value": float(val)})
                self.send("P?")               # re-dump so derived values refresh
                return
            if line.startswith(("PE,", "PS!", "PL!", "PD!")):
                self._log(line)
                self._publish({"type": "param_msg", "text": line})
                if line.startswith(("PS!", "PL!", "PD!")):
                    for p in self.params.values():
                        p.dirty = False
                    self.send("P?")
                return
        except (ValueError, IndexError):
            pass

        self._log(line)
        self._publish({"type": "log", "text": line})

    # ---------------- reader task ----------------
    async def run(self):
        buf = b""
        last_tel = 0.0
        next_open = 0.0
        while True:
            if not self.connected and time.monotonic() >= next_open:
                try:
                    self.open()
                    await asyncio.sleep(0.3)
                    self.request_dump()
                except Exception as e:
                    self._log(f"[console] serial unavailable: {e}")
                    self.close()
                    self.ser = None
                    next_open = time.monotonic() + 3
            if self.connected:
                try:
                    data = self.ser.read(4096)
                    if data:
                        buf += data
                        while b"\n" in buf:
                            raw, buf = buf.split(b"\n", 1)
                            self._handle(raw.decode(errors="replace").strip())
                except Exception as e:
                    self._log(f"[console] serial error: {e}")
                    self.connected = False
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    buf = b""
                    next_open = time.monotonic() + 2

            now = time.time()
            if now - last_tel >= 0.1:         # 10 Hz telemetry push
                last_tel = now
                self.tel["stale"] = not self.connected or (now - self.tel["last_rx"]) > 1.0
                if self.tel["stale"]:
                    self.tel["rx_hz"] = 0.0
                self._publish({"type": "telemetry", "tel": self.tel,
                               "debug": self.debug})
            await asyncio.sleep(0.002 if self.connected else 0.1)

    def params_list(self):
        return [p.as_dict() for p in self.params.values()]

    def groups(self):
        out = {}
        for p in self.params.values():
            out.setdefault(p.group, []).append(p.key)
        return out


link = SerialLink()
