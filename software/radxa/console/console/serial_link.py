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
        self._expected_param_count = 0
        self.maintenance = False
        self._serial_paused = False
        self._release_flash_lock = False
        self.nav={"token":0,"state":0,"pad":False,"last_rx":0}
        self.pose_epoch=0
        self.last_odom_monotonic=0.0
        self.log = collections.deque(maxlen=400)

        # live telemetry
        self.tel = {
            "ms": 0, "encL": 0, "encR": 0, "pitch": 0.0, "yaw": 0.0,
            "x_m": 0.0, "v_ms": 0.0, "heading": 0.0,
            "armed": False, "balancing": False, "rx_hz": 0.0, "last_rx": 0.0,
            "stale": True,
        }
        self.debug = {}                       # latest D-line fields
        self.hub = {}                         # HM: raw motor feedback + forward-frame commands
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
        self._prev_enc = None  # MCU reset/reconnect must not inject an odometry jump
        self.pose_epoch+=1;self.tel["pose_epoch"]=self.pose_epoch
        self.nav={"token":0,"state":0,"pad":False,"last_rx":0}
        self.tel["v_ms"] = 0.0
        self._release_flash_lock = True
        self.derived.clear()
        self.hub.clear()
        self.tel["host_ready"] = False
        self.connected = True
        self._log(f"[console] opened {SERIAL_PORT} @ {SERIAL_BAUD}")

    def close(self):
        if self.ser:
            self.ser.close()
        self.connected = False

    # ---------------- outbound ----------------
    def send(self, line: str):
        if not self.ser or self.maintenance:
            return False
        if not line.endswith("\n"):
            line += "\n"
        self.ser.write(line.encode())
        return True

    async def begin_firmware_maintenance(self):
        if not self.connected or self.tel.get("stale",True) or self.tel.get("balancing"):
            raise ValueError("Stop with local SELECT and wait for fresh IDLE telemetry")
        if self.derived.get("FLASHLOCK")!=1:
            raise ValueError("Install FLASHLOCK-capable controller firmware once before app reflashing")
        self.maintenance=True  # inhibit all ordinary writers, but keep reading acknowledgements
        self.tel["firmware_lock"]=False
        try:
            self.ser.write(b"FU,1\n");self.ser.flush()
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                if self.tel.get("firmware_lock") and not self.tel.get("balancing"):break
                await asyncio.sleep(.02)
            else:raise ValueError("Controller did not acknowledge the maintenance/arming lock")
            self.dump_complete=False;self.params.clear()
            self.ser.write(b"P?\nBR,0\n");self.ser.flush()
            deadline=time.monotonic()+5
            retry_at=time.monotonic()+1
            while time.monotonic()<deadline:
                if self.dump_complete and not self.tel.get("host_ready"):
                    self.hold_for_firmware();return
                if time.monotonic()>=retry_at:
                    self.ser.write(b"P?\nBR,0\n");self.ser.flush();retry_at=time.monotonic()+1
                await asyncio.sleep(.02)
            raise ValueError("Locked controller snapshot/readiness acknowledgement timed out")
        except BaseException:
            self.maintenance=False;self._release_flash_lock=True;raise

    def hold_for_firmware(self):
        self.maintenance=True;self._serial_paused=True
        self.close();self.ser=None;self.debug={}
        self.tel.update(stale=True,rx_hz=0.0,startup_state="FLASHING",startup_ready=False)

    def resume_after_firmware(self):
        self._serial_paused=False;self.maintenance=False;self._release_flash_lock=True
        self._prev_enc=None;self.dump_complete=False
        self.tel.update(stale=True,v_ms=0.0,host_ready=False,startup_ready=False)

    def request_dump(self):
        if self.maintenance:return False
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
        if self._prev_enc is not None and ms<self.tel["ms"]-1000:
            self._prev_enc=None;self.pose_epoch+=1;self.tel["pose_epoch"]=self.pose_epoch
            self.tel["v_ms"]=0.0
        cpm = self.derived.get("COUNTSM") or (ENCODER_CPR / (2 * math.pi * 0.05))
        if self._prev_enc is not None:
            sign=self.params["ENCSGN"].value if "ENCSGN" in self.params else 1
            track=self.params["TRACKW"].value if "TRACKW" in self.params else TRACK_WIDTH_M
            dL = sign*(encL - self._prev_enc[0]) / cpm
            dR = sign*(encR - self._prev_enc[1]) / cpm
            d = (dL + dR) / 2.0
            dth = (dR - dL) / max(.05,track)
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
        self.pose_epoch+=1;self.tel["pose_epoch"]=self.pose_epoch
        self._pose = [0.0, 0.0, 0.0]
        self._prev_enc = None
        self.path.clear()

    def _handle(self, line: str):
        if not line:
            return
        try:
            if line.startswith("AN!,") or line.startswith("AN?,"):
                self.nav.update(token=int(line[4:]),state=1 if line.startswith("AN!") else 9,last_rx=time.time(),received_monotonic=time.monotonic())
                return
            if line.startswith("NR,"):
                _,ms,token,state,sequence,age,pad,_reserved=line.split(",")
                self.nav={"ms":int(ms),"token":int(token),"state":int(state),"sequence":int(sequence),
                           "age_ms":int(age),"pad":bool(int(pad)),"last_rx":time.time(),"received_monotonic":time.monotonic()}
                self._publish({"type":"navigation","nav":self.nav})
                return
            if line in ("FU!,1","FU!,0"):
                self.tel["firmware_lock"]=line.endswith("1")
                return
            if line == "BR!,1":
                self.tel["host_ready"] = True
                return
            if line == "BR?,0":
                self.tel["host_ready"] = False
                return
            if line.startswith("BS,"):
                phase = line[3:]
                if phase in ("WAIT_HOST", "INITIALIZING", "CALIBRATING", "READY", "ERROR"):
                    self.tel["startup_state"] = phase
                    self.tel["startup_ready"] = phase == "READY"
                    self._log("[startup] " + phase)
                    self._publish({"type": "startup", "state": phase})
                return
            if line.startswith("O,"):
                p = line.split(",")
                if len(p) >= 6:
                    self._odometry(int(p[2]), int(p[3]), float(p[4]), float(p[5]), int(p[1]))
                    now = time.time()
                    self.last_odom_monotonic=time.monotonic()
                    self._rx_times.append(now)
                    self.tel["last_rx"] = now
                    if len(self._rx_times) > 5:
                        span = self._rx_times[-1] - self._rx_times[0]
                        self.tel["rx_hz"] = round((len(self._rx_times) - 1) / span, 1) if span else 0.0
                return

            if line.startswith("D,"):
                f = line.split(",")[1:]
                names = ["ms", "pF", "w", "u", "v", "ex", "vCmd", "steer",
                         "satV", "satA", "loopMax", "spdHi", "bal", "fwd", "str",
                         "accelMag", "roughness", "soften", "terrainState", "drops",
                         "fpRisk", "fpPred", "fpFlags", "driveReq", "driveRef", "driveEff",
                         "balScale", "wheelL", "wheelR",
                         "acEnabled", "acState", "acPolicy", "acBlend", "acError", "acBias", "acFollow",
                         "acFlags", "clZ", "clEx", "clRef", "fpHazard", "clRecover", "acStable",
                         "acKind", "acExit", "acExitBlock", "acCooldown", "dsLimit", "dsExcess", "dsScale",
                         "dsActive", "dsGoal", "acOverspeed"]
                self.debug = {n: float(x) for n, x in zip(names, f) if x not in ("", None)}
                self.tel["balancing"] = bool(self.debug.get("bal", 0))
                return

            if line.startswith("HM,"):
                fields=line.split(",")[1:]
                if len(fields)!=14:return
                values=[float(value) for value in fields]
                if not all(math.isfinite(value) for value in values):return
                names=("ms","current_cmd_l","current_cmd_r","current_l","current_r","rpm_l","rpm_r",
                       "temp_l","temp_r","fault_l","fault_r","saturated","link_ok","mode_ok")
                self.hub=dict(zip(names,values))
                self.hub["received_monotonic"]=time.monotonic()
                self.tel["hub"]=self.hub
                return

            if line.startswith("P#,"):
                self.dump_complete = False
                self._expected_param_count=int(line[3:])
                self.params.clear()
                return
            if line == "P.":
                self.dump_complete = self._expected_param_count>0 and len(self.params)==self._expected_param_count
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
        next_dump = 0.0
        while True:
            if not self._serial_paused and not self.connected and time.monotonic() >= next_open:
                try:
                    self.open()
                    await asyncio.sleep(0.3)
                    self.request_dump()
                    next_dump = time.monotonic() + 2
                except Exception as e:
                    self._log(f"[console] serial unavailable: {e}")
                    self.close()
                    self.ser = None
                    next_open = time.monotonic() + 3
            if self.connected and not self._serial_paused:
                try:
                    # A reset-held ESP32 may discard the first request before
                    # Serial.begin()/sensor calibration. Retry until P. arrives.
                    if not self.dump_complete and time.monotonic() >= next_dump:
                        self.send("P?")
                        next_dump = time.monotonic() + 2
                    data = self.ser.read(4096)
                    if data:
                        buf += data
                        while b"\n" in buf:
                            raw, buf = buf.split(b"\n", 1)
                            self._handle(raw.decode(errors="replace").strip())
                    if self._release_flash_lock and self.dump_complete and self.tel.get("startup_ready") and not self.maintenance:
                        if self.derived.get("FLASHLOCK")==1:self.send("FU,0")
                        self._release_flash_lock=False
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

            if self._serial_paused:buf=b""

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
