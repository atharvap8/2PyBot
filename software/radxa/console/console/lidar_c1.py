"""lidar_c1.py — RPLIDAR C1 driver (pure Python + pyserial, no ROS, no SDK).

Owns ONLY the lidar UART (default /dev/ttyAS4, 40-pin UART4, 460800 8N1).
Protocol constants come from Slamtec's SDK (sl_lidar_cmd.h / sl_lidar_protocol.h):
    request      0xA5 <cmd> [len payload checksum]   checksum = XOR of all bytes
    descriptor   0xA5 0x5A <len:30 | mode:2 (4 bytes LE)> <type>
    legacy node  5 bytes: [S | !S<<1 | quality<<2] [angle_q6<<1 | C (LE16)] [dist_q2 (LE16)]
Angles from the sensor run clockwise, 0 deg = the C1's front mark.

Not yet verified on hardware: that the C1 answers legacy SCAN (0x20) with
5-byte nodes, and whether it needs a motor-speed command first. The driver
tries SCAN; if no samples arrive it sends the HQ motor-speed command (0xA8)
and retries, and reports what happened in status["note"].
"""
from __future__ import annotations

import struct
import threading
import time

try:
    import serial  # pyserial is already a console dependency
except ImportError:  # pragma: no cover - reported in status
    serial = None

SYNC, ANS1, ANS2 = 0xA5, 0xA5, 0x5A
CMD_STOP, CMD_RESET, CMD_SCAN = 0x25, 0x40, 0x20
CMD_INFO, CMD_HEALTH, CMD_HQ_MOTOR = 0x50, 0x52, 0xA8
TYPE_INFO, TYPE_HEALTH, TYPE_MEAS = 0x04, 0x06, 0x81
HEALTH_TEXT = {0: "good", 1: "warning", 2: "error"}
DEFAULT_RPM = 1200            # 20 Hz on the C1M1R2 (owner-confirmed); 600 = 10 Hz


class LidarC1:
    def __init__(self, port: str, baud: int = 460800, on_scan=None, rpm: int = DEFAULT_RPM):
        self.port, self.baud = port, baud
        self.rpm = max(300, min(1500, int(rpm)))
        self.on_scan = on_scan                 # fn(scan_dict) per full rotation
        self.ser = None
        self._stop = threading.Event()
        self._thread = None
        self.status = {"state": "stopped", "port": port, "baud": baud, "error": "", "note": "",
                       "model": None, "firmware": None, "hardware": None, "serial": None,
                       "health": None, "scan_hz": 0.0, "points": 0, "bad_nodes": 0, "scans": 0,
                       "rpm": self.rpm}

    # ------------------------------------------------------------ lifecycle
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="lidar-c1", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._close()
        self.status["state"] = "stopped"

    # ------------------------------------------------------------ low level
    def _close(self):
        try:
            if self.ser:
                self.ser.write(bytes([SYNC, CMD_STOP]))
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _send(self, cmd, payload: bytes = b""):
        if payload:
            pkt = bytes([SYNC, cmd, len(payload)]) + payload
            chk = 0
            for b in pkt:
                chk ^= b
            pkt += bytes([chk])
        else:
            pkt = bytes([SYNC, cmd])
        self.ser.write(pkt)

    def _read_exact(self, n, timeout=1.0):
        buf = bytearray()
        end = time.monotonic() + timeout
        while len(buf) < n and time.monotonic() < end and not self._stop.is_set():
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf += chunk
        return bytes(buf)

    def _descriptor(self, timeout=1.0):
        """Find 0xA5 0x5A and return (length, mode, type) or None."""
        end = time.monotonic() + timeout
        prev = None
        while time.monotonic() < end and not self._stop.is_set():
            b = self.ser.read(1)
            if not b:
                continue
            if prev == ANS1 and b[0] == ANS2:
                rest = self._read_exact(5, timeout)
                if len(rest) < 5:
                    return None
                word = struct.unpack("<I", rest[:4])[0]
                return word & 0x3FFFFFFF, word >> 30, rest[4]
            prev = b[0]
        return None

    def _request(self, cmd, want_type, timeout=1.0):
        self.ser.reset_input_buffer()
        self._send(cmd)
        d = self._descriptor(timeout)
        if not d or d[2] != want_type:
            return None
        return self._read_exact(d[0], timeout)

    # ------------------------------------------------------------ session
    def _open_and_identify(self):
        if serial is None:
            raise RuntimeError("pyserial not installed")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        try:
            self.ser.dtr = False                # USB adapters use DTR for motor; UARTs may lack modem lines
        except (OSError, serial.SerialException):
            pass
        self._send(CMD_STOP)
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        info = self._request(CMD_INFO, TYPE_INFO)
        if not info or len(info) < 20:
            raise RuntimeError("no answer to GET_INFO: check wiring (TX<->RX), 5 V, baud and UART4 overlay")
        model, fw, hw = info[0], struct.unpack("<H", info[1:3])[0], info[3]
        self.status.update(model=model, firmware=f"{fw >> 8}.{fw & 0xFF:02d}", hardware=hw,
                           serial=info[4:20].hex().upper())
        health = self._request(CMD_HEALTH, TYPE_HEALTH)
        if health and len(health) >= 3:
            code = health[0]
            self.status["health"] = HEALTH_TEXT.get(code, str(code))
            if code == 2:
                self._send(CMD_RESET)
                time.sleep(1.0)
                raise RuntimeError("lidar reports a hardware error; sent RESET")

    def _start_scan(self):
        self.ser.reset_input_buffer()
        self._send(CMD_SCAN)
        d = self._descriptor(1.5)
        return bool(d and d[2] == TYPE_MEAS)

    def _scan_loop(self):
        angles, dists, quals = [], [], []
        t_start = time.monotonic()
        last_scan_t = None
        first = True
        buf = bytearray()
        got_any_until = time.monotonic() + 1.5
        while not self._stop.is_set():
            chunk = self.ser.read(512)
            if chunk:
                buf += chunk
            elif time.monotonic() > got_any_until:
                return "no samples"
            i = 0
            n = len(buf)
            while n - i >= 5:
                b0, b1 = buf[i], buf[i + 1]
                s, ns, c = b0 & 1, (b0 >> 1) & 1, b1 & 1
                if s == ns or c != 1:                  # misaligned: resync one byte
                    self.status["bad_nodes"] += 1
                    i += 1
                    continue
                got_any_until = time.monotonic() + 1.5
                angle = (struct.unpack_from("<H", buf, i + 1)[0] >> 1) / 64.0
                dist = struct.unpack_from("<H", buf, i + 3)[0] / 4.0
                q = b0 >> 2
                if s and angles:
                    now = time.monotonic()
                    if not first:
                        self._emit(angles, dists, quals, t_start, now, last_scan_t)
                    first = False
                    last_scan_t = now
                    angles, dists, quals = [], [], []
                    t_start = now
                if dist > 0:
                    angles.append(angle)
                    dists.append(dist)
                    quals.append(q)
                i += 5
            del buf[:i]
        return "stopped"

    def _emit(self, angles, dists, quals, t0, t1, prev_t):
        st = self.status
        st["scans"] += 1
        st["points"] = len(angles)
        if prev_t:
            hz = 1.0 / max(1e-3, t1 - prev_t)
            st["scan_hz"] = hz if not st["scan_hz"] else 0.8 * st["scan_hz"] + 0.2 * hz
        if self.on_scan:
            self.on_scan({"t0": t0, "t1": t1, "angle_deg": angles, "dist_mm": dists, "quality": quals})

    def _run(self):
        while not self._stop.is_set():
            self.status.update(state="connecting", error="")
            try:
                self._open_and_identify()
                # Set the motor speed first: legacy SCAN only streams after the
                # HQ motor command, and it selects the scan rate (rpm / 60 Hz).
                self._send(CMD_HQ_MOTOR, struct.pack("<H", self.rpm))
                time.sleep(1.2)
                ok = self._start_scan()
                if ok:
                    self.status["state"] = "scanning"
                why = self._scan_loop() if ok else "no scan descriptor"
                if why in ("no samples", "no scan descriptor") and not self._stop.is_set():
                    self.status["note"] = "SCAN gave no samples; sending motor-speed command and retrying"
                    self._send(CMD_HQ_MOTOR, struct.pack("<H", self.rpm))
                    time.sleep(0.8)
                    if self._start_scan():
                        self.status["state"] = "scanning"
                        why = self._scan_loop()
                if not self._stop.is_set():
                    self.status.update(state="error", error=f"scan ended: {why}")
            except Exception as e:  # serial errors, unplugged, wrong port
                self.status.update(state="error", error=str(e))
            finally:
                self._close()
            if not self._stop.is_set():
                time.sleep(2.0)
