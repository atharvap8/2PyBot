"""Odometry core for the standalone SLAM (ported unchanged from the ROS bridge).

No rclpy imports: everything here can be replayed offline from a recorded
console tap, which keeps the maths reviewable apart from ROS plumbing.

Input is raw firmware lines from SerialLink.add_line_listener() (read-only):
    O,ms,encL,encR,pitchDeg,yawDeg[,gxDps,gyDps,gzDps]   50 Hz
    D,ms,pitchF,rateF,...,balancing(13th field),...       ~50 Hz

Conventions (REP-103/105): x forward, y left, z up, yaw CCW positive.
Encoder counts arrive already forward-positive per wheel (firmware applies
per-wheel signs), so turning left means the right wheel advances more.
"""
import math
from collections import deque
from typing import NamedTuple, Optional

AXES = {"x": 0, "y": 1, "z": 2}


class OSample(NamedTuple):
    ms: int
    enc_l: int
    enc_r: int
    pitch: float
    yaw: float
    gyro: Optional[tuple]     # sensor-frame deg/s, or None


class DSample(NamedTuple):
    ms: int
    pitch_f: float      # forward-positive degrees
    rate_f: float       # forward-positive deg/s
    balancing: bool


def parse_line(line: str):
    """Return OSample / DSample, or None for anything else or malformed input."""
    try:
        if line.startswith("O,"):
            p = line.strip().split(",")
            if len(p) < 6:
                return None
            gyro = None
            if len(p) >= 9:
                gyro = (float(p[6]), float(p[7]), float(p[8]))
            return OSample(int(p[1]), int(p[2]), int(p[3]), float(p[4]), float(p[5]), gyro)
        if line.startswith("D,"):
            p = line.strip().split(",")
            if len(p) < 14:
                return None
            return DSample(int(p[1]), float(p[2]), float(p[3]), float(p[13]) >= 0.5)
    except ValueError:
        return None
    return None


def parse_axis(spec: str):
    """'auto' -> None; 'z' / '-z' / '+y' -> (index, sign)."""
    s = spec.strip().lower()
    if s == "auto":
        return None
    sign = -1.0 if s.startswith("-") else 1.0
    name = s.lstrip("+-")
    if name not in AXES:
        raise ValueError(f"imu yaw axis must be auto, x, y or z with optional sign, got {spec!r}")
    return AXES[name], sign


def axis_name(axis) -> str:
    idx, sign = axis
    return ("-" if sign < 0 else "") + "xyz"[idx]


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ClockMap:
    """Maps the ESP32 millisecond counter onto host (ROS) time.

    offset = host_receive_time - esp_time. Serial/USB/UDP only ever add delay,
    so the minimum offset over a sliding window is the best latency-free
    estimate. A monotonic deque keeps that minimum O(1) per sample.
    """

    def __init__(self, window_s: float = 10.0):
        self.window = window_s
        self._mins = deque()        # (host_t, offset), offsets increasing
        self.last_ms = None

    def update(self, ms: int, host_t: float) -> bool:
        """Feed one sample. Returns True if the ESP32 clock restarted/wrapped."""
        restarted = self.last_ms is not None and ms < self.last_ms - 1000
        if restarted:
            self._mins.clear()
        self.last_ms = ms
        off = host_t - ms / 1000.0
        while self._mins and self._mins[-1][1] >= off:
            self._mins.pop()
        self._mins.append((host_t, off))
        while self._mins and host_t - self._mins[0][0] > self.window:
            self._mins.popleft()
        return restarted

    def ready(self) -> bool:
        return bool(self._mins)

    def stamp(self, ms: int) -> float:
        return ms / 1000.0 + self._mins[0][1]


class AxisDetector:
    """Finds which gyro axis (and sign) is the robot's vertical axis.

    Only uses samples while the wheels say the robot is clearly turning, then
    compares each sensor axis with the encoder yaw rate. Also reports the
    gyro/encoder rate ratio, which gives the effective (scrub) track width.
    """

    MIN_RATE = 0.3      # rad/s of encoder yaw rate before a sample counts
    NEED = 150          # ~3 s of turning at 50 Hz
    MIN_CORR = 0.90
    RATIO_RANGE = (0.6, 1.6)

    def __init__(self):
        self.reset()

    def reset(self):
        self.n = 0
        self.see = 0.0
        self.abs_e = 0.0
        self.sgg = [0.0, 0.0, 0.0]
        self.seg = [0.0, 0.0, 0.0]
        self.abs_g = [0.0, 0.0, 0.0]

    def add(self, w_enc: float, gyro_dps):
        if abs(w_enc) < self.MIN_RATE:
            return None
        g = [math.radians(v) for v in gyro_dps]
        self.n += 1
        self.see += w_enc * w_enc
        self.abs_e += abs(w_enc)
        for k in range(3):
            self.sgg[k] += g[k] * g[k]
            self.seg[k] += w_enc * g[k]
            self.abs_g[k] += abs(g[k])
        if self.n < self.NEED:
            return None

        def corr(k):
            return self.seg[k] / math.sqrt(self.see * self.sgg[k] + 1e-12)

        best = max(range(3), key=lambda k: abs(corr(k)))
        c = corr(best)
        ratio = self.abs_g[best] / max(self.abs_e, 1e-9)
        if abs(c) >= self.MIN_CORR and self.RATIO_RANGE[0] <= ratio <= self.RATIO_RANGE[1]:
            return (best, 1.0 if c > 0 else -1.0), abs(c), ratio
        if self.n >= 4 * self.NEED:
            self.reset()          # inconsistent data: start a fresh window
        return None


class Odometry:
    """Planar dead reckoning: wheel distance + gyro yaw (encoder yaw fallback)."""

    MAX_SPEED = 3.0     # m/s; larger steps are encoder glitches, not motion
    MAX_YAW_RATE = 15.0  # rad/s
    MAX_GAP = 0.5       # s; longer gaps re-anchor instead of integrating

    def __init__(self, radius: float, track: float, cpr: int, yaw_axis: str = "auto",
                 encoder_yaw_sign: float = 1.0):
        self.m_per_count = 2.0 * math.pi * radius / cpr
        self.track = track
        self.enc_yaw_sign = 1.0 if encoder_yaw_sign >= 0 else -1.0
        self.axis = parse_axis(yaw_axis)
        self.detector = AxisDetector() if self.axis is None else None
        self.detected = None            # (axis, corr, ratio) once auto-detection succeeds
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.prev = None
        self.bias = 0.0
        self.still_s = 0.0
        self.jumps = 0
        self.source = "encoder"
        self.slip_acc = 0.0             # integrated |gyro yaw - wheel yaw| (rad); read+reset by fusion

    def re_anchor(self):
        self.prev = None

    def step(self, s: OSample) -> bool:
        """Integrate one O sample. Returns True when the pose advanced."""
        if self.prev is None:
            self.prev = (s.ms, s.enc_l, s.enc_r)
            return False
        dt = (s.ms - self.prev[0]) / 1000.0
        dl = (s.enc_l - self.prev[1]) * self.m_per_count
        dr = (s.enc_r - self.prev[2]) * self.m_per_count
        self.prev = (s.ms, s.enc_l, s.enc_r)
        if dt <= 0.0 or dt > self.MAX_GAP:
            return False
        ds = 0.5 * (dl + dr)
        dth_enc = self.enc_yaw_sign * (dr - dl) / self.track
        if abs(ds) > self.MAX_SPEED * dt or abs(dth_enc) > self.MAX_YAW_RATE * dt:
            self.jumps += 1
            return False
        v = ds / dt
        w_enc = dth_enc / dt
        dth = dth_enc
        self.source = "encoder"

        if s.gyro is not None:
            if self.axis is None and self.detector is not None:
                found = self.detector.add(w_enc, s.gyro)
                if found:
                    self.axis = found[0]
                    self.detected = found
                    self.detector = None
            if self.axis is not None:
                idx, sign = self.axis
                w_raw = math.radians(s.gyro[idx]) * sign
                if abs(v) < 0.01 and abs(w_enc) < 0.02:
                    self.still_s += dt
                else:
                    self.still_s = 0.0
                if self.still_s > 1.5:          # learn residual gyro bias at rest
                    self.bias += 0.02 * (w_raw - self.bias)
                dth = (w_raw - self.bias) * dt
                self.source = "gyro"
                # Wheel slip shows as the wheels' yaw disagreeing with the gyro
                # (scaled by the learned scrub ratio of this chassis).
                ratio = self.detected[2] if self.detected else 1.0
                self.slip_acc += abs((w_raw - self.bias) - ratio * w_enc) * dt

        mid = self.th + 0.5 * dth
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th = wrap(self.th + dth)
        self.v = v
        self.w = dth / dt
        return True


class Attitude:
    """Pitch deviation from the balancing equilibrium, for TF and scan gating.

    The baseline is the slow average pitch while balancing, so firmware trim
    and IMU mounting offsets cancel out. The lidar should be mounted level
    while the robot balances.
    """

    def __init__(self, sign: float = 1.0, tau_s: float = 20.0):
        self.sign = sign
        self.tau = tau_s
        self.baseline = None
        self.dev = 0.0
        self.rate = 0.0
        self.balancing = False
        self._last_t = None
        self._last_p = None
        self._far_s = 0.0

    def reset(self):
        """Forget the baseline (e.g. pitch source changed)."""
        self.baseline = None
        self._far_s = 0.0
        self._last_t = self._last_p = None

    def update(self, t: float, pitch_deg: float, rate_dps: Optional[float] = None,
               balancing: Optional[bool] = None):
        p = self.sign * pitch_deg
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        if rate_dps is None:
            rate_dps = 0.0 if not dt or self._last_p is None else (p - self._last_p) / dt
        else:
            rate_dps *= self.sign
        self._last_t, self._last_p = t, p
        if self.baseline is None:
            self.baseline = p
        learning = balancing if balancing is not None else True
        if learning and dt:
            if abs(p - self.baseline) < 5.0:
                self._far_s = 0.0
                self.baseline += min(1.0, dt / self.tau) * (p - self.baseline)
            else:
                # Started while lying down / self-righting: once a new posture
                # has been held for 3 s, adopt it instead of locking out.
                self._far_s += dt
                if self._far_s > 3.0:
                    self.baseline = p
                    self._far_s = 0.0
        self.dev = p - self.baseline
        self.rate = rate_dps
        if balancing is not None:
            self.balancing = balancing
