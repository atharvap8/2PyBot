"""SLAM engine: ties odometry, the C1 scans and the grid together.

Listen-only: reads raw O/D lines from the console's serial link and scans
from the lidar driver. It never sends anything to the robot.
"""
from __future__ import annotations

import collections
import json
import math
import os
import re
import threading
import time

import numpy as np

from .fusion import PoseFilter
from .grid import OccupancyGrid
from .tof_layer import ToFLayer
from .odom_core import Attitude, DSample, Odometry, OSample, axis_name, parse_line

# Mount (robot frame: x forward, y left). Lidar centre is 31 mm right of the
# centreline and on the axle line (assembly A1); 0 deg mark faces forward.
DEFAULTS = {
    "lidar_x": 0.0, "lidar_y": -0.031, "lidar_yaw_deg": 0.0,
    "lidar_height": 0.265,           # measured scan-plane height above floor (owner, upright)
    "self_radius": 0.12,             # guard fence + body
    "min_quality": 5,
    "max_range": 8.0,
    # Per-beam floor clipping handles normal balancing lean; whole scans are
    # dropped only for a large lean (resting/falling) or fast rocking.
    "max_tilt_deg": 12.0, "max_tilt_rate": 60.0, "plane_tol_deg": 0.75,
    "deskew": True,
    "tof_height": 0.21,              # VL53L5CX above the floor (owner, wheel contact to sensor)
    "tof_x": 0.0,                    # forward offset from the axle (not measured)
    "tof_y": 0.04,                   # 4 cm LEFT of the centreline (owner-measured)
    "wheel_radius": 0.052, "track": 0.155, "cpr": 4096,
    "yaw_axis": "auto", "encoder_yaw_sign": 1.0,
    "match_points": 200,
    "map_every_m": 0.10, "map_every_deg": 5.0,
    "min_match_score": 0.12,         # mapping: below this, trust the wheels for this scan
    "min_loc_score": 0.30,           # localizing: below this, say "not sure" instead of "localized"
}
SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]{1,40}$")


class SlamEngine:
    def __init__(self, maps_dir: str, settings: dict | None = None):
        self.cfg = dict(DEFAULTS, **(settings or {}))
        self.maps_dir = maps_dir
        self.lock = threading.Lock()
        self.odom = Odometry(self.cfg["wheel_radius"], self.cfg["track"], self.cfg["cpr"],
                             self.cfg["yaw_axis"], self.cfg["encoder_yaw_sign"])
        self.att = Attitude()
        self.odom_hist = collections.deque(maxlen=400)        # (t, x, y, th)
        self.att_hist = collections.deque(maxlen=400)         # (t, dev, rate)
        self.rates = {"O": 0, "D": 0, "scan": 0}
        self._rate_t = time.monotonic()
        self.rate_hz = {"O": 0.0, "D": 0.0, "scan": 0.0}
        self.last_o = self.last_d = self.last_scan = 0.0
        self.mode = "idle"                     # idle | mapping | localizing
        self.map_name = ""
        self.grid = OccupancyGrid()
        self.pose = np.zeros(3)                # map frame (published copy of filter.mu)
        self.filter = PoseFilter()
        self.tof_layer = ToFLayer(height=self.cfg["tof_height"], x_offset=self.cfg["tof_x"], y_offset=self.cfg["tof_y"])
        self.tof_source = None                 # console ToF supervisor (history deque), set by server
        self._odom_ref = None                  # odom pose at the last processed scan
        self._last_map_pose = None
        self.path = collections.deque(maxlen=3000)
        self.scan_pts = np.zeros((0, 2))
        self.gate = {"passed": 0, "tilt": 0, "rate": 0, "no_robot": 0, "usable_range": 0.0, "floor_clipped": 0}
        self.match = {"score": 0.0, "ok": False, "lost": 0}
        self.message = "Waiting for the lidar."
        self.tests = {}
        self._need_find = False
        self._last_find = 0.0
        self._find_lock = threading.Lock()
        self._scan_q = collections.deque(maxlen=2)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker, name="slam", daemon=True)

    def start(self):
        self._thread.start()

    def shutdown(self):
        self._stop.set()

    # ------------------------------------------------------------- inputs
    def on_line(self, line: str, t: float):
        msg = parse_line(line)
        if isinstance(msg, OSample):
            self.rates["O"] += 1
            self.last_o = t
            with self.lock:
                if self.odom.step(msg):
                    self.odom_hist.append((t, self.odom.x, self.odom.y, self.odom.th))
                if time.monotonic() - self.last_d > 0.5:
                    self.att.update(msg.ms / 1000.0, msg.pitch)
                    self.att_hist.append((t, self.att.dev, self.att.rate))
        elif isinstance(msg, DSample):
            self.rates["D"] += 1
            self.last_d = t
            with self.lock:
                self.att.update(msg.ms / 1000.0, msg.pitch_f, msg.rate_f, msg.balancing)
                self.att_hist.append((t, self.att.dev, self.att.rate))

    def on_scan(self, scan: dict):
        self.rates["scan"] += 1
        self.last_scan = time.monotonic()
        self._scan_q.append(scan)

    # ------------------------------------------------------------- helpers
    def _odom_at(self, t):
        with self.lock:                     # the serial thread appends concurrently
            h = list(self.odom_hist)
        if not h:
            return None
        if t <= h[0][0]:
            return np.array(h[0][1:])
        for a, b in zip(reversed(h[:-1]), reversed(h)):
            if a[0] <= t <= b[0]:
                f = (t - a[0]) / max(1e-6, b[0] - a[0])
                th = a[3] + f * math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
                return np.array([a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), th])
        return np.array(h[-1][1:])

    def _scan_points(self, scan):
        c = self.cfg
        a = np.radians(-np.asarray(scan["angle_deg"], float)) + math.radians(c["lidar_yaw_deg"])
        r = np.asarray(scan["dist_mm"], float) / 1000.0
        q = np.asarray(scan["quality"], float)
        # Points arrive in rotation order; their fraction of the sweep gives
        # each one's capture time between scan t0 and t1 (used for de-skew).
        frac = np.linspace(0.0, 1.0, len(r)) if len(r) > 1 else np.zeros(len(r))
        keep = (q >= c["min_quality"]) & (r >= c["self_radius"])
        return a[keep], r[keep], frac[keep]

    def _floor_limit(self, a, tilt_deg, spread_deg):
        """Per-beam range before the tilted scan plane reaches the floor.

        Forward pitch lowers forward beams by atan(tan(tilt)*cos(azimuth)); side
        beams stay level and rearward beams rise, so they keep full range.
        """
        c = self.cfg
        dip = np.degrees(np.arctan(math.tan(math.radians(tilt_deg)) * np.cos(a)))
        down = dip + spread_deg + c["plane_tol_deg"]
        limit = np.full(len(a), float(c["max_range"]))
        hit = down > 1e-3
        limit[hit] = np.minimum(limit[hit], c["lidar_height"] / np.tan(np.radians(down[hit])))
        return limit

    def _deskew(self, pts, frac, t0, t1):
        """Express every point in the robot frame at mid-scan using odometry."""
        knots = np.linspace(t0, t1, 5)
        poses = [self._odom_at(t) for t in knots]
        if any(p is None for p in poses):
            return pts
        poses = np.array(poses)
        mid = self._odom_at(0.5 * (t0 + t1))
        tq = frac * 4.0
        x = np.interp(tq, np.arange(5), poses[:, 0])
        y = np.interp(tq, np.arange(5), poses[:, 1])
        th = np.interp(tq, np.arange(5), np.unwrap(poses[:, 2]))
        # pose_i relative to mid, in the mid robot frame
        cm, sm = math.cos(mid[2]), math.sin(mid[2])
        dx, dy = x - mid[0], y - mid[1]
        rx, ry = cm * dx + sm * dy, -sm * dx + cm * dy
        dth = th - mid[2]
        cd, sd = np.cos(dth), np.sin(dth)
        return np.c_[rx + cd * pts[:, 0] - sd * pts[:, 1], ry + sd * pts[:, 0] + cd * pts[:, 1]]

    def _gate(self, t0, t1, robot_live):
        c = self.cfg
        if not robot_live:
            return 0.0                                      # lidar-only bench mode: no tilt info
        with self.lock:                     # the serial thread appends concurrently
            hist = list(self.att_hist)
        win = [x for x in hist if t0 - 0.05 <= x[0] <= t1 + 0.05]
        if not win:
            return None
        devs = [x[1] for x in win]
        rate = max(abs(x[2]) for x in win)
        if max(abs(d) for d in devs) > c["max_tilt_deg"]:
            self.gate["tilt"] += 1
            return "tilt"
        if rate > c["max_tilt_rate"]:
            self.gate["rate"] += 1
            return "rate"
        # Signed mean lean during the sweep plus its spread as extra margin.
        mean = sum(devs) / len(devs)
        return mean, max(abs(d - mean) for d in devs)

    # ------------------------------------------------------------- worker
    def _worker(self):
        while not self._stop.is_set():
            self._update_rates()
            self._process_tof()
            if not self._scan_q:
                time.sleep(0.01)
                continue
            scan = self._scan_q.popleft()
            try:
                self._process(scan)
            except Exception as e:  # keep the loop alive, show the reason
                self.message = f"Processing error: {e}"

    def _process_tof(self):
        src = self.tof_source
        history = getattr(src, "history", None)
        if not history:
            return
        frame = history[-1]
        if frame.get("sequence") == self.tof_layer.last_seq:
            return                                # nothing new: avoid locking at 100 Hz
        t = frame.get("capture_monotonic_ns", 0) / 1e9
        with self.lock:
            pose = self.pose.copy()
            att = list(self.att_hist)[-25:]
            balancing = self.att.balancing
        near = min(att, key=lambda a: abs(a[0] - t)) if att else (t, 0.0, 0.0)
        steady = balancing and abs(near[1]) < 2.0 and abs(near[2]) < 20.0
        try:
            # 3D voxels build up only while mapping (map frame), so a saved map
            # carries a matching 3D layer.
            self.tof_layer.update(frame, pose, near[1], steady, t,
                                  accumulate=self.mode == "mapping" and self.match.get("ok", False))
        except (KeyError, ValueError, TypeError):
            pass                                  # malformed frame: skip, keep the loop alive

    def _update_rates(self):
        now = time.monotonic()
        dt = now - self._rate_t
        if dt >= 1.0:
            for k in self.rates:
                self.rate_hz[k] = self.rates[k] / dt
                self.rates[k] = 0
            self._rate_t = now

    def _process(self, scan):
        c = self.cfg
        a, r, frac = self._scan_points(scan)
        robot_live = time.monotonic() - self.last_o < 1.0
        g = self._gate(scan["t0"], scan["t1"], robot_live)
        sensor = np.array([c["lidar_x"], c["lidar_y"]])
        pts_full = np.c_[c["lidar_x"] + r * np.cos(a), c["lidar_y"] + r * np.sin(a)]
        with self.lock:
            self.scan_pts = pts_full                         # radar view shows raw scan
        if g in ("tilt", "rate"):
            self.message = ("The robot is leaning a lot, so I'm skipping this scan."
                            if g == "tilt" else "The robot is rocking fast, so I'm skipping this scan.")
            return
        if g is None:
            self.gate["no_robot"] += 1
            return
        tilt, spread = g if isinstance(g, tuple) else (g, 0.0)
        limit = self._floor_limit(a, tilt, spread)
        keep = r <= limit
        pts = pts_full[keep]
        if c["deskew"] and robot_live and len(pts):
            pts = self._deskew(pts, frac[keep], scan["t0"], scan["t1"])
        self.gate["passed"] += 1
        self.gate["usable_range"] = float(limit.min()) if len(limit) else c["max_range"]
        self.gate["floor_clipped"] = int((~keep).sum())
        if self.mode == "idle" or len(pts) < 30:
            if self.mode == "idle":
                self.message = "Lidar is working. Press Start mapping when you're ready."
            else:
                self.message = "Too few points in this scan; move away from open space."
            return

        tmid = 0.5 * (scan["t0"] + scan["t1"])
        od = self._odom_at(tmid) if robot_live else None
        with self.lock:
            grid = self.grid                                 # load/start may swap it from HTTP threads
            if od is not None and self._odom_ref is not None:
                # odometry increment in the previous robot frame -> filter prediction
                d0, d1 = od[0] - self._odom_ref[0], od[1] - self._odom_ref[1]
                cr, sr = math.cos(self._odom_ref[2]), math.sin(self._odom_ref[2])
                dth = math.atan2(math.sin(od[2] - self._odom_ref[2]), math.cos(od[2] - self._odom_ref[2]))
                slip, self.odom.slip_acc = self.odom.slip_acc, 0.0
                self.filter.predict(cr * d0 + sr * d1, -sr * d0 + cr * d1, dth, slip)
            self._odom_ref = od
            guess = self.filter.mu.copy()

        if self.mode == "localizing" and self._need_find:
            self._need_find = False
            self.relocalize(wide=True)
            guess = self.filter.mu.copy()
        sub = pts[np.linspace(0, len(pts) - 1, min(len(pts), c["match_points"])).astype(int)]
        first = grid.version == 0
        if first:
            new_pose, score, ok = guess, 0.0, True
        else:
            z, score, cov = grid.match(sub, guess, with_cov=True)
            good = score >= (c["min_loc_score"] if self.mode == "localizing" else c["min_match_score"])
            # A good match still has to agree with wheels+gyro (Mahalanobis gate);
            # otherwise this scan is carried by the odometry prediction alone.
            ok = good and self.filter.update(z, cov)
            if not good:
                self.filter.stats["skipped"] += 1
            new_pose = self.filter.mu.copy()
        new_pose[2] = math.atan2(math.sin(new_pose[2]), math.cos(new_pose[2]))
        with self.lock:
            self.pose = new_pose
            self.match = {"score": score, "ok": ok, "lost": 0 if ok else self.match["lost"] + 1}
            self.path.append((float(new_pose[0]), float(new_pose[1])))
        if self.mode == "mapping":
            lp = self._last_map_pose
            moved = (lp is None or math.hypot(new_pose[0] - lp[0], new_pose[1] - lp[1]) >= c["map_every_m"]
                     or abs(math.degrees(new_pose[2] - lp[2])) >= c["map_every_deg"])
            if moved and (ok or first):
                grid.integrate(new_pose, pts, sensor)
                self._last_map_pose = new_pose.copy()
            self.message = ("Mapping. Drive slowly and come back to where you started to tidy the map."
                            if ok or first else "I'm not sure where I am. Slow down or drive back to a spot you've mapped.")
        else:
            # lost for ~2 s: search the whole map again (at most every 3 s)
            if self.match["lost"] >= 20 and time.monotonic() - self._last_find > 3.0:
                self._last_find = time.monotonic()
                self.relocalize(wide=True)
            self.message = ("Localized: I know where I am on this map." if ok else
                            "I can't match this view to the map. Drive to a mapped area or press Find me.")

    # ------------------------------------------------------------- commands
    def start_mapping(self, fresh=True):
        with self.lock:
            if fresh:
                self.grid = OccupancyGrid()
                self.pose = np.zeros(3)
                self.path.clear()
                self.map_name = ""
                self.tof_layer.clear_voxels()
            self.filter.reset(self.pose, sigma_xy=0.01, sigma_th=math.radians(0.5))
            self._odom_ref = None
            self._last_map_pose = None
            self.mode = "mapping"
        self.message = "Mapping started."

    def stop_mapping(self):
        self.mode = "idle"
        self.message = "Stopped. Save the map if you like it."

    def relocalize(self, wide=True):
        """Global search: try positions on a 0.25 m lattice over the map's free
        space at 36 headings with a few scan points, then refine the best eight.
        Serialized: "Find me" (HTTP) and the automatic re-search (worker) never
        run at the same time."""
        with self._find_lock:
            return self._relocalize()

    def _relocalize(self):
        g = self.grid
        if g.version == 0 or not len(self.scan_pts):
            return 0.0
        pts = self.scan_pts[np.hypot(self.scan_pts[:, 0], self.scan_pts[:, 1]) < self.cfg["max_range"]]
        if len(pts) < 30:
            return 0.0
        few = pts[np.linspace(0, len(pts) - 1, 90).astype(int)]
        stride = max(1, int(round(0.25 / g.res)))                 # even 0.25 m lattice
        free = g.logodds[::stride, ::stride] < -1.0              # confidently free cells
        rows, cols = np.nonzero(free)
        if not len(rows):
            return 0.0
        xs = g.origin + (cols * stride + 0.5) * g.res
        ys = g.origin + (rows * stride + 0.5) * g.res
        ths = np.radians(np.arange(0, 360, 10))
        cand = np.stack([np.repeat(xs, len(ths)), np.repeat(ys, len(ths)), np.tile(ths, len(xs))], axis=1)
        coarse = g.score(few, cand)
        best, best_s = self.pose, -1.0
        for k in np.argsort(coarse)[-8:]:
            p, sc = g.match(pts[::2], cand[k], lin=0.30, ang=math.radians(8))
            if sc > best_s:
                best, best_s = p, sc
        with self.lock:
            self.pose = best
            self.filter.reset(best)
            self._odom_ref = None
        return best_s

    # ------------------------------------------------------------- motion tests
    def test_start(self, name):
        with self.lock:
            self.tests[name] = {"x": self.odom.x, "y": self.odom.y, "th": self.odom.th, "t": time.monotonic()}

    def test_result(self, name):
        t = self.tests.get(name)
        if not t:
            return None
        d = math.hypot(self.odom.x - t["x"], self.odom.y - t["y"])
        dth = math.degrees(math.atan2(math.sin(self.odom.th - t["th"]), math.cos(self.odom.th - t["th"])))
        if name == "push":
            return {"distance_m": d, "pass": 0.9 <= d <= 1.1, "hint": "Push the robot 1 m straight ahead."}
        if name == "turn":
            return {"turn_deg": dth, "pass": 70 <= dth <= 110,
                    "hint": "Turn it LEFT by about 90 degrees. Negative means left/right is swapped."}
        return None

    # ------------------------------------------------------------- maps
    def save(self, name):
        if not SAFE_NAME.match(name or ""):
            raise ValueError("Use letters, numbers, - or _ (max 40).")
        os.makedirs(self.maps_dir, exist_ok=True)
        base = os.path.join(self.maps_dir, name)
        # Copy under the lock, compress/write outside it: on_line() takes this
        # lock on the ESP32 serial reader thread, which must never stall on disk I/O.
        with self.lock:
            g = self.grid.copy()
        np.savez_compressed(base + ".npz", logodds=g.logodds, touched=g.touched,
                            voxels=self.tof_layer.voxel_array(min_hits=1))
        with open(base + ".png", "wb") as f:
            f.write(g.png(g.known_bbox()))
        meta = {"name": name, "resolution": g.res, "origin": g.origin,
                "size_cells": g.n, "saved": time.time(), "start_pose": [0, 0, 0]}
        with open(base + ".json", "w") as f:
            json.dump(meta, f, indent=1)
        self.map_name = name
        return meta

    def list_maps(self):
        if not os.path.isdir(self.maps_dir):
            return []
        out = []
        for fn in sorted(os.listdir(self.maps_dir)):
            if fn.endswith(".json"):
                try:
                    with open(os.path.join(self.maps_dir, fn)) as f:
                        out.append(json.load(f))
                except (OSError, ValueError):
                    continue
        return out

    def load(self, name):
        if not SAFE_NAME.match(name or ""):
            raise ValueError("bad map name")
        data = np.load(os.path.join(self.maps_dir, name + ".npz"))
        self.tof_layer.load_voxels(data["voxels"] if "voxels" in data.files else np.zeros((0, 4), np.int32))
        g = OccupancyGrid()
        g.logodds[:] = data["logodds"]
        g.touched[:] = data["touched"]
        g.version = 1
        with self.lock:
            self.grid = g
            self.pose = np.zeros(3)                 # assumes the robot starts where mapping started
            self.filter.reset(self.pose, sigma_xy=0.5, sigma_th=math.radians(30))
            self.path.clear()
            self._odom_ref = None
            self.map_name = name
            self.mode = "localizing"
            self._need_find = True               # search the whole map on the first scan
        self.message = "Map loaded. Looking for where the robot is on it…"

    # ------------------------------------------------------------- state for the UI
    def state(self, lidar_status):
        now = time.monotonic()
        with self.lock:
            pose = self.pose.tolist()
            scan = self.scan_pts[:: max(1, len(self.scan_pts) // 360)].round(3).tolist()
            path = list(self.path)[-1500:]
            od = {"x": self.odom.x, "y": self.odom.y, "yaw_deg": math.degrees(self.odom.th),
                  "source": self.odom.source, "jumps": self.odom.jumps,
                  "axis": axis_name(self.odom.axis) if self.odom.axis else None,
                  "axis_detect": ({"corr": self.odom.detected[1], "ratio": self.odom.detected[2],
                                   "track_eff": self.odom.track / self.odom.detected[2]}
                                  if self.odom.detected else None)}
            att = {"dev": self.att.dev, "rate": self.att.rate, "balancing": self.att.balancing}
        return {
            "mode": self.mode, "map_name": self.map_name, "message": self.message,
            "lidar": lidar_status, "rates": self.rate_hz,
            "robot_live": now - self.last_o < 1.0, "odom": od, "attitude": att,
            "pose": pose, "path": path, "scan": scan, "gate": dict(self.gate), "match": dict(self.match),
            "fusion": dict(self.filter.stats, sigma=self.filter.sigma()),
            "tof": self.tof_layer.state(),
            "map_version": self.grid.version,
            "tests": {k: self.test_result(k) for k in self.tests},
            "settings": self.cfg,
        }

    def scene3d(self, max_cells=40000):
        """Tilted-map scene: the same 2D occupancy grid as wall and floor cells
        (integer indices), robot pose and path. The viewer raises walls a little
        and shows it from a 45-degree corner camera."""
        with self.lock:
            g = self.grid
            walls = np.argwhere(g.logodds > 0.6)                      # (row=y, col=x)
            floor = np.argwhere((g.logodds < -0.4) & g.touched)
            pose = self.pose.tolist()
            path = list(self.path)[-1500:]
        if len(walls) > max_cells:
            walls = walls[np.linspace(0, len(walls) - 1, max_cells).astype(int)]
        if len(floor) > max_cells:
            floor = floor[np.linspace(0, len(floor) - 1, max_cells).astype(int)]
        return {"grid_res": g.res, "grid_origin": g.origin,
                "walls": walls[:, ::-1].tolist(), "floor": floor[:, ::-1].tolist(),
                "pose": pose, "path": path, "version": g.version}
    def map_png(self):
        with self.lock:                          # copy only; encode off the lock (see save())
            g = self.grid.copy()
        bbox = g.known_bbox()
        if bbox is None:
            return None, None
        png = g.png(bbox)
        r0, r1, c0, c1 = bbox
        meta = {"x0": g.origin + c0 * g.res, "y0": g.origin + r0 * g.res,
                "w": (c1 - c0) * g.res, "h": (r1 - r0) * g.res, "version": g.version}
        return png, meta
