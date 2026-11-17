"""slam_demo.py — try the SLAM page without the robot.

    python -m console.slam_demo            # then open http://localhost:6970/

Simulates a furnished room, a robot driving a loop (with wheel-odometry drift
and the occasional lean while accelerating), firmware O/D lines and C1 scans.
The real SlamEngine and page run on top of it unchanged.
"""
from __future__ import annotations

import math
import os
import tempfile
import threading
import time

import numpy as np

from .slam.engine import SlamEngine
from .slam_app import create_app

ROOM = [((-3, -2), (4, -2)), ((4, -2), (4, 3)), ((4, 3), (-3, 3)), ((-3, 3), (-3, -2)),   # walls
        ((0.8, 0.4), (1.6, 0.4)), ((1.6, 0.4), (1.6, 1.0)), ((1.6, 1.0), (0.8, 1.0)), ((0.8, 1.0), (0.8, 0.4)),  # table
        ((-2.0, 1.8), (-1.2, 2.6)), ((2.5, -1.2), (3.2, -1.2)), ((-1.0, -2.0), (-1.0, -1.3))]  # sofa, shelf, door frame
CPR, R, TRACK = 4096, 0.052, 0.155


def raycast(x, y, angles, max_r=12.0):
    seg = np.array(ROOM, float)                                    # (S,2,2)
    p1, p2 = seg[:, 0], seg[:, 1]
    d = np.stack([np.cos(angles), np.sin(angles)], 1)               # (A,2)
    e = p2 - p1                                                     # (S,2)
    den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
    w = p1[None] - np.array([x, y])[None, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (w[..., 0] * e[None, :, 1] - w[..., 1] * e[None, :, 0]) / den
        u = (w[..., 0] * d[:, None, 1] - w[..., 1] * d[:, None, 0]) / den
    t = np.where((t > 0) & (u >= 0) & (u <= 1), t, np.inf)
    r = t.min(1)
    return np.where(r < max_r, r, 0.0)


class FakeLidar:
    def __init__(self):
        self.status = {"state": "scanning", "port": "SIMULATED", "baud": 460800, "error": "", "note": "demo mode",
                       "model": 65, "firmware": "1.01", "hardware": 18, "serial": "DEMO", "health": "good",
                       "scan_hz": 10.0, "points": 500, "bad_nodes": 0, "scans": 0}


def run_sim(engine, lidar):
    rng = np.random.default_rng(1)
    x, y, th = 0.0, 0.0, 0.0
    encL = encR = 0.0
    t0 = time.monotonic()
    last_scan = t0
    waypoints = [(2.8, -1.0), (2.8, 2.0), (-2.0, 2.2), (-2.2, -1.2), (0.0, 0.0)]
    wp = 0
    k_slip = 1.03                                                   # odometry over-reads 3 %
    while True:
        now = time.monotonic()
        dt = 0.02
        tx, ty = waypoints[wp]
        err = math.atan2(ty - y, tx - x) - th
        err = math.atan2(math.sin(err), math.cos(err))
        dist = math.hypot(tx - x, ty - y)
        w = max(-0.8, min(0.8, 2.0 * err))
        v = 0.0 if abs(err) > 0.6 else min(0.3, dist)
        if dist < 0.15:
            wp = (wp + 1) % len(waypoints)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        vl, vr = v - w * TRACK / 2, v + w * TRACK / 2
        encL += vl * dt / (2 * math.pi * R) * CPR * k_slip
        encR += vr * dt / (2 * math.pi * R) * CPR
        ms = int((now - t0) * 1000)
        gz = math.degrees(w) + rng.normal(0, 0.3)
        engine.on_line(f"O,{ms},{int(encL)},{int(encR)},0.0,0.0,{rng.normal(0, .2):.2f},{rng.normal(0, .2):.2f},{gz:.2f}",
                       now)
        lean = 1.0 * math.sin(now * 0.7)
        engine.on_line(f"D,{ms},{lean:.2f},{0.7 * math.cos(now * 0.7):.2f},0,{v:.2f},0,0,0,0,0,0,0,1", now)
        if now - last_scan >= 0.1:
            ang = np.radians(np.arange(0, 360, 0.72))               # sensor angles, clockwise
            lx, ly = x + 0.031 * math.sin(th), y - 0.031 * math.cos(th)
            r = raycast(lx, ly, th - ang)
            r = np.where(r > 0, r + rng.normal(0, 0.01, len(r)), 0)
            ok = r > 0
            engine.on_scan({"t0": last_scan, "t1": now, "angle_deg": np.degrees(ang[ok]).tolist(),
                            "dist_mm": (r[ok] * 1000).tolist(), "quality": [40] * int(ok.sum())})
            lidar.status["scans"] += 1
            last_scan = now
        time.sleep(max(0.0, dt - (time.monotonic() - now)))


def main():
    import uvicorn
    maps = os.environ.get("BOT_MAPS_DIR") or os.path.join(tempfile.gettempdir(), "2pybot_demo_maps")
    engine = SlamEngine(maps)
    engine.start()
    lidar = FakeLidar()
    threading.Thread(target=run_sim, args=(engine, lidar), daemon=True).start()
    app = create_app(engine, lidar, lambda: print("[demo] E-STOP pressed"))
    port = int(os.environ.get("BOT_SLAM_PORT", "6970"))
    print(f"SLAM demo on http://localhost:{port}/  (maps in {maps})")
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


if __name__ == "__main__":
    main()
