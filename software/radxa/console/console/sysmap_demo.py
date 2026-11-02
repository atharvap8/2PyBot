"""sysmap_demo.py — run the system map with a simulated robot, no hardware.

    cd software/radxa/console
    python -m console.sysmap_demo                    # healthy robot
    python -m console.sysmap_demo --scenario tof     # ToF stopped answering
    python -m console.sysmap_demo --scenario serial  # USB serial silent
    python -m console.sysmap_demo --scenario wheel   # left M0601 overheat fault
    python -m console.sysmap_demo --scenario cycle   # rotate through all of them

Then open http://localhost:6969. Use it to work on the page from a laptop.
"""
import argparse
import collections
import math
import time

SCENARIOS = ("healthy", "tof", "serial", "wheel", "imu", "pad")


class FakeLink:
    def __init__(self):
        self.connected, self.maintenance, self.dump_complete = True, False, True
        self.ser = None
        self.tel, self.debug, self.hub, self.nav = {}, {}, {}, {}
        self.derived = {}
        self.log = collections.deque(maxlen=400)
        self._subs = {object(), object()}

    def send(self, line):
        self.log.append({"t": time.time(), "text": f"[demo] sent {line}"})
        return True


class FakeToF:
    def __init__(self):
        self.ok = True

    def status(self, link=None):
        if not self.ok:
            return {"state": "ranging", "stale": True, "age_ms": 2400, "valid_count": 0,
                    "bus": 7, "address": "0x29", "ranging_hz": 10, "distance_mm": [], "valid": []}
        d = [int(600 + 140 * math.hypot(i % 8 - 3.5, (i // 8 - 5) * 1.2)) for i in range(64)]
        return {"state": "ranging", "stale": False, "age_ms": 60, "valid_count": 58,
                "bus": 7, "address": "0x29", "ranging_hz": 10,
                "distance_mm": d, "valid": [i % 13 != 0 for i in range(64)]}


class FakeStreamer:
    def status(self):
        return {"running": True, "encoder": "cedar_h264", "error": "",
                "metrics": {"fps": "30", "frame": "18422", "capture_to_publish_ms": "62"}}


def main():
    import uvicorn

    from .sysmap import create_app

    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="healthy", choices=SCENARIOS + ("cycle",))
    ap.add_argument("--port", type=int, default=6969)
    args = ap.parse_args()

    link, tof = FakeLink(), FakeToF()
    for text in ("[BOOT] Waiting for Radxa ACK; IMU calibration and device startup are on HOLD",
                 "[IMU] Initialisation OK", "[HUB] RIGHT id2 OK; LEFT id1 OK",
                 "[PAD] Connected: EVOFOX One S", "[PAY] Pan  -> 165 deg", "[PAY] Zoom -> 120 deg",
                 "[PAY] Torch ON (35%)"):
        link.log.append({"t": time.time(), "text": text})
    t0 = time.time()
    state = {"pad": True}

    def tick():
        t = time.time() - t0
        scenario = SCENARIOS[int(t // 12) % len(SCENARIOS)] if args.scenario == "cycle" else args.scenario
        now = time.time()
        silent = scenario == "serial"
        link.tel.update(pitch=round(0.4 * math.sin(t * 2.1), 2), yaw=round(182 + 3 * math.sin(t / 5), 1),
                        balancing=True, startup_state="READY", host_ready=True, v_ms=0.21,
                        stale=silent, rx_hz=0.0 if silent else 50.0,
                        last_rx=now - 4.2 if silent else now)
        link.debug.update(loopMax=3100, w=round(3 * math.cos(t * 2.1), 1), accelMag=1.01)
        hot = scenario == "wheel"
        link.hub = {"current_cmd_l": 0.62, "current_cmd_r": 0.58, "current_l": 0.0 if hot else 0.6,
                    "current_r": 0.57, "rpm_l": 0 if hot else 41, "rpm_r": 39,
                    "temp_l": 81 if hot else 44, "temp_r": 43, "fault_l": 0x10 if hot else 0, "fault_r": 0,
                    "saturated": 0, "link_ok": 1, "mode_ok": 1, "received_monotonic": time.monotonic()}
        tof.ok = scenario != "tof"
        if scenario == "imu" and not any("[IMU] ERROR" in e["text"] for e in list(link.log)[-5:]):
            link.log.append({"t": now, "text": "[IMU] ERROR: begin() failed — check wiring / address"})
        if scenario != "imu" and any("[IMU] ERROR" in e["text"] for e in list(link.log)[-5:]):
            link.log.append({"t": now, "text": "[IMU] Initialisation OK"})
        pad_up = scenario != "pad"
        if pad_up != state["pad"]:
            state["pad"] = pad_up
            link.log.append({"t": now, "text": "[PAD] Connected: EVOFOX One S" if pad_up
                             else "[PAD] Disconnected — inputs zeroed, balance continues"})

    class TickingToF(FakeToF):
        def status(self, link=None):
            tick()  # first thing the collector reads, so every snapshot is current
            self.ok = tof.ok
            return FakeToF.status(self, link)

    app = create_app(link, TickingToF(), FakeStreamer(), lambda: {"ready": True, "camera": True, "hotspot": True},
                     lambda: link.send("X"))
    print(f"System map demo ({args.scenario}) on http://localhost:{args.port}")
    uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
