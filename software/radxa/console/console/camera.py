"""
camera.py — v4l2 control surface + low-light calibration.

Two jobs:
  1. Enumerate whatever controls THIS camera actually has (no hardcoding —
     UVC cameras differ wildly) so the GUI can render real sliders.
  2. Calibrate for low light: an auto-exposure routine that measures the
     snapshot's mean luma and walks exposure/gain to a target, then stops.
     Fixed exposure is also what makes the frame rate constant (auto mode
     silently drops to 15 or 7.5 fps when it gets dark).

Snapshots come from the stream pipeline (ffmpeg writes a JPEG once a
second to /dev/shm), so we never fight the encoder for the device.
"""
import asyncio, json, os, re, subprocess, threading, time
from typing import Dict, List

from .config import CAMERA_DEV, SNAPSHOT_PATH, CAM_PROFILES
from . import native_camera

# V4L2 absolute exposure is in 100-us units: 300 = 30 ms, below a 30-fps frame.
STREAM_EXPOSURE_MAX = 300

# Sensible starting points. Names vary by driver, so every key is optional
# and silently skipped if this camera doesn't have it.
# Ranges below match the Lenovo FHD Webcam actually fitted to this robot
# (auto_exposure menu is {1=Manual, 3=Aperture Priority}; gain 0-100;
# exposure 1-5000; brightness -64..64; contrast 0..64; gamma 72..500).
# Unknown keys are skipped automatically, so these stay safe on other cameras.
PROFILES = {
    "daylight":   {"auto_exposure": 3, "exposure_dynamic_framerate": 0,
                   "gain": 0, "brightness": 0, "contrast": 32, "gamma": 106,
                   "saturation": 53, "sharpness": 3, "backlight_compensation": 1},
    "indoor":     {"auto_exposure": 1, "exposure_time_absolute": 330, "gain": 25,
                   "brightness": 6, "contrast": 34, "gamma": 120,
                   "saturation": 55, "sharpness": 3, "backlight_compensation": 2},
    "low_light":  {"auto_exposure": 1, "exposure_time_absolute": 1200, "gain": 65,
                   "brightness": 16, "contrast": 28, "gamma": 160,
                   "saturation": 45, "sharpness": 2, "backlight_compensation": 4},
    "night":      {"auto_exposure": 1, "exposure_time_absolute": 2500, "gain": 100,
                   "brightness": 28, "contrast": 24, "gamma": 220,
                   "saturation": 20, "sharpness": 1, "backlight_compensation": 6},
    # Short exposure keeps the frame rate up and kills motion blur while the
    # robot is driving; gain and gamma pay for the lost light.
    "driving":    {"auto_exposure": 1, "exposure_time_absolute": 200, "gain": 85,
                   "brightness": 12, "contrast": 34, "gamma": 180,
                   "saturation": 50, "sharpness": 4},
}

_CTRL_RE = re.compile(
    r"^\s*(?P<name>\w+)\s+0x[0-9a-f]+\s+\((?P<type>\w+)\)\s*:\s*(?P<rest>.*)$")


def _run(args, timeout=5):
    try:
        return subprocess.run(args, capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:
        return ""


def list_controls(dev: str = CAMERA_DEV) -> List[Dict]:
    """Parse `v4l2-ctl --list-ctrls-menus` into something a GUI can render."""
    native = native_camera.status() if dev == CAMERA_DEV else None
    if native:
        return native_camera.controls(native)
    out, ctrls = _run(["v4l2-ctl", "-d", dev, "--list-ctrls-menus"]), []
    cur = None
    for line in out.splitlines():
        m = _CTRL_RE.match(line)
        if m:
            d = {"name": m.group("name"), "type": m.group("type"), "menu": {}}
            for kv in re.finditer(r"(\w+)=(-?\d+)", m.group("rest")):
                d[kv.group(1)] = int(kv.group(2))
            if "flags" in m.group("rest"):
                d["flags"] = m.group("rest").split("flags=")[-1].strip()
            ctrls.append(d)
            cur = d
        elif cur is not None and re.match(r"^\s+\d+:", line):
            k, v = line.strip().split(":", 1)
            cur["menu"][int(k)] = v.strip()
    return ctrls


def get_control(name: str, dev: str = CAMERA_DEV):
    native = native_camera.status() if dev == CAMERA_DEV else None
    if native:
        return native["values"].get(name)
    out = _run(["v4l2-ctl", "-d", dev, "--get-ctrl", name])
    m = re.search(r":\s*(-?\d+)", out)
    return int(m.group(1)) if m else None


def set_control(name: str, value: int, dev: str = CAMERA_DEV):
    if dev == CAMERA_DEV and native_camera.status():
        return native_camera.set_control(name, int(value))
    r = subprocess.run(["v4l2-ctl", "-d", dev, "--set-ctrl", f"{name}={int(value)}"],
                       capture_output=True, text=True)
    return r.returncode == 0, (r.stderr or r.stdout).strip()


def apply_profile(name: str, dev: str = CAMERA_DEV):
    """Apply a named profile; returns which keys this camera accepted."""
    if dev == CAMERA_DEV and native_camera.status():
        prof = native_camera.PROFILES.get(name)
        return (True, native_camera.apply_values(prof)) if prof else (False, {"error": f"unknown CSI profile {name}"})
    prof = PROFILES.get(name)
    if not prof:
        return False, {"error": f"unknown profile {name}"}
    have = {c["name"] for c in list_controls(dev)}
    applied, skipped = {}, []
    # auto_exposure must be set to manual BEFORE exposure_time_absolute sticks
    order = sorted(prof.items(), key=lambda kv: 0 if "auto" in kv[0] else 1)
    for k, v in order:
        if k not in have:
            skipped.append(k)
            continue
        if k in ("exposure_time_absolute", "exposure_absolute"):
            v = min(v, STREAM_EXPOSURE_MAX)
        ok, _ = set_control(k, v, dev)
        (applied if ok else skipped).__setitem__(k, v) if ok else skipped.append(k)
    return True, {"applied": applied, "skipped": skipped}


def measure_luma(path: str = SNAPSHOT_PATH):
    """Mean brightness 0-255 without spawning a decoder for every UI poll."""
    if not os.path.exists(path):
        return None
    try:
        from PIL import Image, ImageStat
        with Image.open(path) as image:
            return ImageStat.Stat(image.convert("L")).mean[0]
    except Exception:
        out = subprocess.run(
            ["ffmpeg", "-v", "error", "-i", path, "-vf", "signalstats,metadata=print:file=-",
             "-f", "null", "-"], capture_output=True, text=True, timeout=10).stdout
        match = re.search(r"YAVG=([\d.]+)", out)
        return float(match.group(1)) if match else None


_CALIBRATING = threading.Lock()      # one exposure walk at a time (button or watcher)


def autocalibrate(target: float = 110.0, tol: float = 8.0, max_iter: int = 12,
                  dev: str = CAMERA_DEV, progress=None):
    """
    Walk exposure then gain until mean luma lands near `target`.
    Exposure first (free, but costs frame rate and adds motion blur),
    gain second (keeps the frame rate, costs noise). Stops as soon as
    it is inside tolerance, so it doesn't gratuitously crank the gain.
    """
    if not _CALIBRATING.acquire(blocking=False):
        return {"ok": False, "error": "a calibration is already running"}
    try:
        return _autocalibrate(target, tol, max_iter, dev, progress)
    finally:
        _CALIBRATING.release()


def _autocalibrate(target, tol, max_iter, dev, progress):
    have = {c["name"]: c for c in list_controls(dev)}
    exp_key = next((k for k in ("exposure_us", "exposure_time_absolute", "exposure_absolute") if k in have), None)
    gain_key = next((k for k in ("gain", "analogue_gain") if k in have), None)
    auto_key = next((k for k in ("auto_exposure", "exposure_auto") if k in have), None)
    if auto_key:
        set_control(auto_key, 1, dev)          # 1 = manual on both spellings
    if not exp_key:
        if "imx708" in dev:
            return {"ok": False, "error": "CSI publisher control channel is not ready; wait for the video pipeline"}
        return {"ok": False, "error": "camera exposes no manual exposure control"}

    steps = []
    lo = have[exp_key].get("min", 1)
    hi = have[exp_key]["max"] if exp_key == "exposure_us" else max(lo, min(have[exp_key].get("max", 2000), STREAM_EXPOSURE_MAX))
    current = get_control(exp_key, dev)
    if current is not None and current > hi:
        set_control(exp_key, hi, dev)
    for i in range(max_iter):
        time.sleep(1.2)                        # let a fresh snapshot land
        y = measure_luma()
        if y is None:
            return {"ok": False, "error": "no snapshot; is the stream running?"}
        cur_e = get_control(exp_key, dev) or lo
        cur_g = get_control(gain_key, dev) if gain_key else None
        steps.append({"iter": i, "luma": round(y, 1), "exposure": cur_e, "gain": cur_g})
        if progress:
            progress(steps[-1])
        if abs(y - target) <= tol:
            return {"ok": True, "converged": True, "luma": y, "exposure": cur_e,
                    "gain": cur_g, "steps": steps}
        ratio = max(0.35, min(2.8, target / max(y, 1.0)))
        new_e = int(min(hi, max(lo, cur_e * ratio)))
        if new_e != cur_e:
            set_control(exp_key, new_e, dev)
            continue
        # Keep exposure within one frame; use gain for the remaining light.
        if gain_key and cur_g is not None:
            gmax = have[gain_key].get("max", 255)
            new_g = int(min(gmax, max(have[gain_key].get("min", 0), cur_g * ratio if cur_g else 16)))
            if new_g == cur_g:
                break
            set_control(gain_key, new_g, dev)
        else:
            break
    return {"ok": True, "converged": False, "steps": steps,
            "note": "hit a control limit before reaching target; the sensor "
                    "may simply be out of light. Add illumination or a faster lens."}


class ExposureWatch:
    """Re-exposes the camera by itself when the picture stays too dark or too bright.

    A fixed exposure keeps the frame rate steady but is only right for the light it
    was set in: drive from a bright room into a dim one and the picture stays dark.
    Every PERIOD seconds this measures the picture's mean brightness. If it has been
    outside LOW..HIGH for NEED checks in a row, it runs the same exposure walk as the
    Auto-calibrate button, then leaves the camera alone for a while. The gap between
    the trigger band and the walk's target is what stops it hunting. When the sensor
    runs out of range (a dark room, a covered lens) it rests much longer, unless the
    picture swings to the other extreme.
    """
    PERIOD, NEED = 5.0, 3
    LOW, HIGH, TARGET = 55.0, 185.0, 110.0
    REST, REST_AT_LIMIT = 30.0, 300.0        # s left alone after a walk; longer when the sensor ran out of range

    def __init__(self, log=print, settings=None, measure=None, calibrate=None, clock=time.monotonic):
        self.log, self.clock = log, clock
        self.settings = settings or os.path.join(os.path.dirname(CAM_PROFILES), "auto_exposure.json")
        self.measure = measure or self.fresh_luma
        self.calibrate = calibrate or (lambda: autocalibrate(target=self.TARGET))
        self.enabled = True
        try:
            with open(self.settings) as f:
                self.enabled = bool(json.load(f).get("enabled", True))
        except (OSError, ValueError, AttributeError):
            pass
        self.bad, self.rest_until, self.busy = 0, 0.0, False
        self.walked_at, self.limit_side = -1e9, None     # the long rest only holds while the picture is bad the same way
        self.last = {"luma": None, "action": "", "at": None}

    @staticmethod
    def fresh_luma(path=SNAPSHOT_PATH, max_age=5.0):
        """Brightness of the latest snapshot, or None when the camera is off or the picture is old."""
        try:
            if time.time() - os.path.getmtime(path) > max_age:
                return None
        except OSError:
            return None
        return measure_luma(path)

    def set_enabled(self, on):
        self.enabled, self.bad = bool(on), 0
        try:
            os.makedirs(os.path.dirname(self.settings), exist_ok=True)
            with open(self.settings, "w") as f:
                json.dump({"enabled": self.enabled}, f)
        except OSError:
            pass                                    # still applies until the console restarts
        return self.status()

    def status(self):
        return {"enabled": self.enabled, "busy": self.busy, "low": self.LOW, "high": self.HIGH, "target": self.TARGET,
                "resting_s": max(0.0, round(self.rest_until - self.clock(), 1)), **self.last}

    def due(self, luma, now):
        """One brightness reading in; True when it is time to re-expose."""
        self.last["luma"] = None if luma is None else round(luma, 1)
        if not self.enabled or luma is None or self.LOW <= luma <= self.HIGH:
            self.bad = 0
            return False
        side = "dark" if luma < self.LOW else "bright"
        # Out of range in the dark, then the light comes on (or the lens is uncovered): do not sit out the long rest.
        flipped = self.limit_side is not None and side != self.limit_side and now >= self.walked_at + self.REST
        if now < self.rest_until and not flipped:
            self.bad = 0
            return False
        self.bad += 1
        return self.bad >= self.NEED

    def finished(self, result, luma, now):
        self.bad = 0
        settled = bool(result.get("ok") and result.get("converged"))
        self.rest_until = now + (self.REST if settled else self.REST_AT_LIMIT)
        self.walked_at, self.limit_side = now, None if settled else ("dark" if luma < self.LOW else "bright")
        why = "too dark" if luma < self.LOW else "too bright"
        outcome = (f"now {result.get('luma', 0):.0f}" if settled
                   else result.get("error") or "the sensor ran out of range for this light")
        self.last.update(action=f"Picture was {why} ({luma:.0f}): re-exposed, {outcome}", at=time.time())
        self.log("[camera] " + self.last["action"])

    async def run(self):
        loop = asyncio.get_running_loop()
        while True:
            await asyncio.sleep(self.PERIOD)
            try:
                luma = await loop.run_in_executor(None, self.measure) if self.enabled else None
                if self.due(luma, self.clock()) and not _CALIBRATING.locked():
                    self.busy = True
                    try:
                        result = await loop.run_in_executor(None, self.calibrate)
                    finally:
                        self.busy = False
                    self.finished(result or {}, luma, self.clock())
            except asyncio.CancelledError:
                raise
            except Exception as error:  # noqa: BLE001 - a camera hiccup must not end the watch
                self.bad = 0
                self.rest_until = self.clock() + self.REST
                self.last.update(action=f"Exposure check failed: {error}", at=time.time())


def snapshot_bytes():
    try:
        with open(SNAPSHOT_PATH, "rb") as f:
            return f.read()
    except OSError:
        return None


def save_profile(name: str, values: Dict[str, int]):
    os.makedirs(os.path.dirname(CAM_PROFILES), exist_ok=True)
    data = load_profiles()
    data[name] = values
    with open(CAM_PROFILES, "w") as f:
        json.dump(data, f, indent=1)
    return data


def load_profiles() -> Dict:
    try:
        with open(CAM_PROFILES) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def current_values(dev: str = CAMERA_DEV) -> Dict[str, int]:
    native = native_camera.status() if dev == CAMERA_DEV else None
    if native:
        return native["values"]
    return {c["name"]: get_control(c["name"], dev) for c in list_controls(dev)
            if c["type"] in ("int", "bool", "menu")}


def profile_names():
    return list(native_camera.PROFILES if native_camera.status() else PROFILES) + list(load_profiles())
