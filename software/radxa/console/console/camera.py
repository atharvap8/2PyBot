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
import json, os, re, subprocess, time
from typing import Dict, List

from .config import CAMERA_DEV, SNAPSHOT_PATH, CAM_PROFILES

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
    out = _run(["v4l2-ctl", "-d", dev, "--get-ctrl", name])
    m = re.search(r":\s*(-?\d+)", out)
    return int(m.group(1)) if m else None


def set_control(name: str, value: int, dev: str = CAMERA_DEV):
    r = subprocess.run(["v4l2-ctl", "-d", dev, "--set-ctrl", f"{name}={int(value)}"],
                       capture_output=True, text=True)
    return r.returncode == 0, (r.stderr or r.stdout).strip()


def apply_profile(name: str, dev: str = CAMERA_DEV):
    """Apply a named profile; returns which keys this camera accepted."""
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


def autocalibrate(target: float = 110.0, tol: float = 8.0, max_iter: int = 12,
                  dev: str = CAMERA_DEV, progress=None):
    """
    Walk exposure then gain until mean luma lands near `target`.
    Exposure first (free, but costs frame rate and adds motion blur),
    gain second (keeps the frame rate, costs noise). Stops as soon as
    it is inside tolerance, so it doesn't gratuitously crank the gain.
    """
    have = {c["name"]: c for c in list_controls(dev)}
    exp_key = next((k for k in ("exposure_time_absolute", "exposure_absolute") if k in have), None)
    gain_key = next((k for k in ("gain", "analogue_gain") if k in have), None)
    auto_key = next((k for k in ("auto_exposure", "exposure_auto") if k in have), None)
    if auto_key:
        set_control(auto_key, 1, dev)          # 1 = manual on both spellings
    if not exp_key:
        return {"ok": False, "error": "camera exposes no manual exposure control"}

    steps = []
    lo = have[exp_key].get("min", 1)
    hi = max(lo, min(have[exp_key].get("max", 2000), STREAM_EXPOSURE_MAX))
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
            new_g = int(min(gmax, max(0, cur_g * ratio if cur_g else 16)))
            if new_g == cur_g:
                break
            set_control(gain_key, new_g, dev)
        else:
            break
    return {"ok": True, "converged": False, "steps": steps,
            "note": "hit a control limit before reaching target; the sensor "
                    "may simply be out of light. Add illumination or a faster lens."}


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
    return {c["name"]: get_control(c["name"], dev) for c in list_controls(dev)
            if c["type"] in ("int", "bool", "menu")}
