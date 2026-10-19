"""Bounded IPC to the CSI publisher: no competing capture opens or guessed UVC units."""
import asyncio
import json
import os
import socket
import tempfile
import threading
from pathlib import Path

CONTROL_SOCKET = os.environ.get("BOT_CAM_SOCKET", "/dev/shm/2pybot-camera.sock")
TUNING_FILE = os.environ.get("BOT_CAM_TUNING", "/var/lib/2pybot/camera_tuning.json")
_lock = threading.RLock()

PROFILES = {
    "rpi_imx708": {"exposure_auto": 0, "color_temperature": 5000, "white_balance_auto": 1, "contrast": 100, "saturation": 100, "black_point": 0, "gamma": 100},
    "natural": {"exposure_auto": 0, "color_temperature": 5000, "white_balance_auto": 1, "contrast": 100, "saturation": 100, "black_point": 0, "gamma": 100},
    "daylight": {"exposure_auto": 0, "color_temperature": 5910, "white_balance_auto": 0, "contrast": 100, "saturation": 100, "black_point": 0, "gamma": 100},
    "indoor": {"exposure_auto": 0, "color_temperature": 4640, "white_balance_auto": 1, "contrast": 100, "saturation": 100, "black_point": 0, "gamma": 100},
    "low_light": {"exposure_auto": 0, "white_balance_auto": 1, "contrast": 110, "saturation": 125, "black_point": 0, "gamma": 115},
    "night": {"exposure_auto": 0, "white_balance_auto": 1, "contrast": 100, "saturation": 110, "black_point": 0, "gamma": 125},
    "driving": {"exposure_auto": 1, "exposure_us": 6000, "gain": 64, "white_balance_auto": 1, "contrast": 120, "saturation": 140, "black_point": 8, "gamma": 100},
}


def request(command):
    with tempfile.TemporaryDirectory(prefix="2pybot-cam-", dir="/dev/shm") as folder, socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
        sock.settimeout(0.8)
        sock.bind(str(Path(folder) / "reply.sock"))
        sock.sendto(command.encode("ascii"), CONTROL_SOCKET)
        return json.loads(sock.recv(16384))


def status():
    if not Path(CONTROL_SOCKET).exists():
        return None
    try:
        data = request("GET")
        return data if data.get("ok") and data.get("backend") == "imx708-isp" else None
    except (OSError, ValueError):
        return None


def controls(info):
    definitions = [
        ("exposure_auto", 0, 1, 0, "menu", "0 Auto / 1 Manual"),
        ("exposure_us", 54, info["exposure_max_us"], 8000, "int", "microseconds; limited to one sensor frame"),
        ("gain", 16, 256, 16, "int", "Q4 analogue gain: 16 = 1x, 64 = 4x"),
        ("white_balance_auto", 0, 1, 1, "bool", "neutral-reference white balance; freezes without a neutral reference"),
        ("color_temperature", 2964, 7590, 5000, "int", "Kelvin; IMX708-calibrated white balance and linear colour matrix"),
        ("red_balance", 64, 4095, 512, "int", "Q8.8 Bayer gain: 256 = 1x"),
        ("blue_balance", 64, 4095, 512, "int", "Q8.8 Bayer gain: 256 = 1x"),
        ("contrast", 80, 180, 100, "int", "%; 100 = sensor-profile output unchanged"),
        ("saturation", 0, 200, 100, "int", "%; sensor colour matrix supplies the calibrated colour"),
        ("black_point", 0, 48, 0, "int", "output adjustment; default 0 with the sensor profile"),
        ("gamma", 70, 160, 100, "int", "%; 100 = no shadow lift"),
    ]
    return [{"name": name, "min": lo, "max": hi, "default": default,
             "step": 1, "type": kind, "unit": unit,
             "menu": {0: "Auto", 1: "Manual"} if name == "exposure_auto" else {}}
            for name, lo, hi, default, kind, unit in definitions]


def _save_value(name, value):
    path = Path(TUNING_FILE)
    with _lock:
        try:
            values = json.loads(path.read_text())
        except (OSError, ValueError):
            values = {}
        values[name] = value
        if name in ("exposure_us", "gain"):
            values["exposure_auto"] = 1
        if name in ("red_balance", "blue_balance", "color_temperature"):
            values["white_balance_auto"] = 0
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as handle:
            json.dump(values, handle)
            temp_path = handle.name
        os.replace(temp_path, path)


def set_control(name, value, persist=True):
    info = status()
    if not info:
        return False, "CSI publisher control channel is unavailable"
    descriptor = next((c for c in controls(info) if c["name"] == name), None)
    if not descriptor or not descriptor["min"] <= value <= descriptor["max"]:
        return False, "unknown or out-of-range camera control"
    try:
        result = request(f"SET {name} {int(value)}")
        if not result.get("ok"):
            return False, result.get("error", "camera rejected control")
        if persist:
            _save_value(name, int(value))
        return True, "applied through capture owner"
    except (OSError, ValueError) as exc:
        return False, str(exc)


def apply_values(values, persist=True):
    applied, skipped = {}, []
    mode = int(values.get("exposure_auto", 0))
    wb_auto = int(values.get("white_balance_auto", 1))
    ordered = [(k, v) for k, v in values.items() if k not in ("exposure_auto", "white_balance_auto")]
    for key, value in ordered:
        if mode == 0 and key in ("exposure_us", "gain"):
            continue
        if wb_auto and key in ("red_balance", "blue_balance"):
            continue
        ok, _ = set_control(key, int(value), persist)
        if ok:
            applied[key] = value
        else:
            skipped.append(key)
    for key in ("white_balance_auto", "exposure_auto"):
        if key in values:
            ok, _ = set_control(key, int(values[key]), persist)
            if ok:
                applied[key] = values[key]
            else:
                skipped.append(key)
    return {"applied": applied, "skipped": skipped}


async def restore():
    for _ in range(40):
        if status():
            try:
                values = json.loads(Path(TUNING_FILE).read_text())
            except (OSError, ValueError):
                return
            await asyncio.get_running_loop().run_in_executor(None, lambda values=values: apply_values(values, False))
            return
        await asyncio.sleep(.5)
