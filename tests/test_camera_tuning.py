import importlib
import sys
from pathlib import Path

import pytest

APP = Path(__file__).resolve().parents[1] / "software/radxa/console"
sys.path.insert(0, str(APP))
native = importlib.import_module("console.native_camera")
camera = importlib.import_module("console.camera")


def info():
    return {"ok": True, "backend": "imx708-isp", "exposure_max_us": 17208,
            "values": {"exposure_auto": 0, "exposure_us": 8000, "gain": 16,
                       "contrast": 125, "saturation": 150, "gamma": 100, "black_point": 16}}


def test_csi_controls_use_microseconds_and_real_gain_bounds():
    controls = {c["name"]: c for c in native.controls(info())}
    assert controls["exposure_us"]["max"] == 17208
    assert controls["gain"]["min"] == 16
    assert controls["gain"]["max"] == 256
    assert "microseconds" in controls["exposure_us"]["unit"]


def test_native_enumeration_never_opens_busy_capture_device(monkeypatch):
    monkeypatch.setattr(native, "status", info)
    monkeypatch.setattr(camera, "_run", lambda *args, **kwargs: pytest.fail("competing v4l2-ctl query"))
    assert any(c["name"] == "exposure_us" for c in camera.list_controls())
    assert camera.get_control("exposure_us") == 8000


def test_manual_exposure_persists_manual_mode_and_rejects_long_frames(monkeypatch, tmp_path):
    monkeypatch.setattr(native, "status", info)
    calls = []
    monkeypatch.setattr(native, "request", lambda command: calls.append(command) or info())
    monkeypatch.setattr(native, "TUNING_FILE", str(tmp_path / "tuning.json"))
    assert native.set_control("exposure_us", 6000)[0]
    assert calls == ["SET exposure_us 6000"]
    import json
    saved = json.loads((tmp_path / "tuning.json").read_text())
    assert saved["exposure_auto"] == 1
    assert not native.set_control("exposure_us", 30000)[0]


def test_auto_profile_does_not_reapply_stale_manual_exposure(monkeypatch):
    calls = []
    monkeypatch.setattr(native, "set_control", lambda key, value, persist=True: calls.append((key, value)) or (True, ""))
    native.apply_values({"exposure_auto": 0, "exposure_us": 15000, "gain": 128, "contrast": 125}, False)
    assert ("exposure_us", 15000) not in calls
    assert ("gain", 128) not in calls
    assert calls[-1] == ("exposure_auto", 0)
