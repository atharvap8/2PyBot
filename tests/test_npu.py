import importlib.util
from pathlib import Path

source = Path(__file__).resolve().parents[1] / "software/radxa/console/console/npu.py"
spec = importlib.util.spec_from_file_location("npu", source)
npu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(npu)


def _env(monkeypatch, device, lib_dir, model):
    monkeypatch.setenv("BOT_NPU_DEVICE", str(device))
    monkeypatch.setenv("BOT_NPU_LIB_DIR", str(lib_dir))
    monkeypatch.setenv("BOT_NPU_MODEL", str(model))
    # DEVICE is read at import time into a module constant; refresh it.
    npu.DEVICE = Path(str(device))


def test_usable_when_all_three_present(tmp_path, monkeypatch):
    dev = tmp_path / "vipcore"; dev.write_text("")
    lib = tmp_path / "lib"; lib.mkdir(); (lib / "libVIPhal.so").write_text("")
    model = tmp_path / "ssd.nb"; model.write_text("")
    _env(monkeypatch, dev, lib, model)
    s = npu.status()
    assert s["usable"] is True and s["reason"] == ""
    assert npu.available() is True
    assert s["backend"] == "viplite"


def test_missing_model_is_the_blocker(tmp_path, monkeypatch):
    dev = tmp_path / "vipcore"; dev.write_text("")
    lib = tmp_path / "lib"; lib.mkdir(); (lib / "libVIPhal.so").write_text("")
    _env(monkeypatch, dev, lib, tmp_path / "absent.nb")
    s = npu.status()
    assert s["usable"] is False and s["device"] and s["driver"] and not s["model"]
    assert "NBG" in s["reason"]


def test_missing_device_and_driver_reported(tmp_path, monkeypatch):
    _env(monkeypatch, tmp_path / "absent", tmp_path / "absent_lib", tmp_path / "absent.nb")
    s = npu.status()
    assert s["usable"] is False
    assert not s["device"] and not s["driver"] and not s["model"]
    assert "VIPLite kernel driver not loaded" in s["reason"] and "VIPLite driver libraries" in s["reason"]
