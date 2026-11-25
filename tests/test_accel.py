import importlib.util
from pathlib import Path

source=Path(__file__).resolve().parents[1]/"software/radxa/console/console/accel.py"
spec=importlib.util.spec_from_file_location("accel",source)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_npu_is_picked_first_when_available(monkeypatch):
    monkeypatch.delenv("BOT_ONNX_PROVIDERS",raising=False)
    chosen=module.select_providers(["CPUExecutionProvider","XnnpackExecutionProvider","VSINPUExecutionProvider"])
    assert chosen[0]=="VSINPUExecutionProvider"
    assert chosen[-1]=="CPUExecutionProvider"
    assert module.provider_label(chosen)=="NPU (VeriSilicon)"


def test_xnnpack_beats_plain_cpu(monkeypatch):
    monkeypatch.delenv("BOT_ONNX_PROVIDERS",raising=False)
    chosen=module.select_providers(["CPUExecutionProvider","XnnpackExecutionProvider"])
    assert chosen==["XnnpackExecutionProvider","CPUExecutionProvider"]


def test_cpu_only_host_still_works(monkeypatch):
    monkeypatch.delenv("BOT_ONNX_PROVIDERS",raising=False)
    assert module.select_providers(["CPUExecutionProvider"])==["CPUExecutionProvider"]
    assert module.provider_label(["CPUExecutionProvider"])=="CPU"


def test_cpu_is_always_appended(monkeypatch):
    monkeypatch.delenv("BOT_ONNX_PROVIDERS",raising=False)
    chosen=module.select_providers(["VSINPUExecutionProvider"])
    assert chosen==["VSINPUExecutionProvider","CPUExecutionProvider"]


def test_override_wins_but_drops_unavailable_and_keeps_cpu(monkeypatch):
    monkeypatch.setenv("BOT_ONNX_PROVIDERS","CUDAExecutionProvider,XnnpackExecutionProvider")
    chosen=module.select_providers(["CPUExecutionProvider","XnnpackExecutionProvider"])
    assert chosen==["XnnpackExecutionProvider","CPUExecutionProvider"]


def test_override_can_force_cpu_order(monkeypatch):
    monkeypatch.setenv("BOT_ONNX_PROVIDERS","CPUExecutionProvider")
    chosen=module.select_providers(["CPUExecutionProvider","XnnpackExecutionProvider","VSINPUExecutionProvider"])
    assert chosen==["CPUExecutionProvider"]
