"""Pick the fastest ONNX Runtime execution provider that is actually available.

On-board detection runs on the CPU today. This module selects an accelerated
provider when the running ONNX Runtime reports one (the robot's Allwinner A733
has a VeriSilicon NPU, reachable through the VSINPU execution provider), and it
always appends the plain CPU provider as a guaranteed fallback. The result is
the same code path on a laptop, in CI and on the robot: it just runs faster
where faster hardware exists.

See docs/NPU_ACCELERATION.md for the hardware and the roadmap.
"""
import os

# Preference order, fastest or most specialised first. CPU is always appended.
# NPU first (VeriSilicon, the A733's accelerator, via the VSINPU EP), then other
# accelerator EPs that may appear on other hosts, then XNNPACK (SIMD CPU kernels
# that ship in the stock wheel and beat the default CPU provider on ARM), then
# the plain CPU provider.
PREFERRED = (
    "VSINPUExecutionProvider",      # VeriSilicon NPU (TIM-VX) - the A733 target
    "VitisAIExecutionProvider",
    "MIGraphXExecutionProvider",
    "ROCMExecutionProvider",
    "CUDAExecutionProvider",
    "OpenVINOExecutionProvider",
    "ArmNNExecutionProvider",
    "ACLExecutionProvider",         # Arm Compute Library (NEON or GPU)
    "XnnpackExecutionProvider",     # SIMD CPU kernels, in the stock wheel
    "CPUExecutionProvider",
)

_LABELS = {
    "VSINPUExecutionProvider": "NPU (VeriSilicon)",
    "VitisAIExecutionProvider": "NPU (Vitis AI)",
    "MIGraphXExecutionProvider": "GPU (MIGraphX)",
    "ROCMExecutionProvider": "GPU (ROCm)",
    "CUDAExecutionProvider": "GPU (CUDA)",
    "OpenVINOExecutionProvider": "OpenVINO",
    "ArmNNExecutionProvider": "Arm NN",
    "ACLExecutionProvider": "Arm Compute (NEON/GPU)",
    "XnnpackExecutionProvider": "CPU (XNNPACK SIMD)",
    "CPUExecutionProvider": "CPU",
}


def select_providers(available=None):
    """Return the ordered provider list to hand to InferenceSession.

    ``available`` defaults to ``onnxruntime.get_available_providers()``. An
    operator can override the choice with ``BOT_ONNX_PROVIDERS`` (a comma list of
    provider names); unavailable names in the override are dropped. The plain CPU
    provider is always present as the last resort.
    """
    if available is None:
        import onnxruntime as ort
        available = ort.get_available_providers()
    available = list(available)
    override = os.environ.get("BOT_ONNX_PROVIDERS", "").strip()
    if override:
        wanted = [name.strip() for name in override.split(",") if name.strip()]
    else:
        wanted = [name for name in PREFERRED if name in available]
    chosen = [name for name in wanted if name in available]
    if "CPUExecutionProvider" not in chosen:
        chosen.append("CPUExecutionProvider")
    return chosen


def provider_label(providers):
    """Short human label for the primary (first) provider, for status and overlays."""
    primary = providers[0] if providers else "CPUExecutionProvider"
    return _LABELS.get(primary, primary)
