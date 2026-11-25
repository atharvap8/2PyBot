"""VIPLite NPU backend detection for the on-board detector.

On the robot's Allwinner A733 the NPU is reached through the VIPLite driver and
an ACUITY-compiled NBG model, not through ONNX Runtime: the Unified OpenVX and
galcore stack that ONNX Runtime's VSINPU provider needs is absent on this board
(verified; see docs/NPU_ACCELERATION.md). The NPU itself works: a sample NBG runs
on it in about 3 ms.

What is missing is only our model in NBG form, which has to be compiled from the
ONNX on an x86 host with the ACUITY toolkit. This module reports whether the
three pieces are in place (the kernel device, the VIPLite driver libraries and a
compiled NBG) so the console can show NPU readiness and switch to it the moment a
`.nb` is dropped in. Until then detection stays on ONNX/CPU and this explains why.

Pure path and environment checks only; safe to import and call anywhere.
"""
import os
from pathlib import Path

# The VIPLite kernel driver (aw_nna_vip) exposes this node on the A733.
DEVICE = Path(os.environ.get("BOT_NPU_DEVICE", "/dev/vipcore"))
# The A733 VIPLite v2.0 user libraries from Radxa's ai-sdk. Override with
# BOT_NPU_LIB_DIR if the sdk lives elsewhere.
DEFAULT_LIB_DIR = "/home/radxa/ai-sdk/viplite-tina/lib/aarch64-none-linux-gnu/v2.0"
# An ACUITY-compiled NBG for the detector. Produced off-board on an x86 host.
DEFAULT_MODEL = "/var/lib/2pybot/models/ssd-mobilenet-v1.nb"


def _lib_dir():
    d = Path(os.environ.get("BOT_NPU_LIB_DIR", DEFAULT_LIB_DIR))
    return d if (d / "libVIPhal.so").is_file() else None


def _model_path():
    p = Path(os.environ.get("BOT_NPU_MODEL", DEFAULT_MODEL))
    return p if p.is_file() else None


def status():
    """Readiness of the VIPLite NPU path, with a human reason when not usable."""
    lib = _lib_dir()
    model = _model_path()
    device = DEVICE.exists()
    reasons = []
    if not device:
        reasons.append(f"no {DEVICE} (VIPLite kernel driver not loaded)")
    if lib is None:
        reasons.append("VIPLite driver libraries not found (set BOT_NPU_LIB_DIR)")
    if model is None:
        reasons.append("no compiled NBG model (compile the ONNX with ACUITY on x86, set BOT_NPU_MODEL)")
    usable = bool(device and lib is not None and model is not None)
    return {
        "backend": "viplite",
        "device": bool(device),
        "driver": lib is not None,
        "model": model is not None,
        "usable": usable,
        "lib_dir": str(lib) if lib else None,
        "model_path": str(model) if model else None,
        "reason": "" if usable else "; ".join(reasons),
    }


def available():
    return status()["usable"]
