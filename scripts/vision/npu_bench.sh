#!/bin/sh
# Prove and benchmark the Allwinner A733 (Vivante VIP9000) NPU on the robot.
#
# This reproduces the verified Phase 1 result: it fetches Radxa's ai-sdk (which
# carries the A733 VIPLite v2.0 driver and the vpm_run runner), builds vpm_run
# with the stock compiler, and runs a sample NBG on the NPU, printing the
# per-inference time. It changes nothing on the system and needs no sudo.
#
# Usage: sh npu_bench.sh [loops]   (default 10)
# See docs/NPU_ACCELERATION.md. The Unified OpenVX / ONNX Runtime VSINPU path is
# not available on this board; the NPU runs ACUITY-compiled NBG via VIPLite.
set -e
LOOPS="${1:-10}"
SDK="${AI_SDK_DIR:-$HOME/ai-sdk}"
VIPLIB="$SDK/viplite-tina/lib/aarch64-none-linux-gnu/v2.0"

[ -e /dev/vipcore ] || { echo "no /dev/vipcore: VIPLite kernel driver not loaded"; exit 1; }

if [ ! -d "$SDK/.git" ]; then
    echo "cloning ai-sdk into $SDK ..."
    git clone --depth 1 https://github.com/ZIFENG278/ai-sdk.git "$SDK"
fi

echo "building vpm_run (native, a733) ..."
make -C "$SDK/examples/vpm_run" clean >/dev/null 2>&1 || true
make -C "$SDK/examples/vpm_run" AI_SDK_PLATFORM=a733 vpm_run

RUN="$(mktemp -d)"
cp "$SDK/examples/vpm_run/vpm_run" "$RUN/"
cp "$SDK/examples/vpm_run/operator/v3/network_binary.nb" "$RUN/"
cp "$SDK/examples/vpm_run/operator/input_0.dat" "$RUN/"
cp "$SDK/examples/vpm_run/operator/sample.txt" "$RUN/"

echo "running sample NBG on the NPU ($LOOPS loops) ..."
cd "$RUN"
LD_LIBRARY_PATH="$VIPLIB" ./vpm_run -s sample.txt -l "$LOOPS" -b 1 | grep -iE "inference time|avg inference|vip lite init|ret="
rm -rf "$RUN"
