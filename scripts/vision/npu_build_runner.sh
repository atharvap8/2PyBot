#!/bin/sh
# Build the persistent NPU inference coprocess (awnn_runner) on the robot.
# Reuses Radxa's ai-sdk awnn/VIPLite helper. Installs to /usr/local/bin/awnn_runner.
# Run on the bot. Needs the ai-sdk checkout (scripts/vision/npu_bench.sh fetches it).
set -e
SDK="${AI_SDK_DIR:-$HOME/ai-sdk}"
HERE="$(cd "$(dirname "$0")" && pwd)"
VIPLIB="$SDK/viplite-tina/lib/aarch64-none-linux-gnu/v2.0"
AWNN="$SDK/examples/libawnn_viplite"
OUT="${1:-/usr/local/bin/awnn_runner}"

[ -d "$SDK" ] || { echo "no ai-sdk at $SDK; run scripts/vision/npu_bench.sh first"; exit 1; }

# -fcommon: awnn_internal.h defines tentative globals (time_begin/_end) that are
# pulled into every unit that includes it; merge them instead of erroring.
gcc -O2 -fcommon -o /tmp/awnn_runner \
  "$HERE/awnn_runner.c" "$AWNN/awnn_lib.c" "$AWNN/awnn_quantize.c" \
  -I"$SDK" -I"$AWNN" -I"$VIPLIB/inc" \
  -L"$VIPLIB" -lNBGlinker -lVIPhal -lpthread -lm

sudo install -m 0755 /tmp/awnn_runner "$OUT"
echo "installed $OUT"
echo "runtime needs: LD_LIBRARY_PATH=$VIPLIB  (BOT_NPU_LIB_DIR)"
