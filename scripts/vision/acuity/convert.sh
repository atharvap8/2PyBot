#!/bin/bash
# Convert the detector ONNX to an A733 NBG with the ACUITY toolkit.
# Run INSIDE the ubuntu-npu:v2.0.10.2 container, from this directory
# (/workspace/scripts/vision/acuity). See README.md for the host setup.
#
# Produces: out/network_binary.nb  (copy to the bot at BOT_NPU_MODEL)
#
# Config (override via env):
#   MODEL_NAME  model directory name under model/ and ai-sdk/models  (ssd_mobilenet_v1)
#   QUANT       pcq | int16 | uint8 | float   (pcq = per-channel int8, best accuracy)
#   ITER        calibration iterations for quantization              (10)
set -e

MODEL_NAME="${MODEL_NAME:-ssd_mobilenet_v1}"
QUANT="${QUANT:-pcq}"
ITER="${ITER:-10}"
HERE="$(cd "$(dirname "$0")" && pwd)"
SDK="${AI_SDK_DIR:-$HOME/ai-sdk}"

echo "== model=$MODEL_NAME quant=$QUANT iter=$ITER =="

[ -d "$HERE/model/$MODEL_NAME" ] || {
  echo "ERROR: put the model in $HERE/model/$MODEL_NAME (see model/README-model.txt)"; exit 1; }

# 1. ai-sdk provides env.sh (NPU version) and the pegasus_* wrapper scripts
if [ ! -d "$SDK/.git" ]; then
  git clone --depth 1 https://github.com/ZIFENG278/ai-sdk.git "$SDK"
fi
cd "$SDK/models"
# A733 is NPU version v3 (software v2.0); see Radxa NPU version table
source env.sh v3
cp ../scripts/* .

# 2. stage our model dir into ai-sdk/models
rm -rf "./$MODEL_NAME"
cp -r "$HERE/model/$MODEL_NAME" "./$MODEL_NAME"

# 3. import (parses the framework model -> .json/.data + templates)
echo "== pegasus_import =="
./pegasus_import.sh "$MODEL_NAME/"

# 4. input preprocessing: our tensor is 300x300 uint8 RGB with no normalisation,
#    so mean 0 and scale 1. Best-effort patch; review the yml if the shape differs.
META="$MODEL_NAME/${MODEL_NAME}_inputmeta.yml"
if [ -f "$META" ]; then
  python3 - "$META" <<'PY' || echo "NOTE: could not auto-set mean/scale; edit $META by hand"
import re, sys
p = sys.argv[1]; t = open(p).read()
t = re.sub(r'(mean:\s*\n)(\s*-\s*[-\d.]+\n)+', lambda m: m.group(1)+''.join('  - 0\n' for _ in re.findall(r'-\s*[-\d.]+', m.group(0))), t)
t = re.sub(r'(scale:\s*\n)(\s*-\s*[-\d.]+\n)+', lambda m: m.group(1)+''.join('  - 1\n' for _ in re.findall(r'-\s*[-\d.]+', m.group(0))), t)
open(p,'w').write(t); print("patched mean=0 scale=1 in", p)
PY
  echo "----- $META -----"; sed -n '1,60p' "$META"; echo "-----"
fi

# 5. quantize (skip for an already-quantized source: set QUANT=float)
if [ "$QUANT" != "float" ]; then
  echo "== pegasus_quantize ($QUANT) =="
  ./pegasus_quantize.sh "$MODEL_NAME" "$QUANT" "$ITER"
fi

# 6. export OpenVX + NBG
echo "== pegasus_export_ovx ($QUANT) =="
./pegasus_export_ovx.sh "$MODEL_NAME" "$QUANT"

# 7. collect the NBG
NB="$(find "$MODEL_NAME/wksp" -name network_binary.nb -path '*nbg*' | head -1)"
[ -n "$NB" ] || NB="$(find "$MODEL_NAME/wksp" -name network_binary.nb | head -1)"
mkdir -p "$HERE/out"
if [ -n "$NB" ]; then
  cp "$NB" "$HERE/out/network_binary.nb"
  echo "== DONE: $HERE/out/network_binary.nb =="
  echo "scp it to: radxa@2pybot.local:/var/lib/2pybot/models/ssd-mobilenet-v1.nb"
else
  echo "ERROR: no network_binary.nb produced; inspect $SDK/models/$MODEL_NAME/wksp"; exit 1
fi
