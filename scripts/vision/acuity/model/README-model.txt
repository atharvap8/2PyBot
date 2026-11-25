Put the detector source model for ACUITY here, in a directory named to match
MODEL_NAME in convert.sh (default: ssd_mobilenet_v1/).

    model/ssd_mobilenet_v1/
      ssd_mobilenet_v1.onnx        the source model (see note on quantization)
      dataset.txt                  one calibration image path per line
      inputs_outputs.txt           input and output node names
      calib/                       the calibration images referenced by dataset.txt

Quantization source model
-------------------------
ACUITY quantizes a FLOAT model itself. Prefer a float SSD-MobileNet-V1 ONNX whose
outputs are num_detections / detection_boxes / detection_scores / detection_classes
(same decode as console/object_detection.py). Then convert.sh runs pcq (per-channel
int8), which kept accuracy in Radxa's example.
Our shipped ssd-mobilenet-v1-int8.onnx is already INT8; importing a pre-quantized
QDQ graph can fail. If you use it, run convert.sh with QUANT=float and verify the
detections match the CPU baseline.

dataset.txt
-----------
20 to 50 representative frames from the robot's own camera. Include at least one
frame with the EVOFOX gamepad held face-on to the camera, because the visual
re-pair gesture depends on the "remote" class surviving quantization. Grab frames
from /dev/shm/2pybot_snap.jpg on the bot, or experiments/visual_pair/.
Example lines (paths are relative to the model dir inside the container):
    ./calib/room_01.jpg
    ./calib/gamepad_01.jpg

inputs_outputs.txt
------------------
The input and output node names of the ONNX. If import fails to find outputs,
set them explicitly here. Inspect with: python3 -c "import onnx;m=onnx.load('x.onnx');
print([i.name for i in m.graph.input],[o.name for o in m.graph.output])".

Input preprocessing
-------------------
SSD-MobileNet-V1 takes a 300x300 uint8 RGB tensor with no normalisation. convert.sh
sets mean 0 and scale 1 in the generated *_inputmeta.yml. Review that file if the
input shape or layout differs.
