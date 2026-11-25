# Converting the detector to an NPU model (ACUITY, x86 Linux)

The robot's A733 NPU runs ACUITY-compiled NBG models through the VIPLite driver
(proven: `scripts/vision/npu_bench.sh`, about 3 ms per inference). ACUITY, the
tool that turns our ONNX into an NBG, is an x86-64 Linux Docker image, so this
step runs on a PC, not on the robot. Do it from the Arch install (or any x86-64
Linux with Docker), then copy the resulting `.nb` to the bot.

Everything below is the host side. The on-board side is already wired:
`console/npu.py` picks up the model at `BOT_NPU_MODEL`
(default `/var/lib/2pybot/models/ssd-mobilenet-v1.nb`).

## 1. Docker

Arch: `sudo pacman -S docker && sudo systemctl enable --now docker`
(Add yourself to the `docker` group or use sudo.)

## 2. Get the ACUITY image (A733, v2.0)

Download the A733 ACUITY archive from Allwinner's netdisk (linked from Radxa's
"ACUITY Toolkit Environment Configuration" doc):
<https://netstorage.allwinnertech.com:5001/sharing/Mh23BhPHq>

```
unzip docker_images_v2.0.x.zip
cd docker_images_v2.0.x
unzip ubuntu-npu_v2.0.10.2.tar.zip
sudo docker load -i ubuntu-npu_v2.0.10.2.tar      # -> ubuntu-npu:v2.0.10.2
```

## 3. Start the container with this repo mounted

Run from the repo root so the container sees `scripts/vision/acuity`:

```
sudo docker run --ipc=host -itd \
  -v "$PWD":/workspace --name acuity ubuntu-npu:v2.0.10.2 /bin/bash
sudo docker exec -it acuity /bin/bash
```

## 4. Convert, inside the container

```
cd /workspace/scripts/vision/acuity
./convert.sh
```

`convert.sh` clones `ai-sdk`, selects the A733 NPU version (`v3`), imports the
model, quantizes per-channel int8 (pcq, the method that kept accuracy in Radxa's
example), and exports the NBG. It writes `out/network_binary.nb`.

Before the first run, prepare the model directory it expects (see
`model/README-model.txt`): the ONNX, `dataset.txt` listing calibration images
(include at least one frame with the EVOFOX gamepad held to the camera, since the
re-pair gesture relies on the "remote" class), and `inputs_outputs.txt` with the
input and output node names. SSD-MobileNet-V1 takes a 300x300 uint8 RGB tensor
with no normalization, so mean 0 and scale 1 in the generated `*_inputmeta.yml`.

## 5. Sanity-check the NBG on the bot

Copy it over and run it through the proven runner:

```
scp out/network_binary.nb radxa@2pybot.local:/var/lib/2pybot/models/ssd-mobilenet-v1.nb
# on the bot, point vpm_run at it, or just confirm console/npu.py reports usable:
#   python3 -c "from console.npu import status; print(status())"
```

Once `BOT_NPU_MODEL` resolves and `/dev/vipcore` and the VIPLite libs are present,
`console/npu.py` reports `usable: true`. The thin VIPLite inference call that
feeds `decode_outputs()` is the last code step (tracked in
docs/NPU_ACCELERATION.md, Phase 2).

## Notes

- Our shipped `ssd-mobilenet-v1-int8.onnx` is already INT8. ACUITY prefers to
  quantize a float model itself; importing a pre-quantized QDQ graph can error.
  The robust route is a float SSD-MobileNet-V1 ONNX plus pcq here. If you must use
  the int8 graph, export with `float` (no re-quantization) and verify accuracy.
- NPU version for A733 is `v3` / software `v2.0` (Radxa NPU version table).
- If import fails on output nodes, set them explicitly in `inputs_outputs.txt`.
