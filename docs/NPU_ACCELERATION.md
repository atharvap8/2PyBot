# NPU and GPU acceleration for on-board detection

The console runs one small vision model: an INT8 SSD-MobileNet-V1 COCO detector
(`object_detection.py` for the camera workspace, `visual_pair.py` for the
show-the-gamepad re-pair gesture). Today both run ONNX Runtime on the CPU with a
single thread. This note records what the robot's hardware can do, the plan to
move the detector off the CPU, and the small first step that is already in the
tree.

## The hardware we actually have

The robot's computer is a Radxa Cubie A7A, Allwinner A733 SoC:

- CPU: 2x Cortex-A76 plus 6x Cortex-A55.
- NPU: 3 TOPS, a VeriSilicon Vivante NPU. Radxa's toolchain for it is the
  ACUITY Toolkit, which converts a float or quantised model into a Network
  Binary Graph (`.nb`, "NBG"), run on device through VeriSilicon OVXLib. Radxa
  ships `vpm_run` and `NBinfo` to test an NBG, plus a model zoo.
- GPU: Imagination BXM-4-64 MC1, with Vulkan 1.3 and OpenCL 3.0.

Two consequences follow immediately:

1. The NPU is a VeriSilicon part. That is the one family ONNX Runtime can target
   on this board through the **VSINPU execution provider**, which is built on
   VeriSilicon TIM-VX. This is the straight line from "we have an ONNX model" to
   "it runs on the NPU".
2. The GPU is Imagination, not Mali or Adreno or NVIDIA. ONNX Runtime has no
   PowerVR execution provider, so there is no drop-in ORT GPU path here. A GPU
   route exists but means leaving ORT for that one model (see GPU section).

## Why bother

The detector is bounded on purpose (one 300x300 inference about once per second),
so the CPU cost is not large today. The reasons to accelerate it anyway:

- Head-room. The A55 cores also carry SLAM, the web console, WebRTC and serial.
  Taking the ~tens of milliseconds of INT8 convolution off the CPU frees those
  cores for control-loop-adjacent work and keeps the board cooler (the doctor
  already watches `cpu_hot`).
- Head-room to do more. Once detection is cheap we can raise the rate, run a
  second model, or use a larger, more accurate detector.

So the goal is correctness and head-room, not fixing a present fire.

## Plan, in order of effort and payoff

### Phase 0: pick the best available provider (done, in this commit)

`console/accel.py` chooses the fastest execution provider ONNX Runtime actually
reports as available, and always appends `CPUExecutionProvider` as the fallback,
so the console behaves identically on a laptop, in CI and on the robot. Order of
preference: an operator override (`BOT_ONNX_PROVIDERS`), then the VeriSilicon NPU
provider, then any other accelerator providers that happen to be present, then
XNNPACK (SIMD CPU kernels that ship in the stock wheel and are a free win over
the default CPU provider on ARM), then plain CPU.

`object_detection.py` and `visual_pair.py` now build their sessions through this
helper and report the provider they are actually using (the camera overlay and
`/api/object_detection` status show it instead of the hard-coded "ONNX CPU").

This phase changes no hardware behaviour on its own. On the stock Radxa image it
picks XNNPACK when present, otherwise CPU. It is the plumbing every later phase
needs, and it makes "did the NPU actually get used" observable.

### Phase 1: run the detector on the NPU through ACUITY and vpm_run (measurement)

Vendor path, lowest risk, proves the NPU works for our model before we commit to
a runtime.

1. Install the ACUITY Toolkit per Radxa's NPU docs.
2. Convert `ssd-mobilenet-v1-int8.onnx` to NBG, feeding the same INT8
   quantisation we already use. Use ACUITY's quantisation-accuracy tooling to
   confirm the "remote" class the re-pair gesture relies on survives conversion.
3. Benchmark with `vpm_run`; compare latency and detections against the CPU
   baseline on the same snapshots from `experiments/visual_pair/`.

Deliverable: a measured NPU-vs-CPU number and a known-good NBG. No console change
yet. If the numbers do not justify the integration work, we stop here and keep
XNNPACK.

### Phase 2: drive the NPU from the console

Two ways to go, decided by Phase 1 and by what the stock image provides:

- **A. ONNX Runtime VSINPU EP (preferred, keeps our ONNX pipeline).** Build ORT
  with the VSINPU EP against the board's TIM-VX and OVXLib, package the wheel for
  the deploy venv, and add `VSINPUExecutionProvider` as the top auto pick (the
  `accel.py` order already lists it first). Our Python, pre- and post-processing
  and `decode_outputs` stay exactly as they are. Nodes the EP cannot take fall
  back to CPU automatically.
- **B. NBG plus a thin runner.** If a maintained VSINPU wheel is not practical,
  wrap `vpm_run`/OVXLib behind the same small interface `accel.py` exposes, load
  the NBG from Phase 1, and keep ONNX+CPU as the fallback path. More glue code,
  but no custom ORT build to maintain.

Either way the gate stays: accelerated when available, CPU otherwise, with the
provider reported in status so a regression is visible.

### Phase 3 (optional): GPU path

Only if the NPU route stalls. The Imagination GPU does Vulkan 1.3 and OpenCL
3.0, but ONNX Runtime cannot target it. Realistic options, all of which mean
running this one model outside ORT:

- **ncnn with its Vulkan backend.** Convert ONNX to ncnn, run on the BXM GPU
  over Vulkan. Mature, self-contained, good ARM and mobile-GPU support.
- **OpenCL via CLBlast or a TVM-compiled module.** More work, more control.

Given the NPU is purpose-built for this and is simpler to reach through VSINPU,
GPU is the backup, not the plan.

## Writing our own / contributing upstream

If a usable VSINPU wheel or NBG runner is not available for this exact image, the
work needed is real but bounded, and it is the kind of thing worth sending back:

- The open stack under the NPU is **TIM-VX** (VeriSilicon's own, open source) and
  the ONNX Runtime **VSINPU EP** (already upstream in ORT). Making these build
  and run against the A733's OVXLib and kernel driver, then publishing the build
  recipe and wheel, is the highest-value contribution: it turns every A733 board
  into an ORT NPU target, not just ours.
- The fully open driver route is **mesa Teflon** (the TensorFlow Lite delegate on
  the etnaviv/VeriSilicon NPU driver). This avoids the vendor blob entirely but
  targets TFLite, not ONNX, so it only helps if we also keep a TFLite copy of the
  model.
- Any fixes (build flags, missing ops, quantisation mismatches, packaging) go to
  Radxa's open-source repositories and to the ORT VSINPU EP so the next person
  does not repeat Phase 1 and 2.

Scope control: we write our own integration glue and build recipe. We do not
write a kernel driver or an NPU compiler from scratch; ACUITY, OVXLib, TIM-VX and
the VSINPU EP already exist and are the supported path.

## Status today

- `accel.py` provider selection: in the tree, unit-tested, wired into both
  detectors, provider shown in status. CPU fallback guaranteed.
- Phases 1 to 3: not started; they need the robot and the Radxa NPU toolchain.
