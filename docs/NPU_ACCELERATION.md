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

1. The NPU is a VeriSilicon part. In general that is the one family ONNX Runtime
   can target, through the **VSINPU execution provider** built on TIM-VX. But see
   the verified on-device findings below: on this exact board the runtime that
   VSINPU and TIM-VX need is not present, so the real path here is ACUITY to NBG
   run through the VIPLite driver, not ONNX Runtime.
2. The GPU is Imagination, not Mali or Adreno or NVIDIA. ONNX Runtime has no
   PowerVR execution provider, so there is no drop-in ORT GPU path here. A GPU
   route exists but means leaving ORT for that one model (see GPU section).

## On-device findings (verified on the robot, Nov 2026)

Probed and tested over SSH on the actual board. These change the plan, so they
are recorded here rather than left as assumptions:

- Board is a Radxa Cubie A7A, `allwinner,sun60i-a733`, Debian 11, kernel
  `5.15.147-18-a733`. NPU is the Vivante **VIP9000** (cid `0x1000003b`), clocked
  to 1.2 GHz.
- The loaded NPU kernel driver is **`aw_nna_vip`**, which exposes **`/dev/vipcore`**.
  This is the **VIPLite** stack. There is **no `galcore` module and no
  `/dev/galcore`** on the board.
- The VeriSilicon **Unified OpenVX** userspace driver (`libGAL.so`) opens
  `/dev/galcore`. Because galcore is absent, the Unified OpenVX runtime does not
  work here. **TIM-VX and the ONNX Runtime VSINPU EP both sit on that Unified
  OpenVX runtime, so they cannot drive this NPU.** Building ONNX Runtime with
  VSINPU would not help on this board.
- The supported path is **offline: ACUITY compiles the model to an NBG, and the
  lightweight VIPLite driver (`/dev/vipcore`) runs it.** This is proven working:
  Radxa's `ai-sdk` repo (`github.com/ZIFENG278/ai-sdk`) carries the A733 VIPLite
  driver (`viplite-tina/.../v2.0`) and the `vpm_run` runner. Built on-device with
  the stock gcc and ran a sample NBG on the NPU in a **measured ~2.9 to 3.4 ms
  per inference** over ten loops. So the NPU itself, the driver and the toolchain
  all work; what is missing is only our model in NBG form.

Net effect on the plan: keep `accel.py` (it still correctly picks XNNPACK or CPU,
and VSINPU on any board that does expose it), but the actual acceleration of this
robot's detector goes through ACUITY to NBG to VIPLite, described next.

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

### Phase 1: prove the NPU runs a model on the board (DONE)

Verified on hardware (see on-device findings): cloned Radxa's `ai-sdk`, built
`vpm_run` on-device with the stock gcc, and ran a sample NBG on the NPU at about
2.9 to 3.4 ms per inference. The NPU, the VIPLite driver and the toolchain all
work. What is left for Phase 1 is converting our own model:

1. On an x86 Linux host (ACUITY runs there, in Docker, not on the aarch64 board):
   install the ACUITY Toolkit per Radxa's docs.
2. Convert `ssd-mobilenet-v1-int8.onnx` to NBG, matching the INT8 quantisation we
   already use. Use ACUITY's quantisation-accuracy tooling to confirm the
   "remote" class the re-pair gesture relies on survives conversion.
3. Copy the `.nb` to the robot and benchmark it with `vpm_run` against the CPU
   baseline on the snapshots in `experiments/visual_pair/`.

Blocker: step 1 and 2 need an x86 Linux machine with the ACUITY Docker image.
That is the one piece not reachable from the robot itself.

### Phase 2: drive the NPU from the console (VIPLite + NBG)

On-device findings rule out the ONNX Runtime VSINPU EP on this board (it needs the
Unified OpenVX driver and `galcore`, which are absent). So the integration is the
VIPLite route:

- Load the NBG from Phase 1 through the VIPLite driver (`libVIPhal` /
  `libNBGlinker`, v2.0, in `ai-sdk/viplite-tina`). The repo's `libawnn_viplite`
  is a small C helper around this; `vpm_run.c` is a complete worked example.
- Expose it to the console behind a tiny interface: a C extension or `ctypes`
  wrapper that takes the 300x300 uint8 tensor we already build and returns the
  same raw output tensors `decode_outputs()` expects, so the Python pre- and
  post-processing stay unchanged.
- Keep ONNX plus CPU as the fallback whenever the NBG or the driver is missing,
  and report which backend is live in status (the detector already surfaces the
  provider string).

This avoids both a custom ONNX Runtime build (which would not work here anyway)
and keeps the rest of the detector identical.

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
  detectors, provider shown in status. CPU fallback guaranteed. On this board it
  resolves to CPU (no XNNPACK in the aarch64 wheel, no VSINPU driver).
- Phase 1: NPU proven on the robot (VIPLite + `vpm_run`, about 3 ms on a sample
  NBG). Converting our own model is blocked only on an x86 ACUITY host.
- Phase 2: path decided (VIPLite + NBG + a thin ctypes/C bridge, not VSINPU).
  Not started; needs the NBG from Phase 1 first.
- Phase 3 (GPU): not needed unless the NBG route stalls.
