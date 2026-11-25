"""NPU object detection via the VIPLite runner + SSD post-processing.

The ACUITY-compiled NBG for SSD-MobileNet-V1 outputs the raw SSD tensors (box
encodings and class logits over 1917 anchors); the TensorFlow detection
post-process op is not on the NPU, so we do it here: decode the box encodings
against the SSD anchors, softmax the class logits, threshold and run per-class
NMS. The result matches `object_detection.decode_outputs` so the rest of the
detector is unchanged.

The NPU itself is driven by `awnn_runner`, a small persistent coprocess that keeps
the network resident (see scripts/vision/awnn_runner.c). This module speaks its
pipe protocol and converts frames to the model's planar uint8 input.
"""
import os
import struct
import subprocess
import threading

import numpy as np

from . import npu

RUNNER = os.environ.get("BOT_NPU_RUNNER", "/usr/local/bin/awnn_runner")
INPUT_SIZE = 300
NUM_CLASSES = 91            # background + 90 COCO ids
# TF SSD box coder scales (y, x, h, w)
_SCALES = np.array([10.0, 10.0, 5.0, 5.0], dtype=np.float32)


def ssd_anchors():
    """The 1917 SSD-MobileNet-V1 (300) COCO anchors as [cy, cx, h, w], normalised.

    Matches the TF ssd_mobilenet_v1_coco anchor generator
    (reduce_boxes_in_lowest_layer=True, min 0.2, max 0.95).
    """
    fmaps = [19, 10, 5, 3, 2, 1]
    num = len(fmaps)
    lo, hi = 0.2, 0.95
    scales = [lo + (hi - lo) * i / (num - 1) for i in range(num)] + [1.0]
    ratios = [1.0, 2.0, 0.5, 3.0, 1.0 / 3.0]
    out = []
    for k, f in enumerate(fmaps):
        if k == 0:
            specs = [(0.1, 1.0), (scales[0], 2.0), (scales[0], 0.5)]
        else:
            specs = [(scales[k], ar) for ar in ratios]
            specs.append(((scales[k] * scales[k + 1]) ** 0.5, 1.0))
        for i in range(f):
            for j in range(f):
                cy, cx = (i + 0.5) / f, (j + 0.5) / f
                for s, ar in specs:
                    r = ar ** 0.5
                    out.append((cy, cx, s / r, s * r))
    return np.array(out, dtype=np.float32)


def decode_boxes(box_enc, anchors):
    """box_enc (N,4)=[ty,tx,th,tw] -> boxes (N,4)=[xmin,ymin,xmax,ymax] in 0..1."""
    ya, xa, ha, wa = anchors[:, 0], anchors[:, 1], anchors[:, 2], anchors[:, 3]
    t = box_enc / _SCALES
    yc = t[:, 0] * ha + ya
    xc = t[:, 1] * wa + xa
    h = np.exp(t[:, 2]) * ha
    w = np.exp(t[:, 3]) * wa
    ymin, xmin = yc - h / 2, xc - w / 2
    ymax, xmax = yc + h / 2, xc + w / 2
    return np.clip(np.stack([xmin, ymin, xmax, ymax], axis=1), 0.0, 1.0)


def _softmax(x):
    e = np.exp(x - x.max(axis=1, keepdims=True))
    return e / e.sum(axis=1, keepdims=True)


def _nms(boxes, scores, iou_thr=0.5, top=100):
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    area = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size and len(keep) < top:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        iou = inter / (area[i] + area[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_thr]
    return keep


def postprocess(box_enc, class_logits, anchors, threshold=0.5, people_only=False, max_det=100):
    """Raw SSD tensors -> detections in decode_outputs() format.

    Most of the 1917 anchors are background, so we softmax, drop anchors whose
    best foreground class is below threshold, and only decode/NMS the survivors.
    """
    from .object_detection import COCO
    probs = _softmax(class_logits)
    fg = probs[:, 1:]                          # drop background column
    best = fg.max(axis=1)
    cand = np.where(best >= threshold)[0]
    if cand.size == 0:
        return []
    boxes = decode_boxes(box_enc[cand], anchors[cand])
    classes = fg[cand]
    dets = []
    present = [1] if people_only else range(1, NUM_CLASSES)
    for c in present:
        col = c - 1
        sc = classes[:, col]
        idx = np.where(sc >= threshold)[0]
        if idx.size == 0:
            continue
        for k in _nms(boxes[idx], sc[idx]):
            b = boxes[idx[k]]
            if b[2] <= b[0] or b[3] <= b[1]:
                continue
            dets.append({"class_id": c, "label": COCO.get(c, "object " + str(c)),
                         "score": round(float(sc[idx[k]]), 4),
                         "box": [float(b[0]), float(b[1]), float(b[2]), float(b[3])],
                         "distance_m": None})
    dets.sort(key=lambda d: d["score"], reverse=True)
    return dets[:max_det]


class NpuDetector:
    """Persistent NPU inference via awnn_runner. Thread-safe single inference."""

    def __init__(self, model_path=None, lib_dir=None, runner=RUNNER):
        self.model_path = model_path or npu.status()["model_path"]
        self.lib_dir = lib_dir or npu.status()["lib_dir"]
        self.runner = runner
        self.proc = None
        self.insz = 0
        self.out_elems = []
        self.anchors = ssd_anchors()
        self.lock = threading.Lock()

    def start(self):
        env = dict(os.environ)
        if self.lib_dir:
            env["LD_LIBRARY_PATH"] = self.lib_dir + os.pathsep + env.get("LD_LIBRARY_PATH", "")
        self.proc = subprocess.Popen([self.runner, str(self.model_path)],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, bufsize=0, env=env)
        header = b""
        while not header.endswith(b"\n"):
            ch = self.proc.stdout.read(1)
            if not ch:
                raise RuntimeError("awnn_runner exited before READY")
            header += ch
        parts = header.decode().split()
        self.insz = int(next(p for p in parts if p.startswith("insz=")).split("=")[1])
        outs = int(next(p for p in parts if p.startswith("outs=")).split("=")[1])
        self.out_elems = [int(x) for x in parts[3:3 + outs]]

    def close(self):
        if self.proc:
            try:
                self.proc.stdin.close()
                self.proc.terminate()
            except Exception:  # noqa: BLE001
                pass
            self.proc = None

    def _read(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.proc.stdout.read(n - len(buf))
            if not chunk:
                raise RuntimeError("awnn_runner output truncated")
            buf += chunk
        return buf

    def _raw(self, planar_bytes):
        self.proc.stdin.write(planar_bytes)
        self.proc.stdin.flush()
        outs = [np.frombuffer(self._read(e * 4), dtype="<f4") for e in self.out_elems]
        return outs

    def infer(self, image_bgr, threshold=0.5, people_only=False):
        """image_bgr: HxWx3 BGR uint8 -> detections (decode_outputs format)."""
        import cv2
        with self.lock:
            if self.proc is None or self.proc.poll() is not None:
                self.start()
            small = cv2.resize(image_bgr, (INPUT_SIZE, INPUT_SIZE), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
            planar = np.ascontiguousarray(np.transpose(rgb, (2, 0, 1)))   # C,H,W (R,G,B planes)
            outs = self._raw(planar.tobytes())
        # map outputs by size: 4*1917 box encodings, 91*1917 class logits
        n = len(self.anchors)
        box = next(o for o in outs if o.size == n * 4).reshape(n, 4)
        cls = next(o for o in outs if o.size == n * NUM_CLASSES).reshape(n, NUM_CLASSES)
        return postprocess(box, cls, self.anchors, threshold, people_only)
