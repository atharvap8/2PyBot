"""Unit tests for the NPU SSD post-processing (pure numpy, no hardware)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "software", "radxa", "console"))

from console.npu_detect import ssd_anchors, decode_boxes, postprocess, NUM_CLASSES  # noqa: E402


def test_anchor_count_is_1917():
    a = ssd_anchors()
    assert a.shape == (1917, 4)
    assert a.min() >= -1.0 and a.max() <= 2.0


def test_zero_encoding_returns_anchor_boxes():
    a = ssd_anchors()[:5]
    boxes = decode_boxes(np.zeros((5, 4), dtype=np.float32), a)
    for i, (cy, cx, h, w) in enumerate(a):
        assert abs(boxes[i, 0] - max(0, cx - w / 2)) < 1e-5
        assert abs(boxes[i, 1] - max(0, cy - h / 2)) < 1e-5


def test_postprocess_picks_the_high_scoring_class():
    anchors = ssd_anchors()
    n = len(anchors)
    box_enc = np.zeros((n, 4), dtype=np.float32)
    logits = np.full((n, NUM_CLASSES), -10.0, dtype=np.float32)
    logits[:, 0] = 5.0                      # background dominates everywhere
    logits[100, 0] = -10.0
    logits[100, 18] = 10.0                  # anchor 100 is clearly class 18 (dog)
    dets = postprocess(box_enc, logits, anchors, threshold=0.5)
    assert dets and dets[0]["class_id"] == 18 and dets[0]["label"] == "dog"
    assert dets[0]["score"] > 0.9


def test_people_only_filters_other_classes():
    anchors = ssd_anchors()
    n = len(anchors)
    logits = np.full((n, NUM_CLASSES), -10.0, dtype=np.float32)
    logits[:, 0] = 5.0
    logits[5, 0] = -10.0; logits[5, 18] = 10.0     # a dog
    assert postprocess(np.zeros((n, 4), "f4"), logits, anchors, 0.5, people_only=True) == []
