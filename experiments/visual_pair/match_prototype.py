"""Offline feasibility prototype for visual re-pair (see DESIGN.md).

Recognises the gamepad in a still image with ORB feature matching against one or
more reference photos, using a RANSAC homography inlier count as the score. This
is deliberately standalone: it needs only OpenCV, runs on a folder of images and
touches no robot, serial port or console service.

Usage:
  # score one image
  python match_prototype.py score path/to/frame.jpg

  # evaluate score separation on a labelled set (pos/ and neg/ subfolders)
  python match_prototype.py eval path/to/testset

  # simulate the 3-second hold on a live snapshot the console already writes
  python match_prototype.py watch /run/2pybot/snapshot.jpg

References default to the manual's gamepad photos; override with --refs DIR.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
DEFAULT_REFS = [REPO / "docs/manual/img/cut/photo_gamepad.png",
                REPO / "docs/manual/img/photo_gamepad.jpg"]
HOLD_S = 3.0
SCORE_THRESHOLD = 18      # min RANSAC inliers; tune from `eval`
RATIO = 0.75              # Lowe ratio test


def load_gray(path, max_side=900):
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        return None
    h, w = img.shape[:2]
    s = max_side / max(h, w)
    if s < 1.0:
        img = cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


class Matcher:
    def __init__(self, ref_paths):
        self.orb = cv2.ORB_create(nfeatures=1500)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.refs = []
        for p in ref_paths:
            g = load_gray(p)
            if g is None:
                continue
            kp, des = self.orb.detectAndCompute(g, None)
            if des is not None and len(kp) >= 10:
                self.refs.append((Path(p).name, kp, des))
        if not self.refs:
            raise SystemExit("no usable reference images; pass --refs with photos of the pad")

    def score(self, gray):
        """Best RANSAC inlier count across all references (0 if none)."""
        kp2, des2 = self.orb.detectAndCompute(gray, None)
        if des2 is None or len(kp2) < 10:
            return 0, None
        best, best_ref = 0, None
        for name, kp1, des1 in self.refs:
            pairs = self.bf.knnMatch(des1, des2, k=2)
            good = [m for m, n in (p for p in pairs if len(p) == 2) if m.distance < RATIO * n.distance]
            if len(good) < 8:
                continue
            src = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            _, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
            inliers = int(mask.sum()) if mask is not None else 0
            if inliers > best:
                best, best_ref = inliers, name
        return best, best_ref


def centre_roi(gray, frac=0.8):
    h, w = gray.shape
    cw, ch = int(w * frac), int(h * frac)
    x, y = (w - cw) // 2, (h - ch) // 2
    return gray[y:y + ch, x:x + cw]


def cmd_score(m, args):
    g = load_gray(args.image)
    if g is None:
        raise SystemExit(f"cannot read {args.image}")
    s, ref = m.score(centre_roi(g))
    print(f"{args.image}: inliers={s} ref={ref} -> {'MATCH' if s >= args.threshold else 'no'}")


def cmd_eval(m, args):
    root = Path(args.dir)
    rows = {"pos": [], "neg": []}
    for label in ("pos", "neg"):
        for p in sorted((root / label).glob("*")):
            g = load_gray(p)
            if g is None:
                continue
            s, _ = m.score(centre_roi(g))
            rows[label].append(s)
            print(f"[{label}] {p.name}: {s}")
    for label, vals in rows.items():
        if vals:
            vals = np.array(vals)
            print(f"{label}: n={len(vals)} min={vals.min()} med={int(np.median(vals))} max={vals.max()}")
    if rows["pos"] and rows["neg"]:
        lo, hi = min(rows["pos"]), max(rows["neg"])
        print(f"suggested threshold ~ {int((lo + hi) / 2)} (pos min {lo}, neg max {hi}; "
              f"{'separable' if lo > hi else 'OVERLAP, add references or use embeddings'})")


def cmd_watch(m, args):
    seen_since = None
    print(f"watching {args.snapshot}; hold the pad in view for {HOLD_S:.0f}s (Ctrl+C to stop)")
    while True:
        g = load_gray(args.snapshot)
        s = m.score(centre_roi(g))[0] if g is not None else 0
        now = time.monotonic()
        if s >= args.threshold:
            seen_since = seen_since or now
            held = now - seen_since
            bar = "#" * int(held / HOLD_S * 20)
            print(f"\rseen inliers={s:3d}  held {held:4.1f}s [{bar:<20}]", end="")
            if held >= HOLD_S:
                print("\nTRIGGER: would call POST /api/gamepad/repair now")
                seen_since = now + 60  # cooldown placeholder
        else:
            seen_since = None
            print(f"\rno pad    inliers={s:3d}                                   ", end="")
        time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser(description="Visual re-pair ORB prototype")
    ap.add_argument("mode", choices=["score", "eval", "watch"])
    ap.add_argument("target", help="image file, test dir, or snapshot path")
    ap.add_argument("--refs", help="folder of reference photos of the pad")
    ap.add_argument("--threshold", type=int, default=SCORE_THRESHOLD)
    args = ap.parse_args()

    ref_paths = sorted(Path(args.refs).glob("*")) if args.refs else [p for p in DEFAULT_REFS if p.exists()]
    m = Matcher(ref_paths)
    print(f"references: {[n for n, _, _ in m.refs]}")

    if args.mode == "score":
        args.image = args.target
        cmd_score(m, args)
    elif args.mode == "eval":
        args.dir = args.target
        cmd_eval(m, args)
    else:
        args.snapshot = args.target
        cmd_watch(m, args)


if __name__ == "__main__":
    sys.exit(main())
