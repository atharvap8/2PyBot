"""The map outline tracer used by the console's 3D view."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("cv2")
source = Path(__file__).resolve().parents[1] / "software/radxa/console/console/slam/vectorize.py"
spec = importlib.util.spec_from_file_location("vectorize", source)
vec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vec)

RES, ORIGIN = 0.05, -5.0


def room(n=60, lo=10, hi=50):
    """A closed square room: a one-cell wall ring with free floor inside."""
    occupied = np.zeros((n, n), bool); free = np.zeros((n, n), bool)
    occupied[lo, lo:hi + 1] = occupied[hi, lo:hi + 1] = True
    occupied[lo:hi + 1, lo] = occupied[lo:hi + 1, hi] = True
    free[lo + 1:hi, lo + 1:hi] = True
    return occupied, free


def bounds(rings):
    xs = [p[0] for ring in rings for p in ring]; ys = [p[1] for ring in rings for p in ring]
    return min(xs), max(xs), min(ys), max(ys)


def test_a_room_becomes_one_ring_with_a_hole_and_one_floor():
    occupied, free = room()
    out = vec.vectorize(occupied, free, RES, ORIGIN)
    assert len(out["walls"]) == 1 and len(out["walls"][0]) == 2, "one wall outline with the room as its hole"
    assert len(out["floor"]) == 1
    assert sum(len(r) for r in out["walls"][0]) <= 16, "straight walls must come out as a handful of corners, not one point per cell"
    x0, x1, y0, y1 = bounds(out["walls"][0])
    assert abs(x0 - (ORIGIN + 10 * RES)) < RES and abs(x1 - (ORIGIN + 51 * RES)) < RES
    assert abs(y0 - (ORIGIN + 10 * RES)) < RES and abs(y1 - (ORIGIN + 51 * RES)) < RES


def test_crop_offsets_place_the_outline_in_map_coordinates():
    occupied, free = room()
    full = vec.vectorize(occupied, free, RES, ORIGIN)
    cropped = vec.vectorize(occupied[5:56, 8:58], free[5:56, 8:58], RES, ORIGIN, row0=5, col0=8)
    assert bounds(cropped["walls"][0]) == pytest.approx(bounds(full["walls"][0]), abs=1e-6)


def test_specks_are_dropped_and_small_gaps_bridged():
    occupied, free = room()
    occupied[30, 30] = True                      # a lone cell in the middle of the room: noise
    occupied[10, 30] = False                     # a one-cell hole in the top wall
    out = vec.vectorize(occupied, free, RES, ORIGIN)
    assert len(out["walls"]) == 1, "the speck must go and the gap must not split the wall"
    assert len(out["walls"][0]) == 2, "with the gap bridged the room is still enclosed"


def test_a_real_gap_such_as_a_doorway_stays_open():
    occupied, free = room()
    occupied[10, 26:35] = False                  # a 45 cm doorway
    free[10, 26:35] = True
    out = vec.vectorize(occupied, free, RES, ORIGIN)
    assert len(out["walls"]) == 1 and len(out["walls"][0]) == 1, "an open doorway means the wall no longer encloses a hole"


def test_empty_map_and_bad_input():
    empty = np.zeros((20, 20), bool)
    assert vec.vectorize(empty, empty, RES, ORIGIN) == {"walls": [], "floor": []}
    with pytest.raises(ValueError):
        vec.vectorize(np.zeros((4, 4), bool), np.zeros((5, 5), bool), RES, ORIGIN)


def test_output_is_much_smaller_than_the_cells_it_replaces():
    rng = np.random.default_rng(3)
    occupied, free = room(200, 20, 180)
    for _ in range(40):                          # ragged thick walls, like a real lidar map
        r, c = rng.integers(21, 179, 2)
        occupied[r:r + 2, 20:23] = True; occupied[20:23, c:c + 2] = True
    out = vec.vectorize(occupied, free, RES, ORIGIN)
    points = sum(len(ring) for poly in out["walls"] for ring in poly)
    assert points < occupied.sum() / 6
