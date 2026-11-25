"""Clean outlines of the occupancy grid, for the console's 3D view.

The raw map is a grid of 5 cm cells. Drawn one box per cell it looks blocky and
noisy. This turns it into a few smooth polygons instead: small gaps are bridged,
specks are dropped, the outline is traced on cell corners and straightened.

Pure function of the grid arrays (no locks, no I/O). Needs OpenCV; `available()`
says whether it is importable, and the caller falls back to raw cells when not.
"""
import numpy as np

try:  # OpenCV is already on the robot for the camera; keep the console importable without it
    import cv2
except Exception:  # noqa: BLE001
    cv2 = None

WALL_EPS = 1.1        # straightening tolerance, in cells
FLOOR_EPS = 1.4
MIN_WALL_CELLS = 6    # smaller blobs are noise
MIN_FLOOR_CELLS = 40
MIN_HOLE_CELLS = 6
UPSCALE = 4           # trace on cell corners rather than cell centres


def available():
    return cv2 is not None


def _outlines(mask, to_world, eps, min_cells):
    """[[outer, hole, ...], ...] for the white regions of a uint8 mask."""
    big = cv2.resize(mask, None, fx=UPSCALE, fy=UPSCALE, interpolation=cv2.INTER_NEAREST)
    found = cv2.findContours(big, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = found[-2], found[-1]
    if hierarchy is None:
        return []
    hierarchy = hierarchy[0]
    area = UPSCALE * UPSCALE
    out = []
    for i, contour in enumerate(contours):
        if hierarchy[i][3] != -1 or cv2.contourArea(contour) < min_cells * area:
            continue
        outer = cv2.approxPolyDP(contour, eps * UPSCALE, True)
        if len(outer) < 3:
            continue
        polygon = [[to_world(p[0]) for p in outer]]
        child = hierarchy[i][2]
        while child != -1:
            if cv2.contourArea(contours[child]) >= MIN_HOLE_CELLS * area:
                hole = cv2.approxPolyDP(contours[child], eps * UPSCALE, True)
                if len(hole) >= 3:
                    polygon.append([to_world(p[0]) for p in hole])
            child = hierarchy[child][0]
        out.append(polygon)
    return out


def vectorize(occupied, free, res, origin, row0=0, col0=0):
    """Outlines of the walls and of the known floor, in map metres.

    `occupied` and `free` are boolean arrays indexed [row = y, col = x]; cell
    (row, col) covers [origin + (col0 + col) * res, ... + res) in x and likewise in y.
    Returns {"walls": [...], "floor": [...]}, each a list of polygons, each polygon a
    list of rings (outer first, then holes), each ring a list of [x, y].
    """
    if cv2 is None:
        raise RuntimeError("OpenCV is not available")
    occupied = np.asarray(occupied, dtype=bool)
    free = np.asarray(free, dtype=bool)
    if occupied.shape != free.shape or occupied.ndim != 2:
        raise ValueError("occupied and free must be 2D arrays of the same shape")
    if not occupied.any() and not free.any():
        return {"walls": [], "floor": []}
    pad = 3                                       # room for closing at the edges
    walls = np.pad(occupied, pad).astype(np.uint8) * 255
    floor = np.pad(free, pad).astype(np.uint8) * 255

    def to_world(point):
        return [round(origin + (col0 - pad + float(point[0]) / UPSCALE) * res, 3),
                round(origin + (row0 - pad + float(point[1]) / UPSCALE) * res, 3)]

    walls = cv2.morphologyEx(walls, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))       # bridge one-cell gaps
    count, labels, stats, _ = cv2.connectedComponentsWithStats(walls, 8)
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] < MIN_WALL_CELLS:
            walls[labels == i] = 0                                                 # specks
    seen = cv2.morphologyEx(cv2.bitwise_or(floor, walls), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))   # fill pinholes
    return {"walls": _outlines(walls, to_world, WALL_EPS, MIN_WALL_CELLS),
            "floor": _outlines(seen, to_world, FLOOR_EPS, MIN_FLOOR_CELLS)}
