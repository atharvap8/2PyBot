"""Path planning on the occupancy grid (numpy + heapq, no SciPy).

The 5 cm SLAM grid is pooled to 10 cm, cropped to the known area, obstacles
are inflated by the robot's turning radius (it can spin in place), and A*
runs on 8-connected cells with an extra cost near walls so paths keep to the
middle of corridors. Unknown space is not traversable, so the robot only
drives where it has mapped.
"""
from __future__ import annotations

import collections
import heapq
import math

import numpy as np

CELL = 0.10               # planning resolution (m)
ROBOT_RADIUS = 0.17       # body corner radius about the axle (0.156 m) + 1.4 cm
SOFT_RADIUS = 0.45        # walls closer than this add cost (keeps paths centred)
SOFT_WEIGHT = 6.0
OCC = 0.6                 # log-odds: occupied
FREE = -0.4               # log-odds: confidently free


class PlanError(ValueError):
    pass


def _pool(a, k, fn):
    n = (a.shape[0] // k) * k, (a.shape[1] // k) * k
    a = a[:n[0], :n[1]]
    return fn(fn(a.reshape(n[0] // k, k, n[1] // k, k), axis=3), axis=1)


def _distance(blocked, limit_cells):
    """Brushfire distance (in cells, 8-connected chamfer) from blocked cells, capped."""
    h, w = blocked.shape
    dist = np.full((h, w), np.inf)
    q = collections.deque()
    for r, c in zip(*np.nonzero(blocked)):
        dist[r, c] = 0.0
        q.append((r, c))
    steps = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
    while q:
        r, c = q.popleft()
        d = dist[r, c]
        if d >= limit_cells:
            continue
        for dr, dc, s in steps:
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and d + s < dist[rr, cc]:
                dist[rr, cc] = d + s
                q.append((rr, cc))
    return dist


class CostMap:
    def __init__(self, grid, extra_obstacles=None, pad_m=1.0):
        k = max(1, int(round(CELL / grid.res)))
        self.res = grid.res * k
        bbox = grid.known_bbox(pad=int(pad_m / grid.res))
        if bbox is None:
            raise PlanError("The map is empty")
        r0, r1, c0, c1 = bbox
        r0, c0 = (r0 // k) * k, (c0 // k) * k
        lo = grid.logodds[r0:r1, c0:c1]
        to = grid.touched[r0:r1, c0:c1]
        occ = _pool(lo > OCC, k, np.max)
        free = _pool((lo < FREE) & to, k, np.max) & ~occ
        self.x0 = grid.origin + c0 * grid.res          # world coordinate of planning cell (0,0) corner
        self.y0 = grid.origin + r0 * grid.res
        self.shape = occ.shape
        if extra_obstacles is not None and len(extra_obstacles):
            rr, cc = self.cell(extra_obstacles[:, 0], extra_obstacles[:, 1])
            ok = (rr >= 0) & (rr < occ.shape[0]) & (cc >= 0) & (cc < occ.shape[1])
            occ[rr[ok], cc[ok]] = True
            free[rr[ok], cc[ok]] = False
        self.occ, self.free = occ, free
        soft = SOFT_RADIUS / self.res
        dist = _distance(occ, soft + 1)
        self.clear_m = dist * self.res
        self.lethal = (self.clear_m < ROBOT_RADIUS) | ~free
        self.cost = 1.0 + SOFT_WEIGHT * np.clip((SOFT_RADIUS - self.clear_m) / (SOFT_RADIUS - ROBOT_RADIUS), 0, 1) ** 2

    def cell(self, x, y):
        return (np.floor((np.asarray(y) - self.y0) / self.res).astype(int),
                np.floor((np.asarray(x) - self.x0) / self.res).astype(int))

    def world(self, r, c):
        return self.x0 + (c + 0.5) * self.res, self.y0 + (r + 0.5) * self.res

    def inside(self, r, c):
        return 0 <= r < self.shape[0] and 0 <= c < self.shape[1]

    def nearest_free(self, r, c, max_m=0.6):
        """Closest traversable cell (labels and the robot itself may sit near walls)."""
        if self.inside(r, c) and not self.lethal[r, c]:
            return r, c
        best, best_d = None, math.inf
        rad = int(max_m / self.res)
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                rr, cc = r + dr, c + dc
                d = math.hypot(dr, dc)
                if d <= rad and d < best_d and self.inside(rr, cc) and not self.lethal[rr, cc]:
                    best, best_d = (rr, cc), d
        return best

    def line_clear(self, a, b):
        (r0, c0), (r1, c1) = a, b
        n = int(max(abs(r1 - r0), abs(c1 - c0)) * 2) + 1
        for t in np.linspace(0, 1, n):
            r = int(round(r0 + (r1 - r0) * t))
            c = int(round(c0 + (c1 - c0) * t))
            if self.lethal[r, c]:
                return False
        return True


def astar(cm, start, goal, max_expand=250000):
    h, w = cm.shape
    sr, sc = start
    gr, gc = goal
    g = np.full((h, w), np.inf)
    parent = {}
    g[sr, sc] = 0.0
    heap = [(0.0, sr, sc)]
    steps = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
    expanded = 0
    while heap:
        f, r, c = heapq.heappop(heap)
        if (r, c) == (gr, gc):
            path = [(r, c)]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return path[::-1]
        expanded += 1
        if expanded > max_expand:
            break
        base = g[r, c]
        for dr, dc, s in steps:
            rr, cc = r + dr, c + dc
            if not (0 <= rr < h and 0 <= cc < w) or cm.lethal[rr, cc]:
                continue
            if dr and dc and (cm.lethal[r, cc] or cm.lethal[rr, c]):
                continue                                   # no corner cutting
            ng = base + s * 0.5 * (cm.cost[r, c] + cm.cost[rr, cc])
            if ng < g[rr, cc]:
                g[rr, cc] = ng
                parent[(rr, cc)] = (r, c)
                heapq.heappush(heap, (ng + math.hypot(gr - rr, gc - cc), rr, cc))
    return None


def smooth(cm, cells):
    """Line-of-sight shortcutting, then resampled every ~0.15 m."""
    if len(cells) <= 2:
        out = cells
    else:
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not cm.line_clear(cells[i], cells[j]):
                j -= 1
            out.append(cells[j])
            i = j
    pts = [cm.world(r, c) for r, c in out]
    dense = [pts[0]]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.15))
        for k in range(1, n + 1):
            dense.append((x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n))
    return dense


def plan(grid, start_xy, goal_xy, extra_obstacles=None):
    """World path [(x, y), ...] from start to goal, or PlanError with a reason."""
    cm = CostMap(grid, extra_obstacles)
    s = cm.nearest_free(*cm.cell(*start_xy))
    if s is None:
        raise PlanError("The robot is not on mapped free space")
    gl = cm.nearest_free(*cm.cell(*goal_xy))
    if gl is None:
        raise PlanError("The goal is not on mapped free space (or too close to a wall)")
    cells = astar(cm, s, gl)
    if cells is None:
        raise PlanError("No mapped path to the goal")
    path = smooth(cm, cells)
    path[0] = (float(start_xy[0]), float(start_xy[1]))
    return [(round(x, 3), round(y, 3)) for x, y in path]


def frontier_goals(grid, pose_xy, limit=8):
    """Exploration targets: free cells next to unknown space, nearest first."""
    cm = CostMap(grid, pad_m=1.5)
    known = cm.free | cm.occ
    unknown = ~known
    nb = np.zeros_like(unknown)
    nb[1:, :] |= unknown[:-1, :]
    nb[:-1, :] |= unknown[1:, :]
    nb[:, 1:] |= unknown[:, :-1]
    nb[:, :-1] |= unknown[:, 1:]
    rows, cols = np.nonzero(nb & ~cm.lethal)
    if not len(rows):
        return []
    xs, ys = cm.world(rows, cols)
    d = np.hypot(xs - pose_xy[0], ys - pose_xy[1])
    keep = d > 0.6
    order = np.argsort(d[keep])[:limit * 6]
    out = []
    for i in order:
        x, y = float(xs[keep][i]), float(ys[keep][i])
        if all(math.hypot(x - a, y - b) > 0.8 for a, b in out):
            out.append((x, y))
        if len(out) >= limit:
            break
    return out
