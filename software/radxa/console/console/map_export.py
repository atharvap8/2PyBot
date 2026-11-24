"""Export a saved SLAM map as a 3D mesh (binary STL) or a portable zip, and
import a zip back. Pure geometry from the saved occupancy grid; no robot access.

The zip holds the native files (name.npz, name.png, name.json) so a map exported
from one bot imports cleanly into another. The STL raises every wall cell into a
box for viewing or printing; the floor is a single plane.
"""
import io
import json
import os
import re
import struct
import zipfile

import numpy as np

SAFE = re.compile(r"[A-Za-z0-9_-]{1,40}\Z")
WALL_H = 0.5          # metres a wall cell is raised in the mesh
MAX_CELLS = 30000     # cap so an export stays a reasonable download


def _meta(maps_dir, name):
    try:
        with open(os.path.join(maps_dir, name + ".json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def stl_bytes(maps_dir, name, height=WALL_H):
    """Binary STL: a box per wall cell plus a floor plane over the known area."""
    data = np.load(os.path.join(maps_dir, name + ".npz"))
    logodds = np.asarray(data["logodds"], dtype=np.float32)
    touched = np.asarray(data["touched"]) if "touched" in data.files else (logodds != 0)
    m = _meta(maps_dir, name)
    res = float(m.get("resolution", 0.05))
    origin = float(m.get("origin", -logodds.shape[0] * res / 2))

    ys, xs = np.where(logodds > 0.6)
    if len(xs) > MAX_CELLS:
        idx = np.linspace(0, len(xs) - 1, MAX_CELLS).astype(int)
        xs, ys = xs[idx], ys[idx]
    bx = origin + xs * res
    by = origin + ys * res

    # 8 corners and the 12 triangles (as corner indices) of a unit cell box.
    off = np.array([[0, 0, 0], [res, 0, 0], [res, res, 0], [0, res, 0],
                    [0, 0, height], [res, 0, height], [res, res, height], [0, res, height]], np.float32)
    faces = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                      [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]], np.int32)
    n = len(xs)
    tris = np.empty((0, 3, 3), np.float32)
    if n:
        corners = np.zeros((n, 8, 3), np.float32)
        corners[:, :, 0] = bx[:, None] + off[:, 0]
        corners[:, :, 1] = by[:, None] + off[:, 1]
        corners[:, :, 2] = off[:, 2]
        tris = corners[:, faces, :].reshape(-1, 3, 3)

    # floor plane over the touched bbox
    tb = np.argwhere(touched)
    if len(tb):
        y0, x0 = tb.min(0)
        y1, x1 = tb.max(0) + 1
        fx0, fy0 = origin + x0 * res, origin + y0 * res
        fx1, fy1 = origin + x1 * res, origin + y1 * res
        floor = np.array([[[fx0, fy0, 0], [fx1, fy0, 0], [fx1, fy1, 0]],
                          [[fx0, fy0, 0], [fx1, fy1, 0], [fx0, fy1, 0]]], np.float32)
        tris = np.concatenate([tris, floor], axis=0) if len(tris) else floor

    count = len(tris)
    rec = np.zeros(count, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["v"] = tris
    header = b"2PyBot SLAM map: " + name.encode()[:60]
    return struct.pack("<80sI", header.ljust(80, b" "), count) + rec.tobytes()


def zip_bytes(maps_dir, name):
    """Portable map: the native npz/png/json in one zip."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for ext in (".npz", ".png", ".json"):
            p = os.path.join(maps_dir, name + ext)
            if os.path.exists(p):
                z.write(p, arcname=name + ext)
    return buf.getvalue()


def import_zip(maps_dir, name, blob):
    """Write an exported map zip into the maps directory under `name`."""
    if not SAFE.match(name or ""):
        raise ValueError("Use letters, numbers, - or _ (max 40).")
    os.makedirs(maps_dir, exist_ok=True)
    written = []
    with zipfile.ZipFile(io.BytesIO(blob)) as z:
        members = {os.path.splitext(os.path.basename(m))[1]: m for m in z.namelist()
                   if m.endswith((".npz", ".png", ".json"))}
        if ".npz" not in members:
            raise ValueError("Not a 2PyBot map export (no .npz inside).")
        for ext, member in members.items():
            with z.open(member) as src:
                blob_e = src.read()
            if ext == ".json":
                try:
                    meta = json.loads(blob_e.decode())
                    meta["name"] = name
                    blob_e = json.dumps(meta, indent=1).encode()
                except (ValueError, UnicodeDecodeError):
                    pass
            with open(os.path.join(maps_dir, name + ext), "wb") as f:
                f.write(blob_e)
            written.append(ext)
    return written
