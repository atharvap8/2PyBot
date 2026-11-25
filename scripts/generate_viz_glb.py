"""Build the console's 3D robot from the full CAD assembly export (models/2pybot.glb).

The Onshape GLB is every face as its own mesh: 17,000 pieces, 686,000 triangles and
45 MB, most of it in parts nobody can see (thirty-seven 1 mm capacitors take 420,000
triangles). A browser on the robot's hotspot needs something much smaller, so this:

  * turns the export (X sideways, Y up, +Z out of the face) into the robot frame used
    everywhere else: x right, y forward, z up, floor at z = 0, axle over the origin.
    That is a rotation, not a mirror;
  * merges each part's faces into one welded mesh per colour;
  * drops what is far below a pixel (the capacitors) and the three decorative wires,
    and replaces screws and LEDs by their convex hulls;
  * tags parts with the system-map block they belong to, marks the two hub motors as
    the spinning wheels, and works out where the labels attach.

Output: ui/viz/robot2.json (manifest) and ui/viz/robot2.bin (float32 positions, then
uint16/uint32 indices, per part). Requires numpy and trimesh.

    python scripts/generate_viz_glb.py
"""
import collections
import json
import re
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "models/2pybot.glb"
OUT = ROOT / "software/radxa/console/ui/viz"

DROP = ("wire_", "GRM188")                         # decorative wires; 1 mm capacitors
HULL = ("screw_", "neopixel")                      # tiny and convex enough: their hull is all anyone sees
MERGE = (("screw_", "screws"), ("standoff_", "standoffs"), ("neopixel", "leds"))
INNER = ("deck_", "standoff", "ESP32", "IMU_", "RS485", "Radxa", "USB_camera", "battery", "buck_", "indicator_retainer")
# part name prefix -> system-map blocks it lights up for (left and right are added from the side it sits on)
KEYS = (("RPLIDAR", ["lidar"]), ("lidar_cradle", ["lidar"]), ("LedRing", ["lights"]), ("leds", ["lights"]),
        ("led_ring_retainer", ["lights"]), ("ESP32", ["esp32", "gamepad"]), ("RS485", ["esp32"]),
        ("IMU_", ["imu"]), ("Radxa", ["radxa", "clients"]), ("USB_camera", ["camera"]), ("face_inlay", ["camera", "tof"]))


def finish(name, rgb):
    r, g, b = rgb
    if name == "leds":
        return "led"
    if name.startswith("RPLIDAR"):
        return "glass"
    if name.startswith("M0601_hub"):
        return "tyre"
    if name in ("screws", "standoffs"):
        return "metal"
    if max(rgb) < 40:
        return "shell"                              # the black printed shell and logo
    if g > r and g > b:
        return "pcb"
    if r > 150 and b < 90:
        return "accent"                             # the orange prints
    return "plastic"


def clean(node_name):
    name = re.sub(r"^occurrence of ", "", str(node_name))
    name = re.sub(r"\s*\[.*$", "", name)           # "[CAD]", "[ASSUMED 85x56]" ...
    name = re.sub(r"__.*$", "", name)              # LedRing__Aufsatz..., neopixel__WS2812...
    return re.sub(r"\s*<\d+>.*$", "", name).strip()


def load():
    """-> {part: {(r, g, b): [(vertices, faces), ...]}} in the GLB's own frame."""
    scene = trimesh.load(SOURCE, force="scene")
    graph, parents = scene.graph, scene.graph.transforms.parents
    parts = collections.OrderedDict()
    for node in graph.nodes_geometry:
        chain = [node]
        while chain[-1] in parents:
            chain.append(parents[chain[-1]])
        chain = chain[::-1]                         # world, Assembly 1, occurrence, ..., leaf
        top = chain[2] if len(chain) > 2 else chain[-1]
        if str(top).startswith("Assembly 2") and len(chain) > 3:
            top = chain[3]                          # the LED ring board: its parts one level down
        name = clean(top)
        if any(name.startswith(p) for p in DROP):
            continue
        transform, geometry = graph[node]
        mesh = scene.geometry[geometry]
        try:
            rgb = tuple(int(v) for v in mesh.visual.material.main_color[:3])
        except Exception:  # noqa: BLE001 - a face without a material
            rgb = (128, 128, 128)
        parts.setdefault(name, collections.OrderedDict()).setdefault(rgb, []).append(
            (trimesh.transform_points(mesh.vertices, transform), np.asarray(mesh.faces)))
    return parts


def weld(chunks):
    vertices, faces, offset = [], [], 0
    for v, f in chunks:
        vertices.append(v)
        faces.append(f + offset)
        offset += len(v)
    mesh = trimesh.Trimesh(np.vstack(vertices), np.vstack(faces), process=False)
    mesh.merge_vertices(digits_vertex=5)            # 0.01 mm
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    return mesh


def build():
    raw = load()
    bounds = {n: (np.min([v.min(0) for c in cs.values() for v, _ in c], axis=0),
                  np.max([v.max(0) for c in cs.values() for v, _ in c], axis=0)) for n, cs in raw.items()}
    shell = [b for n, b in bounds.items() if n.startswith("shell_")]
    wheel = [b for n, b in bounds.items() if n.startswith("M0601_hub")]
    assert len(shell) == 2 and len(wheel) == 2, "expected two shell halves and two hub motors in the assembly"
    cx = 0.5 * (min(b[0][0] for b in shell) + max(b[1][0] for b in shell))       # the body's centre plane
    floor = min(b[0][1] for b in wheel)                                          # the wheels stand on it
    cz = 0.5 * (wheel[0][0][2] + wheel[0][1][2])                                 # the axle
    radius = 0.5 * (wheel[0][1][1] - wheel[0][0][1])

    def to_robot(v):                                # (x, y, z) -> (-(x - cx), z - cz, y - floor): det = +1
        return np.c_[cx - v[:, 0], v[:, 2] - cz, v[:, 1] - floor]

    merged = collections.OrderedDict()              # (part, rgb) -> [(v, f)]
    for name, colours in raw.items():
        target = next((to for prefix, to in MERGE if name.startswith(prefix)), name)
        if any(name.startswith(p) for p in HULL):   # one hull for the whole part, whatever its faces were coloured
            whole = weld([(to_robot(v), f) for chunks in colours.values() for v, f in chunks])
            if target == "screws":                  # round heads hull into hundreds of facets: snap to 1 mm first
                whole = trimesh.Trimesh(np.round(np.asarray(whole.vertices) / 0.001) * 0.001, whole.faces, process=False)
            hull = whole.convex_hull
            first = next(iter(colours))
            colours = {(255, 255, 255) if target == "leds" else first: [(np.asarray(hull.vertices), np.asarray(hull.faces))]}
            to_frame = lambda v: v                  # noqa: E731 - already in the robot frame
        else:
            to_frame = to_robot
        for rgb, chunks in colours.items():
            chunks = [(to_frame(v), f) for v, f in chunks]
            if target in ("screws", "standoffs", "leds"):
                rgb = next(iter(k[1] for k in merged if k[0] == target), rgb)
            merged.setdefault((target, rgb), []).extend(chunks)

    blob, parts, total = bytearray(), [], 0
    box = {}
    for (name, rgb), chunks in merged.items():
        mesh = weld(chunks)
        v, f = np.asarray(mesh.vertices, np.float32), np.asarray(mesh.faces)
        if not len(f):
            continue
        lo, hi = v.min(0), v.max(0)
        prev = box.get(name)
        box[name] = (lo, hi) if prev is None else (np.minimum(lo, prev[0]), np.maximum(hi, prev[1]))
        side = "l" if 0.5 * (lo[0] + hi[0]) < 0 else "r"              # the robot's own left is -x
        keys = list(next((k for prefix, k in KEYS if name.startswith(prefix)), []))
        spin = None
        if name.startswith("M0601_hub"):
            spin, keys = side, ["wheel_" + side]
        elif name.startswith("m0601_mount"):
            keys = ["wheel_" + side]
        wide = len(v) > 65535
        index = f.astype(np.uint32 if wide else np.uint16).reshape(-1)
        while len(blob) % 4:
            blob.append(0)
        voff = len(blob)
        blob += v.tobytes()
        ioff = len(blob)
        blob += index.tobytes()
        total += len(f)
        parts.append({"name": name, "finish": finish(name, rgb), "color": [round(c / 255, 3) for c in rgb],
                      "keys": keys, "inner": any(name.startswith(p) for p in INNER), "spin": spin,
                      "vertices": len(v), "indices": len(index), "voff": voff, "ioff": ioff, "wide": wide})

    def centre(prefix, top=False):
        lo, hi = next(b for n, b in box.items() if n.startswith(prefix))
        c = 0.5 * (lo + hi)
        return [round(float(c[0]), 4), round(float(c[1]), 4), round(float(hi[2] if top else c[2]), 4)]

    ring = box["leds"]
    hub = {s: next(b for n, b in box.items() if n.startswith("M0601_hub") and (0.5 * (b[0][0] + b[1][0]) < 0) == (s == "l"))
           for s in "lr"}
    anchors = {
        "lidar": centre("RPLIDAR", top=True),
        "lights": [round(float(ring[1][0]), 4), round(float(0.5 * (ring[0][1] + ring[1][1])), 4), round(float(ring[1][2]), 4)],
        "esp32": centre("ESP32"), "imu": centre("IMU_"), "radxa": centre("Radxa"),
        "wheel_l": [round(float(hub["l"][0][0]), 4), 0.0, round(float(radius), 4)],
        "wheel_r": [round(float(hub["r"][1][0]), 4), 0.0, round(float(radius), 4)],
    }
    wheels = {s: {"x": round(float(0.5 * (hub[s][0][0] + hub[s][1][0])), 4), "radius": round(float(radius), 4)} for s in "lr"}
    manifest = {"schema": "2pybot-viz-mesh/2", "units": "m", "axes": "x right / y forward / z up",
                "handedness": "right (a rotation of the CAD export; not mirrored)",
                "source": "models/2pybot.glb (Onshape assembly), reduced by scripts/generate_viz_glb.py",
                "bin": "robot2.bin", "bytes": len(blob), "triangles": total,
                "anchors": anchors, "wheels": wheels, "parts": parts}
    return manifest, bytes(blob)


if __name__ == "__main__":
    manifest, blob = build()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "robot2.bin").write_bytes(blob)
    (OUT / "robot2.json").write_text(json.dumps(manifest, separators=(",", ":")), encoding="utf-8")
    print("triangles:", manifest["triangles"], "| parts:", len(manifest["parts"]), "| bin: %.2f MB" % (len(blob) / 1e6))
    for p in sorted(manifest["parts"], key=lambda p: -p["indices"]):
        print("  %-22s %-8s %6d tris  %s%s%s" % (p["name"], p["finish"], p["indices"] // 3, ",".join(p["keys"]) or "-",
                                                 "  inner" if p["inner"] else "", "  spin " + p["spin"] if p["spin"] else ""))
    print("anchors:", manifest["anchors"])
    print("wheels:", manifest["wheels"])
