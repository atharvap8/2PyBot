"""Generate the console's 3D robot model from the assembled STLs, true to the CAD.

CAD is X lateral, Y backwards, Z up, with the ground 115 mm below the CAD origin.
Turning that into x right / y forward / z up takes a half-turn about Z: negate X
and Y. The older driving and world assets negate only Y, which is a mirror, so the
embossed logo reads backwards and the camera lens sits on the wrong side. This
asset is a proper rotation, so it matches the real robot.

Only the printed parts are exported. Wheels, tyre tread, the lidar and the LED ring
are not in the CAD export; the viewer builds them parametrically.

Requires numpy and trimesh. Run:  python scripts/generate_viz_model.py
"""
import json

import numpy as np
import trimesh

from generate_driving_model import PARTS, ROOT

TARGET = ROOT / "software/radxa/console/ui/viz/robot.json"
# Material class per printed part, matching the real robot (black shell and base,
# orange face, motor legs and front beam).
MATERIAL = {"2pybot_shell": "shell", "2pybot_base_B": "base", "2pybot_base_F": "base",
            "2pybot_face": "accent", "2pybot_motor_L": "accent", "2pybot_motor_R": "accent", "beam_D": "accent"}


def to_robot_frame(vertices_mm):
    """CAD millimetres -> robot metres (x right, y forward, z up). A rotation, not a mirror."""
    v = np.asarray(vertices_mm, dtype=float).copy()
    v[:, 0] = 75 - v[:, 0]
    v[:, 1] = 45 - v[:, 1]
    v[:, 2] = v[:, 2] + 115
    return v / 1000


def build():
    parts = []
    for name in PARTS:
        mesh = trimesh.load_mesh(ROOT / "models" / (name + ".stl"))
        mesh.vertices = to_robot_frame(mesh.vertices)
        parts.append({"name": name, "material": MATERIAL[name],
                      "vertices": np.round(mesh.vertices, 5).reshape(-1).tolist(),
                      "faces": mesh.faces.reshape(-1).tolist()})
    return {"schema": "2pybot-viz-mesh/1", "units": "m", "axes": "x right / y forward / z up",
            "handedness": "right (true to CAD; not mirrored)",
            "source": "Full-resolution printed CAD (models/*.stl), rotated into the robot frame",
            "parts": parts}


if __name__ == "__main__":
    model = build()
    TARGET.parent.mkdir(parents=True, exist_ok=True)
    TARGET.write_text(json.dumps(model, separators=(",", ":")), encoding="utf-8")
    print(TARGET, TARGET.stat().st_size, "bytes, triangles:", sum(len(p["faces"]) // 3 for p in model["parts"]))
