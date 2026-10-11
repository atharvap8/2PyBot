"""Generate matching web/Android meshes from the assembled STL coordinates.

Requires numpy, trimesh, fast-simplification. Printed components preserve their
CAD placement; wheels/hubs are parametric visual approximations, not CAD parts.
"""
import json
from pathlib import Path
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
PARTS = ["2pybot_base_B", "2pybot_base_F", "2pybot_face", "2pybot_motor_L",
         "2pybot_motor_R", "2pybot_shell", "beam_D"]


def build():
    parts = []
    for name in PARTS:
        mesh = trimesh.load_mesh(ROOT / "models" / (name + ".stl"))
        mesh = mesh.simplify_quadric_decimation(face_count=900)
        # CAD: X lateral, Y backwards, Z up. Ground is 115 mm below CAD origin.
        vertices = mesh.vertices.copy()
        vertices[:, 0] -= 75
        vertices[:, 1] = 45 - vertices[:, 1]
        vertices[:, 2] += 115
        mesh.vertices = vertices / 1000
        parts.append(pack(name, mesh, "#bfc3c7" if name == "2pybot_shell" else "#91979d"))
    for side in (-1, 1):
        for radius, width, color, suffix in ((.05, .024, "#34383d", "tyre"), (.027, .025, "#9fa5aa", "hub")):
            mesh = trimesh.creation.cylinder(radius=radius, height=width, sections=24)
            mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
            mesh.apply_translation([side * .092, 0, .05])
            parts.append(pack(f"wheel_{side}_{suffix}", mesh, color))
    return {"schema": "2pybot-driving-mesh/1", "units": "m", "axes": "x right / y forward / z up",
            "source": "Seven current assembled STL components; approximate parametric tyres/hubs",
            "parts": parts}


def pack(name, mesh, color):
    return {"name": name, "color": color, "vertices": np.round(mesh.vertices, 5).reshape(-1).tolist(),
            "faces": mesh.faces.reshape(-1).tolist()}


if __name__ == "__main__":
    data = json.dumps(build(), separators=(",", ":"))
    for path in (ROOT / "software/radxa/console/ui/robot-model.json",
                 ROOT / "software/android/app/src/main/assets/robot-model.json"):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(data, encoding="utf-8")
        print(path, len(data), "bytes")
