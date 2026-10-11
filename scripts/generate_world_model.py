"""Generate full-resolution Android Drive CAD, with the existing assembly axes.

Requires numpy and trimesh. Keeps all printed triangles and the same coordinates
as the existing driving asset; no mirror correction or icon regeneration.
"""
import json
import numpy as np
import trimesh
from generate_driving_model import ROOT, PARTS, pack


def build():
    parts = []
    for name in PARTS:
        mesh = trimesh.load_mesh(ROOT / "models" / (name + ".stl"))
        vertices = mesh.vertices.copy()
        vertices[:, 0] -= 75
        vertices[:, 1] = 45 - vertices[:, 1]
        vertices[:, 2] += 115
        mesh.vertices = vertices / 1000
        parts.append(pack(name, mesh, "#bfc3c7" if name == "2pybot_shell" else "#91979d"))
    for side in (-1, 1):
        for radius, width, color, suffix in ((.05, .024, "#34383d", "tyre"), (.027, .025, "#9fa5aa", "hub")):
            mesh = trimesh.creation.cylinder(radius=radius, height=width, sections=96)
            mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
            mesh.apply_translation([side * .092, 0, .05])
            parts.append(pack(f"wheel_{side}_{suffix}", mesh, color))
    return {"schema": "2pybot-driving-mesh/1", "units": "m", "axes": "x right / y forward / z up",
            "source": "Full-resolution current printed CAD; matching parametric tyres/hubs", "parts": parts}


if __name__ == "__main__":
    target = ROOT / "software/android/app/src/main/assets/robot-world-model.json"
    assert target.parent.is_dir()
    model = build()
    target.write_text(json.dumps(model, separators=(",", ":")), encoding="utf-8")
    print(target, "triangles:", sum(len(part["faces"]) // 3 for part in model["parts"]))
