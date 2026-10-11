"""Render the assembled bot's current STL parts into Android launcher artwork.

Requires numpy, trimesh and Pillow. Uses a software depth-buffer renderer, so no
Blender, OpenGL, browser or network is needed. Tyres/hubs are the same parametric
approximations used by the driving model. Printed parts use full-resolution CAD.
"""
import json

import numpy as np
from PIL import Image, ImageDraw, ImageFilter
import trimesh

from generate_driving_model import PARTS, ROOT

SIZE = 1024
SUPERSAMPLE = 2
ARTWORK = ROOT / "software/android/artwork/launcher"
RES = ROOT / "software/android/app/src/main/res"
DENSITIES = {"mdpi": (48, 108), "hdpi": (72, 162), "xhdpi": (96, 216),
             "xxhdpi": (144, 324), "xxxhdpi": (192, 432)}


def assembled_meshes():
    """Same CAD assembly placement as generate_driving_model, without decimation."""
    transform = np.array([[.001, 0, 0, -.075], [0, -.001, 0, .045],
                          [0, 0, .001, .115], [0, 0, 0, 1]])
    result = []
    for name in PARTS:
        mesh = trimesh.load_mesh(ROOT / "models" / f"{name}.stl")
        mesh.apply_transform(transform)
        color = (196, 207, 214) if name == "2pybot_shell" else (147, 161, 172)
        if name == "2pybot_face":
            color = (43, 210, 200)
        result.append((name, mesh, np.array(color)))
    for side in (-1, 1):
        for radius, width, color, suffix in ((.05, .024, (27, 34, 42), "tyre"),
                                             (.027, .025, (159, 178, 193), "hub")):
            mesh = trimesh.creation.cylinder(radius=radius, height=width, sections=96)
            mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
            mesh.apply_translation([side * .092, 0, .05])
            result.append((f"wheel_{side}_{suffix}", mesh, np.array(color)))
    return result


def normalized(value):
    value = np.asarray(value, dtype=float)
    return value / np.linalg.norm(value)


def render(meshes):
    size = SIZE * SUPERSAMPLE
    view = normalized([.48, 1.0, .50])  # front three-quarter view; show the face
    right = normalized(np.cross([0, 0, 1], view))
    up = np.cross(view, right)
    light = normalized([-.6, .8, 1.6])
    fill = normalized([1, -.5, .5])
    all_vertices = np.concatenate([mesh.vertices for _, mesh, _ in meshes])
    xy = np.column_stack((all_vertices @ right, all_vertices @ up))
    centre = (xy.min(axis=0) + xy.max(axis=0)) / 2
    # All geometry is inside Android's central 66dp safe-zone circle (108dp layer).
    scale = size * .285 / np.linalg.norm(xy - centre, axis=1).max()
    depth = np.full((size, size), -np.inf, dtype=np.float32)
    rgba = np.zeros((size, size, 4), dtype=np.uint8)
    triangles = 0
    for _, mesh, base_color in meshes:
        x = size / 2 + (mesh.vertices @ right - centre[0]) * scale
        y = size / 2 - (mesh.vertices @ up - centre[1]) * scale
        z = mesh.vertices @ view
        normals = mesh.face_normals
        brightness = .30 + .68 * np.maximum(0, normals @ light) + .17 * np.maximum(0, normals @ fill)
        half_vector = normalized(light + view)
        specular = 28 * np.maximum(0, normals @ half_vector) ** 28
        colors = np.clip(base_color * brightness[:, None] + specular[:, None], 0, 255).astype(np.uint8)
        for face_index, indices in enumerate(mesh.faces):
            xx, yy, zz = x[indices], y[indices], z[indices]
            x0, x1 = max(0, int(np.floor(xx.min()))), min(size - 1, int(np.ceil(xx.max())))
            y0, y1 = max(0, int(np.floor(yy.min()))), min(size - 1, int(np.ceil(yy.max())))
            denominator = (yy[1] - yy[2]) * (xx[0] - xx[2]) + (xx[2] - xx[1]) * (yy[0] - yy[2])
            if abs(denominator) < 1e-8 or x0 > x1 or y0 > y1:
                continue
            gy, gx = np.mgrid[y0:y1 + 1, x0:x1 + 1]
            gx, gy = gx + .5, gy + .5
            a = ((yy[1] - yy[2]) * (gx - xx[2]) + (xx[2] - xx[1]) * (gy - yy[2])) / denominator
            b = ((yy[2] - yy[0]) * (gx - xx[2]) + (xx[0] - xx[2]) * (gy - yy[2])) / denominator
            c = 1 - a - b
            local_depth = a * zz[0] + b * zz[1] + c * zz[2]
            region = depth[y0:y1 + 1, x0:x1 + 1]
            visible = (a >= -1e-7) & (b >= -1e-7) & (c >= -1e-7) & (local_depth > region)
            region[visible] = local_depth[visible]
            pixels = rgba[y0:y1 + 1, x0:x1 + 1]
            pixels[visible, :3] = colors[face_index]
            pixels[visible, 3] = 255
            triangles += 1
    image = resize(Image.fromarray(rgba), SIZE)
    mask = image.getchannel("A")
    shadow = Image.new("RGBA", (SIZE, SIZE))
    draw = ImageDraw.Draw(shadow)
    bounds = mask.getbbox()
    assert bounds, "Robot did not render"
    draw.ellipse((bounds[0] - 5, bounds[3] - 22, bounds[2] + 5, bounds[3] + 24), fill=(0, 0, 0, 115))
    shadow = shadow.filter(ImageFilter.GaussianBlur(14))
    foreground = Image.alpha_composite(shadow, image)
    monochrome = Image.new("RGBA", (SIZE, SIZE), (255, 255, 255, 0))
    monochrome.putalpha(mask)
    return foreground, monochrome, triangles


def background():
    yy, xx = np.mgrid[0:SIZE, 0:SIZE] / SIZE
    glow = np.exp(-(((xx - .39) / .5) ** 2 + ((yy - .30) / .6) ** 2) * 2)
    rgb = np.array([10, 14, 23]) + glow[..., None] * np.array([10, 33, 37])
    return Image.fromarray(rgb.astype(np.uint8)).convert("RGBA")


def resize(image, size):
    return image.convert("RGBa").resize((size, size), Image.Resampling.LANCZOS).convert("RGBA")


def save(image, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, optimize=True)


def circle(image):
    mask = Image.new("L", image.size)
    ImageDraw.Draw(mask).ellipse((0, 0, image.width - 1, image.height - 1), fill=255)
    result = image.copy()
    result.putalpha(mask)
    return result


def main():
    assert (ROOT / "models").is_dir() and RES.is_dir()
    meshes = assembled_meshes()
    foreground, monochrome, triangles = render(meshes)
    backdrop = background()
    composite = Image.alpha_composite(backdrop, foreground)
    # Android displays the central 72dp of its padded 108dp adaptive layers.
    inset = round(SIZE / 6)
    icon = resize(composite.crop((inset, inset, SIZE - inset, SIZE - inset)), SIZE)
    save(icon, ARTWORK / "robot-icon-1024.png")
    save(circle(icon), ARTWORK / "robot-icon-round-1024.png")
    save(foreground, ARTWORK / "robot-foreground-1024.png")
    for name, (legacy_size, layer_size) in DENSITIES.items():
        save(resize(icon, legacy_size), RES / f"mipmap-{name}/ic_launcher.png")
        save(circle(resize(icon, legacy_size)), RES / f"mipmap-{name}/ic_launcher_round.png")
        for filename, image in (("ic_launcher_foreground", foreground),
                                ("ic_launcher_background", backdrop),
                                ("ic_launcher_monochrome", monochrome)):
            save(resize(image, layer_size), RES / f"drawable-{name}/{filename}.png")
    metadata = {"source": "Seven full-resolution assembled robot STL parts",
                "parts": PARTS, "wheels": "Matching parametric tyre/hub approximations from the driving model",
                "view": "Front three-quarter orthographic", "rendered_triangles": triangles,
                "master_size_px": SIZE, "adaptive_layer_dp": 108, "safe_zone_diameter_dp": 66,
                "regenerate": "python scripts/generate_launcher_icon.py"}
    (ARTWORK / "render-info.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"Rendered {triangles:,} triangles from {len(meshes)} assembled components")
    print(f"Master icon: {ARTWORK / 'robot-icon-1024.png'}")
    print("Generated mdpi through xxxhdpi launcher, adaptive and monochrome resources")


if __name__ == "__main__":
    main()
