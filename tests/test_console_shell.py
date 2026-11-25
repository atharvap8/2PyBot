"""Static checks for the console shell and its 3D view (no browser, no robot)."""
import importlib.util
import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / "software/radxa/console/ui"
SHELL_CODE = [UI / "index.html", UI / "shell/shell.js", UI / "shell/demo.js", UI / "shell/shell.css", UI / "viz/engine.js"]


def text(path):
    return Path(path).read_text(encoding="utf-8")


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ------------------------------------------------------------------ the robot model
def signed_volume(vertices, faces):
    total = 0.0
    for i in range(0, len(faces), 3):
        a, b, c = (vertices[faces[i + k] * 3: faces[i + k] * 3 + 3] for k in range(3))
        total += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6
    return total


def test_robot_model_is_true_to_the_cad_not_mirrored():
    model = json.loads(text(UI / "viz/robot.json"))
    assert model["units"] == "m" and model["axes"] == "x right / y forward / z up"
    assert {p["name"] for p in model["parts"]} == {"2pybot_base_B", "2pybot_base_F", "2pybot_face", "2pybot_motor_L",
                                                   "2pybot_motor_R", "2pybot_shell", "beam_D"}
    for part in model["parts"]:
        assert part["material"] in ("shell", "base", "accent")
        # A solid exported with outward faces has positive volume. Mirroring it (the old
        # assets negate one axis only) flips the sign, so this catches a backwards logo.
        assert signed_volume(part["vertices"], part["faces"]) > 0, part["name"] + " is inside out: the model is mirrored"
        zs = part["vertices"][2::3]
        assert min(zs) > 0.0 and max(zs) < 0.26, part["name"] + " is not standing on the floor in metres"
    face = next(p for p in model["parts"] if p["name"] == "2pybot_face")
    assert min(face["vertices"][1::3]) > 0.03, "the face (camera side) must be at the front, +y"


def test_the_older_assets_are_the_mirrored_ones_this_replaces():
    old = json.loads(text(ROOT / "software/android/app/src/main/assets/robot-world-model.json"))
    shell = next(p for p in old["parts"] if p["name"] == "2pybot_shell")
    assert signed_volume(shell["vertices"], shell["faces"]) < 0


def test_cad_to_robot_frame_is_a_rotation():
    pytest.importorskip("trimesh")
    import sys
    sys.path.insert(0, str(ROOT / "scripts"))
    gen = load(ROOT / "scripts/generate_viz_model.py", "generate_viz_model")
    o, x, y, z = gen.to_robot_frame([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]) * 1000
    ex, ey, ez = x - o, y - o, z - o
    det = ex[0] * (ey[1] * ez[2] - ey[2] * ez[1]) - ex[1] * (ey[0] * ez[2] - ey[2] * ez[0]) + ex[2] * (ey[0] * ez[1] - ey[1] * ez[0])
    assert round(det, 6) == 1.0


# ------------------------------------------------------------------ the shell page
def test_every_local_asset_the_shell_loads_exists():
    html = text(UI / "index.html")
    refs = re.findall(r'(?:href|src|data-src)="(/[^"#?]*)', html)
    assert "/ui/shell/shell.js" in refs and "/ui/shell/shell.css" in refs
    pages = {"/slam/": "slam/index.html", "/sysmap/": "sysmap/index.html", "/deploy": "deployment.html", "/terminal": "terminal.html"}
    for ref in refs:
        target = UI / pages[ref] if ref in pages else UI / ref[len("/ui/"):]
        assert ref in pages or ref.startswith("/ui/"), ref
        assert target.is_file(), ref
    for imported in re.findall(r"from '(\.\.?/[^']+)'", text(UI / "shell/shell.js") + text(UI / "viz/engine.js")):
        base = UI / ("viz" if "vendor" in imported or imported.startswith("../vendor") else "shell")
        assert (base / imported).resolve().is_file(), imported
    assert (UI / "viz/robot.json").is_file()


def test_the_tab_modules_the_shell_hosts_exist():
    files = re.search(r"const TAB_FILES = \[(.*?)\];", text(UI / "shell/shell.js"), re.S).group(1)
    names = re.findall(r"'(\w+)'", files)
    assert "telemetry" in names and "camera" in names and len(names) >= 10
    for name in names:
        assert (UI / "tabs" / (name + ".js")).is_file(), name


def test_hosted_sub_pages_can_hide_their_own_header():
    for page in ("slam/index.html", "sysmap/index.html", "deployment.html", "terminal.html", "ai.html"):
        assert "classList.add('embed')" in text(UI / page), page
    for sheet in ("style.css", "slam/slam.css", "sysmap/sysmap.css", "ai.css"):
        assert ".embed" in text(UI / sheet), sheet


def test_the_console_works_offline_on_the_robot_hotspot():
    for path in SHELL_CODE:
        urls = [u for u in re.findall(r"https?://[^\s\"'<>)]+", text(path)) if "w3.org" not in u]
        assert not urls, f"{path.name} reaches outside the robot: {urls}"
    vendor = UI / "vendor/three"
    assert (vendor / "LICENSE").is_file() and (vendor / "three.module.min.js").stat().st_size > 400_000
    for addon in ("RoomEnvironment.js", "RoundedBoxGeometry.js", "BufferGeometryUtils.js"):
        assert "from './three.module.min.js'" in text(vendor / addon), addon + " must import three by relative path"


def test_the_shell_can_stop_motion_but_never_start_it():
    code = text(UI / "shell/shell.js") + text(UI / "shell/demo.js") + text(UI / "viz/engine.js")
    for forbidden in ("/api/autonomy/mission/start", "/api/autonomy/start", "/api/autonomy/resume", "/api/autonomy/heartbeat",
                      "/api/autonomy/teach", "/api/mapping/start", "lease_key"):
        assert forbidden not in code, "the shell must leave starting motion to the Maps app: " + forbidden
    for allowed in ("API.estop()", "/api/autonomy/pause", "/api/autonomy/stop"):
        assert allowed in code


def test_the_classic_console_is_kept_as_a_fallback():
    classic = text(UI / "classic.html")
    assert "/ui/app.js" in classic and "href=/ui/classic.html" in classic
    assert "from './core.js'" in text(UI / "app.js")
    core = text(UI / "core.js")
    for name in ("export const API", "export function renderGroup", "export function msg", "export const state"):
        assert name in core
    assert "/ui/classic.html" in text(UI / "index.html"), "the shell must link to the classic page when 3D is unavailable"


def test_preview_server_only_serves_ui_files():
    preview = load(ROOT / "scripts/preview_console.py", "preview_console")
    assert preview.resolve("/?demo") == UI / "index.html"
    assert preview.resolve("/ui/viz/robot.json") == (UI / "viz/robot.json").resolve()
    assert preview.resolve("/slam/static/slam.js") == (UI / "slam/slam.js").resolve()
    assert preview.resolve("/ui/../../console/server.py") is None
    assert preview.resolve("/api/status") is None


# ------------------------------------------------------------------ engine logic (needs node)
@pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")
def test_map_cells_merge_into_few_boxes_and_drop_specks():
    script = """
      import { mergeCells, SECTORS, VIEWS } from './viz/engine.js';
      const wall = []; for (let i = 0; i < 40; i++) wall.push([100 + i, 50]);      // one straight wall
      const room = []; for (let i = 0; i < 20; i++) for (let j = 0; j < 10; j++) room.push([i, j]);
      console.log(JSON.stringify({ wall: mergeCells(wall, true), room: mergeCells(room),
        speck: mergeCells([[5, 5], [300, 300], [301, 300]], true), sectors: SECTORS.length, views: Object.keys(VIEWS) }));
    """
    out = subprocess.run(["node", "--input-type=module", "-e", script], cwd=UI, capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr[-600:]
    got = json.loads(out.stdout.strip().splitlines()[-1])
    assert got["wall"] == [[100, 50, 40, 1]]
    assert got["room"] == [[0, 0, 20, 10]]
    assert got["speck"] == [[300, 300, 2, 1]], "a lone cell is noise and must be dropped; a pair is kept"
    assert got["sectors"] == 8 and got["views"] == ["parked", "drive", "park"]
