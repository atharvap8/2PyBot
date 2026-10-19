"""Export and check the M0601 mounts against the current assembled robot STLs.

Requires OpenSCAD 2021.01+, numpy, trimesh, manifold3d and Pillow.
python scripts/generate_m0601_mount.py --openscad /path/to/openscad
Edit hardware/cad/m0601_mount/rev1/dimensions.scad to confirm physical dimensions.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import zipfile

import numpy as np
import trimesh
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
CAD = ROOT / "hardware/cad/m0601_mount/rev1"
BODY = ["2pybot_base_F", "2pybot_base_B", "2pybot_shell", "2pybot_face", "beam_D"]


def export(openscad, destination, part, side="L", layout="print", maximum=False, extra=()):
    command = [str(openscad), "-o", str(destination), "-D", f'part="{part}"',
               "-D", f'side="{side}"', "-D", f'layout="{layout}"',
               "-D", f"maximum_envelope={'true' if maximum else 'false'}"]
    if destination.suffix == ".stl": command += ["--export-format", "binstl"]
    completed = subprocess.run(command + list(extra) + [str(CAD / "mount.scad")],
                               capture_output=True, text=True, timeout=180)
    if completed.returncode:
        raise RuntimeError(completed.stderr + completed.stdout)
    if "ERROR:" in completed.stderr or "WARNING:" in completed.stderr:
        raise RuntimeError(completed.stderr)
    if destination.suffix != ".echo":
        assert destination.exists() and destination.stat().st_size > 0
    return completed


def checked_mesh(path):
    mesh = trimesh.load_mesh(path)
    assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume > 0, path
    assert not np.any(mesh.area_faces < 1e-9), (path, "degenerate faces")
    assert len(mesh.split()) == 1, (path, "disconnected printable bodies")
    return mesh


def intersection_volume(a, b):
    result = trimesh.boolean.intersection([a, b], engine="manifold")
    # Touching faces can produce a zero-volume manifold. Trimesh also derives
    # its centroid while computing volume, so its zero-volume division is moot.
    with np.errstate(invalid="ignore",divide="ignore"):
        volume=abs(float(result.volume)) if len(result.faces) else 0.0
    assert np.isfinite(volume)
    return volume


def drawing(params, destination):
    image = Image.new("RGB", (1600, 1050), "white")
    draw = ImageDraw.Draw(image)
    draw.text((50, 30), "2PyBot / M0601 - Rev 1 mechanical interfaces (mm)", fill="black", font_size=29)
    draw.text((50, 80), "Prototype: chassis 30 x 30 pattern and 65 mm axle drop require physical confirmation.",
              fill="#ac3d12", font_size=22)
    # Underside plan of the four-strut replacement interface.
    s = 9; ox = 70; oy = 165; size = params["frame_size"]
    draw.rectangle((ox, oy, ox+size*s, oy+size*s), outline="black", width=3)
    margin = (size-params["strut_pitch_x"])/2
    ys = [(size-params["strut_pitch_y"])/2, (size+params["strut_pitch_y"])/2]
    xs = [margin, size-margin]
    for x in xs:
        for y in ys:
            r = 1.7*s
            draw.ellipse((ox+x*s-r, oy+y*s-r, ox+x*s+r, oy+y*s+r), outline="#008c95", width=3)
    draw.text((ox, oy+size*s+20), f'4 x M3 clearance, {params["strut_pitch_x"]:g} x {params["strut_pitch_y"]:g} centres',
              fill="black", font_size=20)
    draw.text((ox, oy+size*s+55), f'{size:g} square fits nominal 44 mm tray opening', fill="black", font_size=20)
    # Motor mounting face, bolts + keyed stator envelope.
    cx = 855; cy = 360; s = 12
    outer = params["socket_outer_diameter"]*s/2
    draw.ellipse((cx-outer,cy-outer,cx+outer,cy+outer),outline="black",width=3)
    angles = np.linspace(0,2*np.pi,720)
    radius = params["socket_diameter"]/2
    clock = np.radians(params["motor_clock_deg"])
    vertices = []
    for angle in angles:
        normals = np.radians([30,150,270])+clock
        positive = np.cos(angle-normals)
        bounds = [params["socket_flat_radius"]/value for value in positive if value>0]
        r = min([radius]+bounds)
        vertices.append((cx+s*r*np.cos(angle), cy-s*r*np.sin(angle)))
    draw.line(vertices+[vertices[0]],fill="#008c95",width=3)
    for angle in np.radians([90,210,330])+clock:
        x = cx+s*params["bolt_circle"]/2*np.cos(angle)
        y = cy-s*params["bolt_circle"]/2*np.sin(angle)
        r = params["bolt_clearance"]*s/2
        draw.ellipse((x-r,y-r,x+r,y+r),outline="#ac3d12",width=3)
    draw.text((650,590), f'3 x M2.5 / {params["bolt_clearance"]:g} clearance / 15.2 PCD / 120 deg',fill="black",font_size=20)
    draw.text((650,625), f'Stator seat: diameter {params["socket_diameter"]:g}, three flats at {params["socket_flat_radius"]:g} radius',
              fill="black",font_size=20)
    draw.text((650,660), '19 mm stator and 3 x 8 EQS flats from manufacturer drawing',fill="black",font_size=20)
    lines = [
        f'Axle centre: Y = {params["anchor_center_y"]:g}, Z = -{params["axle_drop"]:g} in current STL frame.',
        f'Socket entry X = {params["socket_lip_x"]:g}; stator face X = {params["stator_face_x"]:g}; end plate {params["face_plate_thickness"]:g} thick.',
        'Per side: 4 x M3 x 10 + M3 nuts/washers; 3 x M2.5 x 10 + ~0.5 mm washers.',
        'M2.5 screw engagement = 10 - 6 - 0.5 = 3.5 mm; motor thread depth is only 5 mm.',
        'Remove old stepper, metal bracket, four brass struts and motor cover. Keep shell, face and split base.',
        'Print both gauges first. Full mounts are side-flat on the bed; support the socket ceiling and rear web.',
        'Rotor shown in assembly is a conservative envelope, not a supplied manufacturer solid model.'
    ]
    for i, line in enumerate(lines): draw.text((50,750+i*37),line,fill="black",font_size=20)
    image.save(destination)


def build(openscad):
    assert CAD.is_dir() and (ROOT / "models").is_dir()
    out = CAD / "exports"; out.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="m0601-check-", dir=out) as tmp:
        temporary = Path(tmp)
        export(openscad, temporary / "params.echo", "parameters")
        text = (temporary / "params.echo").read_text().strip()
        params = dict(json.loads(text.removeprefix("ECHO: ")))
        reports = {}; assembly_mounts = {}
        for side in ("L", "R"):
            filename = f"m0601_mount_{side}.stl"
            export(openscad, out / filename, "mount", side)
            mesh = checked_mesh(out / filename)
            # Invert the source's rigid print transform on the ACTUAL printable
            # mesh. No second float32 STL rounding at large assembly coordinates.
            xmin=params["anchor_center_x"]-params["frame_size"]/2
            if side=="R": xmin=params["board_width"]-params["anchor_center_x"]-params["frame_size"]/2
            ymin=params["anchor_center_y"]-params["frame_size"]/2
            assembly=mesh.copy()
            assembly.apply_transform([[1,0,0,xmin],[0,0,1,ymin],[0,-1,0,0],[0,0,0,1]])
            assembly_mounts[side]=assembly
            reports[filename] = {"bounds_mm":mesh.bounds.round(4).tolist(), "size_mm":mesh.extents.round(4).tolist(),
                                 "volume_mm3":round(mesh.volume,3),"triangles":len(mesh.faces),"watertight":True,
                                 "sha256":hashlib.sha256((out/filename).read_bytes()).hexdigest()}
            # Check all four chassis screw centre-lines (including nut pockets).
            xs = params["anchor_center_x"] + np.array([-1,1])*params["strut_pitch_x"]/2
            ys = params["anchor_center_y"] + np.array([-1,1])*params["strut_pitch_y"]/2
            probes = np.array([[params["board_width"]-x if side=="R" else x,y,z]
                               for x in xs for y in ys for z in np.linspace(-params["frame_depth"]+.1,-.1,17)])
            assert not np.any(assembly.contains(probes)), "Blocked M3 anchor screw bore"
            # The three M2.5 holes are open through the entire cap.
            probes = []
            for a in np.radians([90,210,330])+np.radians(params["motor_clock_deg"]):
                for x in np.linspace(params["stator_face_x"]+.1,params["stator_face_x"]+params["face_plate_thickness"]-.1,17):
                    probes.append([params["board_width"]-x if side=="R" else x,
                                   params["anchor_center_y"]+params["bolt_circle"]/2*np.cos(a),
                                   -params["axle_drop"]+params["bolt_circle"]/2*np.sin(a)])
            assert not np.any(assembly.contains(probes)), "Blocked M2.5 motor screw bore"
        for filename, part in (("chassis_fit_gauge.stl","chassis_gauge"),("motor_fit_gauge.stl","motor_gauge")):
            export(openscad,out/filename,part);mesh=checked_mesh(out/filename)
            reports[filename]={"size_mm":mesh.extents.round(4).tolist(),"volume_mm3":round(mesh.volume,3),
                               "watertight":True,"sha256":hashlib.sha256((out/filename).read_bytes()).hexdigest()}
        assert abs(assembly_mounts["L"].volume-assembly_mounts["R"].volume)<30, "Unexpected handed geometry difference"
        clearances = []
        for side in ("L", "R"):
            wheel_path = temporary / f"wheel_{side}.stl";stator_path=temporary/f"stator_{side}.stl"
            export(openscad,wheel_path,"envelope",side,"assembly",maximum=True)
            export(openscad,stator_path,"stator",side,"assembly",maximum=True)
            wheel=checked_mesh(wheel_path);stator=checked_mesh(stator_path)
            pairs=[(f'{side} mount / maximum rotating wheel',assembly_mounts[side],wheel),
                   (f'{side} mount / maximum stator profile',assembly_mounts[side],stator)]
            for name in BODY:
                body=trimesh.load_mesh(ROOT/"models"/(name+".stl"))
                pairs.extend([(f'{side} mount / {name}',assembly_mounts[side],body),
                              (f'{side} maximum wheel / {name}',wheel,body)])
            for label,a,b in pairs:
                volume=intersection_volume(a,b)
                clearances.append({"pair":label,"intersection_mm3":round(volume,6)})
                assert volume<.01,(label,volume)
        volume=intersection_volume(assembly_mounts["L"],assembly_mounts["R"])
        assert volume<.01
        clearances.append({"pair":"left / right mounts","intersection_mm3":round(volume,6)})
        nominal_face=params["stator_face_x"]-params["stator_projection"]
        nominal_outside=params["stator_face_x"]-params["overall_width"]
        summary={
            "schema":"2pybot-m0601-mount/1","units":"mm","status":"CAD-checked prototype; physical fit/load test pending",
            "chassis_interface_source":"Current models/*.stl geometry plus estimated rev5 30 x 30 mm anchor pattern",
            "unverified_dimensions":["PCB four-anchor centre spacing/location","existing axle drop","actual motor profile/cable/connector envelope"],
            "parameters":params,"exports":reports,"intersection_checks":clearances,
            "minimum_axial_wheel_socket_clearance_mm":round(params["socket_lip_x"]-(params["stator_face_x"]-params["stator_projection_min"]),3),
            "maximum_wheel_to_tray_floor_vertical_clearance_mm":round(params["axle_drop"]-params["wheel_diameter_max"]/2-9,3),
            "nominal_total_width_mm":round(params["board_width"]-2*nominal_outside,3),
            "approximate_tread_center_track_mm":round(params["board_width"]-(nominal_face+nominal_outside),3),
            "minimum_mount_ground_clearance_mm":round(-params["axle_drop"]-params["socket_outer_diameter"]/2+
                                                         params["axle_drop"]+params["wheel_diameter_min"]/2,3),
            "motor_screw_engagement_mm":round(10-params["face_plate_thickness"]-.5,3),
            "verification":"Closed meshes, positive volume, single connected body, screw-bore probes and manifold intersections with current chassis and maximum motor envelopes. Not structural FEA or measured hardware fit."
        }
        (out/"fit-report.json").write_text(json.dumps(summary,indent=2)+"\n")
        drawing(params,out/"mounting-dimensions.png")
        for filename,part,side,layout,camera in (
            ("assembly-preview.png","assembly","L","assembly","75,45,5,68,0,35,400"),
            ("mount-preview.png","mount","L","assembly","23,45,-40,65,0,45,160"),
            ("print-orientation.png","mount","L","print","20,40,20,65,0,35,160")):
            export(openscad,out/filename,part,side,layout,extra=["--imgsize","1600,1200","--viewall","--autocenter",
                   "--projection","o","--camera",camera,"--colorscheme","Tomorrow"])
    bundle=out/"2PyBot-M0601-mount-rev1.zip"
    with zipfile.ZipFile(bundle,"w",compression=zipfile.ZIP_DEFLATED) as archive:
        for path in CAD.glob("*.scad"):archive.write(path,path.name)
        readme=CAD/"README.md"
        if readme.exists():archive.write(readme,"README.md")
        for path in out.iterdir():
            if path.is_file() and path!=bundle:archive.write(path,"exports/"+path.name)
    print(json.dumps({"folder":str(out),"bundle":str(bundle),"meshes":reports,
                      "intersection_checks_passed":len(clearances),
                      "minimum_axial_clearance_mm":summary["minimum_axial_wheel_socket_clearance_mm"],
                      "maximum_wheel_to_base_clearance_mm":summary["maximum_wheel_to_tray_floor_vertical_clearance_mm"]},indent=2))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--openscad",type=Path,required=True)
    args=parser.parse_args();build(args.openscad)
