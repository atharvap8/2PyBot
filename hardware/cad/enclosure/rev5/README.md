# Enclosure Revision 5

OpenSCAD source and STL exports imported from `2pybot-covers-rev5`.

**Status: designed, not yet printed or physically fit-tested.** Dimensions marked `ROUGH` still need measurement against the assembled robot. Mesh checks do not establish physical clearance or printability.

## Files

| Source | Export | Part |
| :--- | :--- | :--- |
| `base.scad` | `base.stl` | Bottom board tray and shell attachment pads |
| `shell.scad` | `shell.stl` | Main cover, lid, ring channel, and cable/camera openings |
| `face.scad` | `face.stl` | Camera/ToF face plate and logo |
| `motor.scad` with `side="L"` | `motor_L.stl` | Left motor/standoff cover |
| `motor.scad` with `side="R"` | `motor_R.stl` | Right motor/standoff cover |
| `dims.scad` | None | Shared dimensions, fit allowances, fasteners, and coordinate frame |
| `logo.scad`, `logo_B.svg` | Included in face export | Extruded face logo |

Keep the source and SVG files together. `face.scad` uses `logo.scad`, which imports `logo_B.svg` by relative path.

## Coordinate frame

Origin is the bottom-deck front-left corner, with z=0 at the solder-side face. X runs left to right when viewed from the front, Y front to back, and Z upward. Exports use assembly coordinates; orient and place each part on the build plate in the slicer.

## Current dimensions

| Feature | Source value |
| :--- | :--- |
| Board width/depth/thickness | 150 / 90 / 1.6 mm |
| Bottom-to-mid clear gap | 60 mm |
| Mid-to-top clear gap | 40 mm |
| Sliding fit allowance | 0.3 mm per side |
| M3 pilot / clearance diameter | 2.5 / 3.4 mm |
| Corner hole inset / diameter | 3.5 / 3.2 mm, estimated |
| Motor standoff square | 30 mm, estimated |
| Tray motor openings | Two 44 mm squares |
| Lens opening | 26 x 18 mm, estimated |
| Ring outer/inner diameter | 68 / 54 mm |
| Ring channel depth | 4.5 mm |

Shared dimensions live in `dims.scad`. Ring, camera, fan, and face-opening dimensions also have local definitions in `shell.scad` and `face.scad`.

## Export

Run from this directory with OpenSCAD installed:

```bash
openscad -o base.stl base.scad
openscad -o shell.stl shell.scad
openscad -o face.stl face.scad
openscad -D 'side="L"' -o motor_L.stl motor.scad
openscad -D 'side="R"' -o motor_R.stl motor.scad
```

Text uses `DejaVu Sans:style=Bold`; install that font when reproducing the exports.

## Intended assembly

The tray supports the board on ledges and corner posts. The shell slides over the tray and attaches through side pads and top rear screws. Motor covers slide outboard 3 mm below their final position, then lift into the tray openings and attach through two tabs each.

The supplied design note calls for fitting the left cover before mounting the right motor, or tilting it into place. Verify that sequence on the printed parts. Motor, connector, encoder, standoff, and cable envelopes include estimated dimensions.

Material, print orientation, supports, and manufacturing tolerances have not been validated on a physical print.

## Imported mesh checks

All five exports have closed edges, consistent edge orientation, and positive signed volume. `face.stl` contains **three zero-area triangles**; the other four exports contain none. Re-export or repair the face mesh before printing. No part-intersection or physical-clearance check was performed during import.

| Export | Triangles | Bounding-box size, mm | Zero-area triangles |
| :--- | :--- | :--- | :--- |
| `base.stl` | 5036 | 156.6 x 96.6 x 16.6 | 0 |
| `shell.stl` | 9474 | 162.8 x 102.8 x 140.6 | 0 |
| `face.stl` | 3904 | 150.0 x 13.29 x 44.0 | 3 |
| `motor_L.stl` | 2140 | 63.5 x 62.8 x 86.4 | 0 |
| `motor_R.stl` | 2496 | 63.5 x 62.8 x 86.4 | 0 |
