# M0601 four-strut replacement mount — Rev 1

Two rigid, keyed motor forks replace the **four brass struts per stepper motor**,
the old metal stepper bracket and the old motor cover. They reuse the existing
four bottom-PCB motor anchor positions. The current split base, shell, face and
camera beam are retained.

**Status:** CAD-checked prototype. The existing chassis source labels its
30 × 30 mm motor-strut pattern as estimated. The original struts/metal bracket
are not represented as measured solids in the current STL assembly. Confirm
their hole locations and axle height on the physical bot; successful mesh checks
do not establish hardware fit or structural capacity.

## Files

| File | Purpose |
| --- | --- |
| `dimensions.scad` | Editable chassis, axle, motor and print-fit parameters |
| `mount.scad` | Parametric source; select side, part and print/assembly layout |
| `exports/m0601_mount_L.stl` | Left mount, already side-flat at Z=0 for printing |
| `exports/m0601_mount_R.stl` | Right mount, already side-flat at Z=0 for printing |
| `exports/chassis_fit_gauge.stl` | Small four-hole gauge for the old strut pattern |
| `exports/motor_fit_gauge.stl` | Keyed socket/three-bolt/cable-exit fit gauge |
| `exports/assembly-preview.png` | Existing robot with replacement mounts and hub-wheel envelopes |
| `exports/mount-preview.png` | Bracket geometry in assembly coordinates |
| `exports/print-orientation.png` | Recommended supplied print orientation |
| `exports/mounting-dimensions.png` | Interface drawing and fastener notes |
| `exports/fit-report.json` | Parameters, mesh checks, clearance checks and STL hashes |
| `exports/2PyBot-M0601-mount-rev1.zip` | Source, printable meshes, images and report together |

The wheel in the preview is a **conservative cylindrical envelope**, not an
accurate manufacturer wheel mesh. The downloaded ZIP's mount/gauge source is
standalone. `part="assembly"` uses the repository's five retained `models/*.stl`
files; the complete assembly preview PNG is included in the ZIP.

## Mechanical interfaces

All dimensions are millimetres. Assembly origin is the bottom PCB's front-left
corner at the solder face. X is lateral, Y points to the rear, Z points upward.

### Existing chassis — confirm before full printing

- Left anchor centre `(23,45)`, right centre `(127,45)` in the 150 mm board frame.
- Four M3 anchors at **30 × 30 mm** centres: left `(8,30)`, `(38,30)`, `(8,60)`,
  `(38,60)`; right mirrored about X=75. These coordinates inherit the estimated
  existing enclosure dimensions, not a PCB manufacturing drawing.
- A **43.4 mm square** upper frame fits the current nominal 44 mm tray opening,
  with 0.30 mm clearance per side. Top pads meet the PCB underside at Z=0.
  The upper outboard rail is relieved around the current split tray's PCB
  support ledge, so the original tray does not need trimming.
- Axle centre at Y=45, Z=−65. The 65 mm drop is a starting layout based on the
  current assembly's wheel/stepper placement, not a measurement of its axle.
  Adjust `axle_drop` to the measured PCB-underside-to-axle distance if needed.
- Four side-loading M3 nut pockets, 5.8 mm across flats and 2.7 mm high;
  four 3.4 mm clearance bores.

### M0601 stator — manufacturer drawing

- **Three M2.5 threaded holes**, 120° apart, **15.20 mm bolt circle**, **5 mm thread
  depth**. Adapter holes are 2.90 mm clearance; default clocking puts one above
  the axle and two below.
- Stator diameter **19 mm**, tolerance +0.05/−0.15; three equally spaced flats
  labelled **3×8 ±0.15 EQS**, interpreted as flats 8 mm from the axle. Their
  normals lie between the three bolt directions. Check this profile on the
  delivered motor using the motor gauge.
- Socket diameter **19.45 mm** and flat distance **8.40 mm** include maximum
  stator tolerances plus 0.20 mm radial / 0.25 mm flat clearance.
- Socket depth **13 mm**, from X=11 to the stator end face at X=24. A **6 mm end
  plate** carries the three screws. Rear cable relief is open along the socket;
  the plate stays intact. Cable/connector shapes are not dimensioned sufficiently
  to prove their full routing clearance; verify the delivered harness and bend.
- Drawing wheel diameter **100.70 ±1.50 mm**, overall axial width **65.50 ±0.50 mm**,
  stator projection **15 +0.95/−0.55 mm**. Checks use the 102.20 mm maximum wheel
  and minimum projection, rather than assuming an exact advertised 102 mm wheel.

Default nominal total width is **233 mm**. The approximate wheel-centre track is
182.5 mm; measure the actual tread-centre distance for motor-control integration.
The maximum wheel envelope clears the tray floor by 4.9 mm vertically and the
socket entry by 1.45 mm axially. These numbers depend on the supplied parameters.

## Fasteners — per side

- 4 × **M3×10** machine screws, 4 × M3 hex nuts and 4 × M3 washers for the PCB
  anchors. Side-load the nuts into the upper frame before installing the mount.
  Confirm screw length against the actual PCB/washer stack.
- 3 × **M2.5×10** machine screws and approximately 0.5 mm-thick washers for the
  motor. With the 6 mm end plate, engagement is `10−6−0.5 = 3.5 mm`, within the
  manufacturer's 5 mm depth. Verify the actual washers/screw lengths before
  tightening; no countersink or head counterbore is used.
- A small cable tie through the web slot for strain relief.

Both motors share the documented stator interface. Use the handed left/right
wheel versions ordered for the corresponding sides and verify cable exit
orientation. `motor_clock_deg` and `cable_direction` are editable in the source.

## Printing and assembly

1. Print **one chassis gauge** and check all four existing strut holes using the
   actual PCB/hardware. Measure X and Y centre spacing if it differs from 30 mm.
   Confirm axle height before building the full pair. Check surrounding PCB
   components/solder against the frame and washer footprints.
2. Print **one motor gauge upright as supplied**, and check the keyed stator,
   three screw positions, insertion depth and rear cable route on the delivered
   motor. The gauge reproduces the full mount's cap thickness and socket fit.
3. Print **one L and one R mount** in the supplied orientation. Suggested starting
   settings: PETG/ASA, 0.20 mm layers, 6 walls, 6 top/bottom layers and 50–60% infill.
   Use dense modifiers around the socket/cap and anchor frame if desired. Enable
   supports for the horizontal socket ceiling and elevated rear web. Remove
   socket supports fully and clean the M3 nut pockets and screw bores.
4. Remove each old stepper, metal bracket, four struts and motor cover. Install
   the M3 captive nuts in the new mount. Seat the hub stator from outboard and
   secure its three screws from the inboard side; that access stays open between
   the two mounts. Hang the assembly on the four original PCB anchors.
5. Route and tie the cable without trapping it against a rotating surface. Check
   free wheel rotation, frame attachment and actual socket seating before motor
   testing. Material stiffness/creep and fatigue under repeated reversals have
   not been measured; this is the first test mount, not a load-rated component.

The M0601 uses an **18 V supply and RS485 integrated FOC control**. The current
TMC/STEP-DIR firmware requires a separate motor-interface integration for these
motors. At the drawing's 100.7 mm diameter, 115 rpm rated speed is approximately
0.61 m/s and 200 rpm no-load speed approximately 1.05 m/s at the specified supply.

## Reproduce

From the repository root, with OpenSCAD and the Python geometry dependencies:

```sh
python -m pip install numpy trimesh networkx rtree manifold3d Pillow
python scripts/generate_m0601_mount.py --openscad /path/to/openscad
```

For one STL directly:

```sh
openscad -D 'part="mount"' -D 'side="L"' -D 'layout="print"' \
  --export-format binstl -o m0601_mount_L.stl mount.scad
```

The generator checks closed meshes, winding, positive volume, single-piece
connectivity, screw-bore access, and volume intersections against all five
retained current robot parts and maximum wheel/stator envelopes. It produces
the report and previews from the same parametric source. These checks do not
include hidden electronics, physical PCB hole metrology, harness bends or FEA.

## Sources

- [DFRobot FIT1038 right motor](https://www.dfrobot.com/product-3076.html)
- [DFRobot installation drawing and thread-depth note](https://wiki.dfrobot.com/fit1038/docs/24652)
- [Evelta handed motor listing](https://evelta.com/m0601-direct-drive-hub-motor-for-agv-right-left-orientation-integrated-foc-servo/)
- Current assembled `models/*.stl` and estimated `enclosure/rev5/dims.scad` anchors.
