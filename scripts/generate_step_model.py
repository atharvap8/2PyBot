"""Generate a STEP file of the 2PyBot simplified model.

Reproduces the geometry shown in the GUI's 3D viewer (chassis stack,
sensor deck, front indicator and two wheels) as real solids, scaled so
that 1 viewer-unit = 100 mm.

Usage:
    python scripts/generate_step_model.py [output.step]

Requires: cadquery  (pip install cadquery)
"""

import sys
from pathlib import Path

import cadquery as cq

# 1 viewer unit = 100 mm
U = 100.0

# Dimensions taken from Robot3DViewer geometry (half-extents * 2 * U)
LOWER_FRAME = (1.24 * U, 0.60 * U, 0.22 * U)   # x, y, z sizes
LOWER_Z0 = 0.06 * U

MAIN_BODY = (1.10 * U, 0.52 * U, 1.07 * U)
MAIN_Z0 = 0.28 * U

SENSOR_DECK = (1.16 * U, 0.58 * U, 0.17 * U)
DECK_Z0 = 1.35 * U

INDICATOR = (0.32 * U, 0.10 * U, 0.20 * U)
INDICATOR_Z0 = 1.05 * U
INDICATOR_Y = 0.29 * U  # offset toward front face

WHEEL_RADIUS = 0.55 * U
WHEEL_THICKNESS = 0.18 * U
WHEEL_X = 0.80 * U      # axle offset from centerline
HUB_RADIUS = 0.12 * U

# NEMA17-style stepper motors (42.3 mm square body, 40 mm long),
# mounted inside the lower frame, shafts pointing out to the wheels.
MOTOR_BODY = 42.3       # square face size (mm)
MOTOR_LENGTH = 40.0     # body length along the axle (mm)
MOTOR_SHAFT_R = 2.5     # 5 mm shaft
MOTOR_BOSS_R = 11.0     # front pilot boss radius (22 mm dia)
MOTOR_BOSS_LEN = 2.0

# ---- Detail features (inspired by concept render) ----
TREAD_COUNT = 24        # tread lugs around each tire
TREAD_DEPTH = 4.0       # radial lug height (mm)
TREAD_WIDTH = 6.0       # lug width along circumference (mm)
RIM_GROOVE_R = 0.42 * U  # glow-ring groove radius on wheel face
RIM_GROOVE_W = 4.0      # groove width (mm)
SPOKE_HOLE_R = 6.0      # lightening holes in wheel face
SPOKE_HOLE_COUNT = 6
SPOKE_HOLE_PITCH_R = 0.27 * U

CAM_POD_R = 9.0         # side camera pod barrel radius
CAM_POD_LEN = 26.0      # pod length (sticks out sideways)
CAM_LENS_R = 5.0
CAM_Z = 1.10 * U        # pod height on the body

CONSOLE = (0.70 * U, 0.34 * U, 0.10 * U)   # top console box
DISPLAY = (0.60 * U, 6.0, 0.34 * U)        # thin tilted display panel
DISPLAY_TILT_DEG = 20.0

VENT_COUNT = 5          # front vent slots
VENT = (0.30 * U, 4.0, 5.0)                # slot size (x, depth, z)
VENT_Z0 = 0.45 * U
VENT_PITCH = 10.0

PANEL_INSET = (0.80 * U, 3.0, 0.70 * U)    # recessed side panel pocket
PANEL_Z0 = 0.42 * U


def box_at(size, z0):
    """Box centered in XY, resting with its base at height z0."""
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2], centered=(True, True, False))
        .translate((0, 0, z0))
    )


def make_motor(side):
    """NEMA17-style stepper: square body + pilot boss + shaft.

    Body sits on the axle line (z = 0), inboard of the wheel; the shaft
    extends outward through the frame to the wheel hub at x = WHEEL_X.
    """
    # Body: extrude inward from the mounting face.
    face_x = side * (WHEEL_X - WHEEL_THICKNESS - MOTOR_BOSS_LEN)
    body = (
        cq.Workplane("YZ")
        .rect(MOTOR_BODY, MOTOR_BODY)
        .extrude(-MOTOR_LENGTH * side)  # extrude toward robot center
        .edges("|X")
        .chamfer(4.0)                   # NEMA corner chamfers
        .translate((face_x, 0, 0))
    )
    boss = (
        cq.Workplane("YZ")
        .circle(MOTOR_BOSS_R)
        .extrude(MOTOR_BOSS_LEN * side)
        .translate((face_x, 0, 0))
    )
    shaft = (
        cq.Workplane("YZ")
        .circle(MOTOR_SHAFT_R)
        .extrude((WHEEL_X - abs(face_x) + WHEEL_THICKNESS) * side)
        .translate((face_x, 0, 0))
    )
    return body.union(boss).union(shaft)


def make_wheel(side):
    """Detailed wheel: treaded tire, glow-ring groove, spoke holes, hub."""
    import math

    x_out = side * WHEEL_X                       # outer face
    x_mid = side * (WHEEL_X - WHEEL_THICKNESS / 2)

    wheel = (
        cq.Workplane("YZ")
        .circle(WHEEL_RADIUS)
        .extrude(WHEEL_THICKNESS * side)
        .translate((side * (WHEEL_X - WHEEL_THICKNESS), 0, 0))
    )

    # Tread lugs around the circumference
    for i in range(TREAD_COUNT):
        a = i * 2 * math.pi / TREAD_COUNT
        cy = (WHEEL_RADIUS + TREAD_DEPTH / 2) * math.cos(a)
        cz = (WHEEL_RADIUS + TREAD_DEPTH / 2) * math.sin(a)
        lug = (
            cq.Workplane("YZ")
            .rect(TREAD_WIDTH, TREAD_DEPTH)
            .extrude(WHEEL_THICKNESS * 0.9)
            .rotate((0, 0, 0), (1, 0, 0), math.degrees(a) + 90)
            .translate((x_mid - WHEEL_THICKNESS * 0.45, cy, cz))
        )
        wheel = wheel.union(lug)

    # Glow-ring groove cut into the outer face
    groove = (
        cq.Workplane("YZ", origin=(x_out, 0, 0))
        .circle(RIM_GROOVE_R + RIM_GROOVE_W / 2)
        .circle(RIM_GROOVE_R - RIM_GROOVE_W / 2)
        .extrude(-3.0 * side)
    )
    wheel = wheel.cut(groove)

    # Spoke lightening holes on the outer face
    for i in range(SPOKE_HOLE_COUNT):
        a = i * 2 * math.pi / SPOKE_HOLE_COUNT
        hy = SPOKE_HOLE_PITCH_R * math.cos(a)
        hz = SPOKE_HOLE_PITCH_R * math.sin(a)
        hole = (
            cq.Workplane("YZ", origin=(x_out, hy, hz))
            .circle(SPOKE_HOLE_R)
            .extrude(-6.0 * side)
        )
        wheel = wheel.cut(hole)

    # Hub cap
    hub = (
        cq.Workplane("YZ")
        .circle(HUB_RADIUS)
        .extrude(WHEEL_THICKNESS * 1.2 * side)
        .translate((side * (WHEEL_X - WHEEL_THICKNESS), 0, 0))
    )
    return wheel.union(hub)


def make_camera_pod(side):
    """Cylindrical camera pod on the upper body side, lens facing forward."""
    x_base = side * (MAIN_BODY[0] / 2)
    barrel = (
        cq.Workplane("YZ")
        .circle(CAM_POD_R)
        .extrude(CAM_POD_LEN * side)
        .translate((x_base, 0, CAM_Z))
    )
    # Mounting arm back to the body
    arm = (
        cq.Workplane("YZ")
        .rect(8.0, 8.0)
        .extrude(CAM_POD_LEN * 0.4 * side)
        .translate((x_base, 0, CAM_Z))
    )
    # Lens ring on the front of the barrel
    lens = (
        cq.Workplane("XZ", origin=(x_base + side * CAM_POD_LEN * 0.6,
                                   CAM_POD_R, CAM_Z))
        .circle(CAM_LENS_R)
        .extrude(4.0)
    )
    return barrel.union(arm).union(lens)


def make_console():
    """Top console box with a tilted display panel (holo screen stand-in)."""
    import math

    console = (
        cq.Workplane("XY")
        .box(CONSOLE[0], CONSOLE[1], CONSOLE[2], centered=(True, True, False))
        .translate((0, 0, DECK_Z0 + SENSOR_DECK[2]))
    )
    display = (
        cq.Workplane("XY")
        .box(DISPLAY[0], DISPLAY[1], DISPLAY[2], centered=(True, True, False))
        .rotate((0, 0, 0), (1, 0, 0), -DISPLAY_TILT_DEG)
        .translate((0, -CONSOLE[1] / 4,
                    DECK_Z0 + SENSOR_DECK[2] + CONSOLE[2]))
    )
    return console.union(display)


def build_robot():
    body = box_at(LOWER_FRAME, LOWER_Z0)
    body = body.union(box_at(MAIN_BODY, MAIN_Z0))
    body = body.union(box_at(SENSOR_DECK, DECK_Z0))

    indicator = box_at(INDICATOR, INDICATOR_Z0).translate((0, INDICATOR_Y, 0))
    body = body.union(indicator)

    # Recessed side panels on the main body
    for side in (-1, 1):
        pocket = (
            cq.Workplane("YZ")
            .rect(PANEL_INSET[0] * 0.6, PANEL_INSET[2])
            .extrude(-PANEL_INSET[1] * side)
            .translate((side * MAIN_BODY[0] / 2, 0,
                        PANEL_Z0 + PANEL_INSET[2] / 2))
        )
        body = body.cut(pocket)

    # Front vent slots (horizontal grille)
    for i in range(VENT_COUNT):
        vent = (
            cq.Workplane("XY")
            .box(VENT[0], VENT[1], VENT[2], centered=(True, True, False))
            .translate((0, MAIN_BODY[1] / 2 - VENT[1] / 2 + 1.0,
                        VENT_Z0 + i * VENT_PITCH))
        )
        body = body.cut(vent)

    # Top console with tilted display panel
    body = body.union(make_console())

    # Side camera pods
    for side in (-1, 1):
        body = body.union(make_camera_pod(side))

    # Stepper motors on the axle line, one per side.
    for side in (-1, 1):
        body = body.union(make_motor(side))

    # Detailed wheels (tread, glow groove, spoke holes, hub)
    for side in (-1, 1):
        body = body.union(make_wheel(side))

    return body


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        Path(__file__).resolve().parent.parent / "models" / "2pybot_simplified.step"
    )
    out.parent.mkdir(parents=True, exist_ok=True)

    robot = build_robot()
    cq.exporters.export(robot, str(out))
    print(f"STEP file written: {out}")


if __name__ == "__main__":
    main()
