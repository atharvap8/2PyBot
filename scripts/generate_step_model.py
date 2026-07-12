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


def box_at(size, z0):
    """Box centered in XY, resting with its base at height z0."""
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2], centered=(True, True, False))
        .translate((0, 0, z0))
    )


def build_robot():
    body = box_at(LOWER_FRAME, LOWER_Z0)
    body = body.union(box_at(MAIN_BODY, MAIN_Z0))
    body = body.union(box_at(SENSOR_DECK, DECK_Z0))

    indicator = box_at(INDICATOR, INDICATOR_Z0).translate((0, INDICATOR_Y, 0))
    body = body.union(indicator)

    # Wheels: cylinders along X axis, centered on the axle (z = 0)
    for side in (-1, 1):
        wheel = (
            cq.Workplane("YZ")
            .circle(WHEEL_RADIUS)
            .extrude(WHEEL_THICKNESS * side)
            .translate((side * (WHEEL_X - WHEEL_THICKNESS / 2), 0, 0))
        )
        hub = (
            cq.Workplane("YZ")
            .circle(HUB_RADIUS)
            .extrude(WHEEL_THICKNESS * 1.2 * side)
            .translate((side * (WHEEL_X - WHEEL_THICKNESS / 2), 0, 0))
        )
        body = body.union(wheel).union(hub)

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
