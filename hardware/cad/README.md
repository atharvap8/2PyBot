# Robot CAD

## Packages

| Path | Contents | Status |
| :--- | :--- | :--- |
| [enclosure/rev5/](enclosure/rev5/README.md) | Parametric OpenSCAD enclosure and five STL exports | Designed; not printed or physically fit-tested |
| [m0601_mount/rev1/](m0601_mount/rev1/README.md) | Left/right M0601 hub-motor forks replacing four struts per stepper, fit gauges, previews and geometry report | CAD prototype; chassis anchor spacing and hardware fit need confirmation |
| [Current assembled STLs](../../models/) | Split base, shell, face, motor covers and camera beam used by the live robot viewer | Reference for the M0601 mount clearance checks |
| [Existing STEP model](../../models/2pybot_simplified.step) | Simplified robot model | Separate reference model |

Revision 5 keeps its SCAD sources, logo SVG, and STL exports together to preserve relative references. Shared dimensions are in `dims.scad`, with part-specific dimensions in the individual sources.

The face STL has three zero-area triangles. Other imported STL edge checks passed. Assembly clearances and printer tolerances still need validation on the physical parts.
