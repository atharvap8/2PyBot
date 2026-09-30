# Robot CAD

## Packages

| Path | Contents | Status |
| :--- | :--- | :--- |
| [enclosure/rev5/](enclosure/rev5/README.md) | Parametric OpenSCAD enclosure and five STL exports | Designed; not printed or physically fit-tested |
| [Existing STEP model](../../models/2pybot_simplified.step) | Simplified robot model | Separate reference model |

Revision 5 keeps its SCAD sources, logo SVG, and STL exports together to preserve relative references. Shared dimensions are in `dims.scad`, with part-specific dimensions in the individual sources.

The face STL has three zero-area triangles. Other imported STL edge checks passed. Assembly clearances and printer tolerances still need validation on the physical parts.
