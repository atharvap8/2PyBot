---
name: kicad-pcb-design
description: Use when designing, editing or checking KiCad schematics and PCBs for 2PyBot, especially hardware/kicad/baselink (STM32G474 + ELRS BaseLink board) - adding parts, changing nets, regenerating .kicad_sch/.kicad_pcb, running ERC/DRC, or reviewing a board for layout and safety rules.
---

# KiCad PCB design for 2PyBot

KiCad 10 is installed at `D:\7_Tools\KiCad` (`bin\kicad-cli.exe`, `bin\python.exe` with `pcbnew`,
official libraries in `share\kicad\symbols` and `share\kicad\footprints`).

## Golden rule: generate, don't hand-draw

The BaseLink project in `hardware/kicad/baselink` is generated:

1. `tools/design.py` is the single source of truth. Every part is a `Part(prefix, "Lib:Symbol",
   value, "Lib:Footprint", {pin_number_or_name: net})` inside a `Group` inside a `Sheet`.
   Rails listed in `RAILS` become power symbols; `"NC"` places a no-connect; any other net is a
   signal (hierarchical label automatically when used on several sheets).
2. `python tools/gen_sch.py` writes the project, root block diagram, one sheet per block, the
   project power-symbol library and `build/netlist.json`. It fails on unknown pin names and on
   symbol pins missing from the footprint, and lists unconnected pins. Fix every ERROR; read every NOTE.
3. `D:\7_Tools\KiCad\bin\python.exe tools/gen_pcb.py` builds the placeholder board (footprints from
   the official libraries, nets, schematic links, block regions, outline, holes, GND plane).
4. Check: `kicad-cli sch erc --severity-all`, `kicad-cli pcb drc --severity-all`, and export the
   schematic PDF; render pages with PyMuPDF and look at them before reporting done.

Never hand-edit generated `.kicad_sch` files. Never do manual or "creative" PCB placement or routing
in files: placement is automatic shelf packing per block. Once real layout has started in KiCad,
stop regenerating the board; change `design.py`, regenerate the schematic, and tell the user to run
*Update PCB from Schematic* (footprint KIID paths match the symbols).

## Adding a part

- Find symbol and footprint names in the installed libraries (search the `.kicad_sym` files for
  `(symbol "NAME"` and the `.pretty` folders). Prefer an exact symbol; if a part has no symbol,
  use a pin-compatible one with the real part number as Value and note "verify pinout".
- Map pins by name where the symbol has meaningful names (ICs), by number for passives/connectors.
- Two-pin parts are drawn horizontal automatically. Multi-unit symbols are placed per unit.
- Keep each `Group` to one function (about 3-12 parts) and give it a clear title.

## Electrical rules for this robot

- Battery: 5S Li-ion, 16-21 V. Battery-side parts rated >=35 V (TVS SMDJ26A, MOSFETs 40 V+).
- Motor power (VMOT) must only flow while the hardware safety chain enables it. Never connect a
  motor rail directly to VBAT. The default (MCU dead, 3V3 lost, E-stop open) must be OFF.
- The host (Radxa) may stop the robot but never arm it. Keep BOOT0/NRST control through series resistors.
- Separate 5 V rails: +5V_SBC (Radxa, lidar, camera), +5V_ACT (servos, LEDs, torch),
  +5V_CTRL (radio, RS485, logic) feeding the 3V3 MCU LDO. Do not merge them.
- RS485: fail-safe receivers, 120 Ohm termination on a jumper, TVS (SM712) at the connector,
  GND in the harness. Motor IDs: left 1, right 2.
- Every IC gets local 100 nF decoupling; MCU VDDA through a ferrite.
- Net classes live in `baselink.kicad_pro` (written by gen_sch.py): Default 0.2/0.127 mm,
  Battery 3.0 mm tracks, Rail5V 0.8 mm, RS485 0.3 mm. Clearances must allow 0.5 mm-pitch parts.

## Layout guidance (for the human doing layout)

- 4 layers: F.Cu signal, In1.Cu solid GND, In2.Cu power, B.Cu signal.
- Battery to hot-swap to motor connectors: short, wide, own copper area; Kelvin-route current-sense lines.
- IMU near the axle line, far from bucks and motor current; crystal next to the MCU with a GND guard.
- Connectors on the board edge; TVS and transceivers next to their connectors.
- Test points on every rail and on WDG_OK, MOTOR_EN, MOTOR_PG before first power-up.

## Reporting

State what was generated, the ERC/DRC counts by type and which remaining items are expected
(unrouted connections on a placeholder board, library silkscreen overlaps). Say clearly which
values still need verification (CubeMX pin map, host header mapping, regulator choice).
