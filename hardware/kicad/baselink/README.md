# BaseLink controller board (KiCad 10)

STM32G474RET6 balance controller for 2PyBot: ELRS radio (CRSF), two RS485
hub-motor links, Radxa Cubie host header, 5S battery input with ideal-diode
reverse protection and current monitor, an LM5069 hot-swap motor bus switch
driven by a hardware safety chain (motor watchdog AND E-stop), three
independent 5 V rails and a 3V3 LDO. Architecture: see the conversation
notes / `docs` (STM32 + ELRS plan).

**Status: schematic draft r0.1 + placeholder PCB.** Footprints are placed
automatically in one region per block; nothing is routed. Verify the STM32 pin
map in STM32CubeMX, the Cubie A7A 40-pin mapping, the ISM6HG256X pinout
(ISM330DHCX symbol used) and the hot-swap/UVLO values before layout.

## Files

| File | What |
|---|---|
| `baselink.kicad_pro` | Project: net classes (Default, Battery, Rail5V, RS485), 4-layer rules |
| `baselink.kicad_sch` | Root: block diagram of the hierarchical sheets |
| `power_input`, `motor_power`, `regulators`, `mcu`, `rs485`, `comms`, `io` `.kicad_sch` | One sheet per block |
| `baselink.kicad_pcb` | Placeholder board: all 144 footprints, nets, outline, M3 holes, GND plane on In1 |
| `baselink.kicad_sym`, `sym-lib-table` | Project power symbols (VBAT, VMOT, +5V_SBC, +5V_ACT, +5V_CTRL, +3V3A) |
| `build/baselink_schematic.pdf` | Schematic PDF |
| `build/erc.rpt`, `build/drc.rpt` | Last check results |
| `tools/design.py` | **The circuit**: every part, footprint and pin-to-net map |
| `tools/gen_sch.py` | Generates the project, symbol library and all sheets |
| `tools/gen_pcb.py` | Generates the board with KiCad's Python (pcbnew) |

## Regenerate

```powershell
cd hardware\kicad\baselink
python tools\gen_sch.py                                  # project + schematic + build\netlist.json
D:\7_Tools\KiCad\bin\python.exe tools\gen_pcb.py         # placeholder board
D:\7_Tools\KiCad\bin\kicad-cli.exe sch erc baselink.kicad_sch -o build\erc.rpt --severity-all
D:\7_Tools\KiCad\bin\kicad-cli.exe pcb drc baselink.kicad_pcb -o build\drc.rpt --severity-all
D:\7_Tools\KiCad\bin\kicad-cli.exe sch export pdf baselink.kicad_sch -o build\baselink_schematic.pdf
```

Edit `tools/design.py`, not the generated `.kicad_sch` files. `gen_sch.py`
checks every pin name against the library symbol and every pin number against
the footprint's pads, and lists unconnected pins.

**Once layout starts, stop regenerating the board.** Edit the schematic source
and use *Tools → Update PCB from Schematic* in KiCad; footprints carry their
schematic links, so placement and routing are kept.

## Known check results

- ERC: 1 warning. IMU `SDx` is tied to GND as the datasheet recommends for an
  unused aux bus; it is bidirectional, so ERC flags it against the GND flag.
- DRC: only unrouted connections and library silkscreen overlaps.

## Sheets

| Sheet | Contents |
|---|---|
| Power input | XT60, 30 A blade fuse, LM74700 + N-FET ideal diode, SMDJ26A, bulk, 1 mΩ shunt + INA228, VBAT ADC divider |
| Motor switch | LM5069 hot-swap + N-FET + 2 mΩ sense, fail-safe UVLO enable (BSS138 + BSS84), 74LVC1G123 motor watchdog, 74LVC1G08 AND with the E-stop loop, VMOT TVS + bulk, 2× XT30 |
| Regulators | 3× Pololu D36V step-down placeholders (+5V_SBC 5 A, +5V_ACT 3 A, +5V_CTRL 0.6 A), TLV75733 3V3 LDO, power LED |
| MCU | STM32G474RET6 LQFP64, decoupling, VDDA ferrite, 24 MHz crystal, BOOT0/NRST, buttons, 10-pin SWD, USB-C with USBLC6 and VBUS OR-ing, status LED |
| RS485 | 2× socketed DIP-8 transceivers (SN65HVD3082E class), termination jumpers, DNP bias, SM712, JST-GH |
| Comms | ELRS JST-GH, Radxa 2×20 socket (5 V, host UART, BOOT0/NRST via 1 k, lidar and ToF pass-through), SN65HVD230 CAN, spare UART |
| IO | IMU (SPI + 2 INT), 2× Qwiic, WS2812 level shifter + connector, 4 servo headers, torch MOSFET, buzzer driver, rail and NTC ADC dividers |
