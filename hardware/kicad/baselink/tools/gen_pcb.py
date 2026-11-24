"""Generate the placeholder BaseLink board with KiCad's own Python (pcbnew).

    D:\\7_Tools\\KiCad\\bin\\python.exe tools/gen_pcb.py

Reads build/netlist.json from gen_sch.py. Every footprint is loaded from the
official KiCad library, linked to its schematic symbol (KIID path) and given
its pad nets, so the ratsnest is complete and "Update PCB from Schematic"
matches 1:1. Placement is automatic: footprints are shelf-packed inside one
labelled region per schematic block. It is a starting point, not a layout:
nothing is routed, connectors are not yet on board edges.

Board: 4 layers (F.Cu signal, In1.Cu GND plane, In2.Cu power, B.Cu signal),
outline with rounded corners, 4x M3 holes, filled GND plane on In1.
"""
import json
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parents[1]
FPROOT = Path(r"D:\7_Tools\KiCad\share\kicad\footprints")
MM = pcbnew.FromMM
GAP = 1.2                     # mm between courtyards
REGION_GAP = 4.0
MARGIN = 8.0
ORIGIN = (40.0, 40.0)          # board top-left on the A3 page
COLUMNS = [["power_input", "motor_power"], ["regulators", "mcu"], ["comms", "rs485"], ["io"]]
COL_W = {"power_input": 50, "motor_power": 50, "regulators": 48, "mcu": 48, "rs485": 56, "comms": 56, "io": 46}


def vec(x, y):
    return pcbnew.VECTOR2I(MM(x + ORIGIN[0]), MM(y + ORIGIN[1]))


def load(fp_id):
    lib, name = fp_id.split(":", 1)
    fp = pcbnew.FootprintLoad(str(FPROOT / (lib + ".pretty")), name)
    if fp is None:
        raise RuntimeError("footprint not found: " + fp_id)
    fp.SetFPIDAsString(fp_id)
    return fp


def size_of(fp):
    bb = fp.GetBoundingBox(False)
    return pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight()), bb


def add_line_rect(board, x0, y0, x1, y1, layer, width=0.15):
    for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(vec(*a))
        s.SetEnd(vec(*b))
        s.SetLayer(layer)
        s.SetWidth(MM(width))
        board.Add(s)


def add_text(board, text, x, y, layer, size=1.2, bold=False, justify_left=True):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetPosition(vec(x, y))
    t.SetLayer(layer)
    t.SetTextSize(vec(size, size))
    t.SetTextThickness(MM(size * (0.2 if bold else 0.15)))
    if justify_left:
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    board.Add(t)


def main():
    data = json.loads((HERE / "build" / "netlist.json").read_text(encoding="utf-8"))
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(4)
    ds = board.GetDesignSettings()
    ds.m_MinClearance = MM(0.127)
    ds.m_TrackMinWidth = MM(0.127)
    tb = board.GetTitleBlock()
    tb.SetTitle("2PyBot BaseLink controller (placeholder placement)")
    tb.SetRevision("0.1")
    tb.SetCompany("2PyBot")
    tb.SetDate("2026-11-24")

    nets = {}

    def net(name):
        if name not in nets:
            ni = pcbnew.NETINFO_ITEM(board, name)
            board.Add(ni)
            nets[name] = ni
        return nets[name]

    by_sheet = {}
    for p in data["parts"]:
        if not p["footprint"]:
            continue
        by_sheet.setdefault(p["sheet"], []).append(p)

    sheets = data["sheets"]
    x_col = MARGIN
    regions = []
    max_h = 0.0
    placed = 0
    for column in COLUMNS:
        y = MARGIN + 6
        width = max(COL_W[k] for k in column)
        for key in column:
            parts = by_sheet.get(key, [])
            fps = []
            for p in parts:
                fp = load(p["footprint"])
                fp.SetReference(p["ref"])
                fp.SetValue(p["value"])
                fp.SetPath(pcbnew.KIID_PATH(p["path"]))
                fp.SetSheetname(sheets[key]["name"])
                fp.SetSheetfile(sheets[key]["file"])
                if p.get("dnp"):
                    fp.SetDNP(True)
                try:
                    fp.SetLocalClearance(None)      # use the board rules, not library overrides
                except (TypeError, AttributeError):
                    pass
                for pad in fp.Pads():
                    try:
                        pad.SetLocalClearance(None)
                    except (TypeError, AttributeError):
                        pass
                    n = p["nets"].get(pad.GetNumber())
                    if n:
                        pad.SetNet(net(n))
                board.Add(fp)
                fps.append(fp)
            # biggest first, shelf packing inside the column width
            fps.sort(key=lambda f: -(size_of(f)[0] * size_of(f)[1]))
            top = y + 5.0                       # room for the region title
            cx, cy, shelf = 0.0, 0.0, 0.0
            for fp in fps:
                w, h, bb = size_of(fp)
                if h > width * 0.8 and w < h:       # long parts (2x20 header) lie along the column
                    fp.SetOrientationDegrees(90)
                    w, h, bb = size_of(fp)
                if cx > 0 and cx + w > width:
                    cy += shelf + GAP
                    cx, shelf = 0.0, 0.0
                # move so the bounding box top-left lands at the target
                pos = fp.GetPosition()
                ox, oy = pcbnew.ToMM(pos.x - bb.GetX()), pcbnew.ToMM(pos.y - bb.GetY())
                fp.SetPosition(vec(x_col + cx + ox, top + cy + oy))
                cx += w + GAP
                shelf = max(shelf, h)
                placed += 1
            region_h = (cy + shelf) + 7.0
            regions.append((key, x_col - 1.5, y - 1.0, x_col + width + 1.5, y + region_h))
            y += region_h + REGION_GAP
        max_h = max(max_h, y)
        x_col += width + REGION_GAP + 3
    # mechanical parts (holes) go to the corners
    W, H = x_col - REGION_GAP - 3 + MARGIN, max_h + MARGIN - REGION_GAP
    holes = [(3.5, 3.5), (W - 3.5, 3.5), (3.5, H - 3.5), (W - 3.5, H - 3.5)]
    for (hx, hy), p in zip(holes, by_sheet.get("root", [])):
        fp = load(p["footprint"])
        fp.SetReference(p["ref"])
        fp.SetValue(p["value"])
        fp.SetPath(pcbnew.KIID_PATH(p["path"]))
        fp.SetPosition(vec(hx, hy))
        board.Add(fp)
        placed += 1

    # outline with rounded corners
    r = 3.0
    edge = pcbnew.Edge_Cuts
    for a, b in (((r, 0), (W - r, 0)), ((W, r), (W, H - r)), ((W - r, H), (r, H)), ((0, H - r), (0, r))):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(vec(*a)); s.SetEnd(vec(*b)); s.SetLayer(edge); s.SetWidth(MM(0.1))
        board.Add(s)
    k = r * (1 - 0.70710678)
    for s0, mid, e0 in (((0, r), (k, k), (r, 0)), ((W - r, 0), (W - k, k), (W, r)),
                        ((W, H - r), (W - k, H - k), (W - r, H)), ((r, H), (k, H - k), (0, H - r))):
        a = pcbnew.PCB_SHAPE(board)
        a.SetShape(pcbnew.SHAPE_T_ARC)
        a.SetArcGeometry(vec(*s0), vec(*mid), vec(*e0))
        a.SetLayer(edge); a.SetWidth(MM(0.1))
        board.Add(a)

    # block regions on the drawing layer, with titles
    for key, x0, y0, x1, y1 in regions:
        add_line_rect(board, x0, y0, x1, y1, pcbnew.Dwgs_User, 0.2)
        add_text(board, sheets[key]["title"], x0 + 1.0, y0 + 2.5, pcbnew.Dwgs_User, 1.4, True)
    add_text(board, "2PyBot BaseLink r0.1", 6.0, H - 1.8, pcbnew.F_SilkS, 1.2, True)

    # GND plane on In1.Cu (filled), keeps the ratsnest readable
    zone = pcbnew.ZONE(board)
    zone.SetLayer(pcbnew.In1_Cu)
    zone.SetNet(net("GND"))
    zone.SetIsRuleArea(False)
    outline = zone.Outline()
    outline.NewOutline()
    for x, y in ((0.5, 0.5), (W - 0.5, 0.5), (W - 0.5, H - 0.5), (0.5, H - 0.5)):
        outline.Append(MM(x), MM(y))
    zone.SetLocalClearance(MM(0.3))
    zone.SetMinThickness(MM(0.25))
    board.Add(zone)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    out = HERE / (data["project"] + ".kicad_pcb")
    pcbnew.SaveBoard(str(out), board)
    text = out.read_text(encoding="utf-8")
    out.write_text(text.replace('(paper "A4")', '(paper "A3")', 1), encoding="utf-8")   # page API is not exposed to Python
    print("board %.1f x %.1f mm, %d footprints, %d nets -> %s" % (W, H, placed, len(nets), out.name))


if __name__ == "__main__":
    main()
