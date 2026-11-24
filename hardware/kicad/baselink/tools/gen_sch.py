"""Generate the hierarchical KiCad 10 schematic for BaseLink from design.py.

    python tools/gen_sch.py

Writes baselink.kicad_sch (root, block diagram of sheet symbols), one
<sheet>.kicad_sch per block, baselink.kicad_sym (project power symbols),
sym-lib-table, and build/netlist.json for gen_pcb.py.

Drawing style
  * every block sheet: title + notes, then coloured, bordered groups
  * every pin: a 2.54 mm wire stub ending in a label, power symbol or
    hierarchical label; unused pins get a no-connect flag
  * nets used on several sheets become hierarchical labels; the root sheet
    shows each block with its pins, joined by labels of the same name
"""
import json
import math
import sys
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kisexpr as K  # noqa: E402
from kisexpr import Q  # noqa: E402
import design as D  # noqa: E402

OUT = Path(__file__).resolve().parents[1]
NS = uuid.UUID("5b1d2a6e-2b0c-4f3e-9a51-2b5a4c1e0b00")
VERSION, GEN_VER = 20260101, "10.0"
G = 2.54
STUB = 2.54
FONT = 1.27
PAPERS = [("A3", 420, 297), ("A2", 594, 420), ("A1", 841, 594)]
SHEET_NAMES = {"power_input": "Power input", "motor_power": "Motor switch", "regulators": "Regulators", "mcu": "MCU",
               "rs485": "RS485", "comms": "Comms", "io": "IO"}
STD_POWER = {"GND": "power:GND", "+3V3": "power:+3V3", "VBUS": "power:VBUS"}


def uid(*key):
    return str(uuid.uuid5(NS, "/".join(str(k) for k in key)))


def snap(v, g=G):
    return round(round(v / g) * g, 4)


def text_w(s, size=FONT):
    return len(s) * size * 0.78 + size


def effects(size=FONT, justify=None, bold=False, color=None, hide=False):
    font = ["font", ["size", size, size]]
    if bold:
        font += [["thickness", round(size * 0.18, 3)], ["bold", "yes"]]
    if color:
        font.append(["color"] + list(color))
    e = ["effects", font]
    if justify:
        e.append(["justify"] + justify.split())
    return e


def prop(name, value, x, y, angle=0, hide=False, size=FONT, justify=None):
    p = ["property", Q(name), Q(value), ["at", x, y, angle]]
    if hide:
        p.append(["hide", "yes"])
    p += [["show_name", "no"], ["do_not_autoplace", "no"], effects(size, justify)]
    return p


# ------------------------------------------------------------------ symbols
class LibSym:
    cache = {}

    def __init__(self, lib_id):
        self.lib_id = lib_id
        if lib_id.startswith("baselink:"):
            self.node = power_symbol(lib_id.split(":", 1)[1])
        else:
            self.node = K.lib_symbol(lib_id)
        base = lib_id.split(":")[-1]
        units = [int(s[1][len(base) + 1:].split("_")[0]) for s in K.find(self.node, "symbol") if str(s[1]).startswith(base + "_")]
        self.units = max([u for u in units] + [1])
        self.props = {p[1]: p for p in K.find(self.node, "property")}
        self.is_power = bool(K.first(self.node, "power"))

    @classmethod
    def get(cls, lib_id):
        if lib_id not in cls.cache:
            cls.cache[lib_id] = LibSym(lib_id)
        return cls.cache[lib_id]

    def pins(self, unit):
        return K.pins(self.node, unit)

    def bbox(self, unit):
        return K.bbox(self.node, unit)


def power_symbol(name):
    """Project power symbol for rails not in the KiCad power library."""
    base = K.lib_symbol("power:GND" if name == "GND" else "power:+5V")
    old = base[1].split(":")[1]
    node = K._deep(base)
    node[1] = Q("baselink:" + name)

    def walk(n):
        for i, x in enumerate(n):
            if isinstance(x, list):
                if x and x[0] == "symbol" and isinstance(x[1], str) and x[1].startswith(old + "_"):
                    x[1] = Q(name + x[1][len(old):])
                if x and x[0] == "property" and x[1] == "Value":
                    x[2] = Q(name)
                if x and x[0] == "property" and x[1] == "Description":
                    x[2] = Q('Power symbol creates a global label with name "%s"' % name)
                if x and x[0] == "name" and len(x) > 1 and isinstance(x[1], str) and x[1] in (old, "~"):
                    x[1] = Q(name)
                walk(x)
    walk(node)
    return node


def xform(px, py, rot):
    """symbol coords (y up) -> schematic offset (y down) for rotation rot (CCW)."""
    if rot == 0:
        return px, -py
    if rot == 90:
        return -py, -px
    if rot == 180:
        return -px, py
    return py, px


def outward(pin_angle, rot):
    a = math.radians((pin_angle + 180 + rot) % 360)
    return round(math.cos(a)), -round(math.sin(a))      # screen vector


def rail_lib(net):
    return STD_POWER.get(net, "baselink:" + net)


# ------------------------------------------------------------------ project model
class Inst:
    """One placed unit of a part."""

    def __init__(self, part, unit, sheet):
        self.part, self.unit, self.sheet = part, unit, sheet
        self.sym = LibSym.get(part.lib)
        rot = part.rot
        if rot is None:
            ps = self.sym.pins(unit)
            rot = 90 if len(ps) == 2 and all(p["angle"] in (90, 270) for p in ps) and not self.sym.is_power else 0
        self.rot = rot
        self.x = self.y = 0.0


def build_project():
    counters = {}
    errors, warnings = [], []
    sheet_insts = {}
    net_sheets = {}
    for sh in D.SHEETS:
        insts = []
        for g in sh.groups:
            g.insts = []
            for part in g.parts:
                sym = LibSym.get(part.lib)
                n = counters.get(part.prefix, 0) + 1
                counters[part.prefix] = n
                part.ref = ("#FLG%02d" % n) if part.prefix == "FLG" else "%s%d" % (part.prefix, n)
                part.sheet = sh
                # resolve pin keys (number or name) -> numbers
                allpins = {}
                for u in range(1, sym.units + 1):
                    for p in sym.pins(u):
                        allpins.setdefault(p["number"], (p, u))
                byname = {}
                for num, (p, u) in allpins.items():
                    byname.setdefault(p["name"], []).append(num)
                nets = {}
                for key, net in part.pins.items():
                    if key in allpins:
                        nets[key] = net
                    elif key in byname:
                        for num in byname[key]:
                            nets[num] = net
                    else:
                        errors.append("%s %s: no pin '%s'" % (part.ref, part.lib, key))
                part.nets = nets
                unmapped = [n for n, (p, u) in allpins.items() if n not in nets and p["type"] != "no_connect"
                            and not (p["hidden"] and p["type"] in ("power_in", "passive") and p["name"] in byname
                                     and any(m in nets for m in byname[p["name"]]))]
                if unmapped and not sym.is_power:
                    warnings.append("%s (%s): unconnected -> NC flag: %s" % (part.ref, part.value, ", ".join(
                        "%s/%s" % (n, allpins[n][0]["name"]) for n in sorted(unmapped, key=lambda s: (len(s), s)))))
                for net in set(nets.values()):
                    if net != "NC" and net not in D.RAILS:
                        net_sheets.setdefault(net, set()).add(sh.key)
                # footprint pad check
                fp = part.fp if part.fp is not None else str(sym.props.get("Footprint", [0, 0, ""])[2])
                part.footprint = fp
                if fp and not sym.is_power:
                    try:
                        pads = set(K.footprint_pads(fp))
                        missing = [n for n in allpins if n not in pads]
                        if missing:
                            errors.append("%s footprint %s lacks pads for pins %s" % (part.ref, fp, missing))
                    except FileNotFoundError:
                        errors.append("%s: footprint not found %s" % (part.ref, fp))
                elif not fp and not sym.is_power:
                    errors.append("%s (%s) has no footprint" % (part.ref, part.lib))
                for u in range(1, sym.units + 1):
                    inst = Inst(part, u, sh)
                    g.insts.append(inst)
                    insts.append(inst)
        sheet_insts[sh.key] = insts
    n = 0
    for part in D.ROOT_PARTS:
        n += 1
        part.ref = "H%d" % n
        part.sheet = None
        part.nets = {}
        part.footprint = part.fp
    hier = {net for net, s in net_sheets.items() if len(s) > 1}
    return sheet_insts, hier, errors, warnings


# ------------------------------------------------------------------ geometry of one instance
def pin_ends(inst):
    out = []
    for p in inst.sym.pins(inst.unit):
        dx, dy = xform(p["x"], p["y"], inst.rot)
        ox, oy = outward(p["angle"], inst.rot)
        out.append((p, dx, dy, ox, oy))
    return out


def label_len(net, hier):
    if net == "NC":
        return 0
    if net in D.RAILS:
        return 6.0
    return text_w(net) + (3.0 if net in hier else 1.0)


def local_box(inst, hier):
    x0, y0, x1, y1 = inst.sym.bbox(inst.unit)
    pts = [xform(x0, y0, inst.rot), xform(x1, y1, inst.rot)]
    bx0, bx1 = min(p[0] for p in pts), max(p[0] for p in pts)
    by0, by1 = min(p[1] for p in pts), max(p[1] for p in pts)
    l = r = t = b = 0.0
    for p, dx, dy, ox, oy in pin_ends(inst):
        net = inst.part.nets.get(p["number"], "NC")
        ext = STUB + label_len(net, hier)
        if ox < 0:
            l = max(l, ext)
        if ox > 0:
            r = max(r, ext)
        if oy < 0:
            t = max(t, ext)
        if oy > 0:
            b = max(b, ext)
    text_pad = 4.0
    return (bx0 - l - 1.27, by0 - max(t, text_pad) - 1.27, bx1 + r + 1.27, by1 + max(b, text_pad) + 1.27)


# ------------------------------------------------------------------ node builders
class SheetWriter:
    def __init__(self, path_str, project):
        self.items = []
        self.path_str = path_str
        self.project = project
        self.libs = {}
        self.wires = set()
        self.power_shown = []

    def use(self, lib_id):
        self.libs[lib_id] = LibSym.get(lib_id).node

    def wire(self, x1, y1, x2, y2):
        key = (x1, y1, x2, y2)
        if key in self.wires:
            return
        self.wires.add(key)
        self.items.append(["wire", ["pts", ["xy", x1, y1], ["xy", x2, y2]],
                           ["stroke", ["width", 0], ["type", "default"]], ["uuid", Q(uid("w", self.path_str, *key))]])

    def label(self, net, x, y, ox, oy, hier):
        ang = {(1, 0): 0, (-1, 0): 180, (0, -1): 90, (0, 1): 270}[(ox, oy)]
        just = "left" if ang in (0, 90) else "right"
        if hier:
            self.items.append(["hierarchical_label", Q(net), ["shape", "bidirectional"], ["at", x, y, ang],
                               effects(FONT, just), ["uuid", Q(uid("hl", self.path_str, net, x, y))]])
        else:
            self.items.append(["label", Q(net), ["at", x, y, ang], effects(FONT, just + " bottom"),
                               ["uuid", Q(uid("l", self.path_str, net, x, y))]])

    def no_connect(self, x, y):
        self.items.append(["no_connect", ["at", x, y], ["uuid", Q(uid("nc", self.path_str, x, y))]])

    def text(self, s, x, y, size=FONT, bold=False, color=None, justify="left bottom"):
        self.items.append(["text", Q(s), ["exclude_from_sim", "no"], ["at", x, y, 0],
                           effects(size, justify, bold, color), ["uuid", Q(uid("t", self.path_str, s, x, y))]])

    def rect(self, x0, y0, x1, y1, color, alpha=0.10, width=0.35, dash="solid"):
        r, g, b = color
        self.items.append(["rectangle", ["start", x0, y0], ["end", x1, y1],
                           ["stroke", ["width", width], ["type", dash], ["color", r, g, b, 1]],
                           ["fill", ["type", "color"], ["color", r, g, b, alpha]],
                           ["uuid", Q(uid("r", self.path_str, x0, y0, x1, y1))]])

    def symbol(self, lib_id, x, y, rot, unit, ref, value, footprint, props_src, sym_uuid, pin_numbers, dnp=False,
               value_pos=None, ref_pos=None, hide_ref=False, hide_value=False):
        self.use(lib_id)
        s = LibSym.get(lib_id)
        node = ["symbol", ["lib_id", Q(lib_id)], ["at", x, y, rot], ["unit", unit], ["body_style", 1],
                ["exclude_from_sim", "no"], ["in_bom", "no" if s.is_power else "yes"], ["on_board", "no" if s.is_power else "yes"],
                ["in_pos_files", "yes"], ["dnp", "yes" if dnp else "no"], ["uuid", Q(sym_uuid)]]

        def lib_pos(name):
            p = s.props.get(name)
            if not p:
                return x, y
            at = K.first(p, "at")
            dx, dy = xform(at[1], at[2], rot)
            return snap(x + dx, 0.635), snap(y + dy, 0.635)
        rx, ry = ref_pos or lib_pos("Reference")
        vx, vy = value_pos or lib_pos("Value")
        node.append(prop("Reference", ref, rx, ry, 0, hide=hide_ref))
        node.append(prop("Value", value, vx, vy, 0, hide=hide_value))
        node.append(prop("Footprint", footprint or "", x, y, 0, hide=True))
        for extra in ("Datasheet", "Description"):
            p = s.props.get(extra)
            node.append(prop(extra, str(p[2]) if p else "", x, y, 0, hide=True))
        for num in pin_numbers:
            node.append(["pin", Q(num), ["uuid", Q(uid("pin", sym_uuid, num, unit))]])
        node.append(["instances", ["project", Q(self.project), ["path", Q(self.path_str), ["reference", Q(ref)], ["unit", unit]]]])
        self.items.append(node)

    def write(self, path, file_uuid, paper, title, extra_top=()):
        tb = ["title_block", ["title", Q(title)], ["date", Q(D.PROJECT["date"])], ["rev", Q(D.PROJECT["rev"])],
              ["company", Q(D.PROJECT["company"])], ["comment", 1, Q(D.PROJECT["title"])]]
        root = ["kicad_sch", ["version", VERSION], ["generator", Q("eeschema")], ["generator_version", Q(GEN_VER)],
                ["uuid", Q(file_uuid)], ["paper", Q(paper)], tb,
                ["lib_symbols"] + [self.libs[k] for k in sorted(self.libs)]] + list(self.items) + list(extra_top) + [["embedded_fonts", "no"]]
        Path(path).write_text(K.dump(root) + "\n", encoding="utf-8")


# ------------------------------------------------------------------ drawing a placed unit with its labels
PWR_COUNTER = [0]


def draw_inst(w, inst, hier, sheet_uuid_path):
    part = inst.part
    x, y = inst.x, inst.y
    sym_uuid = uid("sym", part.ref, inst.unit)
    nums = [p["number"] for p in inst.sym.pins(inst.unit)]
    ref_pos = val_pos = None
    hide_value = inst.sym.is_power
    if inst.rot in (90, 270) and len(nums) == 2:
        ref_pos, val_pos = (x, snap(y - 2.54, 0.635)), (x, snap(y + 2.54, 0.635))
    w.symbol(part.lib, x, y, inst.rot, inst.unit, part.ref, part.value, part.footprint, None, sym_uuid,
             sorted(set(nums), key=lambda s: (len(s), s)), dnp=part.dnp, ref_pos=ref_pos, value_pos=val_pos,
             hide_ref=part.ref.startswith("#"), hide_value=hide_value)
    done = set()
    for p, dx, dy, ox, oy in pin_ends(inst):
        px, py = round(x + dx, 4), round(y + dy, 4)
        if (px, py) in done:
            continue
        done.add((px, py))
        net = part.nets.get(p["number"])
        if net is None:
            if p["type"] == "no_connect":
                continue
            stacked = [q for q in inst.sym.pins(inst.unit) if q["number"] != p["number"]
                       and abs(q["x"] - p["x"]) < 1e-6 and abs(q["y"] - p["y"]) < 1e-6 and q["number"] in part.nets]
            if stacked:
                net = part.nets[stacked[0]["number"]]
            else:
                w.no_connect(px, py)
                continue
        if net == "NC":
            w.no_connect(px, py)
            continue
        lx, ly = round(px + ox * STUB, 4), round(py + oy * STUB, 4)
        if net in D.RAILS and not (inst.sym.is_power and part.lib != "power:PWR_FLAG"):
            w.wire(px, py, lx, ly)
            lib = rail_lib(net)
            up = net != "GND"
            # conventional look: ground hangs down, supplies point up; flip only if
            # that would draw the symbol back over the pin it connects to
            rot = 180 if (up and oy > 0) or (not up and oy < 0) else 0
            PWR_COUNTER[0] += 1
            ref = "#PWR%03d" % PWR_COUNTER[0]
            vdy = (-5.08 if up else 5.08) * (-1 if rot == 180 else 1)
            crowded = any(n == net and abs(ax - lx) < 7.7 and abs(ay - ly) < 7.7 for n, ax, ay in w.power_shown)
            if not crowded:
                w.power_shown.append((net, lx, ly))
            w.symbol(lib, lx, ly, rot, 1, ref, net, "", None, uid("pwr", sheet_uuid_path, lx, ly), ["1"],
                     value_pos=(lx, snap(ly + vdy, 0.635)), hide_ref=True, hide_value=crowded)
        else:
            w.wire(px, py, lx, ly)
            w.label(net, lx, ly, ox, oy, net in hier)


# ------------------------------------------------------------------ sheet layout
def layout_sheet(sh, hier):
    """Place groups in columns and parts in rows; returns (width, height)."""
    top = 22.0 + 4.5 * (1 + len(wrap(sh.note, 150)))
    left = 20.0
    cols = [[] for _ in range(sh.columns)]
    col_h = [0.0] * sh.columns
    for g in sh.groups:
        rows, row, rw, cur_w = [], [], 0.0, 0.0
        boxes = []
        for inst in g.insts:
            bx = local_box(inst, hier)
            boxes.append((inst, bx))
        x = 0.0
        rowh = 0.0
        yrow = 0.0
        content_w = 0.0
        placed = []
        for inst, bx in boxes:
            bw, bh = bx[2] - bx[0], bx[3] - bx[1]
            if x > 0 and x + bw > g.width:
                yrow += rowh + 2.54
                x, rowh = 0.0, 0.0
            placed.append((inst, x - bx[0], yrow - bx[1]))
            x += bw + 2.54
            rowh = max(rowh, bh)
            content_w = max(content_w, x)
        g.local = placed
        g.w = max(content_w, text_w(g.title, 2.0) + 6) + 6
        g.h = yrow + rowh + 12
        c = col_h.index(min(col_h))
        cols[c].append(g)
        col_h[c] += g.h + 8
    x = left
    for c, gs in enumerate(cols):
        y = top
        cw = max([g.w for g in gs] + [0])
        for g in gs:
            g.x, g.y = x, y
            g.w = cw
            for inst, lx, ly in g.local:
                inst.x, inst.y = snap(g.x + 3 + lx), snap(g.y + 9 + ly)
            y += g.h + 8
        x += cw + 10
    return x + 5, max(col_h) + top + 22


def wrap(text, width_mm, size=1.5):
    words, lines, cur = text.split(), [], ""
    per = int(width_mm / (size * 0.78))
    for wd in words:
        if len(cur) + len(wd) + 1 > per:
            lines.append(cur)
            cur = wd
        else:
            cur = (cur + " " + wd).strip()
    if cur:
        lines.append(cur)
    return lines


def pick_paper(w, h):
    for name, pw, ph in PAPERS:
        if w <= pw and h <= ph:
            return name, pw, ph
    return PAPERS[-1]


def dark(c, k=0.55):
    return tuple(int(v * k) for v in c)


def write_project(project, root_uuid, sheet_uuid):
    """KiCad project file: net classes, board rules (JLCPCB-class 4-layer), sheets."""
    def nc(name, track, clear, via=0.6, drill=0.3, colour="rgba(0, 0, 0, 0.000)", priority=2147483647):
        return {"name": name, "track_width": track, "clearance": clear, "via_diameter": via, "via_drill": drill,
                "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_width": 0.2, "diff_pair_gap": 0.25,
                "diff_pair_via_gap": 0.25, "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": colour,
                "schematic_color": colour, "priority": priority, "tuning_profile": ""}
    pro = {
        "meta": {"filename": project + ".kicad_pro", "version": 3},
        "board": {"design_settings": {"defaults": {"board_outline_line_width": 0.1, "copper_line_width": 0.2,
                                                   "silk_line_width": 0.12, "silk_text_size_h": 0.8, "silk_text_size_v": 0.8},
                                      "rules": {"min_clearance": 0.127, "min_track_width": 0.127, "min_via_diameter": 0.5,
                                                "min_through_hole_diameter": 0.3, "min_copper_edge_clearance": 0.3,
                                                "min_hole_to_hole": 0.25, "min_hole_clearance": 0.15, "min_silk_clearance": 0.0},
                                      "track_widths": [0.0, 0.15, 0.25, 0.4, 0.8, 1.5, 3.0],
                                      "via_dimensions": [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.6, "drill": 0.3},
                                                         {"diameter": 1.0, "drill": 0.5}]}},
        "net_settings": {
            "classes": [nc("Default", 0.2, 0.127),
                        nc("Battery", 3.0, 0.15, 1.0, 0.5, "rgba(230, 120, 60, 0.900)", 0),
                        nc("Rail5V", 0.8, 0.15, 0.8, 0.4, "rgba(225, 175, 40, 0.900)", 1),
                        nc("RS485", 0.3, 0.2, colour="rgba(40, 160, 120, 0.900)", priority=2)],
            "meta": {"version": 5}, "net_colors": None, "netclass_assignments": None,
            "netclass_patterns": [{"netclass": "Battery", "pattern": p} for p in ("VBAT*", "/VBAT*", "*VBAT_RAW", "*VBAT_FUSED", "*VBAT_PROT", "VMOT", "*HS_SENSE")]
                                 + [{"netclass": "Rail5V", "pattern": p} for p in ("+5V_*", "VBUS")]
                                 + [{"netclass": "RS485", "pattern": p} for p in ("*RS485?_A", "*RS485?_B", "*CANH", "*CANL")],
        },
        "schematic": {"drawing": {"default_line_thickness": 6.0, "default_text_size": 50.0}, "legacy_lib_dir": "", "legacy_lib_list": []},
        "sheets": [[root_uuid, "Root"]] + [[sheet_uuid[s.key], SHEET_NAMES[s.key]] for s in D.SHEETS],
        "text_variables": {"REV": D.PROJECT["rev"]},
    }
    (OUT / (project + ".kicad_pro")).write_text(json.dumps(pro, indent=2) + "\n", encoding="utf-8")


# ------------------------------------------------------------------ main
def main():
    sheet_insts, hier, errors, warnings = build_project()
    root_uuid = uid("root")
    project = D.PROJECT["name"]
    sheet_uuid = {sh.key: uid("sheet", sh.key) for sh in D.SHEETS}
    netlist = {"project": project, "root_uuid": root_uuid, "parts": [], "sheets": {}}
    sheet_hier = {}
    for page, sh in enumerate(D.SHEETS, start=2):
        path_str = "/%s/%s" % (root_uuid, sheet_uuid[sh.key])
        w = SheetWriter(path_str, project)
        width, height = layout_sheet(sh, hier)
        paper, pw, ph = pick_paper(width, height)
        col = sh.color
        w.text(sh.title.upper(), 20, 18, size=4.0, bold=True, color=dark(col, 0.7) + (1,))
        for i, line in enumerate(wrap(sh.note, 150)):
            w.text(line, 20, 25 + i * 4.5, size=1.5, color=(70, 70, 70, 1))
        for g in sh.groups:
            w.rect(g.x, g.y, g.x + g.w, g.y + g.h, col, alpha=0.10, width=0.4)
            w.rect(g.x, g.y, g.x + g.w, g.y + 7.0, col, alpha=0.35, width=0.4)
            w.text(g.title, g.x + 2.5, g.y + 5.0, size=2.0, bold=True, color=dark(col, 0.45) + (1,))
            for inst, _lx, _ly in g.local:
                draw_inst(w, inst, hier, path_str)
        used = sorted({n for inst in sheet_insts[sh.key] for n in inst.part.nets.values() if n in hier})
        sheet_hier[sh.key] = used
        w.write(OUT / (sh.key + ".kicad_sch"), uid("file", sh.key), paper, sh.title)
        netlist["sheets"][sh.key] = {"uuid": sheet_uuid[sh.key], "name": SHEET_NAMES[sh.key], "file": sh.key + ".kicad_sch",
                                     "title": sh.title, "page": page, "color": col}
        for inst in sheet_insts[sh.key]:
            if inst.unit != 1 or inst.sym.is_power:
                continue
            p = inst.part
            nets = {}
            for num, net in p.nets.items():
                if net == "NC":
                    continue
                nets[num] = net if net in D.RAILS else ("/" + net if net in hier else "/%s/%s" % (SHEET_NAMES[sh.key], net))
            netlist["parts"].append({"ref": p.ref, "value": p.value, "footprint": p.footprint, "sheet": sh.key,
                                     "group": next(g.title for g in sh.groups if p in g.parts), "dnp": p.dnp,
                                     "path": "/%s/%s" % (sheet_uuid[sh.key], uid("sym", p.ref, 1)), "nets": nets})
        print("sheet %-12s %-3s %3d parts, %2d hierarchical nets" % (sh.key, paper, len({i.part.ref for i in sheet_insts[sh.key]}), len(used)))

    # ---------------- root sheet
    w = SheetWriter("/" + root_uuid, project)
    order = [["power_input", "motor_power", "regulators"], ["mcu", "rs485", "comms", "io"]]
    maxlab = max(text_w(n) for n in hier) + 3
    cell_w = 55.88 + 2 * (STUB + maxlab)
    y = 52.0
    sheets_nodes = []
    by_key = {sh.key: sh for sh in D.SHEETS}
    for r, row in enumerate(order):
        rowh = 0
        for c, key in enumerate(row):
            sh = by_key[key]
            pins = sheet_hier[key]
            half = (len(pins) + 1) // 2
            left, right = pins[:half], pins[half:]
            sw = 55.88
            shh = max(len(left), len(right)) * G + 2 * G + 6
            sx = snap(20 + STUB + maxlab + c * cell_w)
            sy = snap(y)
            col = sh.color
            su = sheet_uuid[key]
            node = ["sheet", ["at", sx, sy], ["size", sw, snap(shh)], ["exclude_from_sim", "no"], ["in_bom", "yes"],
                    ["on_board", "yes"], ["dnp", "no"], ["fields_autoplaced", "yes"],
                    ["stroke", ["width", 0.4], ["type", "solid"], ["color"] + list(dark(col, 0.6)) + [1]],
                    ["fill", ["color"] + list(col) + [0.18]], ["uuid", Q(su)],
                    prop("Sheetname", SHEET_NAMES[key], sx, round(sy - 0.7, 4), 0, size=2.0, justify="left bottom"),
                    prop("Sheetfile", key + ".kicad_sch", sx, round(sy + shh + 0.6, 4), 0, size=1.27, justify="left top")]
            for i, n in enumerate(left):
                py = snap(sy + 2 * G + i * G)
                node.append(["pin", Q(n), "bidirectional", ["at", sx, py, 180], ["uuid", Q(uid("sp", key, n))], effects(FONT, "left")])
                w.wire(round(sx - STUB, 4), py, sx, py)
                w.label(n, round(sx - STUB, 4), py, -1, 0, False)
            for i, n in enumerate(right):
                py = snap(sy + 2 * G + i * G)
                node.append(["pin", Q(n), "bidirectional", ["at", sx + sw, py, 0], ["uuid", Q(uid("sp", key, n))], effects(FONT, "right")])
                w.wire(sx + sw, py, round(sx + sw + STUB, 4), py)
                w.label(n, round(sx + sw + STUB, 4), py, 1, 0, False)
            page = netlist["sheets"][key]["page"]
            node.append(["instances", ["project", Q(project), ["path", Q("/" + root_uuid), ["page", Q(str(page))]]]])
            sheets_nodes.append(node)
            rowh = max(rowh, shh)
        y += rowh + 22
    root_w = 25 + 4 * cell_w
    paper, pw, ph = pick_paper(root_w, y + 60)
    w.text("2PyBot BaseLink controller: block diagram", 20, 22, size=5.0, bold=True, color=(40, 40, 40, 1))
    intro = ("STM32G474RET6 balance controller with ELRS radio, two RS485 hub-motor links, Radxa host header and a "
             "hardware motor safety chain. Each block below is a sheet: double-click to open. Labels of the same name join "
             "block pins; power rails (" + ", ".join(D.RAILS) + ") are global power symbols. Generated by tools/gen_sch.py "
             "from tools/design.py: edit the design file and regenerate, do not hand-edit the generated sheets.")
    for i, line in enumerate(wrap(intro, 300)):
        w.text(line, 20, 30 + i * 4.5, size=1.6, color=(70, 70, 70, 1))
    # mounting holes
    hx = snap(pw - 140)
    for i, part in enumerate(D.ROOT_PARTS):
        w.symbol(part.lib, snap(hx + i * 15.24), snap(ph - 52), 0, 1, part.ref, part.value, part.footprint, None,
                 uid("sym", part.ref, 1), [])
        netlist["parts"].append({"ref": part.ref, "value": part.value, "footprint": part.footprint, "sheet": "root",
                                 "group": "Mechanical", "dnp": False, "path": "/" + uid("sym", part.ref, 1), "nets": {}})
    w.text("Mounting holes", hx, snap(ph - 60), size=1.6, bold=True)
    w.items = sheets_nodes + w.items
    w.write(OUT / (project + ".kicad_sch"), root_uuid, paper, D.PROJECT["title"],
            extra_top=[["sheet_instances", ["path", Q("/"), ["page", Q("1")]]]])
    print("root        ", paper, "%d sheets, %d hierarchical nets" % (len(D.SHEETS), len(hier)))

    # ---------------- project symbol library + table
    lib = ["kicad_symbol_lib", ["version", 20251024], ["generator", Q("baselink_gen")], ["generator_version", Q(GEN_VER)]]
    for rail in D.RAILS:
        if rail not in STD_POWER:
            n = K._deep(power_symbol(rail))
            n[1] = Q(rail)
            lib.append(n)
    (OUT / (project + ".kicad_sym")).write_text(K.dump(lib) + "\n", encoding="utf-8")
    (OUT / "sym-lib-table").write_text('(sym_lib_table\n\t(version 7)\n\t(lib (name "baselink")(type "KiCad")(uri "${KIPRJMOD}/baselink.kicad_sym")(options "")(descr "BaseLink power rails"))\n)\n', encoding="utf-8")
    write_project(project, root_uuid, sheet_uuid)
    (OUT / "build").mkdir(exist_ok=True)
    (OUT / "build" / "netlist.json").write_text(json.dumps(netlist, indent=1), encoding="utf-8")
    print("parts with footprints:", len([p for p in netlist["parts"]]))
    for wmsg in warnings:
        print("NOTE  " + wmsg)
    for e in errors:
        print("ERROR " + e)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
