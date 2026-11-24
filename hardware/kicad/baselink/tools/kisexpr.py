"""Minimal S-expression reader/writer for KiCad files, plus library helpers.

Nodes are Python lists: ["symbol", "Device:R", ["at", 0, 0], ...]. Strings that
came from a quoted token are kept as `Q` (a str subclass) so they are written back
quoted; bare atoms are plain str / int / float.
"""
import re
from pathlib import Path

KICAD = Path(r"D:\7_Tools\KiCad")
SYMDIR = KICAD / "share" / "kicad" / "symbols"
FPDIR = KICAD / "share" / "kicad" / "footprints"


class Q(str):
    """A quoted string token."""


TOKEN = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))', re.S)


def parse(text):
    stack, cur = [], []
    pos = 0
    n = len(text)
    while pos < n:
        m = TOKEN.match(text, pos)
        if not m:
            if text[pos:].strip() == "":
                break
            raise ValueError("bad s-expression near %r" % text[pos:pos + 40])
        pos = m.end()
        if m.group(1):
            stack.append(cur)
            cur = []
        elif m.group(2):
            done, cur = cur, stack.pop()
            cur.append(done)
        elif m.group(3) is not None:
            cur.append(Q(m.group(3).replace('\\"', '"').replace("\\\\", "\\")))
        else:
            a = m.group(4)
            try:
                cur.append(int(a))
            except ValueError:
                try:
                    cur.append(float(a))
                except ValueError:
                    cur.append(a)
    return cur[0] if len(cur) == 1 else cur


def _atom(v):
    if isinstance(v, Q):
        return '"' + v.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        s = ("%.4f" % v).rstrip("0").rstrip(".")
        return s if s not in ("-0", "") else "0"
    return str(v)


def dump(node, indent=0):
    if not isinstance(node, list):
        return _atom(node)
    simple = all(not isinstance(x, list) for x in node)
    if simple:
        return "(" + " ".join(_atom(x) for x in node) + ")"
    pad = "\t" * (indent + 1)
    head = [x for x in node if not isinstance(x, list)]
    parts = ["(" + " ".join(_atom(x) for x in head)]
    for x in node:
        if isinstance(x, list):
            parts.append("\n" + pad + dump(x, indent + 1))
    return "".join(parts) + "\n" + "\t" * indent + ")"


def find(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]


def first(node, key, default=None):
    r = find(node, key)
    return r[0] if r else default


_LIBCACHE = {}


def load_lib(lib):
    if lib not in _LIBCACHE:
        _LIBCACHE[lib] = parse((SYMDIR / (lib + ".kicad_sym")).read_text(encoding="utf-8"))
    return _LIBCACHE[lib]


def lib_symbol(lib_id):
    """Flattened library symbol (derived symbols merged into their parent),
    named with the full lib_id as KiCad embeds it in a schematic."""
    lib, name = lib_id.split(":", 1)
    root = load_lib(lib)
    syms = {s[1]: s for s in find(root, "symbol")}
    if name not in syms:
        raise KeyError(lib_id)
    sym = _deep(syms[name])           # never mutate the cached library tree
    ext = first(sym, "extends")
    if ext:
        parent = lib_symbol(lib + ":" + ext[1])
        props = {p[1]: p for p in find(sym, "property")}
        out = [x for x in parent if not (isinstance(x, list) and x and x[0] == "property" and x[1] in props)]
        out[1] = Q(lib_id)
        insert = [i for i, x in enumerate(out) if isinstance(x, list) and x and x[0] == "property"]
        at = insert[-1] + 1 if insert else 2
        out[at:at] = list(props.values())
        pname = ext[1]
        for x in out:
            if isinstance(x, list) and x and x[0] == "symbol" and isinstance(x[1], str) and x[1].startswith(pname + "_"):
                x[1] = Q(name + x[1][len(pname):])
        return out
    out = [x for x in sym]
    out[1] = Q(lib_id)
    return out


def _deep(node):
    return [(_deep(x) if isinstance(x, list) else x) for x in node]


def pins(sym, unit=1):
    """[{number,name,x,y,angle,length,type,hidden}] for units 0 and `unit`, body style 1."""
    out = []
    base = sym[1].split(":")[-1]
    for sub in find(sym, "symbol"):
        m = re.match(re.escape(base) + r"_(\d+)_(\d+)$", sub[1])
        if not m:
            continue
        u, style = int(m.group(1)), int(m.group(2))
        if u not in (0, unit) or style not in (0, 1):
            continue
        for p in find(sub, "pin"):
            at = first(p, "at")
            out.append({"number": str(first(p, "number")[1]), "name": str(first(p, "name")[1]),
                        "x": float(at[1]), "y": float(at[2]), "angle": int(at[3]) if len(at) > 3 else 0,
                        "length": float(first(p, "length")[1]), "type": p[1], "hidden": "hide" in p or bool(first(p, "hide"))})
    return out


def bbox(sym, unit=1):
    """Body bounding box in symbol coordinates (y up): graphics + pin endpoints."""
    xs, ys = [], []
    base = sym[1].split(":")[-1]
    for sub in find(sym, "symbol"):
        m = re.match(re.escape(base) + r"_(\d+)_(\d+)$", sub[1])
        if not m or int(m.group(1)) not in (0, unit) or int(m.group(2)) not in (0, 1):
            continue
        for g in sub:
            if not isinstance(g, list):
                continue
            if g[0] == "rectangle":
                for k in ("start", "end"):
                    v = first(g, k)
                    xs.append(v[1]); ys.append(v[2])
            elif g[0] in ("polyline", "bezier"):
                for p in find(first(g, "pts") or [], "xy"):
                    xs.append(p[1]); ys.append(p[2])
            elif g[0] == "circle":
                c, r = first(g, "center"), first(g, "radius")[1]
                xs += [c[1] - r, c[1] + r]; ys += [c[2] - r, c[2] + r]
            elif g[0] == "arc":
                for k in ("start", "mid", "end"):
                    v = first(g, k)
                    xs.append(v[1]); ys.append(v[2])
    for p in pins(sym, unit):
        xs.append(p["x"]); ys.append(p["y"])
    if not xs:
        return (-2.54, -2.54, 2.54, 2.54)
    return (min(xs), min(ys), max(xs), max(ys))


def footprint_pads(fp_id):
    lib, name = fp_id.split(":", 1)
    path = FPDIR / (lib + ".pretty") / (name + ".kicad_mod")
    tree = parse(path.read_text(encoding="utf-8"))
    return sorted({str(p[1]) for p in find(tree, "pad") if str(p[1])})
