"""SVG diagrams of the 16-LED status ring for the manuals.

Seen from above, robot front at the top, LED numbering clockwise. Angles in
degrees: 0 = front, +90 = right. Used by build_pdf.py through {{ring:NAME}}.
"""
import math

N = 16
STEP = 360 / N
OFF = "#2b2622"
C = {"w": "#ffffff", "g": "#2ee66b", "G": "#0f5a2a", "b": "#3d8bff", "B": "#16315c", "v": "#a64dff", "V": "#3a1f57",
     "c": "#38c8e8", "C": "#164a57", "o": "#ff8a1f", "a": "#ffb020", "m": "#ff2fd0", "r": "#ff2a3a", "R": "#6b1a20",
     "y": "#d8d84a"}


def _all(code):
    return {i * STEP: code for i in range(N)}


def _merge(*parts):
    out = {}
    for p in parts:
        out.update(p)
    return out


# name: (leds {angle: colour code}, arrows [(from_deg, to_deg, dashed)], spin (None | +1 cw | -1 ccw))
STATES = {
    "loading":    (_merge({a: "C" for a in (0, 22.5, 45, 67.5)}, {135: "w", 112.5: "C", 90: "C"}), [], +1),
    "readyfill":  (_all("g"), [], None),
    "waiting":    (_merge(_all("V"), {0: "w"}), [], None),
    "liftme":     ({-22.5: "G", 0: "g", 22.5: "G"}, [], None),
    "uprighting": ({45: "o", 22.5: "R", 225: "o", 202.5: "R"}, [], +1),
    "holding":    (_merge(_all("G"), {0: "g"}), [], None),
    "holdauto":   (_merge(_all("B"), {0: "b"}), [], None),
    "stiff":      (_merge(_all("G"), {0: "g", 90: "a", -90: "a"}), [], None),
    "forward":    ({-45: "g", -67.5: "G", 45: "g", 67.5: "G"}, [(-90, -8, False), (90, 8, False)], None),
    "reverse":    ({-135: "g", -112.5: "G", 135: "g", 112.5: "G"}, [(-90, -172, False), (90, 172, False)], None),
    "turnright":  ({22.5: "w", 0: "#777777"}, [(-90, 88, False)], None),
    "turnleft":   ({-22.5: "w", 0: "#777777"}, [(90, -88, False)], None),
    "autofwd":    ({-45: "b", -67.5: "B", 45: "b", 67.5: "B"}, [(-90, -8, False), (90, 8, False)], None),
    "climbfwd":   ({-45: "v", 45: "v"}, [(-90, -8, True), (90, 8, True)], None),
    "climbturn":  ({0: "v"}, [(-90, 88, True)], None),
    "climbstop":  ({0: "v"}, [], None),
    "pushed":     ({-22.5: "a", 0: "a", 22.5: "a", -45: "a", 45: "a"}, [], None),
    "bubble":     (_merge(_all("G"), {0: "m", -22.5: "m", 22.5: "m"}), [], None),
    "fell":       ({0: "r", -22.5: "r", 22.5: "r", -45: "R", 45: "R"}, [], None),
    "fatal":      (_all("r"), [], None),
}
for n in range(1, 10):
    STATES[f"err{n}"] = ({k * STEP: "r" for k in range(n)}, [], None)


def _pt(deg, r, cx=60, cy=60):
    a = math.radians(deg - 90)
    return cx + r * math.cos(a), cy + r * math.sin(a)


def _arc_path(a0, a1, r):
    x0, y0 = _pt(a0, r)
    x1, y1 = _pt(a1, r)
    sweep = 1 if a1 > a0 else 0
    large = 1 if abs(a1 - a0) > 180 else 0
    return f"M{x0:.1f},{y0:.1f} A{r},{r} 0 {large} {sweep} {x1:.1f},{y1:.1f}"


def svg(name, size=118):
    leds, arrows, spin = STATES[name]
    out = [f'<svg class="ring" width="{size}" height="{size}" viewBox="0 0 120 120" xmlns="http://www.w3.org/2000/svg">',
           '<defs><marker id="ah" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto">'
           '<path d="M0,0 L6,3 L0,6 z" fill="#8a7f74"/></marker></defs>',
           '<circle cx="60" cy="60" r="47" fill="#0c0b0a" stroke="#3a332d" stroke-width="1"/>',
           '<path d="M60,4 l-5,7 h10 z" fill="#f08a4b"/>']                       # front marker
    for i in range(N):
        deg = i * STEP
        col = None
        for a, code in leds.items():
            if abs(((a - deg + 540) % 360) - 180) < 0.1:
                col = C.get(code, code)
        x, y = _pt(deg, 38)
        if col:
            out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="8.5" fill="{col}" opacity="0.28"/>')
            out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5.2" fill="{col}"/>')
        else:
            out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5.2" fill="{OFF}" stroke="#3d3630" stroke-width="0.8"/>')
    for a0, a1, dashed in arrows:
        dash = ' stroke-dasharray="3 3"' if dashed else ""
        out.append(f'<path d="{_arc_path(a0, a1, 25)}" fill="none" stroke="#8a7f74" stroke-width="2"{dash} marker-end="url(#ah)"/>')
    if spin:
        out.append(f'<path d="{_arc_path(200, 330, 25) if spin > 0 else _arc_path(330, 200, 25)}" fill="none" '
                   'stroke="#8a7f74" stroke-width="2" marker-end="url(#ah)"/>')
    out.append("</svg>")
    return "".join(out)
