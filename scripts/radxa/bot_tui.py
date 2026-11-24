#!/usr/bin/env python3
"""2PyBot control centre: full-screen terminal interface (like nmtui).

    2pybot            open it
    2pybot <command>  run one command without the interface (see `2pybot help`)

Keys: 1-9 pages · arrows move · Enter select · Esc back · X STOP motors · Q quit.
Python 3.9, standard library only (curses).
"""
import curses
import locale
import os
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bot_view as V  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
BOT = ["/usr/local/bin/bot"] if Path("/usr/local/bin/bot").exists() else [sys.executable, str(Path(__file__).with_name("bot_cli.py"))]
locale.setlocale(locale.LC_ALL, "")
UNI = "UTF-8" in (locale.getpreferredencoding(False) or "").upper()
DOT = "●" if UNI else "*"
ARROW = "›" if UNI else ">"

# colour pair ids
ACC, OK, WARN, ERR, INFO, DIM, HEAD, SEL, SELDIM, STOP, MAG, TITLE = range(1, 13)
TONE = {"ok": OK, "warn": WARN, "err": ERR, "info": INFO, "dim": DIM, None: 0, "acc": ACC}


def setup_colours():
    curses.start_color()
    try:
        curses.use_default_colors()
        bg = -1
    except curses.error:
        bg = curses.COLOR_BLACK
    rich = curses.COLORS >= 256
    acc = 208 if rich else curses.COLOR_YELLOW
    dim = 244 if rich else curses.COLOR_WHITE
    seld = 238 if rich else curses.COLOR_BLUE
    pairs = {ACC: (acc, bg), OK: (curses.COLOR_GREEN, bg), WARN: (curses.COLOR_YELLOW, bg), ERR: (curses.COLOR_RED, bg),
             INFO: (curses.COLOR_CYAN, bg), DIM: (dim, bg), HEAD: (curses.COLOR_BLACK, acc), SEL: (curses.COLOR_BLACK, acc),
             SELDIM: (curses.COLOR_WHITE, seld), STOP: (curses.COLOR_WHITE, curses.COLOR_RED), MAG: (curses.COLOR_MAGENTA, bg),
             TITLE: (acc, bg)}
    for k, (fg, b) in pairs.items():
        try:
            curses.init_pair(k, fg, b)
        except curses.error:
            curses.init_pair(k, curses.COLOR_WHITE, curses.COLOR_BLACK)


ESC_KEYS = {"[A": curses.KEY_UP, "[B": curses.KEY_DOWN, "[C": curses.KEY_RIGHT, "[D": curses.KEY_LEFT,
            "OA": curses.KEY_UP, "OB": curses.KEY_DOWN, "OC": curses.KEY_RIGHT, "OD": curses.KEY_LEFT,
            "[H": curses.KEY_HOME, "[F": curses.KEY_END, "OH": curses.KEY_HOME, "OF": curses.KEY_END,
            "[1~": curses.KEY_HOME, "[4~": curses.KEY_END, "[5~": curses.KEY_PPAGE, "[6~": curses.KEY_NPAGE,
            "OM": 10}


def read_key(win):
    """getch that also decodes raw escape sequences (terminals that ignore keypad mode)."""
    k = win.getch()
    if k != 27:
        return k
    win.nodelay(True)
    seq = ""
    try:
        for _ in range(4):
            c = win.getch()
            if c == -1:
                break
            seq += chr(c)
            if seq in ESC_KEYS:
                return ESC_KEYS[seq]
    finally:
        win.nodelay(False)
    return 27 if not seq else -1


def cp(n, extra=0):
    return curses.color_pair(n) | extra


def put(win, y, x, text, attr=0, width=None):
    """addstr that never raises: clips to the window and the given width."""
    h, w = win.getmaxyx()
    if y < 0 or y >= h or x >= w:
        return
    if x < 0:
        text, x = text[-x:], 0
    room = w - x - (1 if y == h - 1 else 0)
    if width is not None:
        room = min(room, width)
    if room <= 0:
        return
    try:
        win.addstr(y, x, str(text)[:room], attr)
    except curses.error:
        pass


def frame(win, y, x, h, w, title="", attr=None, focused=False):
    attr = cp(ACC if focused else DIM) if attr is None else attr
    if h < 2 or w < 2:
        return
    try:
        win.attron(attr)
        win.hline(y, x + 1, curses.ACS_HLINE, w - 2)
        win.hline(y + h - 1, x + 1, curses.ACS_HLINE, w - 2)
        win.vline(y + 1, x, curses.ACS_VLINE, h - 2)
        win.vline(y + 1, x + w - 1, curses.ACS_VLINE, h - 2)
        win.addch(y, x, curses.ACS_ULCORNER)
        win.addch(y, x + w - 1, curses.ACS_URCORNER)
        win.addch(y + h - 1, x, curses.ACS_LLCORNER)
        try:
            win.addch(y + h - 1, x + w - 1, curses.ACS_LRCORNER)
        except curses.error:
            pass
        win.attroff(attr)
    except curses.error:
        pass
    if title:
        put(win, y, x + 2, " " + title + " ", cp(TITLE, curses.A_BOLD) if focused else cp(DIM, curses.A_BOLD))


# ====================================================================== pages
class Page:
    title = ""
    hint = ""

    def __init__(self, app):
        self.app = app

    def enter(self):
        pass

    def draw(self, win, y, x, h, w, focused):
        pass

    def key(self, k):
        return False


class ActionPage(Page):
    """Info lines on top, a selectable action list below."""
    hint = "↑↓ choose · Enter run · Esc back" if UNI else "Up/Down choose - Enter run - Esc back"

    def __init__(self, app):
        super().__init__(app)
        self.sel = 0

    def info(self):
        return []                       # [(label, value, tone)]

    def actions(self):
        return []                       # [(label, description, callable)]

    def draw(self, win, y, x, h, w, focused):
        rows = self.info()
        lw = max((len(l) for l, _, _ in rows), default=0) + 2
        yy = y
        for label, value, tone in rows:
            if yy >= y + h:
                break
            put(win, yy, x, label, cp(DIM))
            put(win, yy, x + lw, value, cp(TONE.get(tone, 0)), w - lw)
            yy += 1
        acts = self.actions()
        if not acts:
            return
        yy += 1 if rows else 0
        put(win, yy, x, "ACTIONS", cp(ACC, curses.A_BOLD))
        yy += 1
        self.sel = max(0, min(self.sel, len(acts) - 1))
        lab = min(46, max(len(a[0]) for a in acts) + 2)
        for i, (label, desc, _fn) in enumerate(acts):
            if yy >= y + h:
                break
            on = i == self.sel
            attr = cp(SEL if focused else SELDIM, curses.A_BOLD) if on else 0
            put(win, yy, x, (" " + ARROW + " " if on else "   ") + label.ljust(lab)[:max(10, w - 4)], attr)
            if w > lab + 16 and desc:
                put(win, yy, x + lab + 4, desc, cp(DIM), w - lab - 4)
            yy += 1

    def key(self, k):
        acts = self.actions()
        if k in (curses.KEY_UP, ord("k")):
            self.sel = max(0, self.sel - 1)
        elif k in (curses.KEY_DOWN, ord("j")):
            self.sel = min(len(acts) - 1, self.sel + 1)
        elif k in (10, 13, curses.KEY_ENTER) and acts:
            acts[self.sel][2]()
        else:
            return False
        return True


class Dashboard(Page):
    title = "Dashboard"
    hint = "Live view, refreshes every second"

    def draw(self, win, y, x, h, w, focused):
        s = self.app.snap
        if not s:
            put(win, y, x, "Collecting…" if UNI else "Collecting...", cp(DIM))
            return
        sections = V.dashboard(s)
        ev = s.get("events")
        if ev:
            counts, _ = ev
            bad = [(k.capitalize(), str(v), "warn") for k, v in counts.items() if v]
            sections.append(("Last 24 hours", bad or [("Events", "nothing unusual", "ok")]))
        cols = 2 if w >= 96 else 1
        colw = (w - 3) // cols
        lw = min(20, max(len(l) for _, rows in sections for l, _, _ in rows) + 2)
        heights = [len(r) + 2 for _, r in sections]
        # greedy two-column split
        order, col_y = [[] for _ in range(cols)], [0] * cols
        for i, hh in enumerate(heights):
            c = col_y.index(min(col_y))
            order[c].append(i)
            col_y[c] += hh
        for c in range(cols):
            yy, xx = y, x + c * (colw + 3)
            for i in order[c]:
                title, rows = sections[i]
                if yy >= y + h:
                    break
                put(win, yy, xx, title.upper(), cp(ACC, curses.A_BOLD))
                yy += 1
                for label, value, tone in rows:
                    if yy >= y + h:
                        break
                    put(win, yy, xx, label, cp(DIM), lw)
                    mark = (DOT + " ") if tone in ("ok", "warn", "err") else "  "
                    put(win, yy, xx + lw, mark, cp(TONE.get(tone, 0)))
                    put(win, yy, xx + lw + 2, value, cp(TONE.get(tone, 0), curses.A_BOLD if tone == "err" else 0), colw - lw - 2)
                    yy += 1
                yy += 1
        # recent controller messages fill the space below
        used = max(col_y) + 1
        room = h - used - 1
        if room >= 3:
            put(win, y + used, x, "RECENT CONTROLLER MESSAGES", cp(ACC, curses.A_BOLD))
            lines = [l for l in V.log_tail(200) if "[PAD] btn=" not in l][-(room - 1):]
            for i, line in enumerate(lines):
                put(win, y + used + 1 + i, x, line[11:19] + "  " + line[24:], cp(TONE.get(V.log_tone(line), DIM) or DIM), w)


class Robot(ActionPage):
    title = "Robot"

    def info(self):
        s = self.app.snap or {}
        st = s.get("status") or {}
        state, tone = V.robot_state(s)
        b = s.get("bubble") or {}
        tof = st.get("tof") or {}
        return [("State", state, tone),
                ("Safety bubble on gamepad", "ON" if b.get("enabled") else "off", "ok" if b.get("enabled") else "dim"),
                ("Depth (ToF) guard", "ON" if tof.get("guard_enabled") else "off", "ok" if tof.get("guard_enabled") else "dim")]

    def actions(self):
        a = self.app
        s = a.snap or {}
        bub = (s.get("bubble") or {}).get("enabled")
        guard = ((s.get("status") or {}).get("tof") or {}).get("guard_enabled")
        return [
            ("Stop motors", "zero torque now (same as X anywhere)", a.stop),
            ("Centre camera", "pan and zoom servos to home", lambda: a.call("/api/command", {"line": "N"}, "Camera centred")),
            ("Safety bubble on gamepad: " + ("turn OFF" if bub else "turn ON"), "missions always use it",
             lambda: a.call("/api/bubble", {"enabled": not bub}, "Bubble " + ("off" if bub else "on") + " for gamepad driving")),
            ("Depth guard: " + ("turn OFF" if guard else "turn ON"), "slows forward driving near low obstacles",
             lambda: a.call("/api/tof/guard", {"enabled": not guard}, "Depth guard " + ("off" if guard else "on"))),
            ("LED test: colour compass", "LED 0 white, 1 yellow, 4 red, 8 green, 12 blue", lambda: a.call("/api/led/test", {"mode": 1}, "Compass shown for 2 min")),
            ("LED test: show front", "front white, right green, back red", lambda: a.call("/api/led/test", {"mode": 2}, "Front check shown")),
            ("LED test: end", "back to normal ring", lambda: a.call("/api/led/test", {"mode": 0}, "LED test ended")),
            ("Calibrate gyro", "robot disarmed and perfectly still", self.gyro),
            ("Re-pair gamepad", "forget the old pairing, then hold HOME + A", self.repair),
            ("Record 30 s of data", "for bubble/vibration analysis", self.record),
        ]

    def gyro(self):
        if self.app.moving("calibrate the gyro"):
            return
        if self.app.confirm("Calibrate gyro", "Keep the robot completely still for about 3 seconds.\nContinue?"):
            self.app.call("/api/command", {"line": "C"}, "Gyro calibration started; keep still")

    def repair(self):
        if self.app.moving("re-pair the gamepad"):
            return
        if self.app.confirm("Re-pair gamepad", "Forget the current gamepad pairing?\nAfterwards hold HOME + A on the gamepad until it connects."):
            r = self.app.call("/api/gamepad/repair", {}, "Pairing cleared: hold HOME + A on the gamepad")
            if r:
                self.app.message("Re-pair gamepad", r.get("message", "Hold HOME + A on the gamepad."))

    def record(self):
        r = self.app.call("/api/debug/record", {"seconds": 30, "label": "tui", "fast": True})
        if r:
            self.app.message("Recording", "Recording 30 s of controller data to:\n" + r.get("path", "?") + "\nDrive now.")


class Params(Page):
    title = "Parameters"
    hint = "←→ group · ↑↓ parameter · Enter edit · S save to flash · R reload" if UNI else "Left/Right group - Up/Down - Enter edit - S save - R reload"

    def __init__(self, app):
        super().__init__(app)
        self.data, self.g, self.sel, self.top, self.loaded = None, 0, 0, 0, 0

    def enter(self):
        self.reload()

    def reload(self):
        self.data = self.app.api.get("/api/params", timeout=5)
        self.loaded = time.monotonic()

    def groups(self):
        d = self.data or {}
        return list((d.get("groups") or {}).keys())

    def rows(self):
        d = self.data or {}
        gs = self.groups()
        if not gs:
            return []
        keys = d["groups"][gs[self.g % len(gs)]]
        byk = {p["key"]: p for p in d.get("params", [])}
        return [byk[k] for k in keys if k in byk]

    def draw(self, win, y, x, h, w, focused):
        if time.monotonic() - self.loaded > 10:
            self.reload()
        gs = self.groups()
        if not gs:
            put(win, y, x, "Parameters unavailable (controller not connected?)", cp(WARN))
            return
        self.g %= len(gs)
        xx = x
        for i, g in enumerate(gs):                       # group tabs
            label = " " + g.title() + " "
            if xx + len(label) >= x + w:
                put(win, y, xx, "…" if UNI else "+", cp(DIM))
                break
            put(win, y, xx, label, cp(SEL if focused else SELDIM, curses.A_BOLD) if i == self.g else cp(DIM))
            xx += len(label) + 1
        rows = self.rows()
        self.sel = max(0, min(self.sel, len(rows) - 1))
        vis = h - 3
        if self.sel < self.top:
            self.top = self.sel
        if self.sel >= self.top + vis:
            self.top = self.sel - vis + 1
        put(win, y + 2, x, "{:<10} {:>11}  {:<19} {}".format("KEY", "VALUE", "RANGE", "WHAT IT DOES"), cp(DIM))
        for i, p in enumerate(rows[self.top:self.top + vis]):
            n = i + self.top
            on = n == self.sel
            fixed = p["lo"] == p["hi"]
            val = "{:.6g}".format(p["value"])
            rng = "fixed" if fixed else "{:.4g} … {:.4g}".format(p["lo"], p["hi"]) if UNI else "{:.4g} .. {:.4g}".format(p["lo"], p["hi"])
            line = "{:<10} {:>11}  {:<19} {}".format(p["key"], val + ("*" if p.get("dirty") else ""), rng, p.get("desc", ""))
            attr = cp(SEL if focused else SELDIM, curses.A_BOLD) if on else (cp(DIM) if fixed else 0)
            put(win, y + 3 + i, x, line.ljust(w)[:w] if on else line, attr, w)
        dirty = [p["key"] for p in (self.data or {}).get("params", []) if p.get("dirty")]
        if dirty:
            put(win, y + h - 1, x, "Unsaved: " + ", ".join(dirty[:8]) + ("…" if len(dirty) > 8 else "") + "   press S to save", cp(WARN))

    def key(self, k):
        rows = self.rows()
        if k in (curses.KEY_LEFT,):
            self.g -= 1
            self.sel = self.top = 0
        elif k in (curses.KEY_RIGHT, 9):
            self.g += 1
            self.sel = self.top = 0
        elif k == curses.KEY_UP:
            self.sel = max(0, self.sel - 1)
        elif k == curses.KEY_DOWN:
            self.sel = min(len(rows) - 1, self.sel + 1)
        elif k == curses.KEY_PPAGE:
            self.sel = max(0, self.sel - 10)
        elif k == curses.KEY_NPAGE:
            self.sel = min(len(rows) - 1, self.sel + 10)
        elif k in (10, 13, curses.KEY_ENTER) and rows:
            self.edit(rows[self.sel])
        elif k in (ord("s"), ord("S")):
            if not self.app.moving("save settings") and self.app.confirm("Save to flash", "Store all current settings permanently?"):
                if self.app.call("/api/params/save", {}, "Settings saved to flash", timeout=25):
                    self.reload()
        elif k in (ord("r"), ord("R")):
            self.reload()
            self.app.toast("Parameters reloaded")
        else:
            return False
        return True

    def edit(self, p):
        if p["lo"] == p["hi"]:
            self.app.message(p["key"], "{} is fixed in firmware and cannot be changed.".format(p["key"]))
            return
        text = self.app.prompt("Set " + p["key"], "{}\nRange {:.6g} to {:.6g}".format(p.get("desc", ""), p["lo"], p["hi"]), "{:.6g}".format(p["value"]))
        if text is None:
            return
        try:
            v = float(text)
        except ValueError:
            self.app.message("Not a number", repr(text) + " is not a number.", "err")
            return
        if not p["lo"] <= v <= p["hi"]:
            self.app.message("Out of range", "{} must be between {:.6g} and {:.6g}.".format(p["key"], p["lo"], p["hi"]), "err")
            return
        if self.app.call("/api/params/set", {"key": p["key"], "value": v}, "{} = {:.6g} (not saved yet, press S)".format(p["key"], v), timeout=20):
            self.reload()


class Logs(Page):
    title = "Logs"
    SOURCES = [("Controller", None), ("Console", "2pybot-console.service"), ("Deployment", "2pybot-deploy.service"),
               ("Firmware worker", "2pybot-firmware.service"), ("Camera", "2pybot-mediamtx.service")]
    hint = "Tab source · ↑↓ PgUp PgDn scroll · End follow · / filter · N hide noise" if UNI else "Tab source - scroll - End follow - / filter - N noise"

    def __init__(self, app):
        super().__init__(app)
        self.src, self.lines, self.at, self.follow, self.scroll, self.filter, self.noise = 0, [], 0, True, 0, "", False

    def fetch(self):
        name, unit = self.SOURCES[self.src]
        if unit is None:
            self.lines = V.log_tail(1500)
        else:
            out = V.sh(["journalctl", "--no-pager", "-o", "short", "-n", "600", "-u", unit], timeout=6)
            self.lines = [l for l in out.splitlines() if not l.startswith("Hint:") and "systemd-journal" not in l]
        self.at = time.monotonic()

    def visible(self):
        rows = self.lines
        if not self.noise:
            rows = [l for l in rows if "[PAD] btn=" not in l]
        if self.filter:
            f = self.filter.lower()
            rows = [l for l in rows if f in l.lower()]
        return rows

    def draw(self, win, y, x, h, w, focused):
        if time.monotonic() - self.at > (1 if self.src == 0 else 3):
            self.fetch()
        xx = x
        for i, (name, _u) in enumerate(self.SOURCES):
            label = " " + name + " "
            put(win, y, xx, label, cp(SEL if focused else SELDIM, curses.A_BOLD) if i == self.src else cp(DIM))
            xx += len(label) + 1
        status = ("following" if self.follow else "paused, End to follow") + (("  filter: " + self.filter) if self.filter else "")
        put(win, y, x + w - len(status) - 1, status, cp(INFO if self.follow else WARN))
        rows = self.visible()
        vis = h - 2
        if self.follow:
            self.scroll = max(0, len(rows) - vis)
        self.scroll = max(0, min(self.scroll, max(0, len(rows) - vis)))
        for i, line in enumerate(rows[self.scroll:self.scroll + vis]):
            put(win, y + 2 + i, x, line, cp(TONE.get(V.log_tone(line), 0)), w)
        if not rows:
            put(win, y + 2, x, "(no lines)", cp(DIM))

    def key(self, k):
        rows = self.visible()
        if k == 9:
            self.src = (self.src + 1) % len(self.SOURCES)
            self.fetch()
            self.follow = True
        elif k in (curses.KEY_UP, curses.KEY_PPAGE, curses.KEY_HOME):
            self.follow = False
            self.scroll -= {curses.KEY_UP: 1, curses.KEY_PPAGE: 15, curses.KEY_HOME: 10 ** 6}[k]
        elif k in (curses.KEY_DOWN, curses.KEY_NPAGE):
            self.scroll += 1 if k == curses.KEY_DOWN else 15
            if self.scroll >= len(rows) - 5:
                self.follow = True
        elif k == curses.KEY_END:
            self.follow = True
        elif k == ord("/"):
            t = self.app.prompt("Filter", "Show only lines containing (empty = all):", self.filter)
            if t is not None:
                self.filter = t.strip()
        elif k in (ord("n"), ord("N")):
            self.noise = not self.noise
            self.app.toast("Gamepad button lines " + ("shown" if self.noise else "hidden"))
        else:
            return False
        return True


class Services(Page):
    title = "Services"
    hint = "↑↓ choose · Enter actions" if UNI else "Up/Down choose - Enter actions"

    def __init__(self, app):
        super().__init__(app)
        self.sel = 0

    def draw(self, win, y, x, h, w, focused):
        svc = (self.app.snap or {}).get("services") or V.service_states()
        put(win, y, x, "{:<10} {:<10} {:<10} {}".format("SERVICE", "STATE", "AT BOOT", "PURPOSE"), cp(DIM))
        self.sel = max(0, min(self.sel, len(svc) - 1))
        for i, s in enumerate(svc):
            on = i == self.sel
            tone = OK if s["active"] == "active" else (DIM if s["name"] in ("deploy", "boot", "imx708") else ERR)
            attr = cp(SEL if focused else SELDIM, curses.A_BOLD) if on else 0
            put(win, y + 1 + i, x, "{:<10} ".format(s["name"]), attr)
            put(win, y + 1 + i, x + 11, "{:<10} ".format(s["active"]), attr if on else cp(tone))
            put(win, y + 1 + i, x + 22, "{:<10} {}".format(s["enabled"], s["desc"]), attr if on else cp(DIM), w - 22)
        put(win, y + len(svc) + 2, x, "deploy, boot and imx708 run only when needed; 'inactive' is normal for them.", cp(DIM), w)

    def key(self, k):
        svc = (self.app.snap or {}).get("services") or V.service_states()
        if k == curses.KEY_UP:
            self.sel = max(0, self.sel - 1)
        elif k == curses.KEY_DOWN:
            self.sel = min(len(svc) - 1, self.sel + 1)
        elif k in (10, 13, curses.KEY_ENTER):
            s = svc[self.sel]
            choice = self.app.choose(s["unit"], ["Restart", "Show status", "Follow live log (Ctrl+C to return)", "Stop", "Start"])
            if choice is None:
                return True
            unit = s["unit"]
            if choice in (0, 3, 4):
                if self.app.moving("change services"):
                    return True
                verb = ["restart", None, None, "stop", "start"][choice]
                if self.app.confirm(verb.title() + " " + s["name"], "Run: sudo systemctl {} {}".format(verb, unit)):
                    self.app.external(["sudo", "systemctl", verb, unit], "systemctl {} {}".format(verb, unit))
                    self.app.refresh_now()
            elif choice == 1:
                self.app.external(["systemctl", "--no-pager", "status", unit], "Status of " + unit)
            else:
                self.app.external(["journalctl", "-f", "-n", "50", "-u", unit], "Live log of " + unit + " (Ctrl+C to return)")
        else:
            return False
        return True


class Updates(ActionPage):
    title = "Updates"

    def info(self):
        s = self.app.snap or {}
        fw, dep, cfg = s.get("fw") or {}, s.get("deploy") or {}, s.get("config") or {}
        inst, latest, prog = fw.get("installed") or {}, fw.get("latest") or {}, dep.get("progress") or {}
        rows = [("Branch", cfg.get("branch", "-"), None),
                ("Running commit", V.short((dep.get("applied") or {}).get("commit")), None),
                ("Firmware installed", "{}  ({})".format(inst.get("version", "-"), V.short(inst.get("source_commit"))), "ok" if inst else "warn"),
                ("Firmware latest build", "{}  ({})".format(latest.get("version", "-"), V.short(latest.get("source_commit"))),
                 "ok" if latest.get("build_id") == inst.get("build_id") else "warn"),
                ("Last deployment", "{} · {}".format(prog.get("phase", "-"), prog.get("message", "")),
                 {"success": "ok", "failed": "err"}.get(prog.get("phase"), "warn"))]
        if prog.get("finished_at"):
            rows.append(("Finished", time.strftime("%Y-%m-%d %H:%M", time.localtime(prog["finished_at"])), "dim"))
        return rows

    def actions(self):
        branch = ((self.app.snap or {}).get("config") or {}).get("branch", "main")
        return [("Update to latest " + branch, "pull, install host software, flash firmware", lambda: self.deploy(branch)),
                ("Deploy another branch…" if UNI else "Deploy another branch...", "type a branch name", self.other),
                ("Watch deployment progress", "live progress of a running update", lambda: self.app.external(BOT + ["deploy-status", "--watch"], "Deployment progress")),
                ("Firmware details", "installed and latest controller images", lambda: self.app.external(BOT + ["firmware-status"], "Firmware"))]

    def deploy(self, branch, arg=None):
        if self.app.moving("update"):
            return
        if not self.app.confirm("Update", "Deploy branch '{}'?\nThe robot must be stopped (SELECT) and stay still.\n"
                                "It is reflashed if the firmware changed.".format(branch)):
            return
        cmd = "cd {repo} && git fetch origin {b} && git show FETCH_HEAD:scripts/radxa/deploy_branch.py | sudo python3 - {a}".format(
            repo=shlex.quote(str(REPO)), b=shlex.quote(branch), a=shlex.quote(arg or branch))
        self.app.external(["bash", "-c", cmd], "Deploying " + branch)
        self.app.refresh_now()

    def other(self):
        b = self.app.prompt("Deploy branch", "Branch name (for example main):", "")
        if b and b.strip():
            self.deploy(b.strip())


class Network(ActionPage):
    title = "Network"

    def info(self):
        net = (self.app.snap or {}).get("net") or {}
        rows = []
        for dev, typ, state, conn in net.get("devices", []):
            role = "hotspot" if dev == "wlan1" else ("Wi-Fi" if typ == "wifi" else typ)
            rows.append(("{} ({})".format(dev, role), "{} · {} · {}".format(state, conn or "-", net.get("ips", {}).get(dev, "no IP")),
                         "ok" if state == "connected" else "dim"))
        rows.append(("Hotspot clients", str(net.get("hotspot_clients", 0)), None))
        ip = net.get("ips", {}).get("wlan0")
        rows.append(("Open in a browser", "http://{}:8080  or  http://2pybot.local:8080".format(ip or "10.42.0.1"), "info"))
        return rows

    def actions(self):
        return [("Wi-Fi settings (nmtui)", "join or edit networks", lambda: self.app.external(["nmtui"], "nmtui", pause=False)),
                ("List Wi-Fi networks nearby", "", lambda: self.app.external(["nmcli", "device", "wifi", "list"], "Wi-Fi networks")),
                ("Show all addresses", "", lambda: self.app.external(["ip", "-br", "addr"], "Addresses"))]


class System(ActionPage):
    title = "System"

    def info(self):
        s = self.app.snap or {}
        si = s.get("sys") or {}
        rows = [("Uptime", si.get("uptime", "-"), None), ("Load", si.get("load", "-"), None)]
        if "temp" in si:
            rows.append(("CPU temperature", "{:.0f} °C".format(si["temp"]), "warn" if si["temp"] > 75 else "ok"))
        if "mem_total" in si:
            rows.append(("Memory", "{:.1f} of {:.1f} GB used".format(si["mem_used"], si["mem_total"]), None))
        if "disk_total" in si:
            rows.append(("Disk", "{:.0f} of {:.0f} GB used".format(si["disk_used"], si["disk_total"]),
                         "warn" if si["disk_used"] / si["disk_total"] > 0.85 else "ok"))
        ev = s.get("events")
        if ev:
            counts, last = ev
            for k, v in counts.items():
                rows.append(("24 h: " + k, str(v) + (("   last " + last[k]) if k in last else ""), "warn" if v else "dim"))
        return rows

    def actions(self):
        a = self.app
        return [("Save diagnostics report", "everything needed to debug, as one file", self.report),
                ("Open a shell", "type exit to return", lambda: a.external([os.environ.get("SHELL", "/bin/bash")], "Shell (type exit to return)", pause=False)),
                ("Reboot robot computer", "", lambda: self.power("reboot")),
                ("Power off robot computer", "", lambda: self.power("poweroff"))]

    def report(self):
        path = os.path.expanduser(time.strftime("~/2pybot-report-%Y%m%d-%H%M%S.json"))
        self.app.external(BOT + ["doctor", "--output", path], "Diagnostics report")

    def power(self, what):
        if self.app.moving(what):
            return
        if self.app.confirm(what.title(), "{} the Radxa now? The robot must be stopped.".format(what.title()), danger=True):
            self.app.external(["sudo", "systemctl", what], what, pause=False)


class Help(Page):
    title = "Help"
    TEXT = [
        ("KEYS", None),
        ("1 to 9", "jump to a page"), ("Arrows / Enter", "move and select"), ("Esc or Left", "back to the menu"),
        ("X", "STOP the motors (works on every page)"), ("Q", "quit"),
        ("", ""), ("COMMANDS WITHOUT THIS SCREEN", None),
        ("2pybot status", "dashboard once"), ("2pybot stop", "stop the motors"),
        ("2pybot logs", "controller log (add -f to follow)"), ("2pybot logs console", "a service's log"),
        ("2pybot params list [GROUP]", "settings table"), ("2pybot params set KEY VALUE --save", "change and save a setting"),
        ("2pybot services", "service states"), ("2pybot gamepad repair", "forget and re-pair the gamepad"), ("2pybot firmware-status", "controller firmware"),
        ("2pybot deploy-status --watch", "follow an update"), ("2pybot doctor", "diagnostics summary and report file"),
        ("add --json", "raw data instead of tables"),
    ]

    def draw(self, win, y, x, h, w, focused):
        for i, (a, b) in enumerate(self.TEXT[:h]):
            if b is None:
                put(win, y + i, x, a, cp(ACC, curses.A_BOLD))
            else:
                put(win, y + i, x, a, cp(INFO))
                put(win, y + i, x + 36, b, 0, w - 36)


# ====================================================================== app
class App:
    def __init__(self, scr, api):
        self.scr, self.api = scr, api
        self.collector = V.Collector(api)
        self.snap, self.lock = None, threading.Lock()
        self.pages = [Dashboard(self), Robot(self), Params(self), Logs(self), Services(self), Updates(self), Network(self),
                      System(self), Help(self)]
        self.cur, self.focus = 0, "menu"
        self.toast_text, self.toast_until, self.toast_tone = "", 0, "info"
        self.alive = True
        self.wake = threading.Event()
        threading.Thread(target=self.poll, daemon=True).start()

    # ---- data
    def poll(self):
        while self.alive:
            try:
                s = self.collector.snapshot()
                s["events"] = self.collector._every("events", 30, V.event_summary)
                with self.lock:
                    self.snap = s
            except Exception:  # noqa: BLE001
                pass
            self.wake.wait(1.0)
            self.wake.clear()

    def refresh_now(self):
        self.collector.stamp.clear()
        self.wake.set()

    # ---- helpers used by pages
    def toast(self, text, tone="info"):
        self.toast_text, self.toast_until, self.toast_tone = text, time.monotonic() + 5, tone

    def call(self, path, body=None, success=None, timeout=8):
        try:
            r = self.api.call(path, body, timeout=timeout)
            if success:
                self.toast(success, "ok")
            self.refresh_now()
            return r if r is not None else {}
        except RuntimeError as e:
            self.message("Not done", str(e), "err")
            return None

    def stop(self):
        try:
            self.api.call("/api/estop", {}, timeout=3)
            self.toast("STOP sent: motors at zero torque", "err")
        except RuntimeError as e:
            self.message("STOP failed", str(e) + "\nUse SELECT on the gamepad.", "err")

    def moving(self, what):
        tel = ((self.snap or {}).get("status") or {}).get("telemetry", {})
        if tel.get("balancing") or tel.get("self_righting"):
            self.message("Robot is armed", "Press SELECT on the gamepad (or X here) before you {}.".format(what), "warn")
            return True
        return False

    def external(self, cmd, title, pause=True):
        curses.def_prog_mode()
        curses.endwin()
        print("\033[38;5;208m\033[1m── 2PyBot: {} ──\033[0m\n".format(title) if UNI else "== 2PyBot: {} ==\n".format(title), flush=True)
        try:
            rc = subprocess.call(cmd)
        except KeyboardInterrupt:
            rc = 130
        except OSError as e:
            print("Could not run: " + str(e))
            rc = 127
        if pause:
            print("\n\033[90m(exit code {}) Press Enter to return to 2PyBot…\033[0m".format(rc) if UNI else "\n(exit {}) Press Enter...".format(rc), end="", flush=True)
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                pass
        curses.reset_prog_mode()
        self.scr.clear()
        self.scr.refresh()

    def _box(self, title, lines, width, tone="acc"):
        H, W = self.scr.getmaxyx()
        w = min(W - 4, max(width, len(title) + 6, *(len(l) + 4 for l in lines)))
        h = len(lines) + 4
        y, x = max(0, (H - h) // 2), max(0, (W - w) // 2)
        win = curses.newwin(h, w, y, x)
        win.bkgd(" ", 0)
        win.erase()
        frame(win, 0, 0, h, w, title, cp(TONE.get(tone, ACC), curses.A_BOLD), True)
        for i, l in enumerate(lines):
            put(win, 2 + i, 2, l, 0, w - 4)
        return win, h, w

    def message(self, title, text, tone="acc"):
        lines = []
        for part in str(text).splitlines() or [""]:
            while len(part) > 70:
                cut = part.rfind(" ", 0, 70)
                cut = cut if cut > 20 else 70
                lines.append(part[:cut])
                part = part[cut:].lstrip()
            lines.append(part)
        win, h, w = self._box(title, lines + [""], 40, tone)
        put(win, h - 2, w - 10, "[ OK ]", cp(SEL, curses.A_BOLD))
        win.refresh()
        self.scr.timeout(-1)
        win.keypad(True)
        read_key(win)
        self.scr.timeout(250)

    def confirm(self, title, text, danger=False):
        lines = str(text).splitlines() + [""]
        yes = False
        while True:
            win, h, w = self._box(title, lines, 46, "err" if danger else "acc")
            put(win, h - 2, w - 22, "[ Yes ]", cp(SEL, curses.A_BOLD) if yes else cp(DIM))
            put(win, h - 2, w - 12, "[ No ]", cp(SEL, curses.A_BOLD) if not yes else cp(DIM))
            win.refresh()
            self.scr.timeout(-1)
            win.keypad(True)
            k = read_key(win)
            self.scr.timeout(250)
            if k in (curses.KEY_LEFT, curses.KEY_RIGHT, 9):
                yes = not yes
            elif k in (ord("y"), ord("Y")):
                return True
            elif k in (ord("n"), ord("N"), 27, ord("q")):
                return False
            elif k in (10, 13, curses.KEY_ENTER):
                return yes

    def choose(self, title, options):
        sel = 0
        while True:
            win, h, w = self._box(title, [""] * len(options), 44)
            for i, o in enumerate(options):
                put(win, 2 + i, 2, (" " + ARROW + " " if i == sel else "   ") + o.ljust(w - 8), cp(SEL, curses.A_BOLD) if i == sel else 0)
            win.refresh()
            self.scr.timeout(-1)
            win.keypad(True)
            k = read_key(win)
            self.scr.timeout(250)
            if k == curses.KEY_UP:
                sel = max(0, sel - 1)
            elif k == curses.KEY_DOWN:
                sel = min(len(options) - 1, sel + 1)
            elif k in (10, 13, curses.KEY_ENTER):
                return sel
            elif k in (27, ord("q")):
                return None

    def prompt(self, title, text, default=""):
        buf = list(str(default))
        curses.curs_set(1)
        try:
            while True:
                lines = str(text).splitlines() + ["", ""]
                win, h, w = self._box(title, lines, 52)
                field = "".join(buf)
                put(win, h - 3, 2, "> " + field.ljust(w - 8), cp(SELDIM))
                put(win, h - 2, 2, "Enter accept · Esc cancel" if UNI else "Enter accept - Esc cancel", cp(DIM))
                win.move(h - 3, min(w - 3, 4 + len(field)))
                win.keypad(True)
                win.refresh()
                self.scr.timeout(-1)
                k = win.get_wch()
                self.scr.timeout(250)
                if k in ("\n", "\r", curses.KEY_ENTER):
                    return "".join(buf)
                if k in ("\x1b",):
                    return None
                if k in (curses.KEY_BACKSPACE, "\x7f", "\b"):
                    buf = buf[:-1]
                elif isinstance(k, str) and k.isprintable():
                    buf.append(k)
        finally:
            curses.curs_set(0)

    # ---- drawing
    def draw(self):
        scr = self.scr
        H, W = scr.getmaxyx()
        scr.erase()
        if H < 16 or W < 60:
            put(scr, 0, 0, "Make the terminal bigger (at least 60×16)." if UNI else "Terminal too small (60x16).", cp(WARN))
            scr.refresh()
            return
        s = self.snap or {}
        state, tone = V.robot_state(s) if s else ("…", "dim")
        # header
        put(scr, 0, 0, " " * W, cp(HEAD))
        put(scr, 0, 1, (" ◉ 2PyBot " if UNI else " 2PyBot ") + " control centre", cp(HEAD, curses.A_BOLD))
        pill = " " + state + " "
        put(scr, 0, W - len(pill) - 11, pill, cp({"ok": OK, "warn": WARN, "err": ERR}.get(tone, INFO), curses.A_REVERSE | curses.A_BOLD))
        put(scr, 0, W - 9, s.get("time", ""), cp(HEAD))
        fw = ((s.get("fw") or {}).get("installed") or {}).get("version", "-")
        branch = (s.get("config") or {}).get("branch", "-")
        ip = ((s.get("net") or {}).get("ips") or {}).get("wlan0", "no Wi-Fi")
        sub = " firmware {}  ·  {}  ·  {}  ·  http://{}:8080".format(fw, branch, ip, ip) if UNI else " firmware {} | {} | {}".format(fw, branch, ip)
        put(scr, 1, 0, sub, cp(DIM))
        # menu
        mw = 22
        top, bottom = 2, H - 1
        frame(scr, top, 0, bottom - top, mw, "Menu", focused=self.focus == "menu")
        for i, p in enumerate(self.pages):
            on = i == self.cur
            attr = (cp(SEL, curses.A_BOLD) if self.focus == "menu" else cp(SELDIM, curses.A_BOLD)) if on else 0
            put(scr, top + 2 + i, 1, " {}  {}".format(i + 1, p.title).ljust(mw - 2), attr)
        put(scr, top + 3 + len(self.pages), 2, "Q  Quit", cp(DIM))
        # content
        page = self.pages[self.cur]
        frame(scr, top, mw, bottom - top, W - mw, page.title, focused=self.focus == "page")
        try:
            page.draw(scr, top + 1, mw + 2, bottom - top - 2, W - mw - 4, self.focus == "page")
        except Exception as e:  # noqa: BLE001 - a page bug must not kill the app
            put(scr, top + 1, mw + 2, "Page error: " + repr(e)[:W - mw - 20], cp(ERR))
        # footer
        stop = " X STOP "
        if time.monotonic() < self.toast_until:
            put(scr, H - 1, 1, self.toast_text, cp(TONE.get(self.toast_tone, INFO), curses.A_BOLD), W - len(stop) - 3)
        else:
            hint = page.hint if self.focus == "page" else ("↑↓ choose · Enter open · 1-9 jump · Q quit" if UNI else "Up/Down - Enter - 1-9 - Q quit")
            put(scr, H - 1, 1, hint, cp(DIM), W - len(stop) - 3)
        put(scr, H - 1, W - len(stop) - 1, stop, cp(STOP, curses.A_BOLD))
        scr.refresh()

    def run(self):
        curses.curs_set(0)
        self.scr.keypad(True)
        self.scr.timeout(250)
        last = 0
        while True:
            now = time.monotonic()
            if now - last >= 0.5:
                self.draw()
                last = now
            self.scr.timeout(250)
            k = read_key(self.scr)
            if k == -1:
                continue
            if k == curses.KEY_RESIZE:
                self.scr.clear()
            elif k in (ord("x"), ord("X")):
                self.stop()
            elif self.focus == "page" and self.pages[self.cur].key(k):
                pass
            elif ord("1") <= k <= ord("9") and k - ord("1") < len(self.pages):
                self.cur, self.focus = k - ord("1"), "page"
                self.pages[self.cur].enter()
            elif self.focus == "menu":
                if k in (curses.KEY_UP, ord("k")):
                    self.cur = (self.cur - 1) % len(self.pages)
                elif k in (curses.KEY_DOWN, ord("j")):
                    self.cur = (self.cur + 1) % len(self.pages)
                elif k in (10, 13, curses.KEY_ENTER, curses.KEY_RIGHT):
                    self.focus = "page"
                    self.pages[self.cur].enter()
                elif k in (ord("q"), ord("Q")):
                    break
            elif k in (27, curses.KEY_LEFT, curses.KEY_BACKSPACE, 127, ord("q"), ord("Q")):
                self.focus = "menu"
            self.draw()
            last = time.monotonic()
        self.alive = False


def main(api_base="http://127.0.0.1:8080"):
    if not sys.stdout.isatty():
        print("2pybot needs an interactive terminal. Use `2pybot status` for a one-shot report.", file=sys.stderr)
        return 2
    os.environ.setdefault("ESCDELAY", "25")
    api = V.Api(api_base)
    try:
        curses.wrapper(lambda scr: (setup_colours(), App(scr, api).run()))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
