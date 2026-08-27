#!/usr/bin/env python3
"""
radxa_brain.py v2 -- guard + person follower + WEB UI (replaces the old
2pybot camera/control server: this must be the ONLY process using the
camera and the ONLY process on the serial port).

WEB UI:  http://<cubie-ip>:8080
  - live ANNOTATED camera view (detections, distances, mode, velocity)
  - FOLLOW / GUARD buttons -> same as the ArUco cards / keyboard
DEBUG WINDOW (only if a monitor is attached to the Cubie):
  python3 radxa_brain.py /dev/ttyUSB0 --show
Over SSH, use the web UI -- cv2.imshow cannot draw without a display.

v2 fixes vs v1:
  - camera opened with the V4L2 backend explicitly (the GStreamer
    warnings you saw), scanning /dev/video0..3, MJPG fourcc, and a HARD
    error with diagnosis commands instead of silently reading nothing
  - serial opened with exclusive=True so a still-running old server
    fails loudly instead of interleaving V-lines with us

MODES:  GUARD (default, silent collision stop) / FOLLOW (legs mode)
SWITCH: web buttons | terminal f/g/q | ArUco card 7 = follow, 8 = guard
CARDS:  python3 radxa_brain.py --make-cards
"""
import sys, time, threading, select
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import serial

# ---------------- guard tuning ----------------
STOP_CM        = 70.0
MIN_HOLD_S     = 1.0
CLEAR_S        = 1.0
MOVE_GATE_MS   = 0.05
CORRIDOR       = (0.25, 0.75)
BIG_UNKNOWN_FRAC = 0.45
FOCAL_PX       = 600.0
KNOWN_WIDTHS   = {"person": 45.0, "chair": 45.0, "bottle": 7.0, "cup": 8.5,
                  "laptop": 33.0, "cell phone": 7.5, "backpack": 30.0,
                  "couch": 150.0, "tv": 90.0, "dog": 25.0, "cat": 15.0}
# ---------------- follow tuning ----------------
TARGET_W_FRAC  = 0.34
W_DEADBAND     = 0.05
FWD_KP, FWD_MAX     = 2.2, 0.40
STEER_KP, STEER_MAX = 1.4, 0.55
FOLLOW_STEER_SIGN   = +1.0
CMD_SMOOTH     = 0.35
LOST_SILENT_S  = 0.7
PERSON_STOP_FRAC = 0.55
ARUCO_FOLLOW_ID, ARUCO_GUARD_ID = 7, 8
# ---------------- misc ----------------
HTTP_PORT      = 8080
COUNTS_PER_M   = 4096.0 / (2 * 3.14159265 * 0.0350)
ENC_GUARD_SIGN = +1.0

# ---------------- shared state ----------------
mode = "GUARD"
tx_active, tx_fwd, tx_steer, tx_fresh_t = False, 0.0, 0.0, 0.0
fwd_vel, last_odom_t = 0.0, 0.0
guard_on, trig_t, clear_t = False, 0.0, None
latest_jpeg, stream_clients = None, 0
status_line = "starting"
_lock = threading.Lock()

def set_mode(m):
    global mode, guard_on
    m = m.upper()
    if m not in ("GUARD", "FOLLOW") or m == mode:
        return
    mode = m
    if m == "GUARD":
        guard_on = False
        go_silent()
    print(f">> {mode}")

# ---------------- serial ----------------
def open_port(port):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0
    s.dtr = False; s.rts = False          # never reset the ESP32
    s.exclusive = True                    # fail loudly if old server owns it
    try:
        s.open()
    except serial.SerialException as e:
        sys.exit(f"[SER] Cannot open {port} exclusively: {e}\n"
                 f"      Something else is using it (old 2pybot server?).\n"
                 f"      Find it:  sudo lsof {port}   then stop/disable it.")
    return s

def io_thread(ser):
    global fwd_vel, last_odom_t
    buf, pos_prev, t_prev = b"", None, None
    next_tx = time.monotonic()
    while True:
        now = time.monotonic()
        if now >= next_tx:
            next_tx = now + 0.02
            with _lock:
                active, f, s2 = tx_active, tx_fwd, tx_steer
                stale = (now - tx_fresh_t) > 0.5
            if active:
                ser.write(b"V,0,0,1\n" if stale else f"V,{f:.2f},{s2:.2f},1\n".encode())
        data = ser.read(256)
        if data:
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.startswith(b"O,"):
                    try:
                        _, ms, eL, eR, _, _ = line.decode().split(",")
                        pos = ENC_GUARD_SIGN * (int(eL) + int(eR)) / 2.0 / COUNTS_PER_M
                        t = int(ms) / 1000.0
                        if pos_prev is not None and t > t_prev:
                            fwd_vel += 0.3 * ((pos - pos_prev) / (t - t_prev) - fwd_vel)
                        pos_prev, t_prev = pos, t
                        last_odom_t = time.monotonic()
                    except (ValueError, IndexError):
                        pass
        time.sleep(0.002)

def set_cmd(f, s2):
    global tx_active, tx_fwd, tx_steer, tx_fresh_t
    with _lock:
        tx_active, tx_fwd, tx_steer, tx_fresh_t = True, f, s2, time.monotonic()

def go_silent():
    global tx_active
    with _lock:
        tx_active = False

# ---------------- web UI ----------------
PAGE = b"""<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>
<title>2PyBot</title></head>
<body style='background:#111;color:#eee;font-family:sans-serif;text-align:center'>
<h2>2PyBot &mdash; <span id=m>?</span></h2>
<img src='/stream' style='width:96%;max-width:640px;border-radius:12px'>
<p>
<button style='font-size:1.3em;padding:12px 24px' onclick=go('follow')>FOLLOW</button>
<button style='font-size:1.3em;padding:12px 24px' onclick=go('guard')>GUARD / STOP</button>
</p>
<script>
function go(m){fetch('/mode?m='+m).then(r=>r.text()).then(t=>m_.innerText=t)}
const m_=document.getElementById('m');
setInterval(()=>fetch('/mode').then(r=>r.text()).then(t=>m_.innerText=t),1000);
</script></body></html>"""

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        global stream_clients
        if self.path.startswith("/mode"):
            if "m=" in self.path:
                set_mode(self.path.split("m=")[1].split("&")[0])
            self.send_response(200); self.send_header("Content-Type", "text/plain")
            self.end_headers(); self.wfile.write(f"{mode} | {status_line}".encode())
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=fr")
            self.end_headers()
            stream_clients += 1
            try:
                while True:
                    j = latest_jpeg
                    if j:
                        self.wfile.write(b"--fr\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n")
                    time.sleep(0.08)          # ~12 fps to the browser
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                stream_clients -= 1
        else:
            self.send_response(200); self.send_header("Content-Type", "text/html")
            self.end_headers(); self.wfile.write(PAGE)

# ---------------- vision ----------------
def open_camera(cv2):
    for idx in range(4):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)   # V4L2, NOT GStreamer
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, _ = cap.read()
            if ok:
                print(f"[CAM] using /dev/video{idx}")
                return cap
        cap.release()
    sys.exit("[CAM] No camera delivered a frame. Diagnose on the Cubie:\n"
             "  v4l2-ctl --list-devices          (which nodes exist)\n"
             "  sudo fuser -v /dev/video0        (who is holding it -- the\n"
             "  old 2pybot server must be stopped: sudo systemctl disable --now <it>)")

def load_all():
    import cv2, numpy as np
    net = cv2.dnn.readNetFromDarknet("yolov4-tiny.cfg", "yolov4-tiny.weights")
    names = open("coco.names").read().strip().split("\n")
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters()).detectMarkers
    except AttributeError:
        p = cv2.aruco.DetectorParameters_create()
        det = lambda img: cv2.aruco.detectMarkers(img, d, parameters=p)
    return cv2, np, net, net.getUnconnectedOutLayersNames(), names, det

def detect(cv2, np, net, layers, names, frame):
    """persons: [(cx_frac,w_frac,box)], obstacles: [(dist_cm|None,label,box)],
    min obstacle distance in the corridor."""
    H, W = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (320, 320), swapRB=True, crop=False)
    net.setInput(blob)
    persons, obstacles, best = [], [], None
    for out in net.forward(layers):
        for det in out:
            scores = det[5:]
            cid = int(np.argmax(scores))
            if scores[cid] < 0.5:
                continue
            cx, cy, w, h = float(det[0]), float(det[1]), float(det[2]), float(det[3])
            box = (int((cx - w/2)*W), int((cy - h/2)*H), int(w*W), int(h*H))
            label = names[cid]
            if label == "person":
                persons.append((cx, w, box)); continue
            d = KNOWN_WIDTHS[label] * FOCAL_PX / (w*W) if label in KNOWN_WIDTHS and w > 0 else None
            obstacles.append((d, label, box))
            if CORRIDOR[0] < cx < CORRIDOR[1]:
                if d is not None:
                    best = d if best is None else min(best, d)
                elif w > BIG_UNKNOWN_FRAC:
                    best = STOP_CM - 1 if best is None else min(best, STOP_CM - 1)
    return persons, obstacles, best

def annotate(cv2, frame, persons, obstacles, obst_cm):
    H, W = frame.shape[:2]
    cv2.line(frame, (int(CORRIDOR[0]*W), 0), (int(CORRIDOR[0]*W), H), (60, 60, 60), 1)
    cv2.line(frame, (int(CORRIDOR[1]*W), 0), (int(CORRIDOR[1]*W), H), (60, 60, 60), 1)
    for _, _, (x, y, w, h) in persons:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(frame, "person", (x, y-6), 0, 0.55, (0, 255, 0), 2)
    for d, label, (x, y, w, h) in obstacles:
        hot = d is not None and d < STOP_CM
        col = (0, 0, 255) if hot else (0, 165, 255)
        cv2.rectangle(frame, (x, y), (x+w, y+h), col, 2)
        cv2.putText(frame, f"{label} {d:.0f}cm" if d else label, (x, y-6), 0, 0.55, col, 2)
    bar = f"{mode} v={fwd_vel:+.2f} cmd=({tx_fwd:+.2f},{tx_steer:+.2f})" \
          f"{' TX' if tx_active else ''}{' !GUARD!' if guard_on else ''}"
    cv2.putText(frame, bar, (8, 22), 0, 0.6, (0, 255, 255), 2)
    return frame

def make_cards():
    import cv2
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    gen = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    for mid, name in ((ARUCO_FOLLOW_ID, "FOLLOW-ME"), (ARUCO_GUARD_ID, "STOP-FOLLOWING")):
        cv2.imwrite(f"card_{name}_{mid}.png", gen(d, mid, 600))
        print(f"card_{name}_{mid}.png -- print ~10 cm wide")

# ---------------- main ----------------
def main():
    global guard_on, trig_t, clear_t, latest_jpeg, status_line
    if "--make-cards" in sys.argv:
        make_cards(); return
    show = "--show" in sys.argv
    port = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else "/dev/ttyUSB0"

    ser = open_port(port)
    threading.Thread(target=io_thread, args=(ser,), daemon=True).start()
    cv2, np, net, layers, names, detect_aruco = load_all()
    cap = open_camera(cv2)
    httpd = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"Brain on {port} | web UI: http://<cubie-ip>:{HTTP_PORT} | keys: f g q")

    last_seen_t, sm_f, sm_s = 0.0, 0.0, 0.0
    fps_t, fps_n, fps = time.monotonic(), 0, 0.0

    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            k = sys.stdin.readline().strip().lower()
            if k == "f": set_mode("FOLLOW")
            if k == "g": set_mode("GUARD")
            if k == "q": go_silent(); return

        ok, frame = cap.read()
        if not ok:
            status_line = "camera read failed"; time.sleep(0.3); continue
        now = time.monotonic()
        fps_n += 1
        if now - fps_t > 2:
            fps = fps_n / (now - fps_t); fps_t, fps_n = now, 0

        corners, ids, _ = detect_aruco(frame)
        if ids is not None:
            for mid in ids.flatten():
                if mid == ARUCO_FOLLOW_ID: set_mode("FOLLOW")
                if mid == ARUCO_GUARD_ID:  set_mode("GUARD")

        persons, obstacles, obst_cm = detect(cv2, np, net, layers, names, frame)
        danger = obst_cm is not None and obst_cm < STOP_CM

        if mode == "GUARD":
            moving = fwd_vel > MOVE_GATE_MS and (now - last_odom_t) < 0.5
            if not guard_on and danger and moving:
                guard_on, trig_t, clear_t = True, now, None
                set_cmd(0.0, 0.0)
                print(f">>> GUARD STOP ({obst_cm:.0f} cm)")
            elif guard_on:
                set_cmd(0.0, 0.0)
                clear_t = None if danger else (clear_t or now)
                if now - trig_t > MIN_HOLD_S and clear_t and now - clear_t > CLEAR_S:
                    guard_on = False; go_silent(); print(">>> guard released")
        else:  # FOLLOW
            if persons:
                last_seen_t = now
                cx, w, _ = max(persons, key=lambda p: p[1])
                steer = FOLLOW_STEER_SIGN * STEER_KP * (cx - 0.5) * 2.0
                steer = max(-STEER_MAX, min(STEER_MAX, steer))
                werr = TARGET_W_FRAC - w
                fwd = FWD_KP * werr if abs(werr) > W_DEADBAND else 0.0
                fwd = max(-FWD_MAX, min(FWD_MAX, fwd))
                if (danger and fwd > 0) or w > PERSON_STOP_FRAC:
                    fwd = 0.0
                sm_f += CMD_SMOOTH * (fwd - sm_f)
                sm_s += CMD_SMOOTH * (steer - sm_s)
                set_cmd(sm_f, sm_s)
            elif now - last_seen_t > LOST_SILENT_S:
                sm_f = sm_s = 0.0
                go_silent()
            else:
                set_cmd(0.0, sm_s * 0.5)

        status_line = f"{fps:.1f} fps, nearest " + (f"{obst_cm:.0f} cm" if obst_cm else "clear")

        if stream_clients > 0 or show:
            annotate(cv2, frame, persons, obstacles, obst_cm)
            cv2.putText(frame, f"{fps:.1f} fps", (8, 44), 0, 0.6, (0, 255, 255), 2)
            if stream_clients > 0:
                okj, j = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if okj:
                    latest_jpeg = j.tobytes()
            if show:
                cv2.imshow("brain", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    go_silent(); return

if __name__ == "__main__":
    main()
