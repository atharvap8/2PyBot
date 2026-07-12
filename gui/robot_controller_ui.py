import math
import threading
import time
from collections import deque
import serial
from serial.tools import list_ports
import tkinter as tk
import customtkinter as ctk

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")


class Robot3DViewer:
    """Software-rendered 3D visualisation of the balancing robot.

    Pure tk.Canvas renderer (no extra dependencies). Drag with the left
    mouse button to orbit, scroll to zoom. Robot pitch, heading and wheel
    rotation are driven live from telemetry.
    """

    BODY_COLOR = (0, 173, 181)  # teal body panels
    TOP_COLOR = (238, 238, 238)  # sensor deck
    WHEEL_COLOR = (60, 64, 72)
    HUB_COLOR = (241, 196, 15)

    def __init__(self, canvas):
        self.canvas = canvas
        self.w, self.h = 420, 320
        self.cam_yaw = 0.75
        self.cam_pitch = 0.38
        self.zoom = 1.0
        self._drag = None

        # Robot state (degrees)
        self.pitch = 0.0
        self.heading = 0.0
        self.wheel_l = 0.0
        self.wheel_r = 0.0

        canvas.bind("<Configure>", self._on_resize)
        canvas.bind("<ButtonPress-1>", self._on_press)
        canvas.bind("<B1-Motion>", self._on_drag)
        canvas.bind("<MouseWheel>", self._on_wheel)

    # ---------- interaction ----------
    def _on_resize(self, e):
        self.w, self.h = e.width, e.height

    def _on_press(self, e):
        self._drag = (e.x, e.y)

    def _on_drag(self, e):
        if self._drag:
            dx, dy = e.x - self._drag[0], e.y - self._drag[1]
            self.cam_yaw += dx * 0.01
            self.cam_pitch = max(-1.35, min(1.35, self.cam_pitch + dy * 0.01))
            self._drag = (e.x, e.y)

    def _on_wheel(self, e):
        self.zoom = max(0.4, min(3.0, self.zoom * (1.1 if e.delta > 0 else 0.9)))

    # ---------- math helpers ----------
    @staticmethod
    def _rot_x(p, a):
        s, c = math.sin(a), math.cos(a)
        return (p[0], p[1] * c - p[2] * s, p[1] * s + p[2] * c)

    @staticmethod
    def _rot_z(p, a):
        s, c = math.sin(a), math.cos(a)
        return (p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2])

    def _project(self, p):
        sy, cy = math.sin(self.cam_yaw), math.cos(self.cam_yaw)
        sp, cp = math.sin(self.cam_pitch), math.cos(self.cam_pitch)
        x1 = p[0] * cy - p[1] * sy
        y1 = p[0] * sy + p[1] * cy
        zv = p[2] * cp - y1 * sp
        depth = y1 * cp + p[2] * sp
        f = 6.0 / (6.0 + depth)
        scale = min(self.w, self.h) * 0.30 * self.zoom
        return (
            self.w / 2 + x1 * scale * f,
            self.h * 0.52 - zv * scale * f,
            depth,
        )

    @staticmethod
    def _shade(rgb, normal):
        # Fixed light direction, simple Lambert shading
        lx, ly, lz = -0.45, -0.6, 0.66
        nlen = math.sqrt(sum(n * n for n in normal)) or 1.0
        d = (normal[0] * lx + normal[1] * ly + normal[2] * lz) / nlen
        k = 0.38 + 0.62 * max(0.0, d)
        return "#%02x%02x%02x" % tuple(min(255, int(c * k)) for c in rgb)

    @staticmethod
    def _normal(a, b, c):
        u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        return (
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        )

    # ---------- geometry ----------
    def _box_faces(self, hx, hy, z0, z1, transform, color):
        v = [
            (-hx, -hy, z0), (hx, -hy, z0), (hx, hy, z0), (-hx, hy, z0),
            (-hx, -hy, z1), (hx, -hy, z1), (hx, hy, z1), (-hx, hy, z1),
        ]
        v = [transform(p) for p in v]
        idx = [
            (0, 1, 5, 4), (2, 3, 7, 6),  # front / back
            (1, 2, 6, 5), (3, 0, 4, 7),  # right / left
            (4, 5, 6, 7), (3, 2, 1, 0),  # top / bottom
        ]
        return [([v[i] for i in quad], color) for quad in idx]

    def _wheel_faces(self, side, angle_deg, heading):
        """Wheel as a segmented cylinder at x = side * 0.80."""
        r, half_t, n = 0.55, 0.09, 14
        ang = math.radians(angle_deg)
        cx = side * 0.80
        faces = []
        ring_out, ring_in = [], []
        for i in range(n):
            a = ang + i * 2 * math.pi / n
            y, z = r * math.cos(a), r * math.sin(a)
            for ring, xoff in ((ring_out, half_t * side + cx), (ring_in, -half_t * side + cx)):
                p = (xoff, y, z)
                p = self._rot_z(p, heading)
                ring.append(p)
        # tread segments
        for i in range(n):
            j = (i + 1) % n
            quad = [ring_out[i], ring_out[j], ring_in[j], ring_in[i]]
            faces.append((quad, self.WHEEL_COLOR))
        # side disc (outer face only, as a fan of triangles)
        center = self._rot_z((cx + half_t * side, 0, 0), heading)
        for i in range(n):
            j = (i + 1) % n
            faces.append(([center, ring_out[i], ring_out[j]], self.WHEEL_COLOR))
        # spokes to visualise rotation
        spokes = []
        for k in range(3):
            a = ang + k * 2 * math.pi / 3
            tip = (cx + half_t * side * 1.02, r * 0.82 * math.cos(a), r * 0.82 * math.sin(a))
            spokes.append((center, self._rot_z(tip, heading)))
        return faces, spokes

    # ---------- render ----------
    def render(self):
        c = self.canvas
        c.delete("all")
        w, h = self.w, self.h

        pitch = math.radians(self.pitch)
        heading = math.radians(self.heading)
        ground = -0.55

        def body_tf(p):
            p = self._rot_x(p, -pitch)  # lean about the axle
            return self._rot_z(p, heading)

        # Ground grid
        for i in range(-4, 5):
            g = i * 0.5
            for a, b in (((g, -2, ground), (g, 2, ground)), ((-2, g, ground), (2, g, ground))):
                p1, p2 = self._project(a), self._project(b)
                c.create_line(p1[0], p1[1], p2[0], p2[1], fill="#26313a")

        # Drop shadow
        sh = [self._project(self._rot_z(p, heading)) for p in
              ((-0.95, -0.45, ground + 0.01), (0.95, -0.45, ground + 0.01),
               (0.95, 0.45, ground + 0.01), (-0.95, 0.45, ground + 0.01))]
        c.create_polygon([xy for p in sh for xy in p[:2]], fill="#151c22", outline="")

        faces = []
        # Chassis: lower frame, main body, sensor deck
        faces += self._box_faces(0.62, 0.30, 0.06, 0.28, body_tf, (35, 40, 46))
        faces += self._box_faces(0.55, 0.26, 0.28, 1.35, body_tf, self.BODY_COLOR)
        faces += self._box_faces(0.58, 0.29, 1.35, 1.52, body_tf, self.TOP_COLOR)
        # Front indicator (marks robot's forward direction)
        faces += self._box_faces(0.16, 0.05, 1.05, 1.25,
                                 lambda p: body_tf((p[0], p[1] + 0.29, p[2])),
                                 (231, 76, 60))

        all_spokes = []
        for side, wang in ((-1, self.wheel_l), (1, self.wheel_r)):
            f, spokes = self._wheel_faces(side, wang, heading)
            faces += f
            all_spokes += spokes

        # Painter's algorithm
        drawable = []
        for pts, color in faces:
            proj = [self._project(p) for p in pts]
            depth = sum(p[2] for p in proj) / len(proj)
            n = self._normal(pts[0], pts[1], pts[2])
            drawable.append((depth, proj, self._shade(color, n)))
        drawable.sort(key=lambda item: -item[0])
        for _, proj, fill in drawable:
            c.create_polygon([xy for p in proj for xy in p[:2]],
                             fill=fill, outline="#10151a")

        # Wheel spokes (drawn on top)
        for a, b in all_spokes:
            p1, p2 = self._project(a), self._project(b)
            c.create_line(p1[0], p1[1], p2[0], p2[1],
                          fill="#f1c40f", width=2)

        # HUD
        c.create_text(12, 12, anchor="nw", fill="#8395a7",
                      font=("Consolas", 10),
                      text=f"Pitch {self.pitch:+6.2f}\u00b0   Heading {self.heading:+7.1f}\u00b0")
        c.create_text(12, h - 14, anchor="sw", fill="#576574",
                      font=("Consolas", 9), text="Drag: orbit  |  Scroll: zoom")


class ModernRobotController:
    def __init__(self, root):
        self.root = root
        self.root.title("Self-Balancing Robot - Engineering Console")
        self.root.geometry("1400x850")
        self.root.minsize(1000, 700)

        self.serial_port = None
        self.is_connected = False

        self.angle_data = deque(maxlen=200)
        self.target_data = deque(maxlen=200)
        for _ in range(200):
            self.angle_data.append(0.0)
            self.target_data.append(0.0)

        self.telemetry_vars = {}
        self.pressed_keys = {"w": False, "s": False, "a": False, "d": False}

        # Action tracking
        self.prev_l_ticks = 0
        self.prev_r_ticks = 0
        self.last_state_time = 0
        self.robot_state_var = tk.StringVar(value="Stationary")

        self.setup_ui()

        # 3D viewer animation loop (~30 FPS, independent of serial thread)
        self.root.after(100, self._animate_3d)

        # Ensure focus stealing for WASD overrides
        self.root.bind_all(
            "<Button-1>",
            lambda event: (
                event.widget.focus_set() if hasattr(event.widget, "focus_set") else None
            ),
        )
        self.root.bind_all("<KeyPress>", self.on_key_press)
        self.root.bind_all("<KeyRelease>", self.on_key_release)

    def setup_ui(self):
        # Grid Layout: Left Sidebar (Actions), Center (Plot+Telemetry), Right (Sliders)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        # ==========================================
        # LEFT SIDEBAR - CONNECTION & STATE
        # ==========================================
        self.sidebar = ctk.CTkFrame(self.root, corner_radius=0, width=220)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.sidebar.grid_rowconfigure(5, weight=1)

        header = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        header.grid(row=0, column=0, padx=20, pady=(20, 10))
        ctk.CTkLabel(
            header, text="2PyBot", font=ctk.CTkFont(size=24, weight="bold")
        ).pack()
        self.status_label = ctk.CTkLabel(
            header,
            text="\u25cf Disconnected",
            text_color="#e74c3c",
            font=ctk.CTkFont(size=13),
        )
        self.status_label.pack()

        self.port_combo = ctk.CTkComboBox(self.sidebar, values=self.get_ports())
        self.port_combo.grid(row=1, column=0, padx=20, pady=10)

        self.btn_refresh = ctk.CTkButton(
            self.sidebar,
            text="Refresh Ports",
            command=lambda: self.port_combo.configure(values=self.get_ports()),
        )
        self.btn_refresh.grid(row=2, column=0, padx=20, pady=5)

        self.btn_connect = ctk.CTkButton(
            self.sidebar,
            text="Connect",
            command=self.toggle_connection,
            fg_color="green",
            hover_color="#006400",
        )
        self.btn_connect.grid(row=3, column=0, padx=20, pady=(5, 20))

        # Actions
        ctk.CTkLabel(
            self.sidebar,
            text="Global Actions",
            font=ctk.CTkFont(size=16, weight="bold"),
        ).grid(row=4, column=0, padx=20, pady=(20, 10))

        btn_enable = ctk.CTkButton(
            self.sidebar,
            text="Enable Motors (E)",
            command=lambda: self.send_command("E"),
            fg_color="#d35400",
            hover_color="#e67e22",
        )
        btn_enable.grid(row=5, column=0, padx=20, pady=10, sticky="n")

        btn_kill = ctk.CTkButton(
            self.sidebar,
            text="KILL SWITCH (X)",
            command=lambda: self.send_command("X"),
            fg_color="#c0392b",
            hover_color="#e74c3c",
        )
        btn_kill.grid(row=6, column=0, padx=20, pady=10, sticky="n")

        btn_calib = ctk.CTkButton(
            self.sidebar,
            text="Calibrate Gyro (C)",
            command=lambda: self.send_command("C"),
        )
        btn_calib.grid(row=7, column=0, padx=20, pady=10, sticky="n")

        # ==========================================
        # CENTER - PLOTTING & TELEMETRY DASHBOARD
        # ==========================================
        self.main_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        self.main_frame.grid(row=0, column=1, sticky="nsew", padx=20, pady=20)
        self.main_frame.grid_rowconfigure(0, weight=2)
        self.main_frame.grid_rowconfigure(1, weight=1)
        self.main_frame.grid_columnconfigure(0, weight=3)
        self.main_frame.grid_columnconfigure(1, weight=2)

        # Matplotlib/Tk Canvas approach manually built with lines for pure speed
        self.plot_frame = ctk.CTkFrame(self.main_frame, corner_radius=10)
        self.plot_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 20), padx=(0, 10))
        ctk.CTkLabel(
            self.plot_frame,
            text="Live Pitch Angle Tracking",
            font=ctk.CTkFont(size=15, weight="bold"),
        ).pack(pady=(10, 0))

        self.canvas = tk.Canvas(self.plot_frame, bg="#1a2026", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=15, pady=15)
        self.canvas.bind("<Configure>", self.on_canvas_resize)
        self.cw = 600
        self.ch = 300

        # 3D Robot Viewer
        self.viewer_frame = ctk.CTkFrame(self.main_frame, corner_radius=10)
        self.viewer_frame.grid(row=0, column=1, sticky="nsew", pady=(0, 20))
        ctk.CTkLabel(
            self.viewer_frame,
            text="3D Robot Viewer",
            font=ctk.CTkFont(size=15, weight="bold"),
        ).pack(pady=(10, 0))

        viewer_canvas = tk.Canvas(
            self.viewer_frame, bg="#12181d", highlightthickness=0
        )
        viewer_canvas.pack(fill="both", expand=True, padx=15, pady=15)
        self.viewer = Robot3DViewer(viewer_canvas)

        # Deep Telemetry Grid
        self.telemetry_frame = ctk.CTkFrame(self.main_frame, corner_radius=10)
        self.telemetry_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")
        self.telemetry_frame.grid_columnconfigure((0, 1, 2, 3), weight=1)

        top_tel = ctk.CTkFrame(self.telemetry_frame, fg_color="transparent")
        top_tel.grid(row=0, column=0, columnspan=4, pady=(10, 0), sticky="ew")

        ctk.CTkLabel(
            top_tel,
            text="Real-Time Subsystems",
            font=ctk.CTkFont(size=15, weight="bold"),
        ).pack(side="left", padx=20)

        self.log_var = tk.BooleanVar(value=True)
        log_switch = ctk.CTkSwitch(
            top_tel,
            text="Enable Telemetry Stream",
            variable=self.log_var,
            command=lambda: self.send_command("L"),
        )
        log_switch.pack(side="right", padx=20)

        self.tel_labels = {
            "Ax": (0, 2),
            "Ay": (0, 3),
            "Az": (0, 4),
            "Gx": (1, 2),
            "Gy": (1, 3),
            "Gz": (1, 4),
            "Mx": (2, 2),
            "My": (2, 3),
            "Mz": (2, 4),
            "P": (0, 6),
            "I": (1, 6),
            "D": (2, 6),
            "L_Speed": (3, 2),
            "R_Speed": (3, 3),
            "L_Ticks": (3, 6),
            "R_Ticks": (3, 7),
            "L_WhlAng": (3, 8),
            "R_WhlAng": (3, 9),
        }

        ctk.CTkLabel(
            self.telemetry_frame, text="Accelerometer", font=ctk.CTkFont(weight="bold")
        ).grid(row=1, column=0, padx=10)
        ctk.CTkLabel(
            self.telemetry_frame, text="Gyroscope", font=ctk.CTkFont(weight="bold")
        ).grid(row=1, column=1, padx=10)
        ctk.CTkLabel(
            self.telemetry_frame, text="Magnetometer", font=ctk.CTkFont(weight="bold")
        ).grid(row=1, column=2, padx=10)
        ctk.CTkLabel(
            self.telemetry_frame,
            text="Hardware Outputs",
            font=ctk.CTkFont(weight="bold"),
        ).grid(row=1, column=3, padx=10)

        for key in self.tel_labels.keys():
            self.telemetry_vars[key] = tk.StringVar(value="0.0")

        row_offset = 2

        # Accel
        self.create_tel_row("Ax", 0, row_offset)
        self.create_tel_row("Ay", 0, row_offset + 1)
        self.create_tel_row("Az", 0, row_offset + 2)
        # Gyro
        self.create_tel_row("Gx", 1, row_offset)
        self.create_tel_row("Gy", 1, row_offset + 1)
        self.create_tel_row("Gz", 1, row_offset + 2)
        # Mag
        self.create_tel_row("Mx", 2, row_offset)
        self.create_tel_row("My", 2, row_offset + 1)
        self.create_tel_row("Mz", 2, row_offset + 2)

        # PIDs + Motors
        ctk.CTkLabel(
            self.telemetry_frame,
            text="Balance PID Terms",
            font=ctk.CTkFont(weight="bold"),
        ).grid(row=row_offset + 3, column=0, columnspan=3, pady=(15, 5))
        self.create_tel_row("P", 0, row_offset + 4)
        self.create_tel_row("I", 1, row_offset + 4)
        self.create_tel_row("D", 2, row_offset + 4)

        # Motor speeds + Encoder data (right column)
        self.create_tel_row("L_Speed", 3, row_offset)
        self.create_tel_row("R_Speed", 3, row_offset + 1)

        # Encoder section header
        ctk.CTkLabel(
            self.telemetry_frame, text="Wheel Encoders", font=ctk.CTkFont(weight="bold")
        ).grid(row=row_offset + 3, column=3, pady=(15, 5))
        self.create_tel_row("L_Ticks", 3, row_offset + 4)
        self.create_tel_row("R_Ticks", 3, row_offset + 5)
        self.create_tel_row("L_WhlAng", 3, row_offset + 6)
        self.create_tel_row("R_WhlAng", 3, row_offset + 7)

        # Action Display in center (under Balance PID Terms)
        action_frame = ctk.CTkFrame(
            self.telemetry_frame, corner_radius=10, fg_color="#1e272e"
        )
        action_frame.grid(
            row=row_offset + 5,
            column=0,
            columnspan=3,
            rowspan=3,
            padx=10,
            pady=10,
            sticky="nsew",
        )
        ctk.CTkLabel(
            action_frame,
            text="Current Robot Action",
            font=ctk.CTkFont(weight="bold", size=14),
        ).pack(pady=(5, 0))
        ctk.CTkLabel(
            action_frame,
            textvariable=self.robot_state_var,
            font=ctk.CTkFont(size=24, weight="bold"),
            text_color="#f1c40f",
        ).pack(pady=10)

        # ==========================================
        # RIGHT SIDEBAR - PID SLIDERS & OPTIONS
        # ==========================================
        self.tuning_frame = ctk.CTkScrollableFrame(
            self.root, width=320, corner_radius=0
        )
        self.tuning_frame.grid(row=0, column=2, sticky="nsew")

        ctk.CTkLabel(
            self.tuning_frame,
            text="Tuning Parameters",
            font=ctk.CTkFont(size=20, weight="bold"),
        ).pack(pady=20)

        self.switches = {}

        # Options Group inside Tuning
        opt_group = ctk.CTkFrame(self.tuning_frame)
        opt_group.pack(fill="x", padx=10, pady=10)

        self.create_switch(opt_group, "Enable Position Hold", "EN_P", False)
        self.create_switch(opt_group, "Enable Heading / Yaw PID", "EN_Y", False)
        self.create_switch(opt_group, "Invert Yaw PID Direction", "INV_Y", True)
        self.create_switch(opt_group, "Fall Protection", "FP_EN", True)

        # Fall Protection tuning
        self.create_slider_group(
            self.tuning_frame,
            "Fall Protection",
            [
                ("Predict Horizon (s)", "FP_H", 0.05, 0.6, 0.25),
                ("Risk Start (\u00b0)", "FP_S", 3.0, 20.0, 8.0),
                ("Risk Full (\u00b0)", "FP_F", 10.0, 45.0, 20.0),
            ],
        )

        # Balance
        self.create_slider_group(
            self.tuning_frame,
            "Balance PID",
            [
                ("Kp", "KP", 0, 5000, 1250),
                ("Ki", "KI", 0, 200, 42.86),
                ("Kd", "KD", 0, 200, 1.786),
                ("Target Angle", "T", -10.0, 10.0, 0.071),
            ],
        )

        # Position Hold
        self.create_slider_group(
            self.tuning_frame,
            "Position Hold (Drift Correction)",
            [
                ("Pos Kp", "PKP", 0.0, 0.005, 0.0006),
                ("Pos Kd", "PKD", 0.0, 0.01, 0.003),
                ("Max Hold Tilt (\u00b0)", "PMT", 0.5, 5.0, 3.0),
            ],
            decimal_places=4,
        )

        # Yaw
        self.create_slider_group(
            self.tuning_frame,
            "Yaw (Heading) PID",
            [
                ("Yaw Kp", "YKP", 0, 100, 20),
                ("Yaw Kd", "YKD", 0, 50, 5),
                ("Max Turn", "YMT", 100, 10000, 4000),
            ],
        )

        # Drive Controls
        self.create_slider_group(
            self.tuning_frame,
            "Manual Drive Settings",
            [("Drive Tilt (W/S)", "MDT", 0.0, 5.0, 3.0)],
        )

    def create_tel_row(self, key, col, row):
        frame = ctk.CTkFrame(self.telemetry_frame, fg_color="transparent")
        frame.grid(row=row, column=col, sticky="w", padx=10, pady=2)
        ctk.CTkLabel(
            frame, text=f"{key}:", font=ctk.CTkFont(weight="bold"), width=70, anchor="e"
        ).pack(side="left")
        ctk.CTkLabel(
            frame,
            textvariable=self.telemetry_vars[key],
            font=ctk.CTkFont(family="Consolas"),
            width=80,
            anchor="w",
        ).pack(side="left")

    def create_switch(self, parent, text, cmd_prefix, default):
        var = tk.BooleanVar(value=default)
        switch = ctk.CTkSwitch(
            parent,
            text=text,
            variable=var,
            command=lambda cmd=cmd_prefix, v=var: self.send_config(
                cmd, 1 if v.get() else 0
            ),
        )
        switch.pack(anchor="w", padx=10, pady=8)
        self.switches[cmd_prefix] = var

    def create_slider_group(self, parent, title, layout, decimal_places=3):
        group = ctk.CTkFrame(parent)
        group.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(group, text=title, font=ctk.CTkFont(weight="bold", size=14)).pack(
            pady=5
        )

        fmt = f".{decimal_places}f"
        for name, cmd, vmin, vmax, val in layout:
            f = ctk.CTkFrame(group, fg_color="transparent")
            f.pack(fill="x", padx=10, pady=5)

            val_var = tk.StringVar(value=f"{val:{fmt}}")

            top_frame = ctk.CTkFrame(f, fg_color="transparent")
            top_frame.pack(fill="x")

            ctk.CTkLabel(top_frame, text=name).pack(side="left")
            ctk.CTkLabel(
                top_frame, textvariable=val_var, font=ctk.CTkFont(weight="bold")
            ).pack(side="right")

            slider = ctk.CTkSlider(
                f,
                from_=vmin,
                to=vmax,
                command=lambda v, var=val_var, dp=fmt: var.set(f"{v:{dp}}"),
            )
            slider.set(val)
            slider.pack(fill="x", pady=2)

            slider.bind(
                "<ButtonRelease-1>",
                lambda event, c=cmd, s=slider: self.send_config(c, s.get()),
            )

    def on_canvas_resize(self, event):
        self.cw = event.width
        self.ch = event.height

    def _animate_3d(self):
        try:
            self.viewer.render()
        except tk.TclError:
            return  # window closed
        self.root.after(33, self._animate_3d)

    def get_ports(self):
        return [port.device for port in list_ports.comports()] or ["No Comports Found"]

    def toggle_connection(self):
        if self.is_connected:
            self.is_connected = False
            if self.serial_port:
                self.serial_port.close()
            self.btn_connect.configure(
                text="Connect", fg_color="green", hover_color="#006400"
            )
            self.status_label.configure(
                text="\u25cf Disconnected", text_color="#e74c3c"
            )
            print("Disconnected.")
        else:
            port = self.port_combo.get()
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=0.1)
                self.is_connected = True
                self.btn_connect.configure(
                    text="Disconnect", fg_color="red", hover_color="#8b0000"
                )
                self.status_label.configure(
                    text=f"\u25cf {port}", text_color="#2ecc71"
                )
                print(f"Connected to {port}")

                def read_thread():
                    buffer = ""
                    # Variables for UI update throttling
                    last_ui_update = time.time()
                    ui_refresh_rate = 1.0 / 30.0  # 30 FPS maximum inside Python

                    while self.is_connected:
                        try:
                            waiting = self.serial_port.in_waiting
                            if waiting > 0:
                                chunk = self.serial_port.read(waiting).decode(
                                    "ascii", errors="ignore"
                                )
                                buffer += chunk

                                lines = buffer.split("\n")
                                buffer = lines.pop()

                                updated = False
                                for line in lines:
                                    if line.startswith("["):
                                        print(line)
                                    else:
                                        parts = line.split("\t")
                                        if len(parts) >= 16:
                                            try:
                                                vals = [float(p) for p in parts]
                                                self.angle_data.append(vals[0])
                                                self.target_data.append(vals[1])

                                                # Update 3D viewer state
                                                self.viewer.pitch = vals[0]
                                                self.viewer.heading = math.degrees(
                                                    math.atan2(vals[9], vals[8])
                                                )

                                                self.telemetry_vars["Ax"].set(
                                                    f"{vals[2]:<8.3f}"
                                                )
                                                self.telemetry_vars["Ay"].set(
                                                    f"{vals[3]:<8.3f}"
                                                )
                                                self.telemetry_vars["Az"].set(
                                                    f"{vals[4]:<8.3f}"
                                                )

                                                self.telemetry_vars["Gx"].set(
                                                    f"{vals[5]:<8.3f}"
                                                )
                                                self.telemetry_vars["Gy"].set(
                                                    f"{vals[6]:<8.3f}"
                                                )
                                                self.telemetry_vars["Gz"].set(
                                                    f"{vals[7]:<8.3f}"
                                                )

                                                self.telemetry_vars["Mx"].set(
                                                    f"{vals[8]:<8.3f}"
                                                )
                                                self.telemetry_vars["My"].set(
                                                    f"{vals[9]:<8.3f}"
                                                )
                                                self.telemetry_vars["Mz"].set(
                                                    f"{vals[10]:<8.3f}"
                                                )

                                                self.telemetry_vars["P"].set(
                                                    f"{vals[11]:<8.3f}"
                                                )
                                                self.telemetry_vars["I"].set(
                                                    f"{vals[12]:<8.3f}"
                                                )
                                                self.telemetry_vars["D"].set(
                                                    f"{vals[13]:<8.3f}"
                                                )

                                                self.telemetry_vars["L_Speed"].set(
                                                    f"{int(vals[14]):<6d}"
                                                )
                                                self.telemetry_vars["R_Speed"].set(
                                                    f"{int(vals[15]):<6d}"
                                                )

                                                # Encoder data (fields 16-19, present in 20-field packets)
                                                if len(vals) >= 20:
                                                    curr_l = int(vals[16])
                                                    curr_r = int(vals[17])

                                                    self.telemetry_vars["L_Ticks"].set(
                                                        f"{curr_l:<8d}"
                                                    )
                                                    self.telemetry_vars["R_Ticks"].set(
                                                        f"{curr_r:<8d}"
                                                    )
                                                    self.telemetry_vars["L_WhlAng"].set(
                                                        f"{vals[18]:<7.1f}°"
                                                    )
                                                    self.telemetry_vars["R_WhlAng"].set(
                                                        f"{vals[19]:<7.1f}°"
                                                    )

                                                    self.viewer.wheel_l = vals[18]
                                                    self.viewer.wheel_r = vals[19]

                                                    # State estimation logic
                                                    dl = curr_l - self.prev_l_ticks
                                                    dr = curr_r - self.prev_r_ticks

                                                    # Threshold to ignore minor balancing jitter
                                                    t = 3
                                                    new_state = "Stationary"
                                                    if abs(dl) < t and abs(dr) < t:
                                                        new_state = "Stationary"
                                                    elif dl >= t and dr >= t:
                                                        new_state = "Moving Forward"
                                                    elif dl <= -t and dr <= -t:
                                                        new_state = "Moving Backward"
                                                    elif dl >= t and dr <= -t:
                                                        new_state = "Turning Right"
                                                    elif dl <= -t and dr >= t:
                                                        new_state = "Turning Left"

                                                    # Visual delay: keep text for at least 0.2s to prevent flickering
                                                    now = time.time()
                                                    if (
                                                        new_state
                                                        != self.robot_state_var.get()
                                                    ):
                                                        if (
                                                            now - self.last_state_time
                                                        ) > 0.2:
                                                            self.robot_state_var.set(
                                                                new_state
                                                            )
                                                            self.last_state_time = now
                                                    else:
                                                        # Mixed movement or small changes
                                                        pass

                                                    self.prev_l_ticks = curr_l
                                                    self.prev_r_ticks = curr_r

                                                updated = True
                                            except ValueError:
                                                pass

                                if (
                                    updated
                                    and (time.time() - last_ui_update) > ui_refresh_rate
                                ):
                                    self.root.after_idle(self.draw_plot)
                                    last_ui_update = time.time()

                        except Exception as e:
                            print(f"Serial Error: {e}")
                            break
                        time.sleep(0.005)

                threading.Thread(target=read_thread, daemon=True).start()
            except Exception as e:
                print(f"Failed to connect: {e}")

    def send_config(self, key, value):
        if self.is_connected and self.serial_port:
            packet = f"${key}={value}\n".encode("ascii")
            self.serial_port.write(packet)
            print(f"Sent: {packet.decode().strip()}")

    def send_command(self, cmd_char):
        if self.is_connected and self.serial_port:
            packet = f"${cmd_char}\n".encode("ascii")
            self.serial_port.write(packet)
            print(f"Command Sent: {cmd_char}")

    def send_drive(self, cmd_char):
        if self.is_connected and self.serial_port:
            # Send native instant hash-character packet
            self.serial_port.write(f"#{cmd_char}".encode("ascii"))

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key in self.pressed_keys and not self.pressed_keys[key]:
            self.pressed_keys[key] = True
            if key == "w":
                self.send_drive("W")
            elif key == "s":
                self.send_drive("S")
            elif key == "a":
                self.send_drive("A")
            elif key == "d":
                self.send_drive("D")

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in self.pressed_keys:
            self.pressed_keys[key] = False
            if key in ["a", "d"] and not (
                self.pressed_keys["a"] or self.pressed_keys["d"]
            ):
                self.send_drive(" ")
            elif key in ["w", "s"] and not (
                self.pressed_keys["w"] or self.pressed_keys["s"]
            ):
                self.send_drive("X")

    def draw_plot(self):
        self.canvas.delete("all")
        w, h = self.cw, self.ch

        # Horizontal grid lines every 5 degrees
        y_scale_grid = h / 40.0
        for deg in range(-15, 20, 5):
            y = h / 2 - deg * y_scale_grid
            color = "#3a4750" if deg == 0 else "#232b32"
            self.canvas.create_line(0, y, w, y, fill=color, dash=(4, 4))
            if deg != 0:
                self.canvas.create_text(
                    w - 6, y - 2, anchor="se", text=f"{deg:+d}°",
                    fill="#4b5a66", font=("Consolas", 8),
                )

        y_scale = h / 40.0
        points_angle = []
        points_target = []

        for i, (ang, tgt) in enumerate(zip(self.angle_data, self.target_data)):
            x = (i / 200.0) * w
            y_ang = h / 2 - (ang * y_scale)
            y_tgt = h / 2 - (tgt * y_scale)
            points_angle.extend([x, y_ang])
            points_target.extend([x, y_tgt])

        if len(points_target) >= 4:
            self.canvas.create_line(
                *points_target, fill="#d63031", dash=(2, 2), smooth=False
            )
        if len(points_angle) >= 4:
            self.canvas.create_line(*points_angle, fill="#00cec9", smooth=True, width=2)

        # Legend
        self.canvas.create_text(
            10,
            10,
            anchor="nw",
            text=f"Actual Angle: {self.angle_data[-1]:.2f}°",
            fill="#00cec9",
            font=("Consolas", 11, "bold"),
        )
        self.canvas.create_text(
            10,
            30,
            anchor="nw",
            text=f"Target Angle: {self.target_data[-1]:.2f}°",
            fill="#e74c3c",
            font=("Consolas", 11, "bold"),
        )


if __name__ == "__main__":
    app = ctk.CTk()
    gui = ModernRobotController(app)
    app.mainloop()
