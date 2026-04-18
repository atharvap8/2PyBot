"""2PyBot Desktop Dashboard — Real-time telemetry and PID tuning over Bluetooth serial."""

import threading
import time
from collections import deque
import serial
from serial.tools import list_ports
import tkinter as tk
import customtkinter as ctk

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")  # Default theme

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

        self.pressed_keys = {'w': False, 'a': False, 's': False, 'd': False}
        self.telemetry_vars = {}
        
        self.setup_ui()
        
        # Ensure focus stealing for WASD overrides
        self.root.bind_all("<Button-1>", lambda event: event.widget.focus_set() if hasattr(event.widget, "focus_set") else None)
        self.root.bind_all('<KeyPress>', self.on_key_press)
        self.root.bind_all('<KeyRelease>', self.on_key_release)

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

        ctk.CTkLabel(self.sidebar, text="Connection", font=ctk.CTkFont(size=20, weight="bold")).grid(row=0, column=0, padx=20, pady=(20, 10))
        
        self.port_combo = ctk.CTkComboBox(self.sidebar, values=self.get_ports())
        self.port_combo.grid(row=1, column=0, padx=20, pady=10)
        
        self.btn_refresh = ctk.CTkButton(self.sidebar, text="Refresh Ports", command=lambda: self.port_combo.configure(values=self.get_ports()))
        self.btn_refresh.grid(row=2, column=0, padx=20, pady=5)
        
        self.btn_connect = ctk.CTkButton(self.sidebar, text="Connect", command=self.toggle_connection, fg_color="green", hover_color="#006400")
        self.btn_connect.grid(row=3, column=0, padx=20, pady=(5, 20))
        
        # Actions
        ctk.CTkLabel(self.sidebar, text="Global Actions", font=ctk.CTkFont(size=16, weight="bold")).grid(row=4, column=0, padx=20, pady=(20, 10))
        
        btn_enable = ctk.CTkButton(self.sidebar, text="Enable Motors (E)", command=lambda: self.send_command('E'), fg_color="#d35400", hover_color="#e67e22")
        btn_enable.grid(row=5, column=0, padx=20, pady=10, sticky="n")
        
        btn_kill = ctk.CTkButton(self.sidebar, text="KILL SWITCH (X)", command=lambda: self.send_command('X'), fg_color="#c0392b", hover_color="#e74c3c")
        btn_kill.grid(row=6, column=0, padx=20, pady=10, sticky="n")

        btn_calib = ctk.CTkButton(self.sidebar, text="Calibrate Gyro (C)", command=lambda: self.send_command('C'))
        btn_calib.grid(row=7, column=0, padx=20, pady=10, sticky="n")
        
        # ==========================================
        # CENTER - PLOTTING & TELEMETRY DASHBOARD
        # ==========================================
        self.main_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        self.main_frame.grid(row=0, column=1, sticky="nsew", padx=20, pady=20)
        self.main_frame.grid_rowconfigure(0, weight=2)
        self.main_frame.grid_rowconfigure(1, weight=1)
        self.main_frame.grid_columnconfigure(0, weight=1)

        # Matplotlib/Tk Canvas approach manually built with lines for pure speed
        self.plot_frame = ctk.CTkFrame(self.main_frame, corner_radius=10)
        self.plot_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 20))
        ctk.CTkLabel(self.plot_frame, text="Live Pitch Angle Tracking", font=ctk.CTkFont(size=15, weight="bold")).pack(pady=10)
        
        self.canvas = tk.Canvas(self.plot_frame, bg="#2b2b2b", highlightthickness=0)
        self.canvas.pack(fill='both', expand=True, padx=15, pady=15)
        self.canvas.bind("<Configure>", self.on_canvas_resize)
        self.cw = 600
        self.ch = 300
        

        # Deep Telemetry Grid
        self.telemetry_frame = ctk.CTkFrame(self.main_frame, corner_radius=10)
        self.telemetry_frame.grid(row=1, column=0, sticky="nsew")
        self.telemetry_frame.grid_columnconfigure((0,1,2,3), weight=1)
        
        top_tel = ctk.CTkFrame(self.telemetry_frame, fg_color="transparent")
        top_tel.grid(row=0, column=0, columnspan=4, pady=(10, 0), sticky="ew")
        
        ctk.CTkLabel(top_tel, text="Real-Time Subsystems", font=ctk.CTkFont(size=15, weight="bold")).pack(side="left", padx=20)
        
        self.log_var = tk.BooleanVar(value=True)
        log_switch = ctk.CTkSwitch(top_tel, text="Enable Telemetry Stream", variable=self.log_var, command=lambda: self.send_command("L"))
        log_switch.pack(side="right", padx=20)
        
        self.tel_labels = {
            "Ax": (1, 0), "Ay": (2, 0), "Az": (3, 0),
            "Gx": (1, 1), "Gy": (2, 1), "Gz": (3, 1),
            "Mx": (1, 2), "My": (2, 2), "Mz": (3, 2),
            "P": (5, 0), "I": (5, 1), "D": (5, 2),
            "L_Speed": (5, 3), "R_Speed": (6, 3)
        }
        
        ctk.CTkLabel(self.telemetry_frame, text="Accelerometer", font=ctk.CTkFont(weight="bold")).grid(row=1, column=0, padx=10)
        ctk.CTkLabel(self.telemetry_frame, text="Gyroscope", font=ctk.CTkFont(weight="bold")).grid(row=1, column=1, padx=10)
        ctk.CTkLabel(self.telemetry_frame, text="Magnetometer", font=ctk.CTkFont(weight="bold")).grid(row=1, column=2, padx=10)
        ctk.CTkLabel(self.telemetry_frame, text="Hardware Outputs", font=ctk.CTkFont(weight="bold")).grid(row=1, column=3, padx=10)

        for key in self.tel_labels.keys():
            self.telemetry_vars[key] = tk.StringVar(value="0.0")
            
        row_offset = 2
        
        # Accel
        self.create_tel_row("Ax", 0, row_offset)
        self.create_tel_row("Ay", 0, row_offset+1)
        self.create_tel_row("Az", 0, row_offset+2)
        # Gyro
        self.create_tel_row("Gx", 1, row_offset)
        self.create_tel_row("Gy", 1, row_offset+1)
        self.create_tel_row("Gz", 1, row_offset+2)
        # Mag
        self.create_tel_row("Mx", 2, row_offset)
        self.create_tel_row("My", 2, row_offset+1)
        self.create_tel_row("Mz", 2, row_offset+2)
        
        # PIDs + Motors
        ctk.CTkLabel(self.telemetry_frame, text="Balance PID Terms", font=ctk.CTkFont(weight="bold")).grid(row=row_offset+3, column=0, columnspan=3, pady=(15,5))
        self.create_tel_row("P", 0, row_offset+4)
        self.create_tel_row("I", 1, row_offset+4)
        self.create_tel_row("D", 2, row_offset+4)
        
        self.create_tel_row("L_Speed", 3, row_offset)
        self.create_tel_row("R_Speed", 3, row_offset+1)

        # ==========================================
        # RIGHT SIDEBAR - PID SLIDERS & OPTIONS
        # ==========================================
        self.tuning_frame = ctk.CTkScrollableFrame(self.root, width=320, corner_radius=0)
        self.tuning_frame.grid(row=0, column=2, sticky="nsew")

        ctk.CTkLabel(self.tuning_frame, text="Tuning Parameters", font=ctk.CTkFont(size=20, weight="bold")).pack(pady=20)
        
        self.switches = {}
        
        # Options Group inside Tuning
        opt_group = ctk.CTkFrame(self.tuning_frame)
        opt_group.pack(fill="x", padx=10, pady=10)
        
        self.create_switch(opt_group, "Enable Position Hold", "EN_P", False)
        self.create_switch(opt_group, "Enable Heading / Yaw PID", "EN_Y", False)
        self.create_switch(opt_group, "Invert Yaw PID Direction", "INV_Y", True)
        
        # Balance
        self.create_slider_group(self.tuning_frame, "Balance PID", [
            ("Kp", "KP", 0, 5000, 1000),
            ("Ki", "KI", 0, 200, 30),
            ("Kd", "KD", 0, 200, 30),
            ("Target Angle", "T", -10.0, 10.0, -1.469)
        ])
        
        # Position Hold
        self.create_slider_group(self.tuning_frame, "Position Hold (Drift Correction)", [
            ("Pos Kp", "PKP", 0.0, 0.005, 0.0006),
            ("Pos Kd", "PKD", 0.0, 0.01, 0.003),
            ("Max Hold Tilt (\u00b0)", "PMT", 0.5, 5.0, 3.0)
        ], decimal_places=4)

        # Yaw
        self.create_slider_group(self.tuning_frame, "Yaw (Heading) PID", [
            ("Yaw Kp", "YKP", 0, 100, 20),
            ("Yaw Kd", "YKD", 0, 50, 5),
            ("Max Turn", "YMT", 100, 10000, 4000)
        ])
        
        # Drive Controls
        self.create_slider_group(self.tuning_frame, "Manual Drive Settings", [
            ("Drive Tilt (W/S)", "MDT", 0.0, 5.0, 3.0)
        ])
        
    def create_tel_row(self, key, col, row):
        frame = ctk.CTkFrame(self.telemetry_frame, fg_color="transparent")
        frame.grid(row=row, column=col, sticky="w", padx=10, pady=2)
        ctk.CTkLabel(frame, text=f"{key}:", font=ctk.CTkFont(weight="bold"), width=70, anchor="e").pack(side="left")
        ctk.CTkLabel(frame, textvariable=self.telemetry_vars[key], font=ctk.CTkFont(family="Consolas"), width=80, anchor="w").pack(side="left")

    def create_switch(self, parent, text, cmd_prefix, default):
        var = tk.BooleanVar(value=default)
        switch = ctk.CTkSwitch(parent, text=text, variable=var, 
                               command=lambda cmd=cmd_prefix, v=var: self.send_config(cmd, 1 if v.get() else 0))
        switch.pack(anchor="w", padx=10, pady=8)
        self.switches[cmd_prefix] = var

    def create_slider_group(self, parent, title, layout, decimal_places=3):
        group = ctk.CTkFrame(parent)
        group.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(group, text=title, font=ctk.CTkFont(weight="bold", size=14)).pack(pady=5)
        
        fmt = f".{decimal_places}f"
        for name, cmd, vmin, vmax, val in layout:
            f = ctk.CTkFrame(group, fg_color="transparent")
            f.pack(fill="x", padx=10, pady=5)
            
            val_var = tk.StringVar(value=f"{val:{fmt}}")
            
            top_frame = ctk.CTkFrame(f, fg_color="transparent")
            top_frame.pack(fill="x")
            
            ctk.CTkLabel(top_frame, text=name).pack(side="left")
            ctk.CTkLabel(top_frame, textvariable=val_var, font=ctk.CTkFont(weight="bold")).pack(side="right")
            
            slider = ctk.CTkSlider(f, from_=vmin, to=vmax, command=lambda v, var=val_var, dp=fmt: var.set(f"{v:{dp}}"))
            slider.set(val)
            slider.pack(fill="x", pady=2)
            
            slider.bind("<ButtonRelease-1>", lambda event, c=cmd, s=slider: self.send_config(c, s.get()))

    def on_canvas_resize(self, event):
        self.cw = event.width
        self.ch = event.height

    def get_ports(self):
        return [port.device for port in list_ports.comports()] or ["No Comports Found"]

    def toggle_connection(self):
        if self.is_connected:
            self.is_connected = False
            if self.serial_port:
                self.serial_port.close()
            self.btn_connect.configure(text="Connect", fg_color="green", hover_color="#006400")
            print("Disconnected.")
        else:
            port = self.port_combo.get()
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=0.1)
                self.is_connected = True
                self.btn_connect.configure(text="Disconnect", fg_color="red", hover_color="#8b0000")
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
                                chunk = self.serial_port.read(waiting).decode('ascii', errors='ignore')
                                buffer += chunk
                                
                                lines = buffer.split('\n')
                                buffer = lines.pop() 
                                
                                updated = False
                                for line in lines:
                                    if line.startswith('['):
                                        print(line)
                                    else:
                                        parts = line.split('\t')
                                        if len(parts) == 16:
                                            try:
                                                vals = [float(p) for p in parts]
                                                self.angle_data.append(vals[0])
                                                self.target_data.append(vals[1])
                                                
                                                self.telemetry_vars["Ax"].set(f"{vals[2]:<8.3f}")
                                                self.telemetry_vars["Ay"].set(f"{vals[3]:<8.3f}")
                                                self.telemetry_vars["Az"].set(f"{vals[4]:<8.3f}")
                                                
                                                self.telemetry_vars["Gx"].set(f"{vals[5]:<8.3f}")
                                                self.telemetry_vars["Gy"].set(f"{vals[6]:<8.3f}")
                                                self.telemetry_vars["Gz"].set(f"{vals[7]:<8.3f}")
                                                
                                                self.telemetry_vars["Mx"].set(f"{vals[8]:<8.3f}")
                                                self.telemetry_vars["My"].set(f"{vals[9]:<8.3f}")
                                                self.telemetry_vars["Mz"].set(f"{vals[10]:<8.3f}")
                                                
                                                self.telemetry_vars["P"].set(f"{vals[11]:<8.3f}")
                                                self.telemetry_vars["I"].set(f"{vals[12]:<8.3f}")
                                                self.telemetry_vars["D"].set(f"{vals[13]:<8.3f}")
                                                
                                                self.telemetry_vars["L_Speed"].set(f"{int(vals[14]):<6d}")
                                                self.telemetry_vars["R_Speed"].set(f"{int(vals[15]):<6d}")
                                                updated = True
                                            except ValueError:
                                                pass
                                
                                if updated and (time.time() - last_ui_update) > ui_refresh_rate:
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
            packet = f"${key}={value}\n".encode('ascii')
            self.serial_port.write(packet)
            print(f"Sent: {packet.decode().strip()}")

    def send_command(self, cmd_char):
        if self.is_connected and self.serial_port:
            packet = f"${cmd_char}\n".encode('ascii')
            self.serial_port.write(packet)
            print(f"Command Sent: {cmd_char}")
            
    def send_drive(self, cmd_char):
        if self.is_connected and self.serial_port:
            # Send native instant hash-character packet
            self.serial_port.write(f"#{cmd_char}".encode('ascii'))

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key in self.pressed_keys and not self.pressed_keys[key]:
            self.pressed_keys[key] = True
            if key == 'w':
                self.send_drive('W')
            elif key == 's':
                self.send_drive('S')
            elif key == 'a':
                self.send_drive('A')
            elif key == 'd':
                self.send_drive('D')

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in self.pressed_keys:
            self.pressed_keys[key] = False
            if key in ['a', 'd'] and not (self.pressed_keys['a'] or self.pressed_keys['d']):
                self.send_drive(' ')
            elif key in ['w', 's'] and not (self.pressed_keys['w'] or self.pressed_keys['s']):
                self.send_drive('X')

    def draw_plot(self):
        self.canvas.delete("all")
        w, h = self.cw, self.ch
        
        # Center line
        self.canvas.create_line(0, h/2, w, h/2, fill="#3a3a3a", dash=(4, 4))
        
        y_scale = h / 40.0
        points_angle = []
        points_target = []
        
        for i, (ang, tgt) in enumerate(zip(self.angle_data, self.target_data)):
            x = (i / 200.0) * w
            y_ang = h/2 - (ang * y_scale)
            y_tgt = h/2 - (tgt * y_scale)
            points_angle.extend([x, y_ang])
            points_target.extend([x, y_tgt])
            
        if len(points_target) >= 4:
            self.canvas.create_line(*points_target, fill="#d63031", dash=(2, 2), smooth=False)
        if len(points_angle) >= 4:
            self.canvas.create_line(*points_angle, fill="#00cec9", smooth=True, width=2)
            
        # Legend
        self.canvas.create_text(10, 10, anchor="nw", text=f"Actual Angle: {self.angle_data[-1]:.2f}°", fill="#00cec9", font=("Arial", 12, "bold"))
        self.canvas.create_text(10, 30, anchor="nw", text=f"Target Angle: {self.target_data[-1]:.2f}°", fill="#d63031", font=("Arial", 12, "bold"))

if __name__ == "__main__":
    app = ctk.CTk()
    gui = ModernRobotController(app)
    app.mainloop()
