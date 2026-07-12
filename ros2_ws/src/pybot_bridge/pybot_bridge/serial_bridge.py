"""Serial bridge node for the 2PyBot self-balancing robot.

Reads the robot's tab-separated Bluetooth serial telemetry stream and
publishes it as ROS 2 topics; subscribes to /cmd_vel and forwards drive
commands using the robot's native serial protocol.

Telemetry packet (tab-separated, 16 or 20 fields):
    0: pitch (deg)        1: target pitch (deg)
    2-4:  Ax Ay Az (g)    5-7:  Gx Gy Gz (deg/s)
    8-10: Mx My Mz        11-13: PID P I D terms
    14-15: L/R stepper speed (steps/s)
    16-19: L/R encoder ticks, L/R wheel angle (deg)   [extended packet]

Published topics:
    /pybot/imu          sensor_msgs/Imu       (orientation = pitch only)
    /pybot/mag          sensor_msgs/MagneticField
    /pybot/pitch        std_msgs/Float32
    /pybot/joint_states sensor_msgs/JointState (wheel angles/velocities)
    /pybot/odom         nav_msgs/Odometry     (differential-drive estimate)

Subscribed topics:
    /cmd_vel            geometry_msgs/Twist

Serial command protocol (matches serial_tuner.h):
    "$KEY=value\n"  config values    "$X\n" kill    "$E\n" enable
"""

import math
import threading

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, MagneticField
from std_msgs.msg import Float32

G = 9.80665
DEG = math.pi / 180.0


class PybotSerialBridge(Node):
    def __init__(self):
        super().__init__("pybot_serial_bridge")

        # ---- Parameters ----
        self.declare_parameter("port", "COM7")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_radius", 0.055)      # meters
        self.declare_parameter("wheel_separation", 0.16)   # meters
        self.declare_parameter("max_lean_deg", 3.0)        # full-speed lean
        self.declare_parameter("max_linear", 0.5)          # m/s at full lean
        self.declare_parameter("max_angular", 2.0)         # rad/s at full steer

        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_sep = self.get_parameter("wheel_separation").value
        self.max_lean = self.get_parameter("max_lean_deg").value
        self.max_linear = self.get_parameter("max_linear").value
        self.max_angular = self.get_parameter("max_angular").value

        # ---- Publishers ----
        self.pub_imu = self.create_publisher(Imu, "pybot/imu", 10)
        self.pub_mag = self.create_publisher(MagneticField, "pybot/mag", 10)
        self.pub_pitch = self.create_publisher(Float32, "pybot/pitch", 10)
        self.pub_joints = self.create_publisher(JointState, "pybot/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "pybot/odom", 10)

        # ---- Subscriber ----
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        # ---- Odometry state ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_wheel = None  # (left_deg, right_deg)
        self.prev_stamp = None

        # ---- Serial ----
        self.ser = None
        self._lock = threading.Lock()
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self.get_logger().info(f"pybot_serial_bridge up — port={self.port}")

    # ==================================================
    #  Serial handling
    # ==================================================
    def _connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            self.get_logger().info(f"Connected to {self.port}")
            return True
        except serial.SerialException as e:
            self.get_logger().warning(
                f"Serial connect failed ({e}); retrying in 2 s",
                throttle_duration_sec=10.0,
            )
            self.ser = None
            return False

    def _read_loop(self):
        import time as _time

        while self._running and rclpy.ok():
            if self.ser is None:
                if not self._connect():
                    _time.sleep(2.0)
                    continue
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}; reconnecting")
                self.ser = None
                continue

            if not line or line.startswith("["):
                continue
            parts = line.split("\t")
            if len(parts) < 16:
                continue
            try:
                vals = [float(p) for p in parts]
            except ValueError:
                continue
            self._publish_telemetry(vals)

    def _write(self, data: str):
        with self._lock:
            if self.ser is not None:
                try:
                    self.ser.write(data.encode("ascii"))
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial write failed: {e}")
                    self.ser = None

    # ==================================================
    #  Telemetry -> ROS topics
    # ==================================================
    def _publish_telemetry(self, vals):
        now = self.get_clock().now()
        stamp = now.to_msg()
        pitch_deg = vals[0]

        # Pitch
        self.pub_pitch.publish(Float32(data=pitch_deg))

        # IMU
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        half = pitch_deg * DEG / 2.0
        imu.orientation.w = math.cos(half)
        imu.orientation.y = math.sin(half)  # pitch about Y axis
        imu.angular_velocity.x = vals[5] * DEG
        imu.angular_velocity.y = vals[6] * DEG
        imu.angular_velocity.z = vals[7] * DEG
        imu.linear_acceleration.x = vals[2] * G
        imu.linear_acceleration.y = vals[3] * G
        imu.linear_acceleration.z = vals[4] * G
        self.pub_imu.publish(imu)

        # Magnetometer (raw, unitless from QMC5883L -> report as tesla-scaled)
        mag = MagneticField()
        mag.header.stamp = stamp
        mag.header.frame_id = "imu_link"
        mag.magnetic_field.x = vals[8] * 1e-6
        mag.magnetic_field.y = vals[9] * 1e-6
        mag.magnetic_field.z = vals[10] * 1e-6
        self.pub_mag.publish(mag)

        # Joint states + odometry (need extended packet)
        if len(vals) >= 20:
            l_deg, r_deg = vals[18], vals[19]

            js = JointState()
            js.header.stamp = stamp
            js.name = ["left_wheel_joint", "right_wheel_joint"]
            js.position = [l_deg * DEG, r_deg * DEG]
            self.pub_joints.publish(js)

            self._update_odom(l_deg, r_deg, now)

    def _update_odom(self, l_deg, r_deg, now):
        if self.prev_wheel is None:
            self.prev_wheel = (l_deg, r_deg)
            self.prev_stamp = now
            return

        dl = (l_deg - self.prev_wheel[0]) * DEG * self.wheel_radius
        dr = (r_deg - self.prev_wheel[1]) * DEG * self.wheel_radius
        dt = (now - self.prev_stamp).nanoseconds * 1e-9
        self.prev_wheel = (l_deg, r_deg)
        self.prev_stamp = now

        ds = (dl + dr) / 2.0
        dyaw = (dr - dl) / self.wheel_sep

        self.yaw += dyaw
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        if dt > 0:
            odom.twist.twist.linear.x = ds / dt
            odom.twist.twist.angular.z = dyaw / dt
        self.pub_odom.publish(odom)

    # ==================================================
    #  cmd_vel -> robot drive commands
    # ==================================================
    def on_cmd_vel(self, msg: Twist):
        # Map linear.x to a lean command and angular.z to steering,
        # using the robot's live-config protocol ($MDT sets drive tilt).
        lean = max(-1.0, min(1.0, msg.linear.x / self.max_linear)) * self.max_lean
        steer = max(-1.0, min(1.0, msg.angular.z / self.max_angular))

        # Target angle offset drives forward/backward.
        self._write(f"$T={lean:.3f}\n")
        # Steering uses the manual-drive turn channel.
        if steer > 0.1:
            self._write("#A")
        elif steer < -0.1:
            self._write("#D")
        else:
            self._write("# ")

    def destroy_node(self):
        self._running = False
        if self.ser is not None:
            # Safety: zero the target and stop turning on shutdown.
            self._write("$T=0\n")
            self._write("# ")
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PybotSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
