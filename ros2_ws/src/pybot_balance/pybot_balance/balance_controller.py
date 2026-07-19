"""EXPERIMENTAL: full balance control law in ROS 2.

Ported from the on-chip implementation (firmware/BaseLink, tag
checkpoint-esp32-balance). The ESP32 runs firmware/BaseLinkSlave and
only streams IMU/encoder state and applies wheel speed commands.

Pipeline (target 200 Hz, timer-driven):
    /pybot/imu_state  [pitch, rate, yaw, encL, encR, dt]
        -> balance PID (+ manual lean from /cmd_vel)
        -> steering mix from /cmd_vel angular.z
    /pybot/wheel_cmd  [speedL, speedR]  (steps/s)

HONEST WARNING: a Linux/DDS/Python round-trip adds latency and jitter
that a 200 Hz balance loop will feel. This experiment measures whether
USB serial (~1-3 ms RTT) is tight enough. If the robot oscillates in a
way tuning can't fix, that's transport-induced phase lag — revert to
on-chip balancing (git checkout checkpoint-esp32-balance).

Gains are the proven on-chip defaults from config.h.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray


class BalanceController(Node):
    def __init__(self):
        super().__init__("pybot_balance_controller")

        # ---- Parameters (defaults = on-chip values) ----
        self.declare_parameter("kp", 1250.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 1.786)
        self.declare_parameter("target_angle", 0.071)   # equilibrium (deg)
        self.declare_parameter("output_limit", 20000.0)  # steps/s
        self.declare_parameter("integral_limit", 1500.0)
        self.declare_parameter("d_filter_alpha", 0.3)
        self.declare_parameter("accel_limit", 30000.0)   # steps/s^2
        self.declare_parameter("max_lean_deg", 3.0)      # /cmd_vel full lean
        self.declare_parameter("max_linear", 0.5)        # m/s
        self.declare_parameter("steer_gain", 4000.0)     # steps/s per rad/s norm
        self.declare_parameter("max_angular", 2.0)       # rad/s
        self.declare_parameter("state_timeout", 0.1)     # s; stop if IMU stale

        gp = self.get_parameter
        self.kp = gp("kp").value
        self.ki = gp("ki").value
        self.kd = gp("kd").value
        self.target = gp("target_angle").value
        self.out_lim = gp("output_limit").value
        self.i_lim = gp("integral_limit").value
        self.d_alpha = gp("d_filter_alpha").value
        self.accel_lim = gp("accel_limit").value
        self.max_lean = gp("max_lean_deg").value
        self.max_linear = gp("max_linear").value
        self.steer_gain = gp("steer_gain").value
        self.max_angular = gp("max_angular").value
        self.state_timeout = gp("state_timeout").value

        # ---- PID state ----
        self.integral = 0.0
        self.prev_pitch = None
        self.d_filtered = 0.0
        self.smoothed_out = 0.0

        # ---- Inputs ----
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.last_state_time = None
        self.lean_cmd = 0.0
        self.steer_cmd = 0.0
        self.enabled = False

        best_effort = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.pub_wheel = self.create_publisher(
            Float32MultiArray, "pybot/wheel_cmd", best_effort
        )
        self.create_subscription(
            Float32MultiArray, "pybot/imu_state", self.on_state, best_effort
        )
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Bool, "pybot/enable", self.on_enable, 10)

        self.get_logger().warn(
            "EXPERIMENTAL ROS-side balancing active — keep a hand on the kill "
            "switch. Revert: git checkout checkpoint-esp32-balance"
        )

    # --------------------------------------------------
    def on_enable(self, msg: Bool):
        self.enabled = msg.data
        if msg.data:
            self.integral = 0.0
            self.prev_pitch = None
            self.d_filtered = 0.0
            self.smoothed_out = 0.0
            self.get_logger().info("Enabled — PID reset")

    def on_cmd_vel(self, msg: Twist):
        lin = max(-1.0, min(1.0, msg.linear.x / self.max_linear))
        ang = max(-1.0, min(1.0, msg.angular.z / self.max_angular))
        self.lean_cmd = -lin * self.max_lean   # forward = negative lean
        self.steer_cmd = ang

    # --------------------------------------------------
    def on_state(self, msg: Float32MultiArray):
        """Control step runs on every IMU sample (ESP32-paced, 200 Hz)."""
        if len(msg.data) < 6:
            return
        now = self.get_clock().now()
        self.pitch = msg.data[0]
        self.pitch_rate = msg.data[1]
        dt = msg.data[5] if msg.data[5] > 1e-4 else 0.005
        self.last_state_time = now

        if not self.enabled:
            return

        # ---- PID (mirrors firmware pid_controller.h) ----
        setpoint = self.target + self.lean_cmd
        error = setpoint - self.pitch

        self.integral += error * dt * self.ki
        self.integral = max(-self.i_lim, min(self.i_lim, self.integral))

        if self.prev_pitch is None:
            d_raw = 0.0
        else:
            # Derivative on measurement (avoids setpoint kick).
            d_raw = -(self.pitch - self.prev_pitch) / dt
        self.prev_pitch = self.pitch
        self.d_filtered += self.d_alpha * (d_raw - self.d_filtered)

        output = self.kp * error + self.integral + self.kd * self.d_filtered
        output = max(-self.out_lim, min(self.out_lim, output))

        # ---- Accel limiting (prevents stepper stall) ----
        max_delta = self.accel_lim * dt
        delta = output - self.smoothed_out
        if delta > max_delta:
            self.smoothed_out += max_delta
        elif delta < -max_delta:
            self.smoothed_out -= max_delta
        else:
            self.smoothed_out = output

        # ---- Steering mix ----
        yaw_split = self.steer_cmd * self.steer_gain

        cmd = Float32MultiArray()
        cmd.data = [
            float(self.smoothed_out - yaw_split),
            float(self.smoothed_out + yaw_split),
        ]
        self.pub_wheel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Zero the wheels on shutdown.
        stop = Float32MultiArray()
        stop.data = [0.0, 0.0]
        node.pub_wheel.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
