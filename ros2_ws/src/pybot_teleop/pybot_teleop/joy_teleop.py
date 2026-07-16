"""EvoFox One S joystick teleop node for 2PyBot.

Subscribes to /joy (from the standard `joy` node reading the gamepad
paired over Bluetooth) and publishes /cmd_vel with deadzone + cubic
expo shaping, mirroring the feel of the old ESP32 bridge firmware.

Button scheme (X-Input mapping, EvoFox One S in BT mode):
    A (button 0)  -> toggle motor enable (edge-triggered)
    B (button 1)  -> instant kill (enable off)
    Left stick Y  -> linear.x (forward/backward)
    Right stick X -> angular.z (steering)

Enable state is published on /pybot/enable (std_msgs/Bool) so the
serial/micro-ROS bridge can arm or disarm the ESP32.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyTeleop(Node):
    def __init__(self):
        super().__init__("pybot_joy_teleop")

        self.declare_parameter("axis_linear", 1)      # left stick Y
        self.declare_parameter("axis_angular", 3)     # right stick X
        self.declare_parameter("button_enable", 0)    # A
        self.declare_parameter("button_kill", 1)      # B
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("expo", 0.35)
        self.declare_parameter("max_linear", 0.5)     # m/s
        self.declare_parameter("max_angular", 2.0)    # rad/s

        self.axis_lin = self.get_parameter("axis_linear").value
        self.axis_ang = self.get_parameter("axis_angular").value
        self.btn_enable = self.get_parameter("button_enable").value
        self.btn_kill = self.get_parameter("button_kill").value
        self.deadzone = self.get_parameter("deadzone").value
        self.expo = self.get_parameter("expo").value
        self.max_lin = self.get_parameter("max_linear").value
        self.max_ang = self.get_parameter("max_angular").value

        self.enabled = False
        self._last_enable_btn = 0

        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_enable = self.create_publisher(Bool, "pybot/enable", 10)
        self.create_subscription(Joy, "joy", self.on_joy, 10)

        # Watchdog: zero cmd_vel if joy messages stop arriving.
        self._last_joy_time = self.get_clock().now()
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info("pybot_joy_teleop ready — pair the EvoFox One S")

    def _shape(self, raw: float) -> float:
        """Deadzone + rescale + cubic expo."""
        if abs(raw) < self.deadzone:
            return 0.0
        sign = 1.0 if raw > 0 else -1.0
        x = (abs(raw) - self.deadzone) / (1.0 - self.deadzone)
        return sign * ((1.0 - self.expo) * x + self.expo * x ** 3)

    def on_joy(self, msg: Joy):
        self._last_joy_time = self.get_clock().now()

        # Enable toggle (edge) and kill.
        enable_btn = msg.buttons[self.btn_enable] if len(msg.buttons) > self.btn_enable else 0
        kill_btn = msg.buttons[self.btn_kill] if len(msg.buttons) > self.btn_kill else 0

        if enable_btn and not self._last_enable_btn:
            self.enabled = not self.enabled
            self.get_logger().info(f"Motors {'ENABLED' if self.enabled else 'DISABLED'}")
            self.pub_enable.publish(Bool(data=self.enabled))
        self._last_enable_btn = enable_btn

        if kill_btn and self.enabled:
            self.enabled = False
            self.get_logger().warn("KILL pressed — motors disabled")
            self.pub_enable.publish(Bool(data=False))

        cmd = Twist()
        if self.enabled:
            lin_raw = msg.axes[self.axis_lin] if len(msg.axes) > self.axis_lin else 0.0
            ang_raw = msg.axes[self.axis_ang] if len(msg.axes) > self.axis_ang else 0.0
            cmd.linear.x = self._shape(lin_raw) * self.max_lin
            cmd.angular.z = self._shape(ang_raw) * self.max_ang
        self.pub_cmd.publish(cmd)

    def _watchdog(self):
        """Publish zero velocity if the gamepad link is lost."""
        age = (self.get_clock().now() - self._last_joy_time).nanoseconds * 1e-9
        if age > 0.5 and self.enabled:
            self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
