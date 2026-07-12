# 2PyBot ROS 2 Workspace

ROS 2 workspace for the 2PyBot self-balancing robot. Tested against
ROS 2 Humble/Jazzy on Ubuntu; on Windows use WSL2 or a native ROS 2
installation.

## Package: `pybot_bridge`

Bridges the robot's Bluetooth serial telemetry into ROS 2:

| Topic                 | Type                        | Direction |
|-----------------------|-----------------------------|-----------|
| `/pybot/imu`          | `sensor_msgs/Imu`           | publish   |
| `/pybot/mag`          | `sensor_msgs/MagneticField` | publish   |
| `/pybot/pitch`        | `std_msgs/Float32`          | publish   |
| `/pybot/joint_states` | `sensor_msgs/JointState`    | publish   |
| `/pybot/odom`         | `nav_msgs/Odometry`         | publish   |
| `/cmd_vel`            | `geometry_msgs/Twist`       | subscribe |

Also ships a simplified URDF (`urdf/pybot.urdf`) matching the CAD model
in `models/2pybot_simplified.step`, wired to `robot_state_publisher` so
the robot renders live in RViz with wheel rotation from encoder data.

## Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Bind the robot's Bluetooth serial first (Linux):
#   sudo rfcomm bind /dev/rfcomm0 <ROBOT_BT_MAC>
ros2 launch pybot_bridge bridge.launch.py port:=/dev/rfcomm0
```

Visualize in RViz:

```bash
rviz2   # add RobotModel (topic: /robot_description), TF, Odometry
```

Drive with a keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Parameters (`config/bridge.yaml`)

- `port` — serial port (default `/dev/rfcomm0`, launch-arg overridable)
- `wheel_radius`, `wheel_separation` — odometry geometry
- `max_linear` / `max_lean_deg` — maps `cmd_vel` linear.x to lean angle
- `max_angular` — maps `cmd_vel` angular.z to steering
