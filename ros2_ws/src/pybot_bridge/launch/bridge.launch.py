"""Launch the 2PyBot serial bridge with robot_state_publisher.

Usage:
    ros2 launch pybot_bridge bridge.launch.py port:=/dev/rfcomm0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pybot_bridge")
    urdf_path = os.path.join(pkg_share, "urdf", "pybot.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/rfcomm0",
        description="Serial port of the robot Bluetooth link",
    )

    bridge = Node(
        package="pybot_bridge",
        executable="serial_bridge",
        name="pybot_serial_bridge",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "bridge.yaml"),
            {"port": LaunchConfiguration("port")},
        ],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[("joint_states", "pybot/joint_states")],
    )

    return LaunchDescription([port_arg, bridge, rsp])
