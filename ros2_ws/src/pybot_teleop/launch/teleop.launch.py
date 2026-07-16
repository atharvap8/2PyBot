"""Launch EvoFox One S teleop: joy driver + shaping node.

Usage (on the Radxa, gamepad already paired via bluetoothctl):
    ros2 launch pybot_teleop teleop.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("pybot_teleop"), "config", "teleop.yaml"
    )

    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="pybot_teleop",
            executable="joy_teleop",
            name="pybot_joy_teleop",
            output="screen",
            parameters=[config],
        ),
    ])
