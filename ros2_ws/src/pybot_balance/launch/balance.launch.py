"""Launch the EXPERIMENTAL ROS-side balance stack.

Prereqs:
  - ESP32 flashed with firmware/BaseLinkSlave
  - micro-ROS agent running (docker compose microros-agent)

Usage:
    ros2 launch pybot_balance balance.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    balance_cfg = os.path.join(
        get_package_share_directory("pybot_balance"), "config", "balance.yaml"
    )
    teleop_cfg = os.path.join(
        get_package_share_directory("pybot_teleop"), "config", "teleop.yaml"
    )

    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[teleop_cfg],
        ),
        Node(
            package="pybot_teleop",
            executable="joy_teleop",
            name="pybot_joy_teleop",
            output="screen",
            parameters=[teleop_cfg],
        ),
        Node(
            package="pybot_balance",
            executable="balance_controller",
            name="pybot_balance_controller",
            output="screen",
            parameters=[balance_cfg],
        ),
    ])
