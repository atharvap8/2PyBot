"""PLACEHOLDER launch for 2PyBot lidar SLAM.

Planned pipeline:
    lidar driver (TBD: RPLIDAR / LD19 / etc.)
      -> /scan
    slam_toolbox (online_async)
      <- /pybot/odom  (wheel odometry via micro-ROS from ESP32)
      -> /map, TF map->odom

TODO:
  - pick lidar hardware and add its driver node here
  - add slam_toolbox params yaml (online_async config)
  - static TF base_link -> laser mount transform
  - EKF fusing IMU pitch/yaw with wheel odom (robot_localization)
"""

from launch import LaunchDescription


def generate_launch_description():
    # Intentionally empty: hardware not selected yet.
    return LaunchDescription([])
