#!/bin/bash
# ============================================================
#  ros_shell.sh — sourced interactive shell inside the container
# ============================================================
#  Usage: ./ros_shell.sh [command...]
#    no args -> interactive bash with ROS sourced
#    args    -> run a single ros2 command, e.g.:
#               ./ros_shell.sh ros2 topic list
# ============================================================
if [ $# -eq 0 ]; then
    docker exec -it 2pybot_teleop bash -c \
        "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec bash"
else
    docker exec -it 2pybot_teleop bash -c \
        "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $*"
fi
