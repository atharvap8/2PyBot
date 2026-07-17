#!/bin/bash
# ============================================================
#  foxglove.sh — start the Foxglove bridge for remote viz
# ============================================================
#  Connect from any PC: Foxglove Studio -> ws://<radxa-ip>:8765
#  Gives 3D view (TF/URDF), plots, topic inspector — the
#  practical replacement for RViz when the robot is headless.
# ============================================================
set -e

# Install the bridge in the container on first run.
docker exec 2pybot_teleop bash -c \
    "dpkg -s ros-humble-foxglove-bridge >/dev/null 2>&1 || \
     (apt-get update && apt-get install -y ros-humble-foxglove-bridge)"

echo "[foxglove] Bridge starting on ws://$(hostname -I | awk '{print $1}'):8765"
docker exec -it 2pybot_teleop bash -c \
    "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
     ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
