# ============================================================
#  2PyBot ROS 2 image for Radxa Cubie A7A (arm64, Debian 11 host)
# ============================================================
#  Runs the robot-side ROS 2 stack:
#    - pybot_teleop  (EvoFox One S joy -> cmd_vel)
#    - pybot_bridge  (serial telemetry bridge, legacy path)
#    - micro-ROS agent (ESP32 client transport)
#
#  Build (on the Radxa):
#    docker build -t 2pybot:latest -f docker/Dockerfile .
# ============================================================

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ---- System + ROS deps ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-serial \
        joystick \
        bluez \
        ros-humble-joy \
        ros-humble-teleop-twist-joy \
        ros-humble-robot-state-publisher \
        ros-humble-micro-ros-msgs \
    && rm -rf /var/lib/apt/lists/*

# ---- micro-ROS agent (prebuilt) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-micro-ros-agent || true \
    && rm -rf /var/lib/apt/lists/*

# ---- Workspace ----
WORKDIR /ros2_ws
COPY src ./src

# NOTE: no --symlink-install — the install tree must be self-contained
# because build/ and log/ are deleted to slim the image.
RUN . /opt/ros/humble/setup.sh && \
    colcon build && \
    rm -rf build log

# ---- Entrypoint ----
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "pybot_teleop", "teleop.launch.py"]
