#!/bin/bash
# ============================================================
#  joint_states.sh — live wheel/joint state stream in terminal
# ============================================================
#  Usage: ./joint_states.sh [topic]
#    default topic: /pybot/joint_states
#    other useful:  /pybot/pitch  /pybot/state  /pybot/odom  /joy  /cmd_vel
# ============================================================
TOPIC="${1:-/pybot/joint_states}"
DIR="$(dirname "$0")"
if ! "$DIR/ros_shell.sh" "ros2 topic list | grep -q '^${TOPIC}$'"; then
    echo "[joint_states] topic $TOPIC not found."
    echo "Run ./start_stack.sh up and verify /pybot/joint_states is publishing."
    exit 1
fi
exec "$DIR/ros_shell.sh" "ros2 topic echo $TOPIC"
