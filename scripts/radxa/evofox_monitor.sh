#!/bin/bash
# ============================================================
#  evofox_monitor.sh — live EvoFox controller data stream
# ============================================================
#  Usage: ./evofox_monitor.sh [raw|joy|cmd|hz]
#    raw  - host-level jstest on /dev/input/js0 (no ROS needed)
#    joy  - ros2 topic echo /joy        (default)
#    cmd  - ros2 topic echo /cmd_vel    (after shaping/enable)
#    hz   - message rate of /joy
# ============================================================
MODE="${1:-joy}"
DIR="$(dirname "$0")"

case "$MODE" in
    raw)  jstest /dev/input/js0 ;;
    joy)  exec "$DIR/ros_shell.sh" "ros2 topic echo /joy" ;;
    cmd)  exec "$DIR/ros_shell.sh" "ros2 topic echo /cmd_vel" ;;
    hz)   exec "$DIR/ros_shell.sh" "ros2 topic hz /joy" ;;
    *)    echo "usage: $0 [raw|joy|cmd|hz]"; exit 1 ;;
esac
