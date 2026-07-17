#!/bin/bash
# ============================================================
#  monitor_esp32.sh — raw serial monitor on the ESP32 (tio)
# ============================================================
#  NOTE: in micro-ROS serial mode the port carries the binary
#  XRCE-DDS stream (unreadable). Stop the agent first; this
#  script does that for you and restarts it on exit.
#  Quit tio with: Ctrl-t q
# ============================================================
PORT="${1:-/dev/ttyUSB0}"

AGENT_WAS_RUNNING=0
if docker ps --format '{{.Names}}' | grep -q 2pybot_microros_agent; then
    AGENT_WAS_RUNNING=1
    docker stop 2pybot_microros_agent
fi

tio "$PORT" -b 115200

if [ "$AGENT_WAS_RUNNING" = "1" ]; then
    docker start 2pybot_microros_agent
fi
