#!/bin/bash
# ============================================================
#  flash_esp32.sh — compile + upload BaseLink from the Radxa
# ============================================================
#  Usage: ./flash_esp32.sh [sketch_dir] [port]
#    default sketch: ~/2pybot/firmware/BaseLink
#    default port:   /dev/ttyUSB0
#
#  Stops the micro-ROS agent container first (it owns the USB
#  port), flashes, then restarts it.
# ============================================================
set -e

SKETCH="${1:-$HOME/2pybot/firmware/BaseLink}"
PORT="${2:-/dev/ttyUSB0}"
CLI="$HOME/.local/bin/arduino-cli"
# UploadSpeed=115200: 921600 and 460800 are unstable on the Radxa
# USB host (esptool dies after the baud-change handshake).
FQBN="esp32:esp32:esp32:PartitionScheme=huge_app,UploadSpeed=115200"

AGENT_WAS_RUNNING=0
if docker ps --format '{{.Names}}' | grep -q 2pybot_microros_agent; then
    AGENT_WAS_RUNNING=1
    echo "[flash] Stopping micro-ROS agent (owns $PORT)..."
    docker stop 2pybot_microros_agent
fi

echo "[flash] Compiling $SKETCH ..."
"$CLI" compile --fqbn "$FQBN" --warnings default "$SKETCH"

echo "[flash] Uploading to $PORT ..."
"$CLI" upload -p "$PORT" --fqbn "$FQBN" "$SKETCH"

if [ "$AGENT_WAS_RUNNING" = "1" ]; then
    echo "[flash] Restarting micro-ROS agent..."
    sleep 2   # let the ESP32 finish booting first
    docker start 2pybot_microros_agent
fi

echo "[flash] Done."
