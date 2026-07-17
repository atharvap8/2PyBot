#!/bin/bash
# ============================================================
#  start_stack.sh — bring up the 2PyBot ROS 2 stack
# ============================================================
#  Usage: ./start_stack.sh [up|down|restart|status|logs]
# ============================================================
set -e

COMPOSE="docker compose -f $HOME/2pybot/ros2_ws/docker/docker-compose.yaml"
CMD="${1:-up}"

case "$CMD" in
    up)
        $COMPOSE up -d teleop microros-agent
        echo "--- containers ---"
        docker ps --filter name=2pybot --format 'table {{.Names}}\t{{.Status}}'
        ;;
    down)     $COMPOSE down ;;
    restart)  $COMPOSE restart ;;
    status)
        docker ps -a --filter name=2pybot --format 'table {{.Names}}\t{{.Status}}'
        ;;
    logs)     docker logs -f --tail 50 "${2:-2pybot_teleop}" ;;
    *)        echo "usage: $0 [up|down|restart|status|logs <container>]"; exit 1 ;;
esac
