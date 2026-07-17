#!/bin/bash
# ============================================================
#  pair_evofox.sh — pair/reconnect the EvoFox One S over BT
# ============================================================
#  Usage:
#    ./pair_evofox.sh scan          # find the controller MAC
#    ./pair_evofox.sh pair <MAC>    # first-time pairing
#    ./pair_evofox.sh connect <MAC> # reconnect a known pad
#    ./pair_evofox.sh status
#  Put the pad in pairing mode first (HOME+Y, LED double-blink).
# ============================================================
CMD="${1:-status}"
MAC="$2"

case "$CMD" in
    scan)
        echo "[bt] scanning 15 s — watch for 'EvoFox' ..."
        bluetoothctl --timeout 15 scan on
        bluetoothctl devices
        ;;
    pair)
        [ -z "$MAC" ] && { echo "usage: $0 pair <MAC>"; exit 1; }
        bluetoothctl pair "$MAC"
        bluetoothctl trust "$MAC"
        bluetoothctl connect "$MAC"
        ;;
    connect)
        [ -z "$MAC" ] && { echo "usage: $0 connect <MAC>"; exit 1; }
        bluetoothctl connect "$MAC"
        ;;
    status)
        bluetoothctl devices Connected
        ls -l /dev/input/js* 2>/dev/null || echo "no joystick device yet"
        ;;
    *) echo "usage: $0 [scan|pair <MAC>|connect <MAC>|status]"; exit 1 ;;
esac
