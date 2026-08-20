#!/bin/bash
# One-shot agent-driven pairing for the EvoFox One S (Xbox BT mode).
# Finds the pad by name during the scan — its MAC changes between
# pairing attempts (BLE privacy), so never hardcode it.
echo 1 | sudo tee /sys/module/bluetooth/parameters/disable_ertm >/dev/null

# Clean any stale bondings of previous Xbox pads.
for OLD in $(bluetoothctl devices | grep -i xbox | cut -d" " -f2); do
    bluetoothctl remove "$OLD" >/dev/null
done

echo "[pair] scanning 25 s — pad must be blinking..."
bluetoothctl --timeout 25 scan on >/dev/null 2>&1

MAC=$(bluetoothctl devices | grep -i xbox | head -1 | cut -d" " -f2)
if [ -z "$MAC" ]; then
    echo "[pair] NOT FOUND — is the pad still double-blinking?"
    exit 1
fi
echo "[pair] found $MAC — pairing..."

(
  echo "agent NoInputNoOutput"
  echo "default-agent"
  echo "pairable on"
  echo "pair $MAC"
  sleep 12
  echo "trust $MAC"
  sleep 2
  echo "connect $MAC"
  sleep 8
  echo "quit"
) | bluetoothctl | grep -E "Pairing|Paired|Trusted|Connect|Failed"

sleep 2
echo "=== status ==="
bluetoothctl info "$MAC" | grep -E "Name|Paired|Trusted|Connected"
echo "=== input devices ==="
grep -B1 -A4 -i xbox /proc/bus/input/devices || echo NO_INPUT_DEVICE
