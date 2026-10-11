"""Hold CP210x RTS/ESP32 EN until console, camera and hotspot are ready.

Started early by systemd. It notifies systemd once reset is asserted, so the
console can start concurrently and report readiness without a serial connection.
"""
import json
import os
import socket
import time
import urllib.error
import urllib.request
from pathlib import Path

import serial


def notify(message):
    address = os.environ.get("NOTIFY_SOCKET")
    if address:
        if address.startswith("@"):
            address = "\0" + address[1:]
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.connect(address)
            sock.sendall(message.encode())


def main():
    port = os.environ.get("BOT_SERIAL", "/dev/ttyUSB0")
    state_dir = Path(os.environ.get("BOT_BOOT_STATE", "/run/2pybot-boot"))
    endpoint = os.environ.get("BOT_BOOT_READY_URL", "http://127.0.0.1:8080/api/ready")
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.exclusive = port, 460800, 0, True
    s.dtr = False  # GPIO0 high: normal boot, never download mode
    s.rts = True   # standard ESP32 auto-reset circuit: EN held low
    s.open()
    (state_dir / "ready.json").write_text(json.dumps({"ready": False, "reset_held": True}) + "\n")
    print(f"ESP32 reset held on {port}; waiting for {endpoint}", flush=True)
    notify("READY=1\nSTATUS=ESP32 held in reset; waiting for Radxa camera/hotspot")
    started = time.monotonic()
    try:
        while True:
            try:
                with urllib.request.urlopen(endpoint, timeout=1) as response:
                    ready = json.load(response)
                if ready.get("ready") is True:
                    s.rts = False
                    s.close()
                    record = {"ready": True, "wait_seconds": round(time.monotonic() - started, 3),
                              "boot_ready_seconds": float(Path("/proc/uptime").read_text().split()[0]),
                              "conditions": ready}
                    (state_dir / "ready.json").write_text(json.dumps(record) + "\n")
                    print(f"Radxa ready; ESP32 reset released: {record}", flush=True)
                    notify("STATUS=Radxa ready; ESP32 reset released")
                    return
            except (OSError, ValueError, urllib.error.URLError):
                pass
            time.sleep(0.2)
    finally:
        if s.is_open:
            # USB ownership ends here; BOOTWAIT still prevents premature arming.
            s.close()


if __name__ == "__main__":
    main()
