"""Host readiness is independent of serial, avoiding a reset/console deadlock."""
import asyncio
import os
import socket
import time
from pathlib import Path


def readiness(stream_status, snapshot_path, hotspot_address="10.42.0.1"):
    camera_ready = bool(stream_status.get("running") and
                        int(stream_status.get("metrics", {}).get("frame", 0)) > 0)
    try:
        camera_ready = camera_ready and 0 <= time.time() - Path(snapshot_path).stat().st_mtime < 5
    except OSError:
        camera_ready = False
    hotspot_ready = True
    if hotspot_address:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind((hotspot_address, 0))
        except OSError:
            hotspot_ready = False
    return {"ready": camera_ready and hotspot_ready,
            "http": True, "camera": camera_ready, "hotspot": hotspot_ready}


async def publish_ready(link, get_ready):
    while True:
        if (link.connected and link.derived.get("BOOTGATE") == 1 and
                not link.tel.get("host_ready") and get_ready()["ready"]):
            link.send("BR,1")  # release startup interlock; NEVER an enable request
        await asyncio.sleep(0.5)


HOTSPOT_ADDRESS = os.environ.get("BOT_BOOT_HOTSPOT", "10.42.0.1")
