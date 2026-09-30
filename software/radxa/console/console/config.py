"""Console-wide settings. Robot dynamics live in the FIRMWARE parameter
store (63 of them) and are edited through the GUI — nothing here."""
import os

SERIAL_PORT   = os.environ.get("BOT_SERIAL", "/dev/ttyUSB0")
SERIAL_BAUD   = int(os.environ.get("BOT_BAUD", "460800"))
HTTP_PORT     = int(os.environ.get("BOT_PORT", "8080"))

ENCODER_CPR   = 4096          # MT6816 with 4x quadrature decode
TRACK_WIDTH_M = 0.155         # only used for map dead-reckoning

CAMERA_DEV    = os.environ.get("BOT_CAM", "/dev/video0")
SNAPSHOT_PATH = "/dev/shm/2pybot_snap.jpg"
CAM_PROFILES  = "/var/lib/2pybot/camera_profiles.json"

MEDIAMTX_API  = "http://127.0.0.1:9997"
STREAM_NAME   = "cam"
WHEP_URL      = f"/{STREAM_NAME}/whep"   # served by MediaMTX on :8889
