"""Check a compiled BaseLink build against the bot's installed flash layout.

The bot's ESP32 uses the Arduino "Huge APP" table: one application partition
at 0x10000 of 0x300000 bytes (3 MB), no second OTA slot. Every flasher on the
bot (console firmware worker, `bot restore`) reads the live partition table and
refuses any other layout, then writes only the application at 0x10000.

This check applies the same rule at build time, so CI and local builds fail
instead of the bot:
  1. the compiled partition table has exactly that one application partition;
  2. the application image fits it (a warning is printed when it is nearly full).

    python scripts/firmware/check_partitions.py build/BaseLink
"""
import argparse
import os
from pathlib import Path
import struct
import sys

APP_OFFSET = 0x10000
APP_BYTES = 0x300000
WARN_FRACTION = 0.90

_ENTRY = struct.Struct("<HBBII16sI")
_MAGIC = 0x50AA            # bytes AA 50
_TYPE_APP = 0


def app_partitions(table):
    """[(offset, size, label)] for every application entry in a partition table."""
    apps = []
    for at in range(0, len(table) - _ENTRY.size + 1, _ENTRY.size):
        magic, kind, _subtype, offset, size, label, _flags = _ENTRY.unpack_from(table, at)
        if magic != _MAGIC:
            break
        if kind == _TYPE_APP:
            apps.append((offset, size, label.split(b"\0", 1)[0].decode("ascii", "replace")))
    return apps


def check(directory, sketch="BaseLink.ino"):
    """Raise ValueError if the build does not match the bot's layout; return a report."""
    directory = Path(directory)
    table_path = directory / (sketch + ".partitions.bin")
    image_path = directory / (sketch + ".bin")
    if not table_path.is_file():
        raise ValueError("Missing compiled partition table: " + str(table_path))
    if not image_path.is_file():
        raise ValueError("Missing application image: " + str(image_path))
    apps = app_partitions(table_path.read_bytes())
    if [(offset, size) for offset, size, _ in apps] != [(APP_OFFSET, APP_BYTES)]:
        found = ", ".join("%s@0x%X+0x%X" % (label, offset, size) for offset, size, label in apps) or "none"
        raise ValueError("Partition table does not match the bot (needs one app at 0x%X of 0x%X bytes, "
                         "Huge APP); found: %s" % (APP_OFFSET, APP_BYTES, found))
    size = image_path.stat().st_size
    if size > APP_BYTES:
        raise ValueError("Application is %d bytes; the bot's slot at 0x%X holds %d (over by %d)"
                         % (size, APP_OFFSET, APP_BYTES, size - APP_BYTES))
    return {"size_bytes": size, "slot_bytes": APP_BYTES, "used": size / APP_BYTES,
            "headroom_bytes": APP_BYTES - size}


def _annotate(level, text):
    if os.environ.get("GITHUB_ACTIONS") == "true":
        print("::%s::%s" % (level, text))
    else:
        print("%s: %s" % (level.upper(), text), file=sys.stderr)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("directory", type=Path, help="arduino-cli --output-dir of the BaseLink build")
    parser.add_argument("--warn", type=float, default=WARN_FRACTION, help="warn above this fraction of the slot")
    args = parser.parse_args(argv)
    try:
        report = check(args.directory)
    except ValueError as error:
        _annotate("error", str(error))
        return 1
    print("BaseLink application %d / %d bytes (%.1f%%) at 0x%X, %d bytes free"
          % (report["size_bytes"], report["slot_bytes"], 100 * report["used"], APP_OFFSET,
             report["headroom_bytes"]))
    if report["used"] > args.warn:
        _annotate("warning", "BaseLink uses %.1f%% of the bot's 3 MB application slot" % (100 * report["used"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
