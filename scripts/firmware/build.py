"""Compile BaseLink and create the immutable application manifest; optionally publish.

python scripts/firmware/build.py --arduino-cli /path/to/arduino-cli --publish
Only successful compilation/validated application images can replace latest.
"""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"software/radxa/console"))
from console.firmware import inspect_image
from console.versioning import validate_version


def manifest(directory):
    version=(ROOT/"VERSION").read_text().strip();validate_version(version)
    directory=Path(directory);details=inspect_image(directory/"BaseLink.ino.bin")
    try:
        commit=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip()
        dirty=bool(subprocess.check_output(["git","status","--porcelain"],cwd=ROOT,text=True))
    except subprocess.CalledProcessError:commit="";dirty=True
    payload=dict(details,build_id=details["sha256"],schema=1,version=version,
                 built_at=datetime.now(timezone.utc).isoformat(),source_commit=commit,source_dirty=dirty,
                 fqbn="esp32-bluepad32:esp32:esp32:PartitionScheme=huge_app",app_offset=0x10000,partition_bytes=0x300000)
    (directory/"manifest.json").write_text(json.dumps(payload,indent=2)+"\n")
    return payload


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arduino-cli",default="arduino-cli")
    parser.add_argument("--output-dir",type=Path,default=ROOT/"build/BaseLink")
    parser.add_argument("--publish",action="store_true");parser.add_argument("--host",default="2pybot")
    parser.add_argument("--user",default="radxa");parser.add_argument("--address")
    parser.add_argument("--password-stdin",action="store_true")
    args=parser.parse_args();assert ROOT.is_dir();validate_version((ROOT/"VERSION").read_text().strip())
    args.output_dir.mkdir(parents=True,exist_ok=True)
    subprocess.run([args.arduino_cli,"compile","--jobs","2","--fqbn","esp32-bluepad32:esp32:esp32:PartitionScheme=huge_app",
                    "--warnings","default","--output-dir",str(args.output_dir),str(ROOT/"firmware/BaseLink")],check=True)
    built=manifest(args.output_dir);print(json.dumps(built,indent=2))
    if args.publish:
        from publish import publish
        publish(args.output_dir,args.host,args.user,args.address,args.password_stdin)
