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
import re

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"software/radxa/console"))
from console.firmware import inspect_image
from console.versioning import validate_version


def toolchain_versions(cli,lock):
    core=json.loads(subprocess.check_output([cli,"core","list","--format","json"],text=True))
    libraries=json.loads(subprocess.check_output([cli,"lib","list","--format","json"],text=True))
    core_id,version=lock['core'].rsplit('@',1)
    installed={p['id']:p.get('installed_version',p.get('installed','')) for p in core.get('platforms',[])}
    if installed.get(core_id)!=version:raise ValueError('Install locked board core '+lock['core'])
    found={}
    for entry in libraries.get('installed_libraries',[]):
        lib=entry.get('library',entry);found[lib['name']]=lib.get('version')
    selected={}
    for item in lock['libraries']:
        name,wanted=item.rsplit('@',1)
        if found.get(name)!=wanted:raise ValueError('Install locked library '+item)
        selected[name]=wanted
    return {'core':lock['core'],'libraries':selected,'fqbn':lock['fqbn']}


def manifest(directory,source_commit=None,source_fingerprint=None,toolchain=None):
    version=(ROOT/"VERSION").read_text().strip();validate_version(version)
    directory=Path(directory);details=inspect_image(directory/"BaseLink.ino.bin")
    try:
        if source_commit:
            if not re.fullmatch(r"[a-f0-9]{40}",source_commit):raise ValueError("Invalid source commit")
            commit=source_commit;dirty=False
        else:
            commit=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip()
            dirty=bool(subprocess.check_output(["git","status","--porcelain"],cwd=ROOT,text=True))
    except subprocess.CalledProcessError:commit="";dirty=True
    payload=dict(details,build_id=details["sha256"],schema=1,version=version,
                 built_at=datetime.now(timezone.utc).isoformat(),source_commit=commit,source_dirty=dirty,
                 hardware="m0601-rs485",parameter_schema="bldc-v4",
                 fqbn=toolchain['fqbn'] if toolchain else "esp32-bluepad32:esp32:esp32:PartitionScheme=huge_app",app_offset=0x10000,partition_bytes=0x300000)
    if source_fingerprint:
        if not re.fullmatch(r"[a-f0-9]{64}",source_fingerprint):raise ValueError("Invalid source fingerprint")
        payload["source_fingerprint"]=source_fingerprint
    if toolchain:payload['toolchain']=toolchain
    (directory/"manifest.json").write_text(json.dumps(payload,indent=2)+"\n")
    return payload


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arduino-cli",default="arduino-cli")
    parser.add_argument("--output-dir",type=Path,default=ROOT/"build/BaseLink")
    parser.add_argument("--source-commit",help="Commit of an immutable git-archive deployment snapshot")
    parser.add_argument("--source-fingerprint",help="Firmware/toolchain input digest supplied by deployment")
    parser.add_argument("--require-toolchain-lock",action="store_true")
    parser.add_argument("--publish",action="store_true");parser.add_argument("--host",default="2pybot")
    parser.add_argument("--user",default="radxa");parser.add_argument("--address")
    parser.add_argument("--password-stdin",action="store_true")
    args=parser.parse_args();assert ROOT.is_dir();validate_version((ROOT/"VERSION").read_text().strip())
    locked=toolchain_versions(args.arduino_cli,json.loads((ROOT/'scripts/firmware/toolchain.json').read_text())) if args.require_toolchain_lock else None
    args.output_dir.mkdir(parents=True,exist_ok=True)
    subprocess.run([args.arduino_cli,"compile","--jobs","2","--fqbn",locked['fqbn'] if locked else "esp32-bluepad32:esp32:esp32:PartitionScheme=huge_app",
                    "--warnings","default","--output-dir",str(args.output_dir),str(ROOT/"firmware/BaseLink")],check=True)
    built=manifest(args.output_dir,args.source_commit,args.source_fingerprint,locked);print(json.dumps(built,indent=2))
    if args.publish:
        from publish import publish
        publish(args.output_dir,args.host,args.user,args.address,args.password_stdin)
