"""Install the independent firmware worker (run once with sudo on the bot)."""
import argparse
import os
from pathlib import Path
import pwd
import subprocess

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo",type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument("--user",default="radxa")
    parser.add_argument("--venv",type=Path,default=Path("/var/lib/2pybot-deploy/venv"))
    parser.add_argument("--flasher-python",default="/home/radxa/tools/2pybot-flasher/bin/python3")
    args=parser.parse_args();assert os.geteuid()==0,"Run installer with sudo"
    user=pwd.getpwnam(args.user)
    subprocess.run([args.flasher_python,"-c","import esptool,serial"],check=True)
    root=Path("/var/lib/2pybot/firmware");root.mkdir(parents=True,exist_ok=True)
    for directory in [root]+[root/name for name in ("builds","incoming","jobs")]:
        directory.mkdir(exist_ok=True);os.chown(directory,user.pw_uid,user.pw_gid)
    content=(args.repo/"scripts/radxa/systemd/2pybot-firmware.service").read_text()
    for old,new in (("__USER__",args.user),("__REPO__",str(args.repo)),("__VENV__",str(args.venv)),("__FLASHER_PYTHON__",args.flasher_python)):
        content=content.replace(old,new)
    Path("/etc/systemd/system/2pybot-firmware.service").write_text(content)
    subprocess.run(["systemctl","daemon-reload"],check=True)
    subprocess.run(["systemctl","enable","--now","2pybot-firmware.service"],check=True)
    print("Firmware worker installed; compiled builds can now be published")
