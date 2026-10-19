"""Publish a successful compile directory to the bot's latest-build registry.

Uses SSH known-host verification. Password is read from stdin/getpass, never
saved. No arbitrary image/command upload endpoint is exposed by the bot API.
"""
import argparse
import getpass
import json
from pathlib import Path
import shlex
import socket
import sys
import uuid


def publish(directory,host="2pybot",user="radxa",address=None,password_stdin=False,installed_candidate=False):
    import paramiko
    directory=Path(directory);manifest=json.loads((directory/"manifest.json").read_text())
    image=directory/"BaseLink.ino.bin"
    if not image.is_file():image=directory/"application.bin"
    assert image.is_file()
    password=sys.stdin.readline().rstrip("\r\n") if password_stdin else getpass.getpass("Bot SSH password: ")
    client=paramiko.SSHClient();client.load_system_host_keys();client.set_missing_host_key_policy(paramiko.RejectPolicy())
    sock=socket.create_connection((address,22),timeout=15) if address else None
    client.connect(host,username=user,password=password,look_for_keys=False,allow_agent=False,sock=sock,timeout=15)
    try:
        incoming="/var/lib/2pybot/firmware/incoming/"+uuid.uuid4().hex
        with client.open_sftp() as sftp:
            sftp.stat("/var/lib/2pybot/firmware/incoming");sftp.mkdir(incoming)
            sftp.put(str(image),incoming+"/application.bin")
            sftp.put(str(directory/"manifest.json"),incoming+"/manifest.json")
        command="python3 /home/"+user+"/projects/2PyBot/scripts/firmware/register_build.py "+shlex.quote(incoming)
        if installed_candidate:command+=" --installed-candidate"
        _,stdout,stderr=client.exec_command(command,timeout=30)
        output=stdout.read().decode();error=stderr.read().decode();code=stdout.channel.recv_exit_status()
        if code:raise RuntimeError(error or output)
        print(output.strip())
    finally:client.close()
    return manifest


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory",type=Path)
    parser.add_argument("--host",default="2pybot");parser.add_argument("--user",default="radxa")
    parser.add_argument("--address");parser.add_argument("--password-stdin",action="store_true")
    parser.add_argument("--installed-candidate",action="store_true")
    args=parser.parse_args();publish(args.directory,args.host,args.user,args.address,args.password_stdin,args.installed_candidate)
