#!/usr/bin/env python3
"""One-time Linux installation for managed Git pulls, deployment state and ring status."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import pwd
import shutil
import subprocess
import sys
import re
import shlex
import hashlib

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"software/radxa/console"))
from console.deployment_status import atomic,read


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo",type=Path,default=ROOT)
    parser.add_argument("--user",default="radxa")
    parser.add_argument("--state-dir",type=Path,default=Path("/var/lib/2pybot-deploy"))
    parser.add_argument("--run-dir",type=Path,default=Path("/run/2pybot-deploy"))
    parser.add_argument("--arduino-cli",default="arduino-cli")
    parser.add_argument("--provision-toolchain",action="store_true")
    parser.add_argument("--firmware-mode",choices=["artifact","build"],default="artifact")
    parser.add_argument("--bootstrap-host",action="store_true",help="Install versioned host once with controller USB disconnected")
    parser.add_argument("--led-backend",choices=["firmware","spi"],default="firmware")
    parser.add_argument("--spi-bus",type=int,default=0)
    parser.add_argument("--spi-device",type=int,default=0)
    parser.add_argument("--serial-port",default="/dev/ttyUSB0")
    args=parser.parse_args()
    if os.geteuid()!=0:parser.error("Run this one-time installer with sudo")
    args.repo=args.repo.resolve();assert (args.repo/".git").exists()
    account=pwd.getpwnam(args.user)
    wrapper=Path('/usr/local/bin/git')
    if wrapper.exists() and 'scripts/radxa/git_pull.py' not in wrapper.read_text(errors='replace'):
        parser.error('An existing /usr/local/bin/git is not our wrapper; it was left untouched')
    if args.bootstrap_host:
        if Path(args.serial_port).exists():parser.error('Disconnect controller USB for the one-time host bootstrap; reconnect after host setup')
        job=read('/var/lib/2pybot/firmware/current.json',{})
        if job.get('status') in {'preparing','queued','running','recovering','recovery_required'}:
            parser.error('Resolve the existing firmware job before bootstrapping')
    config_path=Path('/etc/2pybot-deploy.json')
    old=read(config_path,{})
    real_git=old.get('real_git',shutil.which('git'))
    if not real_git or Path(real_git)==Path('/usr/local/bin/git'):real_git='/usr/bin/git'
    remote=subprocess.check_output(["runuser","-u",args.user,"--",real_git,"-C",str(args.repo),"remote","get-url","origin"],text=True).strip()
    match=re.fullmatch(r'(?:https://github.com/|git@github.com:)([A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+?)(?:\.git)?',remote)
    if not match:parser.error('Configure the commit-bound bundle URL for this non-GitHub remote manually')
    branch=subprocess.check_output(["runuser","-u",args.user,"--",real_git,"-C",str(args.repo),"branch","--show-current"],text=True).strip()
    if not branch:parser.error('The managed checkout must be on a branch')
    config={"one_pull":True,"repo":str(args.repo),"branch":branch,"user":args.user,"real_git":real_git,"serial_port":args.serial_port,
        "state_dir":str(args.state_dir),"run_dir":str(args.run_dir),"wait_seconds":900,
        "api":"http://127.0.0.1:8080","auto_firmware":True,"firmware_mode":args.firmware_mode,
        "bundle_url":f"https://github.com/{match[1]}/releases/download/bot-{{commit}}/2pybot-deploy-{{commit}}.tar.gz",
        "bundle_api":f"https://api.github.com/repos/{match[1]}/releases/tags/bot-{{commit}}",
        "ci_status_api":f"https://api.github.com/repos/{match[1]}/actions/workflows/bot-bundle.yml/runs?head_sha={{commit}}&per_page=1",
        "arduino_cli":args.arduino_cli,"flasher_python":f"/home/{args.user}/tools/2pybot-flasher/bin/python3",
        "led":{"backend":args.led_backend,"count":16,"brightness":.35,"bus":args.spi_bus,"device":args.spi_device}}
    for directory in [args.state_dir,args.run_dir,args.state_dir/'incoming',args.state_dir/'firmware-cache',args.state_dir/'envs']:
        directory.mkdir(parents=True,exist_ok=True);os.chown(directory,account.pw_uid,account.pw_gid)
    args.run_dir.chmod(0o775)
    request=args.run_dir/'request';request.touch(exist_ok=True);os.chown(request,account.pw_uid,account.pw_gid)
    operation_lock=args.run_dir/'operation.lock';operation_lock.touch(exist_ok=True)
    os.chown(operation_lock,account.pw_uid,account.pw_gid);operation_lock.chmod(0o664)
    atomic(config_path,config)
    downloads=Path('/var/lib/2pybot/downloads');downloads.mkdir(parents=True,exist_ok=True)
    os.chown(downloads,account.pw_uid,account.pw_gid)
    old_downloads=[]
    for file in (args.repo/'software/radxa/console/ui').glob('2PyBot-v*-manifest.json'):
        data=read(file,{})
        version=data.get('version','')
        if not re.fullmatch(r'\d+\.\d+\.\d+',version):continue
        apk=file.with_name(f'2PyBot-v{version}-debug.apk')
        if not apk.is_file() or hashlib.sha256(apk.read_bytes()).hexdigest()!=data.get('sha256'):continue
        for source in (file,apk):
            destination=downloads/source.name
            if not destination.exists():shutil.copy2(source,destination)
        old_downloads.append((tuple(map(int,version.split('.'))),data))
    if old_downloads and not (downloads/'latest.json').exists():atomic(downloads/'latest.json',max(old_downloads,key=lambda item:item[0])[1])
    tmpfiles=Path('/etc/tmpfiles.d/2pybot-deploy.conf')
    tmpfiles.write_text(f'd {args.run_dir} 0775 {args.user} {account.pw_gid} -\nf {request} 0644 {args.user} {account.pw_gid} -\nf {operation_lock} 0664 {args.user} {account.pw_gid} -\n')
    if args.provision_toolchain:
        lock=json.loads((args.repo/'scripts/firmware/toolchain.json').read_text())
        command=['runuser','-u',args.user,'--',args.arduino_cli]
        urls=','.join(lock['board_manager_urls'])
        subprocess.run(command+['core','update-index','--additional-urls',urls],check=True)
        subprocess.run(command+['core','install',lock['core'],'--additional-urls',urls],check=True)
        for library in lock['libraries']:subprocess.run(command+['lib','install',library],check=True)
    spec=importlib.util.spec_from_file_location('one_pull_deploy',args.repo/'scripts/radxa/deploy.py')
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    manager=module.Deployment(args.repo,args.user,args.state_dir,config['api']+'/api/status',config=config_path)
    if args.bootstrap_host:
        # Old hosts cannot expose the new atomic maintenance endpoint. The
        # one-time transition therefore requires the USB controller isolated;
        # no installer resets or stops a potentially balancing controller.
        manager.commit=manager.stable_head()
        plan=module.Plan(console=True,units=True)
        manager.stage_release(plan);manager.prepare(plan)
        manager.switch_release(manager.release);manager.apply(plan);manager.install_runtime_worker()
        previous=manager.read_state()
        previous.update(release=str(manager.release),venv=str(manager.venv),host_commit=manager.commit,bootstrap_host_only=True)
        # Do not claim firmware source was applied by a host-only bootstrap.
        atomic(manager.state_file,previous)
    content='#!/bin/sh\nexec /usr/bin/python3 '+shlex.quote(str(args.repo/'scripts/radxa/git_pull.py'))+' "$@"\n'
    wrapper.write_text(content);wrapper.chmod(0o755)
    manager.install_monitor()
    print('One-pull units installed. Open a new shell or run hash -r once, then git pull --ff-only.')
    print('If the host protocol is not installed, use --bootstrap-host once with controller USB disconnected.')
    if args.led_backend=='firmware':
        print('Original ESP32 LED animations own the ring; Radxa LED control is disabled.')


if __name__=='__main__':main()
