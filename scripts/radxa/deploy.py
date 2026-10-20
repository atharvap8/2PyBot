#!/usr/bin/env python3
"""Apply relevant changes from a completed local Git update. Does not fetch or pull."""

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import hashlib
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import tarfile
import uuid
from urllib.request import urlopen, Request, HTTPRedirectHandler, build_opener
from urllib.parse import urlsplit
from urllib.error import HTTPError, URLError

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"software/radxa/console"))
from console.deployment_status import Progress, read as read_json, atomic


APP = "software/radxa/console/"
MONITOR = "scripts/radxa/systemd/"


@dataclass
class Plan:
    dependencies: bool = False
    native: bool = False
    console: bool = False
    mediamtx: bool = False
    units: bool = False
    rules: bool = False
    camera: bool = False
    monitor: bool = False
    boot: bool = False
    firmware: bool = False
    firmware_worker: bool = False
    android: bool = False
    hotspot: bool = False


def plan_changes(paths, first=False):
    plan = Plan(**{name: first for name in Plan.__dataclass_fields__})
    for path in paths:
        if path=="scripts/radxa/hotspot.json":plan.hotspot=True
        if path.startswith("software/android/") or path=="VERSION":plan.android=True
        if path.startswith("firmware/BaseLink/") or path.startswith("scripts/firmware/") or path=="VERSION":
            plan.firmware=True
        if path in (APP+"console/firmware.py",APP+"console/firmware_worker.py",APP+"console/versioning.py"):
            plan.firmware_worker=True
        if path==APP+"console/deployment_status.py":plan.monitor=True
        if path == APP + "requirements.txt":
            plan.dependencies = plan.console = True
        elif path.startswith(APP + "console/") and path.endswith(".py"):
            plan.console = True
        elif path.startswith(APP + "ui/") and path.endswith((".js", ".html", ".css", ".json")):
            plan.console = True
        elif path.startswith(APP + "native/") and (
            path.endswith((".c", ".h")) or path.endswith("/Makefile")
        ):
            plan.native = plan.console = True
        elif path == APP + "mediamtx.yml":
            plan.mediamtx = plan.console = True
        elif path == APP + "systemd/2pybot-console.service":
            plan.units = plan.console = True
        elif path == APP + "systemd/2pybot-mediamtx.service":
            plan.units = plan.mediamtx = plan.console = True
        elif path == APP + "20-camera-natural.conf":
            plan.camera = plan.console = True
        elif path == APP + "native/99-2pybot-encoder.rules":
            plan.rules = plan.console = True
        elif path in (APP + "40-tof.conf", APP + "45-boot-ready.conf", "scripts/radxa/boot_gate.py",
                      MONITOR + "2pybot-boot-gate.service", MONITOR + "86-2pybot-boot-gate.rules"):
            plan.boot = plan.console = True
        elif path in (MONITOR + "2pybot-deploy.service", MONITOR + "2pybot-deploy.timer"):
            plan.monitor = True
        elif path in ("scripts/radxa/deploy.py","scripts/radxa/git_pull.py","scripts/radxa/deployment_leds.py",
                      MONITOR+"2pybot-deploy.path",MONITOR+"2pybot-deployment-leds.service"):
            plan.monitor=True
    return plan


class GitBusy(RuntimeError):
    pass


class DeploymentDeferred(RuntimeError):
    def __init__(self,message,phase="waiting_idle"):
        super().__init__(message);self.phase=phase


class BundleRedirect(HTTPRedirectHandler):
    def redirect_request(self,request,fp,code,message,headers,new_url):
        redirected=super().redirect_request(request,fp,code,message,headers,new_url)
        if redirected and urlsplit(new_url).netloc!=urlsplit(request.full_url).netloc:
            redirected.remove_header("Authorization")
        return redirected


def controller_ready(status):
    tel=status.get('telemetry',{})
    if not(status.get('connected') and status.get('dump_complete') and not tel.get('stale',True) and tel.get('startup_ready') and tel.get('host_ready')):
        return False
    if status.get('derived',{}).get('FLASHLOCK')==1 and tel.get('firmware_lock',True):return False
    if status.get('derived',{}).get('HUBMOTOR')==1:
        hub=tel.get('hub',{})
        if not hub.get('link_ok') or not hub.get('mode_ok') or hub.get('fault_l') or hub.get('fault_r'):return False
        if not tel.get('balancing') and (abs(hub.get('current_cmd_l',99))>.01 or abs(hub.get('current_cmd_r',99))>.01):return False
    return True


class Deployment:
    def __init__(self, repo, user, state_dir, health_url, system_root=Path("/"), config=None):
        self.repo = Path(repo).resolve()
        self.user = user
        self.state_dir = Path(state_dir)
        self.state_file = self.state_dir / "state.json"
        self.venv = Path(read_json(self.state_file,{}).get("venv",self.state_dir/"venv"))
        self.app = self.repo / APP
        self.health_url = health_url
        self.system_root = Path(system_root)
        self.commit = ""
        self.environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        self.config=read_json(config or self.system_root/"etc/2pybot-deploy.json",{})
        self.progress=Progress(self.state_dir)
        self.current=self.state_dir/"current"
        self.source=self.repo
        self.release=None

    def git(self, *arguments, check=True):
        command = ["git", "-c", "safe.directory=" + str(self.repo), *arguments]
        # Debian's older Git ignores command-line safe.directory exceptions.
        # Reading the checkout as its owner also keeps Git configuration user-scoped.
        if hasattr(os, "geteuid") and os.geteuid() == 0:
            command = ["runuser", "-u", self.user, "--", *command]
        result = subprocess.run(
            command, cwd=self.repo, capture_output=True, text=True, check=False,
        )
        if check and result.returncode:
            raise RuntimeError("Git failed: " + (result.stderr.strip() or str(result.returncode)))
        return result

    def run(self, command, user=False, cwd=None):
        if user:
            command = ["runuser", "-u", self.user, "--", *map(str, command)]
        else:
            command = list(map(str, command))
        print("run: " + " ".join(command), flush=True)
        subprocess.run(command, cwd=cwd, env=self.environment, check=True)

    def read_state(self):
        try:
            return json.loads(self.state_file.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {}

    def stable_head(self):
        if self.config.get('one_pull'):
            from console.deployment_status import process_alive
            if self.config.get('branch') and self.git('branch','--show-current').stdout.strip()!=self.config['branch']:
                raise GitBusy('Checkout is not on the configured deployment branch')
            pull=read_json(Path(self.config.get('run_dir','/run/2pybot-deploy'))/'pull.json',{})
            if pull.get('phase')=='pulling' and process_alive(pull.get('owner_pid')):
                raise GitBusy('Managed git pull is still running')
        git_dir = Path(self.git("rev-parse", "--absolute-git-dir").stdout.strip())
        if (git_dir / "index.lock").exists() or (git_dir / "HEAD.lock").exists():
            raise GitBusy("Git operation in progress")
        if self.git("status", "--porcelain", "--untracked-files=no").stdout.strip():
            raise GitBusy("tracked files have local changes")
        return self.git("rev-parse", "HEAD").stdout.strip()

    def changed_paths(self, previous):
        if not previous or self.git("cat-file", "-e", previous + "^{commit}", check=False).returncode:
            return self.git("ls-files").stdout.splitlines(), True
        return self.git("diff", "--name-only", previous, self.commit, "--").stdout.splitlines(), False

    def write_bytes(self, destination, content, backup=True):
        destination = Path(destination)
        if destination.exists() and destination.read_bytes() == content:
            return False
        if backup and destination.is_file():
            relative = destination.relative_to(self.system_root)
            saved = self.state_dir / "backups" / self.commit / relative
            saved.parent.mkdir(parents=True, exist_ok=True)
            if not saved.exists():
                shutil.copy2(destination, saved)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as output:
                temporary = Path(output.name)
                output.write(content)
            temporary.chmod(0o644)
            os.replace(temporary, destination)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
        return True

    def install_file(self, source, destination):
        return self.write_bytes(self.system_root / destination.lstrip("/"), Path(source).read_bytes())

    def prepare(self, plan):
        if plan.boot:
            source = self.source / "scripts/radxa/boot_gate.py"
            compile(source.read_text(encoding="utf-8"), str(source), "exec")
        if plan.console:
            for source in (self.app / "console").glob("*.py"):
                compile(source.read_text(encoding="utf-8"), str(source), "exec")
        if plan.dependencies or not (self.venv / "bin/python3").exists():
            import pwd
            account = pwd.getpwnam(self.user)
            self.venv.mkdir(parents=True, exist_ok=True)
            os.chown(self.venv, account.pw_uid, account.pw_gid)
            self.run([sys.executable, "-m", "venv", "--system-site-packages", self.venv], user=True)
            self.run([
                self.venv / "bin/python3", "-m", "pip", "install",
                "--disable-pip-version-check", "-r", self.app / "requirements.txt",
            ], user=True)
        if plan.console:
            self.run([
                self.venv / "bin/python3", "-c",
                "import fastapi, uvicorn, serial, pydantic; from PIL import Image; import console.server",
            ], user=True, cwd=self.app)
        if plan.native or (plan.console and not (self.app / "native/2pybot_hwstream").exists()):
            self.run(["make", "-C", self.app / "native"], user=True)

    def install_monitor(self):
        reload_needed = False
        substitutions = {
            "__REPO__": str(self.repo), "__USER__": self.user,
            "__STATE_DIR__": str(self.state_dir), "__HEALTH_URL__": self.health_url,
            "__RUN_DIR__":self.config.get("run_dir","/run/2pybot-deploy"),
        }
        names=["2pybot-deploy.service","2pybot-deploy.timer"]
        if self.config.get("one_pull"):names += ["2pybot-deploy.path","2pybot-deployment-leds.service"]
        for name in names:
            content = (self.repo / MONITOR / name).read_text(encoding="utf-8")
            for original, value in substitutions.items():
                content = content.replace(original, value)
            reload_needed |= self.write_bytes(
                self.system_root / "etc/systemd/system" / name, content.encode("utf-8"),
            )
        if reload_needed:
            self.run(["systemctl", "daemon-reload"])
        self.run(["systemctl", "enable", "--now", "2pybot-deploy.timer"])
        if reload_needed:
            self.run(["systemctl", "restart", "2pybot-deploy.timer"])
        if self.config.get("one_pull"):
            self.run(["systemctl","enable","--now","2pybot-deploy.path","2pybot-deployment-leds.service"])
            self.run(["systemctl","restart","2pybot-deployment-leds.service"])

    def install_boot(self):
        content = (self.source / MONITOR / "2pybot-boot-gate.service").read_text(encoding="utf-8")
        for token, value in {"__USER__": self.user, "__REPO__": str(self.current if self.release else self.repo), "__VENV__": str(self.venv)}.items():
            content = content.replace(token, value)
        changed = self.write_bytes(self.system_root / "etc/systemd/system/2pybot-boot-gate.service", content.encode("utf-8"))
        for name in ("40-tof.conf", "45-boot-ready.conf"):
            changed |= self.install_file(self.app / name, "/etc/systemd/system/2pybot-console.service.d/" + name)
        rule_changed = self.install_file(self.source / MONITOR / "86-2pybot-boot-gate.rules",
                                         "/etc/udev/rules.d/86-2pybot-boot-gate.rules")
        if rule_changed:
            self.run(["udevadm", "control", "--reload-rules"])
        return changed

    def apply(self, plan):
        reload_needed = False
        if plan.boot:
            reload_needed |= self.install_boot()
        if plan.units:
            content = (self.app / "systemd/2pybot-console.service").read_text(encoding="utf-8")
            runtime_app=(self.current/APP) if self.release else self.app
            content = content.replace("__USER__", self.user).replace("__DIR__", str(runtime_app))
            content = content.replace("ExecStart=/usr/bin/python3", "ExecStart=" + str(self.venv / "bin/python3"))
            if self.config.get("one_pull"):
                content += "\n[Service]\nEnvironment=BOT_DEPLOY_STATE="+str(self.state_dir)+"\nEnvironment=BOT_DEPLOY_RUN="+self.config.get("run_dir","/run/2pybot-deploy")+"\nEnvironment=BOT_DOWNLOAD_DIR="+self.config.get("downloads_dir","/var/lib/2pybot/downloads")+"\nEnvironment=BOT_SERIAL="+self.config.get('serial_port','/dev/ttyUSB0')+"\n"
            reload_needed |= self.write_bytes(
                self.system_root / "etc/systemd/system/2pybot-console.service", content.encode("utf-8"),
            )
            reload_needed |= self.install_file(
                self.app / "systemd/2pybot-mediamtx.service", "/etc/systemd/system/2pybot-mediamtx.service",
            )
        if plan.camera:
            reload_needed |= self.install_file(
                self.app / "20-camera-natural.conf",
                "/etc/systemd/system/2pybot-console.service.d/20-camera-natural.conf",
            )
        if plan.mediamtx:
            self.install_file(self.app / "mediamtx.yml", "/etc/2pybot/mediamtx.yml")
        if plan.rules:
            rules_changed = self.install_file(
                self.app / "native/99-2pybot-encoder.rules", "/etc/udev/rules.d/99-2pybot-encoder.rules",
            )
            serial_rules = (
                'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", SYMLINK+="2pybot"\n'
                'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="2pybot"\n'
                'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", SYMLINK+="2pybot"\n'
            )
            rules_changed |= self.write_bytes(
                self.system_root / "etc/udev/rules.d/99-2pybot.rules", serial_rules.encode("utf-8"),
            )
            if rules_changed:
                self.run(["udevadm", "control", "--reload-rules"])
        if reload_needed:
            self.run(["systemctl", "daemon-reload"])
        if plan.boot:
            # Arm the next boot; do not reset an ESP32 owned by the live console.
            self.run(["systemctl", "enable", "2pybot-boot-gate.service"])
        if plan.mediamtx:
            self.run(["systemctl", "enable", "2pybot-mediamtx.service"])
            self.run(["systemctl", "restart", "2pybot-mediamtx.service"])
        if plan.console:
            self.run(["systemctl", "enable", "2pybot-console.service"])
            self.run(["systemctl", "restart", "2pybot-console.service"])

    def stream_running(self):
        try:
            with urlopen(self.health_url, timeout=3) as response:
                return bool(json.load(response).get("stream", {}).get("running"))
        except Exception:
            return False

    def wait_for_health(self, require_stream):
        deadline = time.monotonic() + 60
        last_error = "console not ready"
        while time.monotonic() < deadline:
            try:
                for service in ("2pybot-console.service", "2pybot-mediamtx.service"):
                    subprocess.run(["systemctl", "is-active", "--quiet", service], check=True)
                with urlopen(self.health_url, timeout=3) as response:
                    payload = json.load(response)
                if "telemetry" not in payload:
                    raise RuntimeError("unexpected console status response")
                if require_stream and not payload.get("stream", {}).get("running"):
                    raise RuntimeError("previously running camera stream has not recovered")
                return
            except Exception as error:
                last_error = str(error)
                time.sleep(2)
        raise RuntimeError("deployment health check failed: " + last_error)

    def api(self,path,body=None):
        from urllib.parse import urlsplit
        parts=urlsplit(self.health_url)
        base=parts.scheme+"://"+parts.netloc
        request=Request(base+path,data=None if body is None else json.dumps(body).encode(),
                        headers={"Content-Type":"application/json"})
        try:
            with urlopen(request,timeout=12) as response:return json.load(response)
        except HTTPError as error:
            detail=error.read().decode(errors="replace")
            if error.code in (404,409):raise DeploymentDeferred(detail) from error
            raise RuntimeError(detail) from error
        except (OSError,URLError) as error:
            raise DeploymentDeferred("Console API is unavailable: "+str(error)) from error

    def fingerprint(self):
        inputs=("firmware/BaseLink","scripts/firmware/build.py","scripts/firmware/toolchain.json",
                "scripts/firmware/fingerprint.py","software/radxa/console/console/versioning.py","VERSION")
        return hashlib.sha256(self.git("ls-tree","-r",self.commit,"--",*inputs).stdout.encode()).hexdigest()

    def hotspot_command(self,*arguments):
        result=subprocess.run(arguments,capture_output=True,text=True,timeout=45,
            env=dict(self.environment,LC_ALL='C'),check=False)
        if result.returncode:
            raise RuntimeError('Hotspot command failed: '+(result.stderr.strip() or result.stdout.strip()))
        return result.stdout.strip()

    def hotspot_radio(self,interface):
        output=self.hotspot_command('iw','dev',interface,'info')
        match=re.search(r'\bchannel\s+(\d+)\s+\((\d+)\s+MHz\)',output)
        if not match:raise RuntimeError('Cannot verify the active hotspot radio channel')
        return tuple(map(int,match.groups()))

    def prepare_hotspot(self):
        target=read_json(self.source/'scripts/radxa/hotspot.json',{})
        if set(target)!={'connection','band','channel'}:raise ValueError('Invalid managed hotspot settings')
        name=target.get('connection')
        if name is not None and (not isinstance(name,str) or not name.strip() or any(ord(c)<32 for c in name)):
            raise ValueError('A NetworkManager hotspot connection name is required')
        if target.get('band')!='bg' or type(target.get('channel')) is not int or not 1<=target['channel']<=11:
            raise ValueError('Managed hotspot requires band bg and a 2.4 GHz channel from 1 to 11')
        def field(key):
            return self.hotspot_command('nmcli','--get-values',key,'connection','show','uuid',identifier)
        uuid_pattern=r'[a-fA-F0-9]{8}(?:-[a-fA-F0-9]{4}){3}-[a-fA-F0-9]{12}'
        if name is None:
            candidates=[]
            for identifier in self.hotspot_command('nmcli','--get-values','UUID','connection','show','--active').splitlines():
                if not re.fullmatch(uuid_pattern,identifier):raise RuntimeError('Invalid active connection identity')
                if field('connection.type')=='802-11-wireless' and field('802-11-wireless.mode')=='ap':candidates.append(identifier)
            if len(candidates)!=1:raise RuntimeError('Expected exactly one active Wi-Fi hotspot; set its connection name in hotspot.json')
            identifier=candidates[0];name=field('connection.id')
        else:
            identifier=self.hotspot_command('nmcli','--get-values','connection.uuid','connection','show',name)
        if not re.fullmatch(uuid_pattern,identifier):
            raise RuntimeError('Hotspot connection must resolve to exactly one profile: '+name)
        if field('connection.type')!='802-11-wireless' or field('802-11-wireless.mode')!='ap':
            raise RuntimeError('Selected connection is not a Wi-Fi access point; no settings changed')
        if field('GENERAL.STATE')!='activated':
            raise RuntimeError('The managed hotspot must already be active before changing its band')
        interface=field('GENERAL.DEVICES')
        if not re.fullmatch(r'[A-Za-z0-9_.:-]+',interface):raise RuntimeError('Cannot identify the hotspot Wi-Fi interface')
        band=field('802-11-wireless.band');channel=field('802-11-wireless.channel')
        if band not in {'','a','bg'} or not channel.isdigit():raise RuntimeError('Invalid existing hotspot band/channel')
        radio=self.hotspot_radio(interface)
        return dict(target,connection=name,uuid=identifier,interface=interface,previous_band=band,previous_channel=channel,
            changed=band!=target['band'] or int(channel)!=target['channel'] or radio!=(target['channel'],2407+5*target['channel']))

    def apply_hotspot(self,hotspot):
        if hotspot['changed']:
            # Only band/channel are managed. SSID, PSK, addressing, sharing and
            # the uplink are never copied, printed or rewritten.
            backup=self.state_dir/'hotspot-backups'/(self.commit+'.json')
            if not backup.exists():atomic(backup,hotspot)
            self.progress.update('deploying','Switching hotspot to 2.4 GHz; reconnect Wi-Fi clients',progress=96)
            self.hotspot_command('nmcli','connection','modify','uuid',hotspot['uuid'],
                '802-11-wireless.band',hotspot['band'],'802-11-wireless.channel',str(hotspot['channel']))
            self.hotspot_command('nmcli','--wait','30','connection','up','uuid',hotspot['uuid'])
        if self.hotspot_radio(hotspot['interface'])!=(hotspot['channel'],2407+5*hotspot['channel']):
            raise RuntimeError('Hotspot did not activate on the requested 2.4 GHz channel')
        return {key:hotspot[key] for key in ('connection','uuid','band','channel','interface')}

    def restore_hotspot(self,hotspot):
        backup=read_json(self.state_dir/'hotspot-backups'/(self.commit+'.json'),hotspot)
        if backup.get('uuid')!=hotspot['uuid']:raise RuntimeError('Hotspot rollback profile identity changed')
        self.hotspot_command('nmcli','connection','modify','uuid',hotspot['uuid'],
            '802-11-wireless.band',backup['previous_band'],'802-11-wireless.channel',backup['previous_channel'])
        self.hotspot_command('nmcli','--wait','30','connection','up','uuid',hotspot['uuid'])
        channel,frequency=self.hotspot_radio(hotspot['interface'])
        if int(backup['previous_channel']) and channel!=int(backup['previous_channel']):
            raise RuntimeError('Previous hotspot channel did not recover')
        if (backup['previous_band']=='bg' and not 2400<=frequency<2500) or (backup['previous_band']=='a' and frequency<5000):
            raise RuntimeError('Previous hotspot band did not recover')

    def preflight(self):
        if self.api("/api/deployment/status").get("protocol")!=1:
            raise DeploymentDeferred("One-time host bootstrap is required")
        status=self.api("/api/status");tel=status.get("telemetry",{});debug=status.get("debug",{})
        if not status.get("connected") or tel.get("stale",True) or not tel.get("startup_ready") or not status.get("dump_complete"):
            raise DeploymentDeferred("Waiting for fresh READY controller telemetry")
        if tel.get("balancing") or abs(debug.get("v",0))>.03 or abs(debug.get("w",0))>10:
            raise DeploymentDeferred("Waiting for local SELECT and a stationary bot; deployment will not stop a balancing robot")
        if self.config.get("auto_firmware",True) and status.get("derived",{}).get("HUBSCHEMA")!=4:
            raise DeploymentDeferred("One-time BLDC commissioning is required before automatic source updates")
        parameters={p['key']:p['value'] for p in self.api('/api/params').get('params',[])}
        if parameters.get('BOOTWAIT')!=1:raise DeploymentDeferred('BOOTWAIT=1 is required for reset-safe deployment maintenance')
        job=self.api("/api/firmware/status")
        if job.get("busy") or (job.get("job") or {}).get("status")=="recovery_required":
            raise DeploymentDeferred("Waiting for the existing firmware job/recovery")

    def fetch_bundle(self):
        destination=self.state_dir/"incoming"/self.commit
        if (destination/"bundle.json").is_file():
            self.verify_bundle(destination)
            return destination
        template=self.config.get("bundle_url")
        if not template:raise DeploymentDeferred("Matching CI deployment bundle is not present",phase="waiting_artifact")
        url=template.format(commit=self.commit)
        if not url.startswith("https://"):raise RuntimeError("Deployment bundle must use HTTPS")
        headers={}
        token=os.environ.get("BOT_DEPLOY_TOKEN")
        if token:headers["Authorization"]="Bearer "+token
        opener=build_opener(BundleRedirect())
        self.check_ci(opener,headers)
        if token and self.config.get("bundle_api"):
            try:
                with opener.open(Request(self.config["bundle_api"].format(commit=self.commit),headers=dict(headers,Accept="application/vnd.github+json")),timeout=15) as response:
                    release=json.load(response)
            except HTTPError as error:
                if error.code==404:raise DeploymentDeferred("Waiting for the checked CI release",phase="waiting_artifact") from error
                raise RuntimeError("GitHub release API access failed (HTTP "+str(error.code)+")") from error
            name=f"2pybot-deploy-{self.commit}.tar.gz"
            matches=[asset for asset in release.get("assets",[]) if asset.get("name")==name]
            if len(matches)!=1:raise DeploymentDeferred("CI release bundle is not published yet",phase="waiting_artifact")
            url=matches[0]["url"]
            if not url.startswith("https://api.github.com/"):raise RuntimeError("Invalid GitHub asset API URL")
            headers["Accept"]="application/octet-stream"
        self.progress.update("building","Waiting for/downloading the checked CI bundle",progress=10)
        destination.parent.mkdir(parents=True,exist_ok=True)
        temporary=Path(tempfile.mkdtemp(prefix=self.commit+".download-",dir=destination.parent))
        try:
            try:
                with opener.open(Request(url,headers=headers),timeout=30) as response,(temporary/"bundle.tar.gz").open("wb") as file:
                    size=0
                    while True:
                        block=response.read(1024*1024)
                        if not block:break
                        size+=len(block)
                        if size>180*1024*1024:raise RuntimeError("Deployment bundle exceeds size limit")
                        file.write(block)
            except HTTPError as error:
                if error.code==404:raise DeploymentDeferred("Waiting for the CI bundle for this exact commit",phase="waiting_artifact") from error
                raise RuntimeError("CI bundle download failed (HTTP "+str(error.code)+"); check release access/token") from error
            with tarfile.open(temporary/"bundle.tar.gz") as bundle:
                total=0
                for entry in bundle.getmembers():
                    if entry.name.startswith("/") or ".." in Path(entry.name).parts or not(entry.isfile() or entry.isdir()):
                        raise RuntimeError("Invalid path/link in deployment bundle")
                    total+=entry.size
                    if total>250*1024*1024:raise RuntimeError("Deployment bundle expands beyond size limit")
                bundle.extractall(temporary)
            (temporary/"bundle.tar.gz").unlink()
            self.verify_bundle(temporary)
            if hasattr(os,"geteuid") and os.geteuid()==0:
                import pwd
                account=pwd.getpwnam(self.user)
                for path in [temporary,*temporary.rglob("*")]:os.chown(path,account.pw_uid,account.pw_gid)
            if destination.exists():raise RuntimeError("An incomplete incoming bundle directory already exists")
            temporary.rename(destination)
            return destination
        finally:
            if temporary.exists():shutil.rmtree(temporary)

    def check_ci(self,opener,headers):
        template=self.config.get('ci_status_api')
        if not template:return
        cache=self.state_dir/('ci-'+self.commit+'.json')
        status=read_json(cache,{})
        if time.time()-status.get('checked_at',0)>60:
            try:
                with opener.open(Request(template.format(commit=self.commit),headers=dict(headers,Accept='application/vnd.github+json')),timeout=10) as response:
                    runs=json.load(response).get('workflow_runs',[])
                matching=[r for r in runs if r.get('head_sha')==self.commit]
                run=matching[0] if matching else {}
                status={'checked_at':time.time(),'conclusion':run.get('conclusion'),'status':run.get('status')}
                atomic(cache,status)
            except (HTTPError,OSError,ValueError):return # artifact access itself still provides a clear error
        if status.get('conclusion') in {'failure','cancelled','timed_out','action_required','startup_failure'}:
            raise RuntimeError('CI deployment-bundle checks failed for this commit; inspect GitHub Actions before retrying')

    def verify_bundle(self,directory):
        bundle=read_json(Path(directory)/"bundle.json",{})
        if bundle.get("schema")!=1 or bundle.get("source_commit")!=self.commit:
            raise RuntimeError("CI bundle identity does not match the requested commit")
        allowed={"firmware/application.bin","firmware/manifest.json","android/app.apk","android/manifest.json"}
        if set(bundle.get("sha256",{}))!=allowed:raise RuntimeError("Incomplete deployment bundle")
        for name,digest in bundle["sha256"].items():
            path=Path(directory)/name
            if not path.is_file() or path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
                raise RuntimeError("CI bundle checksum mismatch: "+name)

    def publish_android(self):
        source=self.state_dir/"incoming"/self.commit/"android"
        manifest=read_json(source/"manifest.json",{})
        apk=source/"app.apk"
        if manifest.get("source_commit")!=self.commit or not apk.is_file() or hashlib.sha256(apk.read_bytes()).hexdigest()!=manifest.get("sha256"):
            raise RuntimeError("Android artifact is not the requested checked build")
        from console.versioning import validate_version
        version=manifest.get("version","");validate_version(version,allow_prerelease=True)
        downloads=Path(self.config.get("downloads_dir","/var/lib/2pybot/downloads"));downloads.mkdir(parents=True,exist_ok=True)
        for file,name in [(apk,f"2PyBot-v{version}-debug.apk"),(source/"manifest.json",f"2PyBot-v{version}-manifest.json")]:
            temp=downloads/(name+".new");shutil.copy2(file,temp);temp.chmod(0o644);os.replace(temp,downloads/name)
        atomic(downloads/'latest.json',manifest)

    def stage_release(self,plan):
        releases=self.state_dir/"releases";releases.mkdir(parents=True,exist_ok=True)
        destination=releases/self.commit
        if not (destination/".deployment-snapshot.json").is_file():
            temporary=Path(tempfile.mkdtemp(prefix=self.commit+".staging-",dir=releases))
            archive=Path(str(temporary)+".tar")
            command=["git","-C",str(self.repo),"archive","--format=tar",self.commit]
            if hasattr(os,"geteuid") and os.geteuid()==0:command=["runuser","-u",self.user,"--",*command]
            try:
                with archive.open("wb") as output:subprocess.run(command,stdout=output,check=True)
                with tarfile.open(archive) as bundle:
                    for entry in bundle.getmembers():
                        parts=Path(entry.name).parts
                        if entry.name.startswith("/") or ".." in parts or not (entry.isdir() or entry.isfile()):
                            raise RuntimeError("Unsupported path/link in release archive: "+entry.name)
                    bundle.extractall(temporary)
                atomic(temporary/".deployment-snapshot.json",{"commit":self.commit})
                if hasattr(os,"geteuid") and os.geteuid()==0:
                    import pwd
                    account=pwd.getpwnam(self.user)
                    for path in [temporary,*temporary.rglob("*")]:os.chown(path,account.pw_uid,account.pw_gid)
                if destination.exists():raise RuntimeError("Incomplete release directory already exists: "+str(destination))
                temporary.rename(destination)
            finally:
                archive.unlink(missing_ok=True)
                if temporary.exists():shutil.rmtree(temporary)
        self.release=destination;self.source=destination;self.app=destination/APP
        # Reuse a known unchanged native executable, rather than rebuilding on
        # every Python/UI pull. Never copy a running capture process itself.
        native=self.app/"native/2pybot_hwstream"
        old=self.current/APP/"native/2pybot_hwstream"
        if not old.is_file():old=self.repo/APP/"native/2pybot_hwstream"
        if not plan.native and not native.exists() and old.is_file():
            shutil.copy2(old,native)
            if hasattr(os,"geteuid") and os.geteuid()==0:shutil.chown(native,user=self.user,group=self.user)
        if plan.dependencies or not (self.venv/"bin/python3").is_file():
            requirements=(self.app/"requirements.txt").read_bytes()
            identity=hashlib.sha256(requirements+(sys.version+sys.platform).encode()).hexdigest()[:20]
            self.venv=self.state_dir/"envs"/identity
        else:
            self.venv=Path(self.read_state().get("venv",self.venv))

    def prepare_firmware(self,fingerprint):
        candidate=self.state_dir/"incoming"/self.commit/"firmware"
        if not (candidate/"manifest.json").is_file():
            if self.config.get("firmware_mode","build")!="build":
                raise RuntimeError("Matching CI firmware artifact is unavailable in "+str(candidate))
            candidate=self.state_dir/"firmware-cache"/fingerprint
            candidate.mkdir(parents=True,exist_ok=True)
            if hasattr(os,"geteuid") and os.geteuid()==0:shutil.chown(candidate,user=self.user,group=self.user)
            existing=read_json(candidate/"manifest.json",{})
            if existing.get("source_fingerprint")!=fingerprint or not (candidate/"BaseLink.ino.bin").is_file():
                self.progress.update("building","Compiling controller firmware",progress=15)
                self.run([self.venv/"bin/python3",self.source/"scripts/firmware/build.py",
                    "--arduino-cli",self.config.get("arduino_cli","arduino-cli"),"--output-dir",candidate,
                    "--source-commit",self.commit,"--source-fingerprint",fingerprint,"--require-toolchain-lock"],user=True)
        manifest=read_json(candidate/"manifest.json",{})
        if manifest.get("source_fingerprint")!=fingerprint or manifest.get("source_dirty",True):
            raise RuntimeError("Firmware artifact is not a clean build of the requested inputs")
        if manifest.get("hardware")!="m0601-rs485" or manifest.get("parameter_schema")!="bldc-v4":
            raise RuntimeError("Firmware artifact hardware/schema does not match this bot")
        lock=read_json(self.source/'scripts/firmware/toolchain.json',{})
        if manifest.get('toolchain',{}).get('core')!=lock.get('core') or manifest.get('toolchain',{}).get('libraries')!={item.rsplit('@',1)[0]:item.rsplit('@',1)[1] for item in lock.get('libraries',[])}:
            raise RuntimeError('Firmware artifact was not built with the locked toolchain')
        image=candidate/"application.bin"
        if not image.is_file():
            image=candidate/"BaseLink.ino.bin"
            shutil.copy2(image,candidate/"application.bin")
            if hasattr(os,"geteuid") and os.geteuid()==0:shutil.chown(candidate/"application.bin",user=self.user,group=self.user)
        sys.path.insert(0,str(self.source/APP))
        from console.firmware import inspect_image
        details=inspect_image(image)
        if details["sha256"]!=manifest.get("sha256") or details["size_bytes"]!=manifest.get("size_bytes"):
            raise RuntimeError("Compiled firmware does not match its manifest")
        self.run([self.venv/"bin/python3",self.source/"scripts/firmware/register_build.py",candidate],user=True)
        return manifest

    def switch_release(self,path):
        temporary=self.current.with_name("current.new-"+uuid.uuid4().hex)
        temporary.symlink_to(Path(path),target_is_directory=True)
        os.replace(temporary,self.current)

    def install_runtime_worker(self):
        template=(self.source/MONITOR/"2pybot-firmware.service").read_text()
        replacements={"__USER__":self.user,"__REPO__":str(self.current),"__VENV__":str(self.venv),
            "__FLASHER_PYTHON__":self.config.get("flasher_python","/home/"+self.user+"/tools/2pybot-flasher/bin/python3")}
        for key,value in replacements.items():template=template.replace(key,value)
        template += "\n[Service]\nEnvironment=BOT_DEPLOY_STATE="+str(self.state_dir)+"\nEnvironment=BOT_DEPLOY_RUN="+self.config.get("run_dir","/run/2pybot-deploy")+"\nEnvironment=BOT_SERIAL="+self.config.get('serial_port','/dev/ttyUSB0')+"\n"
        if self.write_bytes(self.system_root/"etc/systemd/system/2pybot-firmware.service",template.encode()):
            self.run(["systemctl","daemon-reload"])
        self.run(["systemctl","enable","2pybot-firmware.service"])
        self.run(["systemctl","restart","2pybot-firmware.service"])

    def wait_firmware(self,job_id):
        deadline=time.monotonic()+self.config.get("firmware_timeout",300)
        while time.monotonic()<deadline:
            status=self.api("/api/firmware/status");job=status.get("job") or {}
            if job.get("id")!=job_id:raise RuntimeError("Firmware job identity changed during deployment")
            phase="calibrating" if job.get("stage") in {"calibrating","restoring"} else "flashing"
            self.progress.update(phase,job.get("message","Updating controller"),progress=job.get("progress",0),firmware_job_id=job_id)
            if job.get("status")=="succeeded":return status
            if job.get("status") in {"failed","recovery_required"}:raise RuntimeError(job.get("error") or job.get("message"))
            time.sleep(.3)
        raise RuntimeError("Firmware job is still active; maintenance remains held")

    def wait_controller(self):
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            status=self.api("/api/status")
            if controller_ready(status):
                return status
            time.sleep(.3)
        raise RuntimeError("Controller did not reconnect with fresh READY metadata")

    def deploy_coordinated(self,plan,state,dry_run=False):
        fingerprint=self.fingerprint()
        plan.firmware=bool(self.config.get("auto_firmware",True) and state.get("firmware_fingerprint")!=fingerprint)
        needs_runtime=plan.console or plan.native or plan.units or plan.firmware_worker or not self.current.is_symlink()
        if dry_run:
            print(json.dumps({"commit":self.commit,"previous":state.get("commit"),"actions":asdict(plan),"one_pull":True}))
            return True
        self.progress.update("checking","Checking requested deployment",commit=self.commit,previous_commit=state.get("commit"),progress=2)
        token=None;activated=False;released=False;previous=state.get("release");had_stream=False
        hotspot=None;hotspot_attempted=False;hotspot_result=state.get('hotspot')
        existing_hold=read_json(Path(self.config.get("firmware_root","/var/lib/2pybot/firmware"))/"deployment-hold.json",{})
        try:
            if needs_runtime or plan.firmware or plan.hotspot:
                if existing_hold and existing_hold.get("commit")!=self.commit:
                    raise DeploymentDeferred("Another interrupted deployment holds the controller; recover it first")
                if existing_hold:
                    if not existing_hold.get('locked'):raise RuntimeError('Interrupted maintenance acquisition needs recovery before deployment')
                    token=existing_hold.get('token')
                    current_job=read_json(Path(self.config.get('firmware_root','/var/lib/2pybot/firmware'))/'current.json',{})
                    if current_job.get('status')=='recovery_required':raise RuntimeError('Controller recovery is required; no service will be restarted')
                    if current_job.get('status') in {'preparing','queued','running','recovering'}:
                        if current_job.get('deployment_token')!=token:raise DeploymentDeferred('Another firmware job owns maintenance')
                        self.wait_firmware(current_job['id']) # never restart the worker halfway through a write
                    elif current_job.get('status')=='failed' and current_job.get('deployment_token')==token:
                        raise RuntimeError(current_job.get('error') or 'The interrupted deployment firmware update failed')
                if not existing_hold:self.preflight()
                if plan.android or (plan.firmware and self.config.get("firmware_mode","build")=="artifact"):
                    self.fetch_bundle()
                self.progress.update("building","Preparing immutable release",progress=8)
                self.stage_release(plan)
                self.prepare(plan)
                hotspot=self.prepare_hotspot() if plan.hotspot else None
                firmware=self.prepare_firmware(fingerprint) if plan.firmware else None
                if self.stable_head()!=self.commit:raise GitBusy("A second pull changed the requested commit during preparation")
                token=existing_hold.get("token") or uuid.uuid4().hex
                self.progress.update("deploying","Acquiring controller maintenance lock",progress=30)
                self.api("/api/internal/deployment/lock",{"token":token,"commit":self.commit})
                if self.stable_head()!=self.commit:raise GitBusy("A second pull changed source before activation")
                had_stream=self.stream_running()
                if needs_runtime:
                    self.switch_release(self.release);activated=True
                    # Migrate the original checkout-based service on first activation.
                    plan.units=plan.units or not previous
                    self.apply(plan)
                    if not plan.console:self.run(["systemctl","restart","2pybot-console.service"])
                    self.install_runtime_worker()
                    self.wait_for_health(had_stream)
                if firmware:
                    deadline=time.monotonic()+15
                    while True:
                        firmware_status=self.api("/api/firmware/status")
                        if firmware_status.get("worker_ready") and self.api("/api/ready").get("ready"):break
                        if time.monotonic()>deadline:raise RuntimeError("Firmware worker/host readiness did not recover")
                        time.sleep(.3)
                    installed=firmware_status.get("installed") or {}
                    if installed.get("build_id")!=firmware["build_id"]:
                        job=firmware_status.get("job") or {}
                        if job.get("deployment_token")!=token or job.get("status") not in {"queued","running","recovering"}:
                            job=self.api("/api/internal/deployment/firmware",{"token":token,"build_id":firmware["build_id"]})
                        self.wait_firmware(job["id"])
                if plan.android:
                    self.progress.update('deploying','Publishing the checked Android download',progress=95)
                    self.publish_android()
                if plan.monitor:self.install_monitor()
                if hotspot:
                    hotspot_attempted=hotspot['changed']
                    hotspot_result=self.apply_hotspot(hotspot)
                self.api("/api/internal/deployment/release",{"token":token});released=True
                self.wait_controller()
            if self.stable_head()!=self.commit:raise GitBusy("Repository changed before deployment completion")
            if plan.android and not (needs_runtime or plan.firmware or plan.hotspot):
                self.fetch_bundle();self.publish_android()
            if plan.monitor and not (needs_runtime or plan.firmware or plan.hotspot):self.install_monitor()
            record={"commit":self.commit,"previous":state.get("commit"),"repo":str(self.repo),
                "release":str(self.release) if activated else previous,"venv":str(self.venv),
                "firmware_fingerprint":fingerprint if plan.firmware else state.get("firmware_fingerprint"),
                "hotspot":hotspot_result,
                "actions":asdict(plan),"applied_at":datetime.now(timezone.utc).isoformat()}
            atomic(self.state_file,record)
            self.progress.update("success","Deployment verified; local START remains required",progress=100,applied_commit=self.commit)
            return True
        except (DeploymentDeferred,GitBusy) as error:
            if hotspot_attempted and not released:
                try:self.restore_hotspot(hotspot)
                except Exception as failure:
                    self.progress.update('failed',str(error),recovery=['Hotspot rollback failed: '+str(failure)])
                    raise RuntimeError('Hotspot rollback failed; maintenance remains held') from failure
            if token and not activated:
                try:self.api("/api/internal/deployment/release",{"token":token})
                except Exception:pass
            self.progress.update(getattr(error,"phase","waiting_idle"),str(error),progress=0)
            raise
        except Exception as error:
            recovery=[]
            network_recovered=True
            if hotspot_attempted and not released:
                try:self.restore_hotspot(hotspot)
                except Exception as failure:
                    network_recovered=False;recovery.append('Hotspot rollback failed: '+str(failure))
            # Never reset/restart a possibly re-armed controller after handback.
            if token and not released:
                job=read_json(Path(self.config.get("firmware_root","/var/lib/2pybot/firmware"))/"current.json",{})
                if job.get("status") not in {"queued","running","recovering","preparing","recovery_required"}:
                    recovery_ok=network_recovered
                    updated_firmware=job.get('deployment_token')==token and job.get('status')=='succeeded'
                    if updated_firmware:
                        recovery.append('New firmware is verified; keeping its host release and maintenance lock together')
                    elif activated and previous:
                        try:
                            self.switch_release(previous)
                            self.source=Path(previous);self.app=self.source/APP
                            self.venv=Path(state.get("venv",self.state_dir/"venv"))
                            backup=self.state_dir/"backups"/self.commit
                            if backup.is_dir():
                                for saved in backup.rglob("*"):
                                    if saved.is_file():self.write_bytes(self.system_root/saved.relative_to(backup),saved.read_bytes(),backup=False)
                            self.apply(Plan(console=True,units=True,mediamtx=plan.mediamtx,
                                boot=plan.boot,camera=plan.camera,rules=plan.rules))
                            self.install_runtime_worker()
                            self.wait_for_health(had_stream)
                        except Exception as failure:
                            recovery_ok=False;recovery.append('Host rollback was not verified: '+str(failure))
                    elif activated:
                        recovery_ok=False;recovery.append('No previous host release is available; maintenance remains held')
                    if not updated_firmware and recovery_ok:
                        try:self.api("/api/internal/deployment/release",{"token":token})
                        except Exception as failure:recovery.append(str(failure))
                else:recovery.append("Firmware job/recovery still owns maintenance; serial remains paused")
            self.progress.update("failed",str(error),recovery=recovery)
            raise

    def deploy(self, force=False, dry_run=False):
        self.commit = self.stable_head()
        state = self.read_state()
        previous = state.get("commit")
        applied=previous==self.commit and not state.get('bootstrap_host_only',False)
        if applied and self.config.get('one_pull') and self.config.get('auto_firmware',True):
            applied=state.get('firmware_fingerprint')==self.fingerprint()
        if applied and not force:
            if self.config.get("one_pull") and not dry_run:
                pull=read_json(Path(self.config.get("run_dir","/run/2pybot-deploy"))/"pull.json",{})
                shown=read_json(self.state_dir/"progress.json",{})
                from console.deployment_status import process_alive
                interrupted=shown.get("phase") in {"checking","building","deploying","calibrating"} and not process_alive(shown.get("owner_pid"))
                if interrupted or (pull.get("commit")==self.commit and pull.get("updated_at",0)>shown.get("updated_at",0)):
                    self.progress.update("success","Already on the verified deployment",commit=self.commit,progress=100)
            return False
        paths, first = self.changed_paths(previous)
        plan = plan_changes(paths, first=first or force)
        if self.config.get("one_pull"):
            old_progress=read_json(self.state_dir/"progress.json",{})
            pull=read_json(Path(self.config.get("run_dir","/run/2pybot-deploy"))/"pull.json",{})
            if not force and old_progress.get("phase")=="failed" and old_progress.get("commit")==self.commit and pull.get("updated_at",0)<=old_progress.get("updated_at",0):
                return False # failure is latched; a new pull or explicit retry is required
            return self.deploy_coordinated(plan,state,dry_run)
        if (plan.firmware or plan.hotspot) and not dry_run:
            raise GitBusy("Firmware/hotspot changed: install the one-pull configuration before marking this commit deployed")
        print(json.dumps({"commit": self.commit, "previous": previous, "actions": asdict(plan)}), flush=True)
        if dry_run:
            return True
        self.state_dir.mkdir(parents=True, exist_ok=True)
        if any(value for name,value in asdict(plan).items() if name not in {"firmware","android"}):
            had_stream = self.stream_running()
            self.prepare(plan)
            if self.stable_head() != self.commit:
                raise GitBusy("repository changed while preparing deployment")
            self.apply(plan)
            if plan.console or plan.mediamtx:
                self.wait_for_health(had_stream)
            if plan.monitor:
                self.install_monitor()
        record = {
            "commit": self.commit, "previous": previous, "repo": str(self.repo),
            "applied_at": datetime.now(timezone.utc).isoformat(), "actions": asdict(plan),
        }
        self.write_bytes(self.state_file, (json.dumps(record, indent=2) + "\n").encode("utf-8"), backup=False)
        print("deployed: " + self.commit, flush=True)
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--user", default="radxa")
    parser.add_argument("--state-dir", type=Path, default=Path("/var/lib/2pybot-deploy"))
    parser.add_argument("--health-url", default="http://127.0.0.1:8080/api/status")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--install-monitor", action="store_true")
    parser.add_argument("--config",type=Path,default=Path("/etc/2pybot-deploy.json"))
    options = parser.parse_args()
    deployment = Deployment(options.repo, options.user, options.state_dir, options.health_url,config=options.config)
    if options.dry_run:
        deployment.deploy(force=options.force, dry_run=True)
        return 0
    if os.geteuid() != 0:
        parser.error("run deployment with sudo; use --dry-run to inspect without changes")
    import fcntl
    lock_path=Path(deployment.config.get('run_dir','/run/2pybot-deploy'))/'operation.lock' if deployment.config.get('one_pull') else Path('/run/2pybot-deploy.lock')
    with lock_path.open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return 0
        try:
            deployment.deploy(force=options.force)
            if options.install_monitor:
                deployment.install_monitor()
        except (GitBusy,DeploymentDeferred) as error:
            print("deferred: " + str(error), flush=True)
        except Exception as error:
            print("deployment failed: " + str(error), file=sys.stderr, flush=True)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
