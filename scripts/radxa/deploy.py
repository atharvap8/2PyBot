#!/usr/bin/env python3
"""Apply relevant changes from a completed local Git update. Does not fetch or pull."""

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from urllib.request import urlopen


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


def plan_changes(paths, first=False):
    plan = Plan(**{name: first for name in Plan.__dataclass_fields__})
    for path in paths:
        if path == APP + "requirements.txt":
            plan.dependencies = plan.console = True
        elif path.startswith(APP + "console/") and path.endswith(".py"):
            plan.console = True
        elif path.startswith(APP + "ui/") and path.endswith((".js", ".html", ".css")):
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
        elif path in (MONITOR + "2pybot-deploy.service", MONITOR + "2pybot-deploy.timer"):
            plan.monitor = True
    return plan


class GitBusy(RuntimeError):
    pass


class Deployment:
    def __init__(self, repo, user, state_dir, health_url, system_root=Path("/")):
        self.repo = Path(repo).resolve()
        self.user = user
        self.state_dir = Path(state_dir)
        self.state_file = self.state_dir / "state.json"
        self.venv = self.state_dir / "venv"
        self.app = self.repo / APP
        self.health_url = health_url
        self.system_root = Path(system_root)
        self.commit = ""
        self.environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")

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
        }
        for name in ("2pybot-deploy.service", "2pybot-deploy.timer"):
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

    def apply(self, plan):
        reload_needed = False
        if plan.units:
            content = (self.app / "systemd/2pybot-console.service").read_text(encoding="utf-8")
            content = content.replace("__USER__", self.user).replace("__DIR__", str(self.app))
            content = content.replace("ExecStart=/usr/bin/python3", "ExecStart=" + str(self.venv / "bin/python3"))
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

    def deploy(self, force=False, dry_run=False):
        self.commit = self.stable_head()
        state = self.read_state()
        previous = state.get("commit")
        if previous == self.commit and not force:
            return False
        paths, first = self.changed_paths(previous)
        plan = plan_changes(paths, first=first or force)
        print(json.dumps({"commit": self.commit, "previous": previous, "actions": asdict(plan)}), flush=True)
        if dry_run:
            return True
        self.state_dir.mkdir(parents=True, exist_ok=True)
        if any(asdict(plan).values()):
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
    options = parser.parse_args()
    deployment = Deployment(options.repo, options.user, options.state_dir, options.health_url)
    if options.dry_run:
        deployment.deploy(force=options.force, dry_run=True)
        return 0
    if os.geteuid() != 0:
        parser.error("run deployment with sudo; use --dry-run to inspect without changes")
    import fcntl
    with open("/run/2pybot-deploy.lock", "w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return 0
        try:
            deployment.deploy(force=options.force)
            if options.install_monitor:
                deployment.install_monitor()
        except GitBusy as error:
            print("deferred: " + str(error), flush=True)
        except Exception as error:
            print("deployment failed: " + str(error), file=sys.stderr, flush=True)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
