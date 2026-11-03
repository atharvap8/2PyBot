#!/usr/bin/env python3
"""Select and deploy a managed bot branch, including narrow-fetch checkouts.

Bootstrap from the bot repository (the helper need not be checked out yet):
git fetch origin hardware/bldc-self-uprighting && git show FETCH_HEAD:scripts/radxa/deploy_branch.py | sudo python3 -
No arming, drive command, hard reset or Git configuration rewrite is performed.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import pwd
import subprocess
import sys
import tempfile
import time
import urllib.request
import uuid

MONITORS = ["2pybot-deploy.path", "2pybot-deploy.timer"]


def atomic(path, value, uid, gid, mode=0o644):
    path = Path(path)
    fd, temporary = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as file:
            json.dump(value, file, indent=2)
            file.write("\n")
            file.flush()
            os.fsync(file.fileno())
            os.fchmod(file.fileno(), mode)
            os.fchown(file.fileno(), uid, gid)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("branch", nargs="?", default="hardware/bldc-self-uprighting")
    parser.add_argument("--config", type=Path, default=Path("/etc/2pybot-deploy.json"))
    args = parser.parse_args()
    if os.geteuid() != 0:
        raise RuntimeError("Run the helper with sudo")
    config = json.loads(args.config.read_text())
    if not config.get("one_pull"):
        raise RuntimeError("This helper needs the existing managed bot deployment")
    repo = Path(config["repo"]).resolve()
    account = pwd.getpwnam(config.get("user", "radxa"))
    git_command = ["runuser", "-u", account.pw_name, "--", config.get("real_git", "/usr/bin/git")]

    def git(*arguments, check=True):
        return subprocess.run(git_command + list(arguments), cwd=repo, check=check,
                              capture_output=True, text=True,
                              env=dict(os.environ, GIT_TERMINAL_PROMPT="0"))

    def api(path):
        with urllib.request.urlopen(config.get("api", "http://127.0.0.1:8080").rstrip("/") + path, timeout=10) as response:
            return json.load(response)

    git("check-ref-format", "--branch", args.branch)
    run_dir = Path(config.get("run_dir", "/run/2pybot-deploy"))
    with (run_dir / "operation.lock").open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("A deployment/pull is already running; let it finish") from error
        status, firmware = api("/api/status"), api("/api/firmware/status")
        deployment = api("/api/deployment/status")
        tel, debug = status.get("telemetry", {}), status.get("debug", {})
        if (not status.get("connected") or tel.get("stale", True) or not tel.get("startup_ready")
                or tel.get("balancing") or tel.get("self_righting")
                or abs(debug.get("v", 0)) > .03 or abs(debug.get("w", 0)) > 10):
            raise RuntimeError("Press SELECT and keep the READY bot stationary, then rerun this same command")
        if firmware.get("busy") or (firmware.get("job") or {}).get("status") == "recovery_required":
            raise RuntimeError("Firmware update/recovery is active; let it finish")
        if deployment.get("maintenance") or deployment.get("deployment_maintenance"):
            raise RuntimeError("Existing maintenance owns the controller; inspect bot deploy-status before changing branch")
        if git("status", "--porcelain", "--untracked-files=no").stdout.strip():
            raise RuntimeError("Tracked local edits found; they were preserved. Commit/stash them before branch deployment")
        previously_active = [unit for unit in MONITORS if subprocess.run(
            ["systemctl", "is-active", "--quiet", unit]).returncode == 0]
        subprocess.run(["systemctl", "stop", *MONITORS], check=True)
        try:
            print("[2PyBot] Fetching " + args.branch, flush=True)
            remote_ref = "refs/remotes/origin/" + args.branch
            git("fetch", "origin", "refs/heads/" + args.branch + ":" + remote_ref)
            target = git("rev-parse", "--verify", remote_ref + "^{commit}").stdout.strip()
            if git("show-ref", "--verify", "--quiet", "refs/heads/" + args.branch, check=False).returncode == 0:
                git("switch", args.branch)
                git("merge", "--ff-only", target)
            else:
                git("switch", "-c", args.branch, target)
            original = args.config.stat()
            config["branch"] = args.branch
            atomic(args.config, config, original.st_uid, original.st_gid, original.st_mode & 0o777)
            queued_at = time.time()
            atomic(run_dir / "pull.json", {
                "schema": 1, "job_id": uuid.uuid4().hex, "phase": "queued", "commit": target,
                "updated_at": queued_at, "owner_pid": os.getpid(), "repo": str(repo),
                "message": "Requested by one-command branch deployment",
            }, account.pw_uid, account.pw_gid)
            request = run_dir / "request"
            request.touch()
            os.chown(request, account.pw_uid, account.pw_gid)
        except BaseException:
            if previously_active:
                subprocess.run(["systemctl", "start", *previously_active], check=False)
            raise
    # Release the coordinator lock before starting its service. Background jobs
    # keep running even if SSH closes; this process only observes their progress.
    subprocess.run(["systemctl", "start", "--no-block", *MONITORS, "2pybot-deploy.service"], check=True)
    print("[2PyBot] Deploying " + target[:8] + "; matching host/firmware will be installed", flush=True)
    deadline, previous = time.monotonic() + config.get("wait_seconds", 900), None
    while time.monotonic() < deadline:
        try:
            progress = api("/api/deployment/status").get("progress", {})
        except (OSError, ValueError):
            time.sleep(1)
            continue  # brief console restart during activation
        if progress.get("commit") == target and progress.get("updated_at", 0) >= queued_at:
            message = (progress.get("phase"), progress.get("message"))
            if message != previous:
                print("[2PyBot] {}: {}".format(*message), flush=True)
                previous = message
            if message[0] == "failed":
                raise RuntimeError("Deployment failed: " + str(message[1]))
            if message[0] == "success":
                installed = api("/api/firmware/status").get("installed") or {}
                params = api("/api/params")
                readback_deadline = time.monotonic() + 10
                while not params.get("complete"):
                    if time.monotonic() > readback_deadline:
                        raise RuntimeError("Deployment finished; complete settings readback is still unavailable")
                    time.sleep(.2)
                    params = api("/api/params")
                print("[2PyBot] Installed firmware: " + str(installed.get("version", "unknown")), flush=True)
                print("[2PyBot] SELFRIGHT=" + str(params.get("derived", {}).get("SELFRIGHT", 0)), flush=True)
                print("[2PyBot] Complete. START remains a local operator action.", flush=True)
                return 0
        time.sleep(1)
    print("[2PyBot] Still queued/running. Follow with: bot deploy-status --watch", flush=True)
    return 75


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError, subprocess.CalledProcessError) as error:
        detail = error.stderr.strip() if isinstance(error, subprocess.CalledProcessError) and error.stderr else str(error)
        print("[2PyBot] " + detail, file=sys.stderr)
        raise SystemExit(1)
