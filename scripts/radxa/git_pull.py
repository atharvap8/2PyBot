#!/usr/bin/env python3
"""Installed git wrapper: ordinary Git passthrough, observed fast-forward pull on the managed bot repo."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid


def command_position(args):
    i = 0
    while i < len(args):
        arg = args[i]
        if arg in {"-C", "-c", "--git-dir", "--work-tree", "--namespace", "--config-env"}:
            i += 2
        elif arg.startswith("-"):
            i += 1
        else:
            return i
    return None


def atomic(path, value):
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        temporary.write_text(json.dumps(value) + "\n", encoding="utf-8")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    config_path = Path(os.environ.get("BOT_DEPLOY_CONFIG", "/etc/2pybot-deploy.json"))
    try:
        config = json.loads(config_path.read_text())
    except (OSError, ValueError):
        config = {}
    real_git = config.get("real_git", "/usr/bin/git")
    if "--help" in args[:1] or "--version" in args[:1]:return subprocess.call([real_git,*args])
    index = command_position(args)
    if index is None or args[index] != "pull" or not config.get("repo"):
        return subprocess.call([real_git, *args])
    result = subprocess.run([real_git, *args[:index], "rev-parse", "--show-toplevel"], capture_output=True, text=True)
    if result.returncode or Path(result.stdout.strip()).resolve() != Path(config["repo"]).resolve():
        return subprocess.call([real_git, *args])
    run_dir = Path(config.get("run_dir", "/run/2pybot-deploy"))
    if not run_dir.is_dir() or not os.access(run_dir, os.W_OK):
        print("Deployment runtime directory is unavailable; install/start the deployment units.", file=sys.stderr)
        return 2
    # The coordinator holds this same lock through activation/handback. Release
    # our pull lock before waiting for it, so neither command can race the other.
    with (run_dir / "operation.lock").open("a+") as lock:
        if os.name == "posix":
            import fcntl
            try:fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                print("A deployment or another pull is in progress; retry after it finishes.", file=sys.stderr)
                return 75
        return managed_pull(args, index, config, real_git, run_dir, lock)


def managed_pull(args, index, config, real_git, run_dir, lock):
    def refuse(message):
        print(message,file=sys.stderr)
        if run_dir.is_dir() and os.access(run_dir,os.W_OK):
            atomic(run_dir/'pull.json',{'phase':'failed','message':message,'updated_at':time.time(),
                'finished_at':time.time(),'owner_pid':os.getpid(),'repo':config['repo']})
        return 2
    branch=subprocess.check_output([real_git,*args[:index],"branch","--show-current"],text=True).strip()
    if config.get('branch') and branch!=config['branch']:
        return refuse('Stay on the configured bot deployment branch: '+config['branch'])
    if any(arg.startswith("--rebase") or arg=='-r' or arg.startswith('-r=') for arg in args[index + 1:]):
        return refuse("The managed bot checkout uses fast-forward pulls; rebase locally on Windows.")
    dirty = subprocess.check_output([real_git, *args[:index], "status", "--porcelain", "--untracked-files=no"], text=True)
    if dirty.strip():
        return refuse("Bot has tracked local edits; preserve them before pulling.")
    status = {"schema": 1, "job_id": uuid.uuid4().hex, "phase": "pulling", "progress": 0,
              "started_at": time.time(), "owner_pid": os.getpid(), "repo": config["repo"]}
    stop = threading.Event()
    def heartbeat():
        while not stop.is_set():
            status["updated_at"] = time.time()
            atomic(run_dir / "pull.json", status)
            stop.wait(.5)
    thread = threading.Thread(target=heartbeat, daemon=True)
    thread.start()
    code=130
    try:
        code = subprocess.call([real_git, *args[:index + 1], "--ff-only", *args[index + 1:]])
    except KeyboardInterrupt:
        code=130
    finally:
        stop.set(); thread.join()
    status.update(updated_at=time.time(), finished_at=time.time())
    if code:
        status.update(phase="failed", message=f"git pull failed (exit {code}); see terminal output")
        atomic(run_dir / "pull.json", status)
        return code
    target = subprocess.check_output([real_git, *args[:index], "rev-parse", "HEAD"], text=True).strip()
    status.update(phase="queued", commit=target, message="Pull completed; waiting for deployment")
    atomic(run_dir / "pull.json", status)
    (run_dir / "request").touch()
    if os.name == "posix":
        import fcntl
        fcntl.flock(lock, fcntl.LOCK_UN)
    deadline = time.monotonic() + config.get("wait_seconds", 900)
    previous = None
    while time.monotonic() < deadline:
        try:
            state = json.loads((Path(config.get("state_dir", "/var/lib/2pybot-deploy")) / "progress.json").read_text())
        except (OSError, ValueError):
            state = {}
        if state.get("commit") == target and state.get("updated_at", 0) >= status["updated_at"]:
            line = (state.get("phase"), state.get("message"))
            if line != previous:
                print("[2PyBot] " + str(line[0]) + ": " + str(line[1]), flush=True); previous = line
            if state.get("phase") == "success":
                return 0
            if state.get("phase") == "failed":
                return 1
            if state.get("phase") == "waiting_idle":
                print("Deployment is queued; it will resume after local stop. No second pull is needed.")
                return 75
        time.sleep(.25)
    print("Source pulled; deployment is still pending. Inspect /api/deployment/status.", file=sys.stderr)
    return 75


if __name__ == "__main__":
    raise SystemExit(main())
