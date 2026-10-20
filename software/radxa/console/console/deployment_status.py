"""Shared deployment state and LED vocabulary. No hardware or service side effects."""
import json
import math
import os
from pathlib import Path
import time
import uuid

STATE_DIR = Path(os.environ.get("BOT_DEPLOY_STATE", "/var/lib/2pybot-deploy"))
RUN_DIR = Path(os.environ.get("BOT_DEPLOY_RUN", "/run/2pybot-deploy"))
PHASES = {"idle": 0, "pulling": 1, "checking": 2, "waiting_artifact":2, "building": 3, "deploying": 4,
          "flashing": 5, "calibrating": 6, "success": 7, "failed": 8, "waiting_idle": 9}
ACTIVE = {"pulling", "queued", "checking", "building", "deploying", "flashing", "calibrating", "waiting_idle", "waiting_artifact"}


def read(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default


def atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as file:
            json.dump(value, file, indent=2, allow_nan=False)
            file.write("\n")
            file.flush()
            os.fsync(file.fileno())
        temporary.chmod(0o644)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def process_alive(pid):
    if not isinstance(pid, int) or pid <= 0:
        return False
    try:
        os.kill(pid, 0)
        return True
    except PermissionError:
        return True
    except OSError:
        return False


def indicator(state_dir=STATE_DIR, run_dir=RUN_DIR, firmware_root=None, now=None):
    """Choose the newest event without mistaking a completed pull for deployment success."""
    now = time.time() if now is None else now
    progress = read(Path(state_dir) / "progress.json", {})
    pull = read(Path(run_dir) / "pull.json", {})
    status = progress
    # A pull heartbeat must be newer than the deployment it is replacing.
    if pull.get("updated_at", 0) > progress.get("updated_at", 0):
        status = pull
    status = dict(status)
    phase = status.get("phase", "idle")
    if phase == "queued":
        phase = "checking"
    unexpected_stop = status.get("phase") in ACTIVE - {"queued", "waiting_idle", "waiting_artifact"} and status.get("owner_pid") and not process_alive(status["owner_pid"])
    if unexpected_stop:
        phase = "failed"
        status["message"] = "Deployment/pull process stopped unexpectedly"
    if firmware_root and phase in {"flashing", "calibrating"}:
        job = read(Path(firmware_root) / "current.json", {})
        if job.get("id") == status.get("firmware_job_id"):
            status["progress"] = job.get("progress", 0)
            if job.get("stage") in {"calibrating", "restoring"}:
                phase = "calibrating"
    recovery_required=firmware_root and read(Path(firmware_root)/"current.json",{}).get('status')=='recovery_required'
    if recovery_required:
        phase='failed';status['message']='Controller recovery required; maintenance remains held'
    finished = status.get("finished_at", status.get("updated_at", 0))
    if unexpected_stop:finished=now # keep an interrupted operation visibly failed until retry
    if recovery_required:finished=now
    if phase == "success" and now - finished > 6:
        phase = "idle"
    elif phase == "failed" and now - finished > 30:
        phase = "idle"
    status.update(phase=phase, phase_code=PHASES.get(phase, 0),
                  progress=max(0, min(100, int(status.get("progress", 0)))),
                  led_active=phase != "idle")
    return status


class Progress:
    def __init__(self, directory=STATE_DIR, clock=time.time):
        self.directory = Path(directory)
        self.clock = clock
        self.value = {}

    def update(self, phase, message, **fields):
        if phase not in PHASES and phase != "queued":
            raise ValueError("Unknown deployment phase")
        now = self.clock()
        self.value.update(fields, phase=phase, message=message, updated_at=now, owner_pid=os.getpid())
        self.value.setdefault("started_at", now)
        self.value.setdefault("job_id", uuid.uuid4().hex)
        if phase in {"success", "failed"}:
            self.value["finished_at"] = now
        atomic(self.directory / "progress.json", self.value)
        return dict(self.value)


def led_frame(phase, progress, seconds, count=16, brightness=.35):
    """One smooth, four-second blended circular spin for deployment ownership."""
    if not 1 <= count <= 256 or not math.isfinite(seconds):
        raise ValueError("Invalid LED geometry/time")
    brightness = max(0, min(.6, brightness))
    first,second=(.02,.75,1),(.6,.12,1)
    if phase=='success':first,second=(.06,1,.25),(.03,.45,.08)
    elif phase=='failed':first,second=(1,.05,.02),(.45,.01,.01)
    elif phase=='waiting_idle':first,second=(1,.5,.05),(1,.15,.01)
    elif phase not in PHASES or phase=='idle':first=second=(0,0,0)
    frame = []
    for i in range(count):
        angle=2*math.pi*(i/count-seconds/4)
        blend=.5+.5*math.sin(angle)
        level=.25+.65*(.5+.5*math.cos(angle))
        frame.append(tuple(round(255*brightness*(a*(1-blend)+b*blend)*level) for a,b in zip(first,second)))
    return frame
