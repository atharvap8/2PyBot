#!/usr/bin/env python3
"""Directory-independent operator commands for the local 2PyBot. Python 3.9+."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import time
import uuid
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bot_view as V  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SERVICES = {
    "console": "2pybot-console.service", "firmware": "2pybot-firmware.service",
    "deploy": "2pybot-deploy.service", "boot": "2pybot-boot-gate.service",
    "camera": "2pybot-mediamtx.service",
}
BUSY = {"preparing", "queued", "running", "recovering"}


def read(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError:
        return default


def atomic(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


JSON_MODE = False
COLOUR = sys.stdout.isatty() and os.environ.get("NO_COLOR") is None


def c(text, tone):
    return V.colour(str(text), tone, COLOUR)


def _scalar(v):
    if isinstance(v, bool):
        return c("yes", "ok") if v else c("no", "dim")
    if isinstance(v, float):
        return "{:.6g}".format(v)
    if v is None:
        return c("-", "dim")
    return str(v)


def print_tree(data, indent=0, out=print):
    """Readable nested key/value view (replaces raw JSON)."""
    pad = "  " * indent
    if isinstance(data, dict):
        width = max((len(str(k)) for k in data), default=0) + 2
        for k, v in data.items():
            label = str(k).replace("_", " ")
            if isinstance(v, (dict, list)) and v and not (isinstance(v, list) and all(not isinstance(x, (dict, list)) for x in v)):
                out(pad + c(label, "acc"))
                print_tree(v, indent + 1, out)
            elif isinstance(v, list):
                out(pad + c(label.ljust(width), "dim") + ", ".join(_scalar(x) for x in v[:12]) + (" ..." if len(v) > 12 else ""))
            else:
                text = _scalar(v)
                out(pad + c(label.ljust(width), "dim") + (text if len(text) < 160 else text[:157] + "..."))
    elif isinstance(data, list):
        for i, item in enumerate(data):
            if isinstance(item, (dict, list)):
                out(pad + c("- item {}".format(i + 1), "dim"))
                print_tree(item, indent + 1, out)
            else:
                out(pad + "- " + _scalar(item))
    else:
        out(pad + _scalar(data))


def show(data):
    if JSON_MODE:
        print(json.dumps(data, indent=2))
    else:
        print_tree(data)


def stamp():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")


class Bot:
    def __init__(self, args):
        self.args = args
        self.config_path = args.config.expanduser().resolve()
        self.config = read(self.config_path, {})
        self.repo = Path(self.config.get("repo", ROOT)).expanduser().resolve()
        self.base = args.api or self.config.get("api", "http://127.0.0.1:8080")
        parts = urlsplit(self.base)
        if parts.scheme != "http" or parts.hostname not in {"127.0.0.1", "localhost", "::1"} or parts.username or parts.password:
            raise ValueError("Bot commands use a local loopback HTTP API, for example http://127.0.0.1:8080")
        self.base = self.base.rstrip("/")
        self.firmware_root = Path(self.config.get("firmware_root", "/var/lib/2pybot/firmware")).expanduser().resolve()
        self.backups = Path(self.config.get("backups", Path.home() / ".local/share/2pybot/backups")).expanduser().resolve()

    def api(self, path, body=None, timeout=5):
        request = Request(self.base + path, data=None if body is None else json.dumps(body).encode(),
                          headers={"Content-Type": "application/json"})
        try:
            with urlopen(request, timeout=timeout) as response:
                return json.load(response)
        except HTTPError as error:
            raise RuntimeError("API refused request: " + error.read().decode(errors="replace")) from error
        except (URLError, OSError) as error:
            raise RuntimeError("Local console is unavailable: " + str(getattr(error, "reason", error))) from error
        except json.JSONDecodeError as error:
            raise RuntimeError("Local console returned invalid JSON") from error

    def firmware_module(self):
        sys.path.insert(0, str(self.repo / "software/radxa/console"))
        from console import firmware
        return firmware

    def idle(self):
        status = self.api("/api/status")
        tel = status.get("telemetry", {})
        if not status.get("connected") or tel.get("stale", True) or not tel.get("startup_ready") or not status.get("dump_complete"):
            raise RuntimeError("Fresh READY controller telemetry is required. Use 'bot restore firmware' for an erased ESP32.")
        if tel.get("balancing") or abs(status.get("debug", {}).get("v", 0)) > .03 or abs(status.get("debug", {}).get("w", 0)) > 5:
            raise RuntimeError("Press local SELECT and keep the bot stationary first.")
        fw = self.api("/api/firmware/status")
        if fw.get("busy") or (fw.get("job") or {}).get("status") == "recovery_required" or tel.get("firmware_lock"):
            raise RuntimeError("Firmware maintenance/recovery currently owns the controller.")
        return status

    def status(self):
        if JSON_MODE:
            show({"host": self.api("/api/ready"), "controller": self.api("/api/status"),
                  "firmware": self.api("/api/firmware/status")})
            return
        snap = V.Collector(V.Api(self.base)).snapshot()
        if not snap.get("status"):
            raise RuntimeError("the console is not answering on " + self.base + " (is 2pybot-console running? try: 2pybot services)")
        V.print_sections(V.dashboard(snap), COLOUR)

    def firmware_status(self):
        fw = self.api("/api/firmware/status")
        if JSON_MODE:
            return show(fw)
        inst, latest, job = fw.get("installed") or {}, fw.get("latest") or {}, fw.get("job") or {}
        same = inst.get("build_id") and inst.get("build_id") == latest.get("build_id")
        rows = [("Installed", "{}  built {}  from {}".format(inst.get("version", "-"), (inst.get("built_at") or "")[:16].replace("T", " "),
                                                             V.short(inst.get("source_commit"))), "ok" if inst else "warn"),
                ("Latest build", "{}  from {}".format(latest.get("version", "-"), V.short(latest.get("source_commit"))), "ok" if same else "warn"),
                ("Up to date", "yes" if same else "no, a newer build is registered", "ok" if same else "warn"),
                ("Can flash now", "yes" if fw.get("can_flash") else "no: " + str(fw.get("blocked_reason") or "-"), "ok" if fw.get("can_flash") else "dim")]
        if job:
            rows.append(("Last update job", "{} · {} · {}".format(job.get("status"), job.get("stage"), job.get("message", "")),
                         {"succeeded": "ok", "failed": "err", "recovery_required": "err"}.get(job.get("status"), "warn")))
        V.print_sections([("Controller firmware", rows)], COLOUR)

    def services(self):
        states = V.service_states()
        if JSON_MODE:
            return show(states)
        tones = {i: ("ok" if s["active"] == "active" else "dim" if s["name"] in ("deploy", "boot", "imx708") else "err")
                 for i, s in enumerate(states)}
        V.table(["SERVICE", "STATE", "AT BOOT", "PURPOSE"], [[s["name"], s["active"], s["enabled"], s["desc"]] for s in states], COLOUR, tones)

    def controller_log(self):
        lines = V.log_tail(self.args.lines)
        for line in lines:
            if "[PAD] btn=" not in line:
                print(c(line, V.log_tone(line)))
        if not self.args.follow:
            return
        path = V.CONTROLLER_LOG
        with open(path, "rb") as f:
            f.seek(0, 2)
            while True:
                chunk = f.readline()
                if not chunk:
                    time.sleep(0.3)
                    continue
                line = chunk.decode("utf-8", "replace").rstrip()
                if "[PAD] btn=" not in line and line.count("\ufffd") < 5:
                    print(c(line, V.log_tone(line)), flush=True)

    def deploy_status(self):
        previous = None
        while True:
            value = self.api("/api/deployment/status")
            if not self.args.watch:
                if JSON_MODE:
                    return show(value)
                p, a = value.get("progress", {}), value.get("applied", {})
                V.print_sections([("Deployment", [
                    ("Phase", "{} ({}%)".format(p.get("phase", "-"), p.get("progress", "-")), {"success": "ok", "failed": "err"}.get(p.get("phase"), "warn")),
                    ("Message", p.get("message", "-"), None),
                    ("Target commit", V.short(p.get("commit")), None),
                    ("Running commit", V.short(a.get("commit")), None),
                    ("Previous commit", V.short(a.get("previous")), "dim")])], COLOUR)
                return
            progress = value.get("progress", {})
            line = (progress.get("phase"), progress.get("message"), progress.get("progress"))
            if line != previous:
                print("{} {}%: {}".format(line[0], line[2], line[1]), flush=True)
                previous = line
            if line[0] in {"success", "failed"}:
                if line[0] == "failed":
                    raise RuntimeError(str(line[1]))
                return
            time.sleep(1)

    def build(self):
        for source in (self.repo / "firmware/BaseLink").iterdir():
            if source.suffix in {".ino", ".h", ".cpp"} and re.search(r"^(?:<<<<<<< |=======\s*$|>>>>>>> )", source.read_text(), re.MULTILINE):
                raise ValueError("Resolve source merge-conflict markers before building: " + str(source))
        if not re.fullmatch(r"\d+\.\d+\.\d+", (self.repo / "VERSION").read_text().strip()):
            raise ValueError("VERSION must contain one release version, for example 4.1.0; resolve any merge-conflict markers")
        output = self.args.output.expanduser().resolve() if self.args.output else self.repo / "build/BaseLink"
        cli = self.args.arduino_cli or self.config.get("arduino_cli", "arduino-cli")
        if not shutil.which(cli):
            raise ValueError("arduino-cli is not installed/on PATH. Install it, then run: bot toolchain install")
        subprocess.run([sys.executable, str(self.repo / "scripts/firmware/build.py"),
                        "--arduino-cli", cli, "--output-dir", str(output), "--require-toolchain-lock"],
                       cwd=self.repo, check=True)
        shutil.copy2(output / "BaseLink.ino.bin", output / "application.bin")
        manifest = self.firmware_module().register_build(self.firmware_root, output)
        print("Built and registered {} ({})".format(manifest["version"], manifest["build_id"]))
        print("Flash it with: flash-fw")

    def flash(self):
        fw = self.api("/api/firmware/status")
        if not fw.get("can_flash"):
            raise RuntimeError(fw.get("blocked_reason") or "Flashing is unavailable")
        build = fw.get("latest") or {}
        if self.args.build and self.args.build != build.get("build_id"):
            raise ValueError("--build must match the latest registered build; run build-firmware or register the intended image first")
        if not build.get("build_id"):
            raise ValueError("No compiled firmware is registered; run build-firmware first")
        result = self.api("/api/firmware/reflash", {"build_id": build["build_id"]}, timeout=20)
        job_id = (result.get("job") or {}).get("id")
        print("Queued firmware job: " + str(job_id), flush=True)
        if self.args.no_wait:
            return
        previous = None
        deadline = time.monotonic() + 900
        while time.monotonic() < deadline:
            job = self.api("/api/firmware/status").get("job") or {}
            if job.get("id") != job_id:
                raise RuntimeError("Firmware job changed; inspect flash status")
            line = (job.get("stage"), job.get("progress"), job.get("message"))
            if line != previous:
                print("{} {}%: {}".format(*line), flush=True)
                previous = line
            if job.get("status") == "succeeded":
                print("Complete. Local START is required to arm.")
                return
            if job.get("status") in {"failed", "recovery_required"}:
                raise RuntimeError(job.get("error") or job.get("message"))
            time.sleep(1)
        raise RuntimeError("Still running on the worker. Use bot firmware-status to inspect it.")

    def set_values(self, values, save=False):
        self.idle()
        current = {p["key"]: p for p in self.api("/api/params")["params"]}
        for key, value in values.items():
            if key not in current or not math.isfinite(value) or not current[key]["lo"] <= value <= current[key]["hi"]:
                raise ValueError("Unknown parameter or out-of-range value: " + key)
        for key, value in values.items():
            answer=self.api("/api/params/set", {"key": key, "value": value},timeout=20)
            if not answer.get("confirmed"):raise RuntimeError("Old bot API did not confirm the write; update the bot to 4.2.0")
        deadline = time.monotonic() + 10
        while True:
            snapshot = self.api("/api/params")
            actual = {p["key"]: p["value"] for p in snapshot["params"]}
            if snapshot.get("complete") and all(abs(actual.get(k, math.inf) - v) <= max(.0001, abs(v) * .00001) for k, v in values.items()):
                break
            if time.monotonic() > deadline:
                raise RuntimeError("Parameter readback did not confirm all writes; nothing was saved to NVS")
            time.sleep(.2)
        if save:
            self.idle()
            answer=self.api("/api/params/save", {},timeout=25)
            if not answer.get("confirmed") or not answer.get("persisted"):raise RuntimeError("NVS persistence was not confirmed")
        if JSON_MODE:
            show({"applied": values, "save_requested": save})
        else:
            for k, v in values.items():
                print("{} = {:.6g}".format(c(k, "acc"), v))
            print(c("Saved to flash." if save else "Applied (not saved: add --save, or run: 2pybot params save).", "ok" if save else "warn"))

    def calibration(self):
        kind = self.args.kind
        if self.args.save and kind != "trim":
            raise ValueError("--save applies to upright trim calibration")
        if self.args.upright and kind != "trim":
            raise ValueError("--upright applies to trim calibration")
        if self.args.reference is not None and kind != "tof":
            raise ValueError("--reference applies to ToF calibration")
        self.idle()
        if kind == "trim":
            if not self.args.upright:
                raise ValueError("Hold the chassis physically upright and use: calibration trim --upright [--save]")
            samples = []
            for _ in range(10):
                status = self.api("/api/status")
                tel = status["telemetry"]
                if tel.get("stale", True) or tel.get("balancing") or abs(status.get("debug", {}).get("w", 0)) > 3:
                    raise RuntimeError("Keep the stopped chassis upright and still throughout trim sampling")
                samples.append(float(tel["pitch"]))
                time.sleep(.1)
            if not all(math.isfinite(v) for v in samples) or max(samples) - min(samples) > .5:
                raise RuntimeError("Upright readings were not stable; trim was not changed")
            self.set_values({"TRIM": sum(samples) / len(samples)}, self.args.save)
        elif kind == "imu":
            started = time.time()
            self.api("/api/command", {"line": "C"})
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                for entry in self.api("/api/log?n=100").get("log", []):
                    if entry.get("t", 0) < started:
                        continue
                    text = entry.get("text", "")
                    if "[IMU] ERROR" in text or "calibration failed" in text:
                        raise RuntimeError(text)
                    if "[IMU] Gyro offsets" in text:
                        print(text)
                        return
                time.sleep(.3)
            raise RuntimeError("Calibration was requested but completion was not observed; inspect bot logs")
        elif kind == "tof":
            if self.args.reference is None or not math.isfinite(self.args.reference) or self.args.reference <= 0:
                raise ValueError("Supply the measured target distance in metres: calibration tof --reference 0.5")
            show(self.api("/api/tof/calibrate", {"reference_m": self.args.reference}))
        else:
            show(self.api("/api/camera/autocalibrate", {}, timeout=90))

    def diagnostics(self):
        report = {"created_at": datetime.now(timezone.utc).isoformat(), "repo": str(self.repo)}
        for name, path in [("status", "/api/status"), ("firmware", "/api/firmware/status"),
                           ("deployment", "/api/deployment/status"), ("params", "/api/params"), ("log", "/api/log?n=100")]:
            try:
                report[name] = self.api(path, timeout=3)
            except RuntimeError as error:
                report[name] = {"error": str(error)}
        commands = {
            "services": ["systemctl", "is-active", *SERVICES.values()],
            "wifi": ["nmcli", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"],
            "radios": ["iw", "dev"], "routes": ["ip", "-4", "route"],
            "dns": ["nmcli", "-f", "GENERAL.DEVICE,IP4.GATEWAY,IP4.DNS", "device", "show"],
            "resolve_github": ["getent", "ahostsv4", "github.com"], "usb": ["lsusb"],
        }
        for name, command in commands.items():
            try:
                result = subprocess.run(command, capture_output=True, text=True, timeout=6)
                report[name] = {"exit": result.returncode, "stdout": result.stdout, "stderr": result.stderr}
            except (OSError, subprocess.TimeoutExpired) as error:
                report[name] = {"error": str(error)}
        if JSON_MODE and not self.args.output:
            return show(report)
        out = (self.args.output or Path("~/2pybot-report-" + time.strftime("%Y%m%d-%H%M%S") + ".json")).expanduser().resolve()
        atomic(out, report)
        try:
            self.status()
        except RuntimeError as error:
            print(c(str(error), "err"))
        counts, last = V.event_summary()
        V.print_sections([("Last 24 hours", [(k, str(v) + (("   last " + last[k]) if k in last else ""), "warn" if v else "dim")
                                             for k, v in counts.items()])], COLOUR)
        print("Full report saved: " + c(str(out), "info"))

    def prefix(self):
        paths = sorted(self.firmware_root.glob("jobs/*/boot-nvs-before.bin"), key=lambda p: p.stat().st_mtime, reverse=True)
        for path in paths:
            try:
                data = path.read_bytes()
                self.validate_prefix(data)
                expected = read(path.parent / "backup.json", {}).get("boot_nvs_sha256")
                if expected and hashlib.sha256(data).hexdigest() != expected:
                    continue
                return path
            except (OSError, ValueError):
                continue
        raise ValueError("No valid saved boot/NVS prefix found. Full erased-chip recovery needs the original bootloader/partition files.")

    @staticmethod
    def validate_prefix(data):
        if len(data) != 0x10000 or data[0x1000] != 0xe9:
            raise ValueError("Invalid ESP32 boot/NVS prefix")
        apps = []
        for offset in range(0x8000, 0x9000, 32):
            if data[offset:offset+2] != b"\xaaP":
                break
            _, kind, _, address, size, _, _ = struct.unpack_from("<HBBII16sI", data, offset)
            if kind == 0:
                apps.append((address, size))
        if apps != [(0x10000, 0x300000)]:
            raise ValueError("Backup does not have the bot's Huge APP partition layout")

    def registered_build(self, identifier=None):
        identifier = identifier or (read(self.firmware_root / "installed.json", {}) or {}).get("build_id")
        if not identifier:
            raise ValueError("No recorded installed image. Specify a registered --build ID.")
        manifest, image = self.firmware_module().load_build(self.firmware_root, identifier, True)
        if manifest.get("hardware") != "m0601-rs485" or manifest.get("parameter_schema") != "bldc-v4":
            raise ValueError("Recovery image must target this M0601 BLDC bot")
        return manifest, image

    def backup(self):
        manifest, image = self.registered_build(self.args.build)
        prefix = self.prefix()
        destination = self.backups / stamp()
        destination.mkdir(parents=True)
        shutil.copy2(image, destination / "application.bin")
        shutil.copy2(prefix, destination / "boot-nvs.bin")
        atomic(destination / "manifest.json", manifest)
        for name, endpoint in [("params", "/api/params"), ("status", "/api/status")]:
            try:
                atomic(destination / (name + ".json"), self.api(endpoint))
            except RuntimeError as error:
                atomic(destination / (name + ".json"), {"error": str(error)})
        checks = {name: hashlib.sha256((destination / name).read_bytes()).hexdigest()
                  for name in ("application.bin", "boot-nvs.bin", "manifest.json", "params.json", "status.json")}
        atomic(destination / "backup.json", {"schema": 1, "created_at": time.time(), "files": checks,
               "source_prefix": str(prefix), "live_flash_read": False,
               "note": "Copies the verified registered image and saved worker boot/NVS prefix; no device flash read is performed."})
        print("Saved backup: " + str(destination))

    def backup_path(self):
        if self.args.backup:
            return self.args.backup.expanduser().resolve()
        candidates = sorted(p.parent for p in self.backups.glob("*/backup.json"))
        if not candidates:
            raise ValueError("No bot backups found; run bot backup or omit --backup for saved-worker firmware recovery")
        return candidates[-1]

    def verify_backup(self, folder):
        metadata = read(folder / "backup.json", {})
        expected_names = {"application.bin", "boot-nvs.bin", "manifest.json", "params.json", "status.json"}
        if metadata.get("schema") != 1 or set(metadata.get("files", {})) != expected_names:
            raise ValueError("Not a complete bot backup")
        for name, checksum in metadata["files"].items():
            if hashlib.sha256((folder / name).read_bytes()).hexdigest() != checksum:
                raise ValueError("Backup checksum mismatch: " + name)

    def restore(self):
        if self.args.save and self.args.kind != "tuning":
            raise ValueError("--save applies to tuning restoration; firmware recovery restores its selected NVS prefix")
        if self.args.offline and self.args.kind != "firmware":
            raise ValueError("--offline applies to USB firmware recovery")
        if self.args.kind == "list":
            show({"backups": [str(p.parent) for p in sorted(self.backups.glob("*/backup.json"))],
                  "installed": read(self.firmware_root / "installed.json"),
                  "builds": [read(p) for p in sorted(self.firmware_root.glob("builds/*/manifest.json"))]})
            return
        if self.args.kind == "tuning":
            folder = self.backup_path()
            self.verify_backup(folder)
            saved = read(folder / "params.json", {})
            if not saved.get("complete") or saved.get("derived", {}).get("HUBSCHEMA") != 4:
                raise ValueError("Backup has no complete BLDC tuning snapshot")
            self.idle()
            atomic(self.backups / ("tuning-before-" + stamp() + ".json"), self.api("/api/params"))
            self.set_values({p["key"]: float(p["value"]) for p in saved["params"]}, self.args.save)
            return
        self.restore_firmware()

    def restore_firmware(self):
        if self.args.backup and self.args.build:
            raise ValueError("Choose either --backup or --build, not both")
        if os.name != "posix":
            raise RuntimeError("USB recovery runs on the Linux bot")
        if os.geteuid() != 0:
            command = ["sudo", sys.executable, str(Path(__file__).resolve()), "--config", str(self.config_path),
                       "--api", self.base, "restore", "firmware"]
            for key in ("backup", "build"):
                value = getattr(self.args, key)
                if value:
                    command += ["--" + key, str(Path(value).expanduser().resolve()) if key == "backup" else value]
            if self.args.offline:
                command.append("--offline")
            subprocess.run(command, check=True)
            return
        job = read(self.firmware_root / "current.json", {})
        if job.get("status") in BUSY:
            raise RuntimeError("An update job is active. Finish/recover that job before taking over USB.")
        try:
            live = self.api("/api/status")
        except RuntimeError:
            if not self.args.offline:
                raise RuntimeError("Console is unavailable. For a stopped/offline bot use: bot restore firmware --offline")
            live = {}
        if live.get("telemetry", {}).get("balancing"):
            raise RuntimeError("Press local SELECT first; recovery will reset the ESP32")
        firmware = self.firmware_module()
        if self.args.backup:
            folder = self.backup_path()
            self.verify_backup(folder)
            prefix, image = folder / "boot-nvs.bin", folder / "application.bin"
            manifest = read(folder / "manifest.json")
            if manifest.get("hardware") != "m0601-rs485" or manifest.get("parameter_schema") != "bldc-v4":
                raise ValueError("Backup image does not target this BLDC bot")
            if firmware.inspect_image(image)["sha256"] != manifest.get("build_id"):
                raise ValueError("Recovery image does not match its manifest")
            firmware.register_build(self.firmware_root, folder, installed_candidate=True)
        else:
            manifest, image = self.registered_build(self.args.build)
            prefix = self.prefix()
        self.validate_prefix(prefix.read_bytes())
        flasher = Path(self.config.get("flasher_python", "/home/radxa/tools/2pybot-flasher/bin/python3"))
        if not flasher.is_file():
            raise ValueError("Flasher interpreter is missing: " + str(flasher))
        port = self.config.get("serial_port", "/dev/ttyUSB0")
        print("Restoring bootloader/partition table/saved NVS and application:", flush=True)
        print("  Prefix: " + str(prefix), flush=True)
        print("  Image:  " + str(image), flush=True)
        subprocess.run(["systemctl", "stop", "2pybot-deploy.timer", "2pybot-deploy.path", "2pybot-deploy.service",
                        SERVICES["firmware"], SERVICES["console"], SERVICES["boot"]], check=True)
        subprocess.run([str(flasher), "-m", "esptool", "--chip", "esp32", "--port", port, "--baud", "230400",
                        "--after", "no_reset", "write_flash", "0x0", str(prefix), "0x10000", str(image)], check=True)
        atomic(self.firmware_root / "installed.json", {"build_id": manifest["build_id"],
               "verified_at": time.time(), "manual_restore": True, "startup_verified": False})
        if job.get("status") == "recovery_required":
            job.update(status="failed", stage="manual_restore", message="Manual USB recovery completed; waiting for startup", updated_at=time.time())
            atomic(self.firmware_root / "current.json", job)
            if re.fullmatch(r"[a-f0-9]{32}", job.get("id", "")):
                atomic(self.firmware_root / "jobs" / job["id"] / "result.json", job)
        hold = self.firmware_root / "deployment-hold.json"
        if hold.exists():
            atomic(self.backups / ("deployment-hold-before-recovery-" + stamp() + ".json"), read(hold))
            hold.unlink() # successful manual recovery supersedes the interrupted hold
        subprocess.run(["systemctl", "start", SERVICES["boot"], SERVICES["console"], SERVICES["firmware"],
                        "2pybot-deploy.path", "2pybot-deploy.timer"], check=True)
        print("USB write completed. Keep still during calibration; use bot status to check READY. Local START is required.")

    def params(self):
        if self.args.operation in {"get", "list"}:
            result = self.api("/api/params")
            if JSON_MODE and not self.args.key:
                return show(result)
            params = result["params"]
            if self.args.key:
                key = self.args.key.upper()
                groups = {g.upper() for g in result.get("groups", {})}
                if key in groups:
                    params = [p for p in params if p["group"].upper() == key]
                else:
                    matches = [p for p in params if p["key"] == key or key in p["key"]]
                    if not matches and key in result.get("derived", {}):
                        return show({"key": key, "value": result["derived"][key], "read_only": True})
                    if not matches:
                        raise ValueError("Unknown parameter or group: " + key + " (groups: " + ", ".join(result.get("groups", {})) + ")")
                    params = matches
                    if JSON_MODE:
                        return show(params)
            group = None
            for p in params:
                if p["group"] != group:
                    group = p["group"]
                    print(c("\n" + group, "acc"))
                fixed = p["lo"] == p["hi"]
                rng = "fixed" if fixed else "{:.4g}..{:.4g}".format(p["lo"], p["hi"])
                line = "  {:<10} {:>11}  {:<17} {}".format(p["key"], "{:.6g}".format(p["value"]) + ("*" if p.get("dirty") else ""), rng, p.get("desc", ""))
                print(c(line, "dim" if fixed else None))
            if any(p.get("dirty") for p in params):
                print(c("\n* changed but not saved (2pybot params save)", "warn"))
        elif self.args.operation == "set":
            if not self.args.key or self.args.value is None:
                raise ValueError("Usage: bot params set KEY VALUE [--save]")
            self.set_values({self.args.key.upper(): self.args.value}, self.args.save)
        elif self.args.operation == "refresh":
            show(self.api("/api/params/refresh", {}))
        else:
            self.idle()
            answer=self.api("/api/params/" + self.args.operation, {},timeout=25)
            if not answer.get("confirmed"):raise RuntimeError("Controller did not confirm settings operation; update the bot to 4.2.0")
            print(c("Done: " + self.args.operation, "ok") if not JSON_MODE else json.dumps(answer, indent=2))

    def toolchain(self):
        cli = self.config.get("arduino_cli", "arduino-cli")
        if not shutil.which(cli):
            raise ValueError("Install arduino-cli first and add it to PATH or set arduino_cli in /etc/2pybot-cli.json")
        if self.args.operation == "status":
            for arguments in (["version"], ["core", "list"], ["lib", "list"]):
                subprocess.run([cli, *arguments], check=True)
            return
        lock = read(self.repo / "scripts/firmware/toolchain.json")
        urls = ",".join(lock["board_manager_urls"])
        subprocess.run([cli, "core", "update-index", "--additional-urls", urls], check=True)
        subprocess.run([cli, "core", "install", lock["core"], "--additional-urls", urls], check=True)
        for library in lock["libraries"]:
            subprocess.run([cli, "lib", "install", library], check=True)

    def restart(self):
        job = read(self.firmware_root / "current.json", {})
        if job.get("status") in BUSY:
            raise RuntimeError("An update job is active; service restart was refused")
        try:
            status = self.api("/api/status")
        except RuntimeError:
            if not self.args.offline:
                raise RuntimeError("Console unavailable. For a stopped bot use: bot restart {} --offline".format(self.args.service))
            status = {}
        if status.get("telemetry", {}).get("balancing"):
            raise RuntimeError("Press local SELECT before restarting bot services")
        command = ["systemctl", "restart", SERVICES[self.args.service]]
        if not hasattr(os, "geteuid") or os.geteuid() != 0:
            command.insert(0, "sudo")
        subprocess.run(command, check=True)

    def deploy(self):
        run_dir = Path(self.config.get("deploy_run", "/run/2pybot-deploy"))
        if not run_dir.is_dir():
            raise RuntimeError("Deployment units are not installed")
        import fcntl
        with (run_dir / "operation.lock").open("a+") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError("A pull or deployment is already running") from error
            git = self.config.get("real_git", "/usr/bin/git")
            commit = subprocess.check_output([git, "rev-parse", "HEAD"], cwd=self.repo, text=True).strip()
            atomic(run_dir / "pull.json", {"schema": 1, "phase": "queued", "commit": commit,
                   "updated_at": time.time(), "owner_pid": os.getpid(), "message": "Requested by bot deploy"})
            (run_dir / "request").touch()
        print("Deployment queued. Use: bot deploy-status --watch")

    def run(self):
        name = self.args.command
        if name in {"status", "check"}:
            if name == "check" and self.args.target == "deploy-status":
                show(self.api("/api/deployment/status"))
            else:
                self.status()
        elif name == "firmware-status":
            self.firmware_status()
        elif name == "services":
            self.services()
        elif name == "deploy-status":
            self.deploy_status()
        elif name == "build-firmware":
            self.build()
        elif name == "flash-fw":
            self.flash()
        elif name == "calibration":
            self.calibration()
        elif name in {"diagnostics", "doctor"}:
            self.diagnostics()
        elif name == "backup":
            self.backup()
        elif name == "restore":
            self.restore()
        elif name == "params":
            self.params()
        elif name == "stop":
            self.api("/api/estop", {})
            print(c("STOP sent: motors at zero torque.", "err"))
        elif name == "deploy":
            self.deploy()
        elif name == "toolchain":
            self.toolchain()
        elif name == "restart":
            self.restart()
        elif name == "update":
            subprocess.run(["git", "pull", "--ff-only"], cwd=self.repo, check=True)
        elif name == "logs" and self.args.service == "controller":
            self.controller_log()
        elif name == "logs":
            command = ["journalctl", "--no-pager", "-u", SERVICES[self.args.service], "-n", str(self.args.lines)]
            if self.args.follow:
                command.append("-f")
            subprocess.run(command, check=True)


def parser():
    result = argparse.ArgumentParser(prog="2pybot", description="2PyBot robot commands. Run without a command for the full-screen interface.",
                                     epilog="Works from any directory. Use '2pybot <command> --help' for options.")
    result.add_argument("--config", type=Path, default=Path(os.environ.get("BOT_CLI_CONFIG", "/etc/2pybot-cli.json")))
    result.add_argument("--api", help="Override the local loopback API URL")
    result.add_argument("--json", action="store_true", help="Print raw data instead of tables")
    result.add_argument("--launcher", default="bot", help=argparse.SUPPRESS)
    sub = result.add_subparsers(dest="command", metavar="<command>")
    sub.add_parser("tui", help="Open the full-screen interface")
    sub.add_parser("services", help="2PyBot service states")
    sub.add_parser("help", help="List commands")
    sub.add_parser("status", help="Host readiness, controller telemetry and firmware versions")
    p = sub.add_parser("check", help="Check host/controller or deployment")
    p.add_argument("target", nargs="?", choices=["status", "deploy-status"], default="status")
    sub.add_parser("firmware-status", help="Installed/latest image and current update job")
    p = sub.add_parser("deploy-status", help="Show or watch the queued deployment")
    p.add_argument("--watch", action="store_true")
    p = sub.add_parser("build-firmware", help="Compile this checkout and register its firmware locally")
    p.add_argument("--output", type=Path)
    p.add_argument("--arduino-cli")
    p = sub.add_parser("flash-fw", help="Flash the latest registered build through the existing worker")
    p.add_argument("--build", help="Require this exact latest build ID")
    p.add_argument("--no-wait", action="store_true")
    p = sub.add_parser("calibration", help="Gyro, upright trim, ToF or camera exposure calibration")
    p.add_argument("kind", choices=["imu", "trim", "tof", "camera"])
    p.add_argument("--upright", action="store_true", help="Chassis is physically upright for trim calibration")
    p.add_argument("--reference", type=float, help="ToF target distance in metres")
    p.add_argument("--save", action="store_true", help="Save verified trim to NVS")
    for name in ("diagnostics", "doctor"):
        p = sub.add_parser(name, help="One-shot telemetry, logs, services, USB, Wi-Fi and DNS report")
        p.add_argument("--output", type=Path)
    p = sub.add_parser("backup", help="Save known firmware/prefix and available live settings")
    p.add_argument("--build", help="Use this registered build instead of the recorded installed build")
    p = sub.add_parser("restore", help="List backups, recover an erased ESP32, or restore tuning")
    p.add_argument("kind", nargs="?", choices=["list", "firmware", "tuning"], default="list")
    p.add_argument("--backup", type=Path, help="Folder created by bot backup")
    p.add_argument("--build", help="Registered application for saved-worker-prefix USB recovery")
    p.add_argument("--offline", action="store_true", help="Recover a stopped bot when the console API is unavailable")
    p.add_argument("--save", action="store_true", help="Persist restored tuning after readback")
    p = sub.add_parser("params", help="Inspect, change, save, load or reset controller settings")
    p.add_argument("operation", choices=["list", "get", "set", "save", "load", "defaults", "refresh"])
    p.add_argument("key", nargs="?")
    p.add_argument("value", nargs="?", type=float)
    p.add_argument("--save", action="store_true")
    sub.add_parser("stop", help="Send operator E-STOP; never arms the bot")
    sub.add_parser("update", help="Run the managed fast-forward pull from the configured checkout")
    sub.add_parser("deploy", help="Queue/retry the current commit without fetching")
    p = sub.add_parser("toolchain", help="Inspect/install the locked Arduino core and libraries")
    p.add_argument("operation", choices=["status", "install"])
    p = sub.add_parser("restart", help="Restart a service on a stopped bot")
    p.add_argument("service", choices=sorted(SERVICES))
    p.add_argument("--offline", action="store_true", help="Bot is stopped but the local API is unavailable")
    p = sub.add_parser("logs", help="Controller log (default) or a service's log")
    p.add_argument("service", nargs="?", choices=["controller"] + sorted(SERVICES), default="controller")
    p.add_argument("--follow", "-f", action="store_true")
    p.add_argument("--lines", "-n", type=int, default=60)
    return result


COMMAND_LIST = [
    ("status", "everything at a glance"), ("stop", "stop the motors"), ("logs [-f]", "controller log (logs console / deploy / ...)"),
    ("services", "service states"), ("params list [GROUP|KEY]", "settings"), ("params set KEY VALUE --save", "change a setting"),
    ("firmware-status", "controller firmware"), ("deploy-status [--watch]", "software update progress"),
    ("doctor", "diagnostics summary + report file"), ("calibration imu|trim|tof|camera", "calibrations"),
    ("restart SERVICE", "restart a service"), ("backup / restore", "firmware and settings backups"),
]


def print_help(launcher):
    print(c("2PyBot", "acc") + " robot commands\n")
    print("  " + c(launcher.ljust(42), "info") + "open the full-screen interface (no command)")
    for cmd, text in COMMAND_LIST:
        print("  " + c((launcher + " " + cmd).ljust(42), "info") + text)
    print("\nAdd " + c("--json", "info") + " for raw data. Details: " + launcher + " <command> --help")


def main(argv=None):
    global JSON_MODE
    args = parser().parse_args(argv)
    JSON_MODE = args.json
    if args.command in (None, "tui", "help"):
        if args.command == "tui" or (args.command is None and args.launcher == "2pybot" and sys.stdout.isatty()):
            import bot_tui
            return bot_tui.main(args.api or "http://127.0.0.1:8080")
        print_help(args.launcher)
        return 0
    try:
        Bot(args).run()
        return 0
    except KeyboardInterrupt:
        print("\nStopped watching; accepted worker jobs continue.", file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(c("2pybot: " + str(error), "err"), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
