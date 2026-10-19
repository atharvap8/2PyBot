"""Progressive camera-calibration sessions using the existing publisher snapshots."""
import asyncio
import copy
import hashlib
import importlib.util
import json
import os
import re
import threading
import time
import uuid
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


def atomic_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    os.replace(temporary, path)


class Calibration:
    def __init__(self, frame_source, metadata_source, directory=None):
        self.frame_source = frame_source
        self.metadata_source = metadata_source
        self.directory = Path(directory or os.environ.get("BOT_CALIBRATION_DIR", "/var/lib/2pybot/vision-calibration"))
        self.lock = threading.RLock()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="camera-calibration")
        self.session = None
        self.preview = None
        self.corrected = None
        self.last_frame = None
        self.previous_metrics = None
        self.stable = 0
        self.last_accept = 0.0
        self.live = {"message": "Print a target, then start the guided capture"}
        self.restore()

    @staticmethod
    def maths():
        try:
            from . import calibration_math
            return calibration_math
        except ImportError as exc:
            raise ValueError("Install the optional vision dependencies (OpenCV, numpy)") from exc

    def restore(self):
        try:
            pointer = json.loads((self.directory / "current.json").read_text())
            identifier = pointer["id"]
            if not re.fullmatch(r"[0-9a-f]{32}", identifier):
                return
            self.session = json.loads((self.directory / identifier / "session.json").read_text())
            if self.session["phase"] in ("capturing", "analyzing"):
                self.session["phase"] = "paused"
                self.session["message"] = "Session restored; resume when the target is ready"
        except (OSError, ValueError, KeyError, TypeError):
            self.session = None

    def folder(self, identifier=None):
        identifier = identifier or self.session["id"]
        if not re.fullmatch(r"[0-9a-f]{32}", identifier):
            raise ValueError("Invalid calibration session")
        return self.directory / identifier

    def persist(self):
        atomic_json(self.folder() / "session.json", self.session)
        atomic_json(self.directory / "current.json", {"id": self.session["id"]})

    def start(self, settings, goal=32):
        math = self.maths()
        settings = math.board_settings(settings)
        if not 24 <= goal <= 60:
            raise ValueError("Choose 24–60 target views")
        metadata = self.metadata_source()
        if metadata.get("telemetry", {}).get("balancing", False):
            raise ValueError("Set the robot down idle before starting camera calibration")
        with self.lock:
            if self.session and self.session["phase"] in ("capturing", "analyzing"):
                raise ValueError("A calibration is already running")
            self.directory.mkdir(parents=True, exist_ok=True)
            identifier = uuid.uuid4().hex
            (self.directory / identifier / "captures").mkdir(parents=True)
            self.session = {"schema": "2pybot-calibration-session/1", "id": identifier,
                            "started_wall_s": time.time(), "phase": "capturing", "board": settings,
                            "goal": goal, "image_size": None, "samples": [], "result": None,
                            "metadata": metadata, "message": "Show the printed target in the centre"}
            self.last_frame = None; self.previous_metrics = None; self.stable = 0; self.last_accept = 0
            self.preview = None; self.corrected = None
            self.persist()
        return self.status()

    def coverage(self, samples=None):
        samples = samples if samples is not None else (self.session["samples"] if self.session else [])
        counts = [0] * 9
        for sample in samples:
            counts[sample["metrics"]["cell"]] += 1
        return {"cells": counts, "covered_cells": sum(c > 0 for c in counts),
                "scales": sorted(set(s["metrics"]["scale_bin"] for s in samples)),
                "tilted_views": sum(s["metrics"]["tilted"] for s in samples)}

    def guide(self):
        coverage = self.coverage()
        order = [4, 0, 2, 6, 8, 1, 3, 5, 7]
        cell = next((i for i in order if coverage["cells"][i] == 0), min(order, key=lambda i: coverage["cells"][i]))
        names = ["top left", "top centre", "top right", "middle left", "centre", "middle right", "bottom left", "bottom centre", "bottom right"]
        message = f"Move the target to the {names[cell]}, then hold it still"
        if coverage["covered_cells"] >= 6 and len(coverage["scales"]) < 2:
            message = "Change distance: bring the board closer or move it farther away"
        elif coverage["covered_cells"] >= 6 and coverage["tilted_views"] < 3:
            message = "Tilt an edge of the board towards the camera; keep its corners visible"
        return {"cell": cell, "message": message}

    def status(self):
        with self.lock:
            if not self.session:
                return {"available": importlib.util.find_spec("cv2") is not None, "phase": "idle",
                        "accepted": 0, "goal": 32, "board": {"kind": "charuco", "columns": 8, "rows": 6, "square_mm": 20},
                        "live": copy.deepcopy(self.live), "sessions": self.list_sessions()}
            result = {key: copy.deepcopy(value) for key, value in self.session.items() if key not in ("samples", "metadata")}
            result.update(available=True, accepted=len(self.session["samples"]), coverage=self.coverage(),
                          guide=self.guide(), live=copy.deepcopy(self.live), sessions=self.list_sessions())
            return result

    def list_sessions(self):
        if not self.directory.is_dir():
            return []
        return sorted([p.name for p in self.directory.iterdir() if p.is_dir() and re.fullmatch(r"[0-9a-f]{32}", p.name)], reverse=True)

    def action(self, action):
        with self.lock:
            if not self.session:
                raise ValueError("Start a session first")
            if action == "pause" and self.session["phase"] == "capturing":
                self.session["phase"] = "paused"
            elif action == "resume" and self.session["phase"] in ("paused", "failed"):
                self.session["phase"] = "capturing"; self.last_frame = None
            elif action == "analyze" and self.session["phase"] in ("capturing", "paused", "failed", "ready"):
                if len(self.session["samples"]) < 20:
                    raise ValueError("Collect at least 20 accepted views before analysis")
                self.session["phase"] = "analyzing"
            elif action == "cancel" and self.session["phase"] != "analyzing":
                self.session["phase"] = "cancelled"
            else:
                raise ValueError("Action is not available in the current phase")
            self.persist()
        return self.status()

    def tick(self):
        with self.lock:
            if not self.session or self.session["phase"] not in ("capturing", "analyzing", "ready"):
                return
            identifier = self.session["id"]
            phase = self.session["phase"]
        if phase == "analyzing":
            self.analyze(identifier)
            return
        jpeg, timing = self.frame_source()
        if not jpeg:
            self.live = {"message": "Waiting for a fresh camera snapshot"}
            return
        fingerprint = timing.get("frame_id") or hashlib.sha256(jpeg).hexdigest()
        if fingerprint == self.last_frame:
            return
        self.last_frame = fingerprint
        math = self.maths()
        cv = math.cv2
        image = cv.imdecode(math.np.frombuffer(jpeg, math.np.uint8), cv.IMREAD_COLOR)
        if image is None:
            self.live = {"message": "Snapshot could not be decoded"}; return
        size = [image.shape[1], image.shape[0]]
        if phase == "ready":
            try:
                corrected = math.rectify(image, self.session["result"])
                self.corrected = cv.imencode(".jpg", corrected, [cv.IMWRITE_JPEG_QUALITY, 88])[1].tobytes()
                self.preview = cv.imencode(".jpg", cv.resize(image, (corrected.shape[1], corrected.shape[0])), [cv.IMWRITE_JPEG_QUALITY, 88])[1].tobytes()
            except ValueError as exc:
                self.live = {"message": str(exc)}
            return
        observation, metrics = math.detect(image, self.session["board"])
        guide = self.guide()
        shown = image.copy()
        if observation:
            for x, y in observation["image_points"]:
                cv.circle(shown, (round(x), round(y)), 5, (40, 220, 90), 2)
        row, column = divmod(guide["cell"], 3)
        cv.rectangle(shown, (round(column*size[0]/3), round(row*size[1]/3)),
                     (round((column+1)*size[0]/3), round((row+1)*size[1]/3)), (220, 180, 40), 4)
        preview = cv.resize(shown, (960, round(960*size[1]/size[0])))
        self.preview = cv.imencode(".jpg", preview, [cv.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
        message = metrics.get("message", guide["message"])
        accepted = False
        if observation:
            previous = self.previous_metrics
            settled = previous is not None and sum(abs(a-b) for a, b in zip(metrics["centre"], previous["centre"])) < .015 and abs(metrics["area"]-previous["area"]) < .02
            self.stable = self.stable + 1 if settled else 0
            if metrics["area"] < .035:
                message = "Bring the target closer; it is too small"
            elif metrics["border_margin"] < .008:
                message = "Move the target slightly inward from the image edge"
            elif metrics["sharpness"] < 45:
                message = "Hold still and improve lighting; the target is blurry"
            elif metrics["brightness"] < 28:
                message = "Add light or use the torch; the target is too dark"
            elif self.stable < 1:
                message = "Hold the target still — capture will happen automatically"
            elif self.duplicate(metrics):
                message = guide["message"]
            elif time.monotonic()-self.last_accept < 1.3:
                message = "Captured. Move to a new position or angle"
            else:
                metadata = self.metadata_source()
                if metadata.get("telemetry", {}).get("balancing"):
                    message = "Robot is balancing; set it down idle to continue capture"
                else:
                    with self.lock:
                        if self.session["id"] != identifier or self.session["phase"] != "capturing":return
                        if self.session["image_size"] and self.session["image_size"] != size:
                            self.session["phase"] = "paused"; message = "Camera resolution changed; start a new session"
                        elif len(self.session["samples"]) >= 60:
                            self.session["phase"] = "paused"; message = "60-view limit reached; analyze this set before starting a new session"
                        else:
                            self.session["image_size"] = size
                            index = len(self.session["samples"])
                            filename = f"captures/{index:03d}.jpg"
                            (self.folder() / filename).write_bytes(jpeg)
                            sample = dict(observation, metrics=metrics, image_file=filename,
                                          timing=timing, metadata=metadata, sha256=hashlib.sha256(jpeg).hexdigest())
                            self.session["samples"].append(sample)
                            self.last_accept = time.monotonic(); accepted = True
                            coverage = self.coverage()
                            if len(self.session["samples"]) >= self.session["goal"] and coverage["covered_cells"] >= 6 and len(coverage["scales"]) >= 2 and coverage["tilted_views"] >= 3:
                                self.session["phase"] = "analyzing"; message = "Capture complete — comparing lens models automatically"
                            else:message = f"Captured view {index+1}. {self.guide()['message']}"
                        self.session["message"] = message; self.persist()
            self.previous_metrics = metrics
        else:
            self.stable = 0; self.previous_metrics = None
        self.live = dict(metrics, message=message, accepted_now=accepted, timing=timing)

    def duplicate(self, metrics):
        for sample in self.session["samples"]:
            old = sample["metrics"]
            angle_difference = abs((metrics["rotation_deg"]-old["rotation_deg"]+180)%360-180)
            if sum(abs(a-b) for a, b in zip(metrics["centre"], old["centre"])) < .075 and abs(metrics["area"]-old["area"]) < .035 and angle_difference < 10 and metrics["tilted"] == old["tilted"]:
                return True
        return False

    def analyze(self, identifier):
        with self.lock:
            samples = copy.deepcopy(self.session["samples"])
            size = self.session["image_size"]
        try:
            result = self.maths().solve(samples, size)
            result.update(calibration_id=identifier, board=self.session["board"], created_wall_s=time.time(),
                          camera_metadata=self.session["metadata"], source_hashes=[s["sha256"] for s in samples])
            with self.lock:
                if self.session["id"] != identifier:return
                self.session["result"] = result
                self.session["phase"] = "ready" if result["quality"]["passed"] else "failed"
                self.session["message"] = "Analysis passed — compare raw and corrected previews" if result["quality"]["passed"] else "Validation needs better views; resume and add sharp tilted captures"
                atomic_json(self.folder() / "intrinsics.json", result)
                if result["quality"]["passed"]:
                    atomic_json(self.directory / "active.json", {"id": identifier})
                self.last_frame = None; self.persist()
        except Exception as exc:
            with self.lock:
                if self.session["id"] == identifier:
                    self.session["phase"] = "failed"; self.session["message"] = str(exc)[-600:]; self.persist()

    async def run(self):
        try:
            while True:
                try:
                    await asyncio.get_running_loop().run_in_executor(self.executor, self.tick)
                except Exception as exc:
                    self.live = {"message": f"Calibration worker: {str(exc)[-300:]}"}
                await asyncio.sleep(.65)
        finally:
            self.executor.shutdown(wait=False)

    def archive(self, identifier):
        folder = self.folder(identifier)
        if not (folder / "session.json").is_file():raise ValueError("Session not found")
        target = folder / "session-export.zip"
        with zipfile.ZipFile(target, "w", zipfile.ZIP_DEFLATED) as output:
            for path in folder.rglob("*"):
                if path.is_file() and path.suffix in (".json", ".jpg", ".pdf"):
                    output.write(path, str(path.relative_to(folder)))
        return target

    def report(self, identifier):
        from reportlab.pdfgen import canvas
        from reportlab.lib.pagesizes import A4
        folder = self.folder(identifier)
        session = json.loads((folder / "session.json").read_text())
        path = folder / "report.pdf"
        report = canvas.Canvas(str(path), pagesize=A4)
        y = 795
        def line(text, size=11):
            nonlocal y
            if y < 50:report.showPage();y=795
            report.setFont("Helvetica", size);report.drawString(36, y, text[:105]);y-=size+9
        line("2PyBot camera calibration report", 18)
        line("Session " + identifier)
        line("Phase: " + session["phase"])
        line("Accepted views: " + str(len(session["samples"])))
        line("Image size: " + str(session["image_size"]))
        line("Board: " + str(session["board"]))
        result = session.get("result")
        if result:
            line("Lens model: " + result["model"])
            quality = result["quality"]
            line(f"Training/refit RMS: {quality['rms_px']:.3f} px")
            line(f"Held-out mean / p95: {quality['heldout_mean_px']:.3f} / {quality['heldout_p95_px']:.3f} px")
            line("Quality gate: " + ("PASS" if quality["passed"] else "MORE VIEWS NEEDED"))
            for row in result["camera_matrix"]:line("K: " + str([round(v, 4) for v in row]))
            line("Distortion: " + str([round(v, 7) for v in result["distortion"]]))
            for i, error in enumerate(quality["per_view_errors_px"]):line(f"View {i+1}: {error:.3f} px", 9)
        line("No camera/ToF extrinsics or physical straight-line validation is claimed.", 9)
        report.save()
        return path
