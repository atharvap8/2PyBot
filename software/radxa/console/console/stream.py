"""
stream.py — camera -> H.264 -> MediaMTX -> WebRTC (WHEP).

On the Cubie A7A, the native capture process uses the Cedar hardware H.264
encoder, a bounded latest-frame queue, and monotonic capture timestamps.
It publishes RTSP to MediaMTX and saves an original JPEG snapshot each second.
The calibration routine never opens the camera for image capture.

Encoder selection is probed, not assumed: hardware h264_v4l2m2m if the
kernel exposes it, else low-latency libx264 ultrafast. Allwinner/Rockchip mainline
support for the VPU is inconsistent, so the fallback matters.
"""
import asyncio, re, shutil, subprocess, threading, time
from functools import lru_cache
from pathlib import Path
from typing import Dict, List, Optional

from .config import CAMERA_DEV, SNAPSHOT_PATH, STREAM_NAME, WHEP_URL

HARDWARE_STREAMER = Path(__file__).resolve().parent.parent / "native" / "2pybot_hwstream"


def probe_formats(dev: str = CAMERA_DEV) -> List[Dict]:
    """What this camera can actually deliver — never trust the box."""
    try:
        out = subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats-ext"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return []
    modes, fmt = [], None
    for line in out.splitlines():
        m = re.search(r"\]:\s*'(\w+)'", line)
        if m:
            fmt = m.group(1)
        s = re.search(r"Size: Discrete (\d+)x(\d+)", line)
        if s and fmt:
            modes.append({"fmt": fmt, "w": int(s.group(1)), "h": int(s.group(2)), "fps": []})
        f = re.search(r"\(([\d.]+) fps\)", line)
        if f and modes:
            modes[-1]["fps"].append(float(f.group(1)))
    return modes


def best_mode(modes: List[Dict], max_w=1920) -> Optional[Dict]:
    """Prefer MJPG (USB bandwidth) at the highest resolution that still
    offers >=25 fps — raw YUYV is capped by USB and looks like a slideshow
    above 640x480."""
    cands = [m for m in modes if m["fps"] and max(m["fps"]) >= 25 and m["w"] <= max_w]
    if not cands:
        cands = modes
    if not cands:
        return None
    cands.sort(key=lambda m: (m["fmt"] != "MJPG", -(m["w"] * m["h"]), -max(m["fps"])))
    return cands[0]


@lru_cache(maxsize=8)
def pick_encoder(width: int = 1920, height: int = 1080) -> str:
    """A compiled-in encoder is not necessarily backed by a usable device."""
    try:
        enc = subprocess.run(["ffmpeg", "-hide_banner", "-encoders"],
                             capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return "libx264"
    for hw in ("h264_v4l2m2m", "h264_rkmpp", "h264_omx"):
        if hw in enc:
            try:
                probe = subprocess.run(
                    ["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin",
                     "-f", "lavfi", "-i", f"color=size={width}x{height}:rate=30",
                     "-frames:v", "1", "-an", "-c:v", hw, "-pix_fmt", "yuv420p",
                     "-f", "null", "-"],
                    capture_output=True, timeout=5)
                if probe.returncode == 0:
                    return hw
            except (OSError, subprocess.TimeoutExpired):
                continue
    return "libx264"


class Streamer:
    def __init__(self):
        self.proc: Optional[subprocess.Popen] = None
        self.mode: Optional[Dict] = None
        self.encoder = "libx264"
        self.bitrate = "6M"
        self.width = 1920
        self.metrics = {}
        self.started_at = 0.0
        self._lock = threading.RLock()
        self.last_error = ""

    def status(self) -> Dict:
        if self.proc is not None and not self.running and not self.last_error:
            self.last_error = f"{self.encoder} exited with status {self.proc.returncode}; see console service logs"
        return {"running": self.running, "mode": self.mode, "encoder": self.encoder,
                "bitrate": self.bitrate, "error": self.last_error,
                "whep": WHEP_URL, "hardware": self.encoder == "cedar_h264",
                "preset": "ultrafast" if self.encoder == "libx264" else "hardware",
                "metrics": self.metrics,
                "uptime_seconds": round(time.monotonic() - self.started_at, 1) if self.running else 0}

    def _read_progress(self, proc):
        """Drain encoder progress continuously and expose measured frame counts."""
        metrics = {}
        for raw in proc.stdout:
            key, _, value = raw.decode(errors="replace").strip().partition("=")
            if key in ("frame", "fps", "dup_frames", "drop_frames", "speed", "out_time",
                       "capture_to_publish_ms", "hardware_encode_ms", "timestamp_fallbacks"):
                metrics[key] = value
            if key == "progress" and self.proc is proc:
                self.metrics = dict(metrics)
        proc.stdout.close()

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def build_cmd(self, mode: Dict) -> List[str]:
        if self.encoder == "cedar_h264":
            match = re.fullmatch(r"(\d+(?:\.\d+)?)([kKmM]?)", self.bitrate)
            if not match:
                raise ValueError("bitrate must be a number, optionally followed by K or M")
            multiplier = {"": 1, "k": 1000, "m": 1000000}[match[2].lower()]
            bitrate = int(float(match[1]) * multiplier)
            fps = min(30, int(max(mode["fps"]) if mode["fps"] else 30))
            return [str(HARDWARE_STREAMER), CAMERA_DEV, str(mode["w"]), str(mode["h"]),
                    str(fps), str(bitrate), f"rtsp://127.0.0.1:8554/{STREAM_NAME}", SNAPSHOT_PATH]
        # V4L2 FOURCC names differ from FFmpeg's codec/pixel-format names.
        input_format = {"MJPG": "mjpeg", "YUYV": "yuyv422", "UYVY": "uyvy422"}.get(
            mode["fmt"], mode["fmt"].lower())
        inp = ["-f", "v4l2", "-input_format", input_format,
               "-video_size", f'{mode["w"]}x{mode["h"]}',
               "-framerate", str(int(max(mode["fps"]) if mode["fps"] else 30)),
               "-i", CAMERA_DEV]
        if self.encoder == "libx264":
            venc = ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
                    "-profile:v", "baseline", "-pix_fmt", "yuv420p"]
        else:
            venc = ["-c:v", self.encoder, "-pix_fmt", "yuv420p"]
        return ["ffmpeg", "-hide_banner", "-loglevel", "warning", "-nostdin",
                "-nostats", "-progress", "pipe:1", "-vsync", "0",
                "-fflags", "nobuffer", "-flags", "low_delay",
                "-probesize", "32", "-analyzeduration", "0",
                *inp,
                # output 1: low-latency h264 to MediaMTX
                *venc, "-b:v", self.bitrate, "-maxrate", self.bitrate,
                "-bufsize", "600k", "-g", "15", "-bf", "0", "-an",
                "-flush_packets", "1", "-muxdelay", "0",
                "-f", "rtsp", "-rtsp_transport", "tcp",
                f"rtsp://127.0.0.1:8554/{STREAM_NAME}",
                # Select real frames instead of synthesizing duplicates across clock jumps.
                "-vf", "select=isnan(prev_selected_t)+gte(t-prev_selected_t\\,1)",
                "-update", "1", "-q:v", "6",
                "-atomic_writing", "1", "-y", SNAPSHOT_PATH]

    def start(self, width: Optional[int] = None, bitrate: Optional[str] = None):
        with self._lock:
            return self._start(width, bitrate)

    def _start(self, width: Optional[int] = None, bitrate: Optional[str] = None):
        self.stop()
        if width is not None:
            self.width = width
        if not HARDWARE_STREAMER.is_file() and not shutil.which("ffmpeg"):
            self.last_error = "ffmpeg not installed"
            return False, self.last_error
        modes = probe_formats()
        mode = best_mode(modes, max_w=self.width)
        if not mode:
            self.last_error = f"no usable mode on {CAMERA_DEV}"
            return False, self.last_error
        self.mode = mode
        self.encoder = ("cedar_h264" if mode["fmt"] == "MJPG" and HARDWARE_STREAMER.is_file()
                        else pick_encoder(mode["w"], mode["h"]))
        if bitrate:
            self.bitrate = bitrate
        try:
            self.metrics = {}
            self.proc = subprocess.Popen(self.build_cmd(mode),
                                         stdout=subprocess.PIPE,
                                         # Inherit journald; an unread PIPE eventually blocks ffmpeg.
                                         stderr=None)
            self.started_at = time.monotonic()
            threading.Thread(target=self._read_progress, args=(self.proc,), daemon=True).start()
            try:
                self.proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
            else:
                self.last_error = f"{self.encoder} exited with status {self.proc.returncode}; see console service logs"
                return False, self.last_error
            self.last_error = ""
            return True, f'{mode["fmt"]} {mode["w"]}x{mode["h"]} via {self.encoder}'
        except Exception as e:
            self.last_error = str(e)
            return False, self.last_error

    def stop(self):
        with self._lock:
            self._stop()

    def _stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.proc = None

    async def supervise(self):
        """Restart the pipeline if its encoder dies (camera unplugged, USB reset)."""
        backoff = 2
        while True:
            if not self.running:
                ok, msg = self.start()
                backoff = 2 if ok else min(30, backoff * 2)
                if not ok:
                    await asyncio.sleep(backoff)
                    continue
            await asyncio.sleep(3)


streamer = Streamer()
