"""Read-only frame metadata and ToF diagnostics. No guessed sensor extrinsics."""
import json
import time
import zlib
from pathlib import Path

STATUS_NAMES = {0: "not updated", 1: "low signal", 2: "phase issue", 3: "high sigma", 4: "target failed",
                5: "valid", 6: "no wrap", 7: "rate failed", 8: "low signal rate", 9: "valid large pulse",
                10: "no target", 11: "measurement failed", 12: "blurred target", 13: "inconsistent target", 255: "no target"}


def snapshot_frame(path):
    path = Path(path)
    try:
        before = path.stat()
        image = path.read_bytes()
        after = path.stat()
        if before.st_mtime_ns != after.st_mtime_ns or time.time_ns()-after.st_mtime_ns > 3_000_000_000:
            return None, {}
        received = time.monotonic_ns()
        metadata = {"frame_id": f"snapshot-{after.st_mtime_ns}", "snapshot_wall_ns": after.st_mtime_ns,
                    "received_monotonic_ns": received, "capture_monotonic_ns": None,
                    "timestamp_source": "snapshot_publication_only", "checksum_verified": False}
        try:
            sidecar_path = Path(str(path)+".meta.json")
            sidecar = json.loads(sidecar_path.read_text())
            captured = sidecar["capture_monotonic_ns"]
            publication_matches = sidecar_path.stat().st_mtime_ns >= after.st_mtime_ns and abs(sidecar["published_wall_ns"]-after.st_mtime_ns) < 100_000_000
            if publication_matches and sidecar["jpeg_size"] == len(image) and sidecar["jpeg_crc32"] == (zlib.crc32(image)&0xffffffff) and isinstance(captured, int) and 0 < captured <= received+100_000_000:
                metadata.update(sidecar)
                metadata["frame_id"] = f"{sidecar['publisher_pid']}:{sidecar['sequence']}:{captured}"
                metadata["checksum_verified"] = True
        except (OSError, ValueError, KeyError, TypeError):
            pass
        return image, metadata
    except OSError:
        return None, {}


def nearest_depth(history, camera_frame):
    stamp = camera_frame.get("capture_monotonic_ns")
    if stamp is None or not history:
        return {"matched": False, "message": "Acquisition metadata is not available yet"}
    frames = [frame for frame in list(history) if isinstance(frame.get("capture_monotonic_ns"), int)]
    if not frames:
        return {"matched": False, "message": "Depth timestamps are unavailable"}
    frame = min(frames, key=lambda value: abs(value["capture_monotonic_ns"]-stamp))
    delta = (frame["capture_monotonic_ns"]-stamp)/1e6
    if abs(delta) > 200:
        return {"matched": False, "delta_ms": round(delta, 3), "message": "No nearby depth frame"}
    return {"matched": True, "delta_ms": round(delta, 3), "depth": frame,
            "timestamp_source": frame.get("timestamp_source"),
            "uncertainty_ms": frame.get("timestamp_uncertainty_ms"),
            "hardware_synchronized": False}


def depth_diagnostics(info):
    valid = info.get("valid", [False]*64)
    distances = info.get("distance_mm", [0]*64)
    statuses = info.get("status", [])
    sigma = info.get("sigma_mm", [])
    counts = {}
    for status in statuses:
        label = STATUS_NAMES.get(status, f"status {status}")
        counts[label] = counts.get(label, 0)+1
    near = sum(0 < d < 50 and i < len(statuses) and statuses[i] in (5,9) for i,d in enumerate(distances))
    usable = sum(valid) if not info.get("stale", True) else 0
    inconsistent = sum(s in (4,12,13) for s in statuses)
    guidance = []
    if info.get("stale", True):guidance.append("Depth is offline/stale; check the worker and I2C connection")
    if near >= 8:guidance.append("Many valid-status returns are inside 50 mm; check the optical window, obstruction or self-reflection before range calibration")
    if inconsistent >= 8:guidance.append("Many far returns fail consistency checks; compare a large flat target with a clear sensor window")
    if not usable and not guidance:guidance.append("No useful depth: inspect raw target counts, status and sigma on a measured flat target")
    if usable:guidance.append("Usable depth is available; collect known-distance targets for bias and mounting validation")
    return {"valid_zones": usable, "near_field_zones": near, "inconsistent_zones": inconsistent,
            "status_counts": counts, "high_sigma_zones": sum(s > 20 for s in sigma),
            "guidance": guidance, "raw_distance_mm": distances, "valid": valid,
            "range_calibrated": info.get("range_calibrated", False),
            "camera_tof_extrinsics_calibrated": False,
            "note": "Diagnostics do not replace physical target or mounting calibration"}
