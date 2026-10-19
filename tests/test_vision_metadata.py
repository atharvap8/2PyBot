"""Frame/sidecar association and honest sparse-depth diagnostics."""
import importlib.util
import json
import time
import zlib
from pathlib import Path

source=Path(__file__).resolve().parents[1]/"software/radxa/console/console/vision.py"
spec=importlib.util.spec_from_file_location("vision_metadata",source)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_sidecar_is_bound_to_the_actual_snapshot_bytes(tmp_path):
    image=tmp_path/"snapshot.jpg";image.write_bytes(b"JPEG-frame-one")
    stamp=time.monotonic_ns()
    sidecar={"jpeg_size":len(image.read_bytes()),"jpeg_crc32":zlib.crc32(image.read_bytes()),
             "publisher_pid":42,"sequence":7,"capture_monotonic_ns":stamp,"timestamp_source":"v4l2_monotonic",
             "published_wall_ns":time.time_ns()}
    metadata=Path(str(image)+".meta.json");metadata.write_text(json.dumps(sidecar))
    _,result=module.snapshot_frame(image)
    assert result["checksum_verified"] and result["capture_monotonic_ns"]==stamp
    image.write_bytes(b"JPEG-frame-two")
    _,result=module.snapshot_frame(image)
    assert not result["checksum_verified"] and result["capture_monotonic_ns"] is None


def test_pairing_uses_acquisition_time_and_reports_uncertainty():
    frames=[{"capture_monotonic_ns":100_000_000,"sequence":1},
            {"capture_monotonic_ns":195_000_000,"sequence":2,"timestamp_source":"host_read_midpoint","timestamp_uncertainty_ms":50}]
    result=module.nearest_depth(frames,{"capture_monotonic_ns":200_000_000})
    assert result["matched"] and result["depth"]["sequence"]==2
    assert result["delta_ms"]==-5 and not result["hardware_synchronized"]
    assert not module.nearest_depth(frames,{"capture_monotonic_ns":999_000_000})["matched"]


def test_near_field_self_returns_are_not_relabelled_as_useful_depth():
    result=module.depth_diagnostics({"stale":False,"valid":[False]*64,"distance_mm":[25]*64,
        "status":[5]*64,"sigma_mm":[1]*64,"range_calibrated":False})
    assert result["near_field_zones"]==64 and result["valid_zones"]==0
    assert not result["camera_tof_extrinsics_calibrated"]
    assert any("window" in message for message in result["guidance"])
