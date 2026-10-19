"""Measured geometry, target compatibility and resumable automated sessions."""
import importlib.util
import json
from pathlib import Path

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")
SOURCE = Path(__file__).resolve().parents[1] / "software/radxa/console/console"


def load(name):
    spec = importlib.util.spec_from_file_location(name, SOURCE / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


maths = load("calibration_math")
manager_module = load("calibration")


def synthetic_samples():
    settings = maths.board_settings({"kind": "checkerboard", "columns": 10, "rows": 7})
    objects = maths.board_points(settings)
    matrix = np.array([[870., 0, 640], [0, 865., 360], [0, 0, 1]])
    distortion = np.array([-.22, .08, .001, -.0008, .01])
    samples = []
    for i in range(30):
        rotation = np.array([-.30+(i%5)*.13, -.34+(i%6)*.12, -.15+(i%4)*.10])
        translation = np.array([-.20+(i%5)*.07, -.14+(i%3)*.06, .55+(i%4)*.13])
        pixels, _ = cv2.projectPoints(objects, rotation, translation, matrix, distortion)
        samples.append({"object_points": objects.tolist(), "image_points": pixels.reshape(-1, 2).tolist()})
    return samples, matrix


def test_both_target_types_are_printable_and_detectable():
    for kind in ("charuco", "checkerboard"):
        settings = maths.board_settings({"kind": kind})
        board = cv2.imdecode(np.frombuffer(maths.board_png(settings, 1000), np.uint8), cv2.IMREAD_GRAYSCALE)
        image = cv2.copyMakeBorder(board, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        observation, quality = maths.detect(image, settings)
        assert observation and quality["detected"]
        assert len(observation["image_points"]) >= 12
        assert b"50 mm" in maths.board_svg(settings)


def test_intrinsic_solver_uses_heldout_views_and_recovers_known_geometry():
    samples, expected = synthetic_samples()
    result = maths.solve(samples, [1280, 720])
    assert result["quality"]["passed"]
    assert result["quality"]["heldout_views"] == 6
    assert result["quality"]["training_views"] == 24
    assert result["quality"]["heldout_mean_px"] < .1
    assert np.asarray(result["camera_matrix"])[0, 0] == pytest.approx(expected[0, 0], rel=.01)
    raw = np.full((720, 1280, 3), 180, np.uint8)
    assert maths.rectify(raw, result).shape[1] == 1100
    with pytest.raises(ValueError, match="resolution"):
        maths.rectify(np.zeros((100, 100, 3), np.uint8), result)


def test_solver_rejects_insufficient_views_and_invalid_board():
    with pytest.raises(ValueError, match="20"):
        maths.solve([], [1280, 720])
    with pytest.raises(ValueError):
        maths.board_settings({"square_mm": float("nan")})


def test_session_restart_retains_data_and_requires_resume(tmp_path):
    source = lambda: (None, {})
    metadata = lambda: {"telemetry": {"balancing": False}}
    manager = manager_module.Calibration(source, metadata, tmp_path)
    manager.maths = lambda: maths
    status = manager.start({"kind": "checkerboard"})
    restored = manager_module.Calibration(source, metadata, tmp_path)
    assert restored.status()["phase"] == "paused"
    assert restored.status()["id"] == status["id"]
    assert restored.action("resume")["phase"] == "capturing"
    with pytest.raises(ValueError):
        restored.action("analyze")
    with pytest.raises(ValueError):
        restored.folder("../camera_tuning.json")
    manager.executor.shutdown(); restored.executor.shutdown()


def test_auto_capture_filters_duplicate_views_and_tracks_coverage(tmp_path):
    settings = maths.board_settings({"kind": "checkerboard"})
    board = cv2.imdecode(np.frombuffer(maths.board_png(settings, 900), np.uint8), cv2.IMREAD_COLOR)
    image = cv2.copyMakeBorder(board, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=(255,255,255))
    jpeg = cv2.imencode(".jpg", image)[1].tobytes()
    frames = iter(range(10))
    manager = manager_module.Calibration(lambda: (jpeg, {"frame_id": str(next(frames))}),
        lambda: {"telemetry": {"balancing": False}}, tmp_path)
    manager.maths = lambda: maths
    manager.start(settings)
    manager.tick(); manager.tick(); manager.tick()
    assert manager.status()["accepted"] == 1
    assert manager.status()["coverage"]["covered_cells"] == 1
    assert len(list(manager.folder().glob("captures/*.jpg"))) == 1
    manager.executor.shutdown()


def test_analysis_persists_validated_result_and_exports_report(tmp_path):
    manager = manager_module.Calibration(lambda: (None, {}), lambda: {"telemetry": {"balancing": False}}, tmp_path)
    manager.maths = lambda: maths
    manager.start({"kind": "checkerboard", "columns": 10, "rows": 7})
    samples, _ = synthetic_samples()
    for i, sample in enumerate(samples):
        sample["sha256"] = "synthetic-" + str(i)
        sample["metrics"] = {"cell": i%9, "scale_bin": i%3, "tilted": True, "centre": [.5,.5], "area": .1, "rotation_deg": i*12}
    manager.session["samples"] = samples
    manager.session["image_size"] = [1280,720]
    manager.session["phase"] = "analyzing"
    identifier = manager.session["id"]
    manager.analyze(identifier)
    assert manager.status()["phase"] == "ready"
    assert json.loads((tmp_path/"active.json").read_text())["id"] == identifier
    assert manager.report(identifier).read_bytes().startswith(b"%PDF")
    assert manager.archive(identifier).is_file()
    manager.executor.shutdown()
