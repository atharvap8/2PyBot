import asyncio
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

SOURCE = Path(__file__).resolve().parents[1] / "software/radxa/console/console/readiness.py"
spec = importlib.util.spec_from_file_location("host_readiness", SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_waits_for_fresh_camera_not_just_process_start(tmp_path):
    snapshot = tmp_path / "frame.jpg"
    snapshot.write_bytes(b"fake snapshot")
    assert not module.readiness({"running": True, "metrics": {}}, snapshot, "")["ready"]
    assert module.readiness({"running": True, "metrics": {"frame": "10"}}, snapshot, "")["ready"]
    os.utime(snapshot, (1, 1))
    assert not module.readiness({"running": True, "metrics": {"frame": "10"}}, snapshot, "")["ready"]


def test_readiness_does_not_depend_on_esp32_serial(tmp_path):
    snapshot = tmp_path / "frame.jpg"
    snapshot.write_bytes(b"fake snapshot")
    assert module.readiness({"running": True, "metrics": {"frame": "10"}}, snapshot, "127.0.0.1")["ready"]
    assert not module.readiness({"running": True, "metrics": {"frame": "10"}}, snapshot, "192.0.2.77")["ready"]


def test_ready_flag_releases_interlock_without_arming():
    async def scenario():
        sent = []
        state = {"ready": False}
        link = SimpleNamespace(connected=True, derived={"BOOTGATE": 1}, tel={"host_ready": False}, send=sent.append)
        task = asyncio.create_task(module.publish_ready(link, lambda: state))
        await asyncio.sleep(0.01)
        assert not sent
        state["ready"] = True
        await asyncio.sleep(0.51)
        assert sent == ["BR,1"]
        link.tel["host_ready"] = True
        await asyncio.sleep(0.51)
        assert sent == ["BR,1"]
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    asyncio.run(scenario())


def test_ack_is_sent_while_firmware_has_no_sensor_telemetry_yet():
    async def scenario():
        sent = []
        link = SimpleNamespace(connected=True, derived={"BOOTGATE": 1, "STARTUPACK": 1},
                               tel={"host_ready": False, "stale": True, "startup_state": "WAIT_HOST"}, send=sent.append)
        task = asyncio.create_task(module.publish_ready(link, lambda: {"ready": True}))
        await asyncio.sleep(.01)
        assert sent == ["BR,1"]
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    asyncio.run(scenario())
