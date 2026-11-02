"""System map HTTP/WS surface with fake console objects (skipped without FastAPI)."""
import collections
import sys
import time
from pathlib import Path

import pytest

pytest.importorskip("fastapi")
pytest.importorskip("httpx")
from fastapi.testclient import TestClient  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software/radxa/console"))
from console.sysmap import create_app  # noqa: E402


class Link:
    def __init__(self):
        self.connected, self.maintenance, self.dump_complete, self.ser = True, False, True, None
        self.tel = {"pitch": float("nan"), "yaw": 10.0, "stale": False, "rx_hz": 50.0,
                    "last_rx": time.time(), "balancing": False, "startup_state": "READY"}
        self.debug, self.hub, self.nav, self.derived = {}, {}, {}, {}
        self.log = collections.deque([{"t": time.time(), "text": "[PAD] Connected: pad"}], maxlen=400)
        self._subs = set()


class ToF:
    def status(self, link=None):
        return {"state": "ranging", "stale": False, "age_ms": 50, "valid_count": 1,
                "distance_mm": [900] * 64, "valid": [True] + [False] * 63}


class Streamer:
    def status(self):
        return {"running": True, "metrics": {"fps": "30", "frame": "5"}}


@pytest.fixture
def client():
    stops = []
    app = create_app(Link(), ToF(), Streamer(), lambda: {"camera": True, "hotspot": True},
                     lambda: stops.append(1))
    with TestClient(app) as c:
        c.stops = stops
        yield c


def test_snapshot_is_valid_json_even_with_nan(client):
    r = client.get("/api/snapshot")
    assert r.status_code == 200
    body = r.json()  # would raise on NaN
    assert body["live"]["pitch"] is None
    assert body["nodes"]["gamepad"]["status"] == "ok"
    assert body["nodes"]["tof"]["metrics"][0] == ["Nearest", "0.90 m"]
    assert len(body["tof_grid"]["distance_mm"]) == 64


def test_websocket_streams_snapshots(client):
    with client.websocket_connect("/ws") as ws:
        first, second = ws.receive_json(), ws.receive_json()
    assert "nodes" in first and second["t"] >= first["t"]


def test_estop_uses_the_console_stop(client):
    assert client.post("/api/estop").json() == {"ok": True}
    assert client.stops == [1]


def test_page_and_assets_are_served(client):
    assert client.get("/").status_code == 200
    for asset in ("sysmap.js", "model.js", "sysmap.css", "bot.jpg"):
        assert client.get(f"/static/{asset}").status_code == 200, asset
