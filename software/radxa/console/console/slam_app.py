"""slam_app.py — "SLAM for beginners" page + API, mounted at /slam/ and on its
own port (BOT_SLAM_PORT, default 6970).

Listen-only: the engine reads raw O/D telemetry through the serial link's line
listener and scans from the lidar driver. The only command it can send to the
robot is the operator stop, through the console's normal E-STOP path.
"""
from __future__ import annotations

import asyncio
import base64
import math
import os

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from . import map_export

UI_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ui", "slam")


class NameBody(BaseModel):
    name: str


class LabelBody(BaseModel):
    name: str
    x: float
    y: float


class EraseBody(BaseModel):
    x: float
    y: float
    r: float = 0.25


def _safe(v):
    if isinstance(v, float) and not math.isfinite(v):
        return None
    if isinstance(v, dict):
        return {k: _safe(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_safe(x) for x in v]
    return v


def create_app(engine, lidar, estop):
    app = FastAPI(title="2PyBot SLAM")

    def state():
        return _safe(engine.state(dict(lidar.status)))

    @app.get("/")
    def index():
        return FileResponse(os.path.join(UI_DIR, "index.html"))

    if os.path.isdir(UI_DIR):
        app.mount("/static", StaticFiles(directory=UI_DIR), name="slam-static")

    @app.get("/api/state")
    def api_state():
        return JSONResponse(state())

    @app.get("/api/map.png")
    def api_map_png():
        png, _ = engine.map_png()
        return Response(png or b"", media_type="image/png")

    @app.get("/api/scene3d")
    def api_scene3d():
        return JSONResponse(_safe(engine.scene3d()))

    @app.get("/api/scene_vector")
    def api_scene_vector():
        """The map as clean outlines for the 3D console view; 501 when the robot
        has no OpenCV, in which case the viewer uses /api/scene3d instead."""
        scene = engine.scene_vector()
        if scene is None:
            return JSONResponse({"error": "outlines need OpenCV"}, status_code=501)
        return JSONResponse(_safe(scene))

    @app.post("/api/mapping/start")
    def mapping_start(fresh: bool = True):
        engine.start_mapping(fresh)
        return {"ok": True}

    @app.post("/api/mapping/stop")
    def mapping_stop():
        engine.stop_mapping()
        return {"ok": True}

    @app.post("/api/test/{name}")
    def test_start(name: str):
        if name not in ("push", "turn"):
            return JSONResponse({"ok": False, "error": "unknown test"}, status_code=404)
        engine.test_start(name)
        return {"ok": True}

    @app.get("/api/maps")
    def maps():
        return engine.list_maps()

    @app.post("/api/maps/save")
    def save(body: NameBody):
        try:
            return {"ok": True, "map": engine.save(body.name.strip())}
        except (ValueError, OSError) as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)

    @app.post("/api/maps/load")
    def load(body: NameBody):
        try:
            engine.load(body.name.strip())
            return {"ok": True}
        except (ValueError, OSError) as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)

    @app.post("/api/maps/extend")
    def extend(body: NameBody):
        """Map correction: load a saved map and keep mapping on top of it."""
        try:
            engine.extend(body.name.strip())
            return {"ok": True}
        except (ValueError, OSError) as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)

    @app.post("/api/maps/delete")
    def delete(body: NameBody):
        try:
            return {"ok": engine.delete_map(body.name.strip()) > 0}
        except (ValueError, OSError) as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)

    @app.get("/api/maps/{name}/stl")
    def export_stl(name: str):
        if not map_export.SAFE.match(name or ""):
            return JSONResponse({"error": "bad map name"}, status_code=400)
        try:
            data = map_export.stl_bytes(engine.maps_dir, name)
        except (OSError, ValueError, KeyError) as e:
            return JSONResponse({"error": str(e)}, status_code=404)
        return Response(data, media_type="model/stl",
                        headers={"Content-Disposition": f'attachment; filename="{name}.stl"'})

    @app.get("/api/maps/{name}/zip")
    def export_zip(name: str):
        if not map_export.SAFE.match(name or ""):
            return JSONResponse({"error": "bad map name"}, status_code=400)
        data = map_export.zip_bytes(engine.maps_dir, name)
        if not data:
            return JSONResponse({"error": "map not found"}, status_code=404)
        return Response(data, media_type="application/zip",
                        headers={"Content-Disposition": f'attachment; filename="{name}.zip"'})

    @app.post("/api/maps/import")
    async def import_map(name: str, request: Request):
        try:
            written = map_export.import_zip(engine.maps_dir, name.strip(), await request.body())
            return {"ok": True, "written": written}
        except (ValueError, OSError) as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)

    @app.post("/api/map/erase")
    def erase(body: EraseBody):
        try:
            return {"ok": True, "cells": engine.erase(body.x, body.y, body.r)}
        except ValueError as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)

    @app.get("/api/labels")
    def labels():
        return engine.labels

    @app.post("/api/labels")
    def add_label(body: LabelBody):
        try:
            return {"ok": True, "labels": engine.add_label(body.name, body.x, body.y)}
        except (ValueError, OSError) as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)

    @app.post("/api/labels/delete")
    def delete_label(body: NameBody):
        return {"ok": engine.remove_label(body.name), "labels": engine.labels}

    @app.post("/api/relocalize")
    def relocalize():
        s = engine.relocalize(wide=True)
        return {"ok": bool(s), "score": s or 0.0}

    @app.post("/api/estop")
    def api_estop():
        estop()
        return {"ok": True}

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        last_map, last_map_t = -1, 0.0
        try:
            while True:
                msg = {"type": "state", "state": state()}
                now = asyncio.get_event_loop().time()
                if engine.grid.version != last_map and now - last_map_t >= 1.0:   # map image at most 1 Hz
                    last_map_t = now
                    # PNG encode off the shared console event loop
                    png, meta = await asyncio.get_event_loop().run_in_executor(None, engine.map_png)
                    if png:
                        msg["map"] = dict(meta, png=base64.b64encode(png).decode())
                        last_map = engine.grid.version
                await sock.send_json(msg)
                await asyncio.sleep(0.2)
        except (WebSocketDisconnect, RuntimeError):
            pass

    return app
