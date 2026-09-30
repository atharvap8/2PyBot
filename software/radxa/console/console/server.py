"""
server.py — the console. One HTTP/WS API, two clients: the browser GUI
that runs on the Radxa's own hotspot, and the Android app.

Everything the app needs is here; the GUI is just the first consumer.
"""
import asyncio, json, os, time
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import FileResponse, Response, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .config import HTTP_PORT, STREAM_NAME
from .serial_link import link
from . import camera, stream

UI_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ui")

app = FastAPI(title="2PyBot Console", version="2.0")


# ============================================================
#  lifecycle
# ============================================================
@app.on_event("startup")
async def _startup():
    asyncio.create_task(link.run())
    asyncio.create_task(stream.streamer.supervise())


@app.on_event("shutdown")
async def _shutdown():
    stream.streamer.stop()
    link.close()


# ============================================================
#  telemetry + parameters
# ============================================================
@app.get("/api/status")
def status():
    return {
        "connected": link.connected,
        "dump_complete": link.dump_complete,
        "telemetry": link.tel,
        "debug": link.debug,
        "derived": link.derived,
        "stream": stream.streamer.status(),
        "param_count": len(link.params),
    }


@app.get("/api/params")
def get_params():
    if not link.params:
        link.request_dump()
    return {"params": link.params_list(), "derived": link.derived,
            "groups": link.groups(), "complete": link.dump_complete}


class SetParam(BaseModel):
    key: str
    value: float


@app.post("/api/params/set")
def set_param(body: SetParam):
    ok, msg = link.set_param(body.key, body.value)
    if not ok:
        raise HTTPException(400, msg)
    return {"ok": True, "key": body.key, "value": body.value}


@app.post("/api/params/{action}")
def param_action(action: str):
    cmd = {"save": "PS", "load": "PL", "defaults": "PD", "refresh": "P?"}.get(action)
    if not cmd:
        raise HTTPException(404, "action must be save|load|defaults|refresh")
    link.send(cmd)
    return {"ok": True, "sent": cmd}


class Cmd(BaseModel):
    line: str


# Commands the app/GUI may send. A remote link may STOP the robot but
# never START it, so 'E' (enable) is deliberately NOT in this list.
ALLOWED_CMDS = {"X", "C", "S", "L", "R", "P?", "PS", "PL", "PD"}
ALLOWED_PREFIX = ("G,", "H,", "A,", "AT,", "K1=", "K2=", "K3=", "K4=", "K5=", "T=", "M=", "P,")


@app.post("/api/command")
def command(body: Cmd):
    line = body.line.strip()
    if line not in ALLOWED_CMDS and not line.startswith(ALLOWED_PREFIX):
        raise HTTPException(403, f"'{line}' is not permitted from a remote client")
    link.send(line)
    return {"ok": True, "sent": line}


@app.post("/api/estop")
def estop():
    link.send("X")
    return {"ok": True}


@app.post("/api/pose/reset")
def pose_reset():
    link.reset_pose()
    return {"ok": True}


@app.get("/api/path")
def path():
    return {"path": list(link.path), "pose": {"x": link.tel.get("x_m", 0),
            "y": link.tel.get("y_m", 0), "heading": link.tel.get("heading", 0)}}


@app.get("/api/log")
def get_log(n: int = 200):
    return {"log": list(link.log)[-n:]}


# ============================================================
#  camera
# ============================================================
@app.get("/api/camera/controls")
def cam_controls():
    return {"device": camera.CAMERA_DEV, "controls": camera.list_controls(),
            "values": camera.current_values(),
            "profiles": list(camera.PROFILES) + list(camera.load_profiles()),
            "formats": stream.probe_formats()}


class CamCtrl(BaseModel):
    name: str
    value: int


@app.post("/api/camera/set")
def cam_set(body: CamCtrl):
    ok, msg = camera.set_control(body.name, body.value)
    if not ok:
        raise HTTPException(400, msg or "v4l2 rejected the value")
    return {"ok": True}


@app.post("/api/camera/profile/{name}")
def cam_profile(name: str):
    user = camera.load_profiles()
    if name in user:
        applied = {}
        for k, v in user[name].items():
            ok, _ = camera.set_control(k, v)
            if ok:
                applied[k] = v
        return {"ok": True, "applied": applied}
    ok, res = camera.apply_profile(name)
    if not ok:
        raise HTTPException(404, res.get("error", "unknown profile"))
    return {"ok": True, **res}


class SaveProfile(BaseModel):
    name: str


@app.post("/api/camera/profile/save")
def cam_profile_save(body: SaveProfile):
    camera.save_profile(body.name, camera.current_values())
    return {"ok": True, "saved": body.name}


@app.post("/api/camera/autocalibrate")
async def cam_autocal(target: float = 110.0):
    loop = asyncio.get_running_loop()
    res = await loop.run_in_executor(None, lambda: camera.autocalibrate(target=target))
    return res


@app.get("/api/camera/snapshot")
def cam_snapshot():
    b = camera.snapshot_bytes()
    if not b:
        raise HTTPException(503, "no snapshot yet — is the stream running?")
    return Response(b, media_type="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@app.get("/api/camera/luma")
def cam_luma():
    return {"luma": camera.measure_luma()}


# ============================================================
#  stream control
# ============================================================
@app.get("/api/stream")
def stream_status():
    return stream.streamer.status()


class StreamCfg(BaseModel):
    width: Optional[int] = None
    bitrate: Optional[str] = None


@app.post("/api/stream/restart")
def stream_restart(body: StreamCfg):
    ok, msg = stream.streamer.start(width=body.width, bitrate=body.bitrate)
    if not ok:
        raise HTTPException(500, msg)
    return {"ok": True, "detail": msg}


@app.post("/api/stream/stop")
def stream_stop():
    stream.streamer.stop()
    return {"ok": True}


# ============================================================
#  websocket — telemetry + param acks + log, 10 Hz
# ============================================================
@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()
    q = link.subscribe()
    try:
        await sock.send_text(json.dumps({"type": "hello", "status": status()},
                                        default=str))
        while True:
            msg = await q.get()
            await sock.send_text(json.dumps(msg, default=str))
    except (WebSocketDisconnect, RuntimeError):
        pass
    finally:
        link.unsubscribe(q)


# ============================================================
#  UI
# ============================================================
if os.path.isdir(UI_DIR):
    app.mount("/ui", StaticFiles(directory=UI_DIR), name="ui")


@app.get("/")
def index():
    return FileResponse(os.path.join(UI_DIR, "index.html"))


def main():
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=HTTP_PORT, log_level="warning")


if __name__ == "__main__":
    main()
