"""
server.py — the console. One HTTP/WS API, two clients: the browser GUI
that runs on the Radxa's own hotspot, and the Android app.

Everything the app needs is here; the GUI is just the first consumer.
"""
import asyncio, json, os
import copy
import hashlib
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Request
from fastapi.responses import FileResponse, Response, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .config import HTTP_PORT, SNAPSHOT_PATH
from .readiness import HOTSPOT_ADDRESS, publish_ready, readiness
from .serial_link import link
from .tof import tof
from . import camera, stream
from .calibration import Calibration
from . import calibration_api
from . import vision
from .object_detection import ObjectDetector
from .firmware import FirmwareManager
from .autonomy import Autonomy
from .deployment_status import STATE_DIR as DEPLOY_STATE, RUN_DIR as DEPLOY_RUN, indicator, read as deployment_read

UI_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ui")

app = FastAPI(title="2PyBot Console", version="2.0")


def calibration_frame():
    image, metadata = vision.snapshot_frame(SNAPSHOT_PATH)
    if image:
        metadata["tof_pair"] = vision.nearest_depth(tof.history, metadata)
    return image, metadata


def calibration_metadata():
    return {"device": camera.CAMERA_DEV, "stream": stream.streamer.status(),
            "telemetry": copy.deepcopy(link.tel), "debug": dict(link.debug),
            "tof": tof.status(link), "payload_pose": "commanded joint state is not in telemetry"}


calibration = Calibration(calibration_frame, calibration_metadata)
calibration_api.register(app, calibration)


def active_intrinsics():
    try:
        identifier = json.loads((calibration.directory / "active.json").read_text())["id"]
        return json.loads((calibration.folder(identifier) / "intrinsics.json").read_text())
    except (OSError, ValueError, KeyError):
        return None


detector = ObjectDetector(calibration_frame, active_intrinsics)


def firmware_snapshot():
    files={}
    active=calibration.directory/"active.json"
    if active.is_file():
        files[str(active)]=hashlib.sha256(active.read_bytes()).hexdigest()
        identifier=json.loads(active.read_text())["id"]
        intrinsics=calibration.folder(identifier)/"intrinsics.json"
        files[str(intrinsics)]=hashlib.sha256(intrinsics.read_bytes()).hexdigest()
    native=camera.native_camera.status() or {}
    return {"params":link.params_list(),"derived":dict(link.derived),"debug":dict(link.debug),"telemetry":copy.deepcopy(link.tel),
            "camera_profile":native.get("colour_profile"),"calibration":files}


firmware=FirmwareManager(link,firmware_snapshot,host_ready=lambda:host_ready().get("ready",False))
autonomy=Autonomy(link,lambda:tof.status(link),firmware.blocked)


def require_loopback(request):
    if not request.client or request.client.host not in ("127.0.0.1","::1"):
        raise HTTPException(403,"Deployment control is local to the bot")


class DeploymentLock(BaseModel):
    token: str
    commit: str


class DeploymentToken(BaseModel):
    token: str


class DeploymentBuild(BaseModel):
    token: str
    build_id: str


class DeploymentLed(BaseModel):
    phase: int
    progress: int
    ttl_ms: int=1500


@app.get("/api/deployment/status")
def deployment_status():
    return {"protocol":1,"progress":deployment_read(DEPLOY_STATE/"progress.json",{}),
        "applied":deployment_read(DEPLOY_STATE/"state.json",{}),
        "indicator":indicator(DEPLOY_STATE,DEPLOY_RUN,firmware.root),
        "maintenance":firmware.blocked(),"deployment_maintenance":bool(firmware.deployment_hold()),"firmware":firmware.status()}


@app.post("/api/internal/deployment/lock")
async def deployment_lock(body:DeploymentLock,request:Request):
    require_loopback(request)
    try:
        hold=await firmware.acquire_deployment(body.token,body.commit)
        return {"locked":True,"token":hold["token"],"commit":hold["commit"]}
    except ValueError as error:raise HTTPException(409,str(error)) from error


@app.post("/api/internal/deployment/release")
async def deployment_release(body:DeploymentToken,request:Request):
    require_loopback(request)
    try:await firmware.release_deployment(body.token);return {"released":True}
    except ValueError as error:raise HTTPException(409,str(error)) from error


@app.post("/api/internal/deployment/firmware")
async def deployment_firmware(body:DeploymentBuild,request:Request):
    require_loopback(request)
    try:return await firmware.queue_deployment_build(body.token,body.build_id)
    except ValueError as error:raise HTTPException(409,str(error)) from error


@app.post("/api/internal/deployment/led")
def deployment_led(body:DeploymentLed,request:Request):
    require_loopback(request)
    return {"sent":link.send_indicator(body.phase,body.progress,body.ttl_ms),
            "backend":"firmware","available_during_esp32_flash":False}


@app.get("/api/autonomy/status")
async def autonomy_status():return autonomy.status()


class RouteInput(BaseModel):
    name: str
    points: list


class TeachInput(BaseModel):
    name: str


class ReplayInput(BaseModel):
    route_id: str
    speed_mps: Optional[float]=None


class SupervisionInput(BaseModel):
    mission_id: str
    lease_key: str


@app.post("/api/autonomy/routes")
async def autonomy_save(body: RouteInput):
    try:return autonomy.save(body.name,body.points)
    except ValueError as error:raise HTTPException(400,str(error)) from error


@app.post("/api/autonomy/teach/start")
async def autonomy_teach(body: TeachInput):
    try:autonomy.begin_teach(body.name);return autonomy.status()
    except ValueError as error:raise HTTPException(409,str(error)) from error


@app.post("/api/autonomy/teach/point")
async def autonomy_teach_point():
    try:autonomy.add_teach_point();return autonomy.status()
    except ValueError as error:raise HTTPException(409,str(error)) from error


@app.post("/api/autonomy/teach/save")
async def autonomy_teach_save():
    try:return autonomy.finish_teach()
    except ValueError as error:raise HTTPException(409,str(error)) from error


@app.post("/api/autonomy/teach/cancel")
async def autonomy_teach_cancel():autonomy.teach=None;return autonomy.status()


@app.post("/api/autonomy/start")
async def autonomy_start(body: ReplayInput):
    try:return await autonomy.start(body.route_id,body.speed_mps)
    except ValueError as error:raise HTTPException(409,str(error)) from error


@app.post("/api/autonomy/resume")
async def autonomy_resume():
    try:return await autonomy.resume()
    except ValueError as error:raise HTTPException(409,str(error)) from error


@app.post("/api/autonomy/heartbeat")
async def autonomy_heartbeat(body: SupervisionInput):
    try:autonomy.heartbeat(body.mission_id,body.lease_key);return {"ok":True}
    except ValueError as error:raise HTTPException(409,str(error)) from error


@app.post("/api/autonomy/pause")
async def autonomy_pause():autonomy.pause();return autonomy.status()


@app.post("/api/autonomy/stop")
async def autonomy_stop():autonomy.stop();return autonomy.status()


@app.middleware("http")
async def firmware_mutation_gate(request,call_next):
    internal=request.url.path.startswith("/api/internal/deployment/")
    if request.method not in ("GET","HEAD","OPTIONS") and not internal and request.url.path not in ("/api/firmware/reflash","/api/estop") and firmware.blocked():
        return JSONResponse({"detail":"Controller firmware maintenance is in progress"},status_code=409)
    return await call_next(request)


@app.get("/api/firmware/status")
def firmware_status():return firmware.status()


class FirmwareRequest(BaseModel):
    build_id: str


@app.post("/api/firmware/reflash")
async def firmware_reflash(body: FirmwareRequest):
    try:return await firmware.start(body.build_id)
    except ValueError as error:raise HTTPException(409,str(error)) from error


class DetectionOptions(BaseModel):
    enabled: bool
    threshold: float = .50
    people_only: bool = False
    corrected: bool = True


@app.get("/api/ml/status")
def detection_status():
    return detector.status()


@app.post("/api/ml/configure")
def detection_configure(body: DetectionOptions):
    try:
        return detector.configure(body.enabled, body.threshold, body.people_only, body.corrected)
    except ValueError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.get("/api/ml/preview.jpg")
def detection_preview():
    image = detector.preview
    if not image:
        image, _ = calibration_frame()
    if not image:raise HTTPException(404, "Camera preview is unavailable")
    return Response(image, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/api/vision/diagnostics")
def vision_diagnostics():
    _, frame = calibration_frame()
    return {"camera_frame": frame, "depth": vision.depth_diagnostics(tof.status(link)),
            "intrinsics_available": (calibration.directory / "active.json").exists(),
            "extrinsics_available": False}


# ============================================================
#  lifecycle
# ============================================================
@app.on_event("startup")
async def _startup():
    app.state.background_tasks = [asyncio.create_task(link.run()),
                                  asyncio.create_task(stream.streamer.supervise()),
                                  asyncio.create_task(tof.acquire()),
                                  asyncio.create_task(tof.publish(link)),
                                  asyncio.create_task(calibration.run()),
                                   asyncio.create_task(detector.run()),
                                   asyncio.create_task(firmware.monitor()),
                                   asyncio.create_task(autonomy.run()),
                                  asyncio.create_task(publish_ready(link, host_ready))]
    app.state.background_tasks.append(asyncio.create_task(camera.native_camera.restore()))


@app.on_event("shutdown")
async def _shutdown():
    for task in app.state.background_tasks:
        task.cancel()
    await asyncio.gather(*app.state.background_tasks, return_exceptions=True)
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
        "tof": tof.status(link),
        "host": host_ready(),
    }


@app.get("/api/ready")
def host_ready():
    return readiness(stream.streamer.status(), SNAPSHOT_PATH, HOTSPOT_ADDRESS)


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
ALLOWED_CMDS = {"X", "C", "S", "L", "R", "N", "P?", "PS", "PL", "PD", "AC,0", "AC,1"}
ALLOWED_PREFIX = ("G,", "H,", "A,", "AT,", "K1=", "K2=", "K3=", "K4=", "K5=", "T=", "M=", "P,")


@app.post("/api/command")
def command(body: Cmd):
    line = body.line.strip()
    if line not in ALLOWED_CMDS and not line.startswith(ALLOWED_PREFIX):
        raise HTTPException(403, f"'{line}' is not permitted from a remote client")
    if line not in {"N","S","L","P?"}:
        autonomy.pause("operator controller command")
    link.send(line)
    return {"ok": True, "sent": line}


@app.post("/api/estop")
def estop():
    autonomy.stop("operator E-STOP","aborted")
    link.send("X")
    return {"ok": True}


@app.post("/api/pose/reset")
def pose_reset():
    autonomy.stop("operator reset odometry","aborted")
    link.reset_pose()
    return {"ok": True}


@app.get("/api/path")
def path():
    return {"path": list(link.path), "pose": {"x": link.tel.get("x_m", 0),
            "y": link.tel.get("y_m", 0), "heading": link.tel.get("heading", 0)}}


@app.get("/api/log")
def get_log(n: int = 200):
    return {"log": list(link.log)[-n:]}


@app.get("/api/tof")
def tof_status():
    return tof.status(link)


class ToFCalibration(BaseModel):
    reference_m: float


@app.post("/api/tof/calibrate")
def tof_calibrate(body: ToFCalibration):
    try:
        tof.calibrate(body.reference_m)
    except ValueError as exc:
        raise HTTPException(409, str(exc)) from exc
    except OSError as exc:
        raise HTTPException(500, "Could not persist ToF calibration") from exc
    return tof.status(link)


class ToFGuard(BaseModel):
    enabled: bool


@app.post("/api/tof/guard")
async def tof_guard(body: ToFGuard):
    if body.enabled and (not link.connected or link.derived.get("TOFGUARD") != 1):
        raise HTTPException(409, "Install ToF-guard firmware and connect the ESP32 first")
    tof.guard_enabled = body.enabled
    if not body.enabled:
        link.send("TG,0,0")
    else:
        link.send(f"TG,1,{tof.status(link)['forward_limit_mps']:.3f}")
    return tof.status(link)


# ============================================================
#  camera
# ============================================================
@app.get("/api/camera/controls")
def cam_controls():
    native = camera.native_camera.status() or {}
    return {"device": camera.CAMERA_DEV, "controls": camera.list_controls(),
            "values": camera.current_values(),
            "colour_profile": native.get("colour_profile"), "ccm_enabled": native.get("ccm_enabled"),
            "profiles": camera.profile_names(),
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


class SaveProfile(BaseModel):
    name: str


@app.post("/api/camera/profile/save")
def cam_profile_save(body: SaveProfile):
    camera.save_profile(body.name, camera.current_values())
    return {"ok": True, "saved": body.name}


@app.post("/api/camera/profile/{name}")
def cam_profile(name: str):
    user = camera.load_profiles()
    if name in user:
        if camera.native_camera.status():
            return {"ok": True, **camera.native_camera.apply_values(user[name])}
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
    @app.get("/ui/android.apk")
    def latest_apk():
        from pathlib import Path
        latest=deployment_read(Path(os.environ.get("BOT_DOWNLOAD_DIR","/var/lib/2pybot/downloads"))/'latest.json',{})
        if not latest.get('version'):raise HTTPException(404,'No checked APK has been published yet')
        return published_apk(latest['version'])

    @app.get("/ui/2PyBot-v{version}-debug.apk")
    def published_apk(version:str):
        from pathlib import Path
        from .versioning import validate_version
        try:validate_version(version,allow_legacy=True,allow_prerelease=True)
        except ValueError:raise HTTPException(404,"Unknown build")
        path=Path(os.environ.get("BOT_DOWNLOAD_DIR","/var/lib/2pybot/downloads"))/f"2PyBot-v{version}-debug.apk"
        # Old checkout installations keep their existing downloads until migrated.
        if not path.is_file():path=Path(UI_DIR)/f"2PyBot-v{version}-debug.apk"
        if not path.is_file():raise HTTPException(404,"APK is not published")
        return FileResponse(path,media_type="application/vnd.android.package-archive",filename=path.name)

    @app.get("/ui/2PyBot-v{version}-manifest.json")
    def published_apk_manifest(version:str):
        from pathlib import Path
        from .versioning import validate_version
        try:validate_version(version,allow_legacy=True,allow_prerelease=True)
        except ValueError:raise HTTPException(404,"Unknown build")
        path=Path(os.environ.get("BOT_DOWNLOAD_DIR","/var/lib/2pybot/downloads"))/f"2PyBot-v{version}-manifest.json"
        if not path.is_file():path=Path(UI_DIR)/f"2PyBot-v{version}-manifest.json"
        if not path.is_file():raise HTTPException(404,'Manifest is not published')
        return FileResponse(path,media_type='application/json')

    app.mount("/ui", StaticFiles(directory=UI_DIR), name="ui")


@app.get("/")
def index():
    return FileResponse(os.path.join(UI_DIR, "index.html"))


def main():
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=HTTP_PORT, log_level="warning")


if __name__ == "__main__":
    main()
