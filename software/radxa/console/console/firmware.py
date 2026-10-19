"""Firmware build registry and durable update requests shared with an independent worker."""
import asyncio
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import struct
import time
import uuid
from .versioning import validate_version

ROOT=Path(os.environ.get("BOT_FIRMWARE_DIR","/var/lib/2pybot/firmware"))
HEARTBEAT=Path(os.environ.get("BOT_FIRMWARE_HEARTBEAT","/run/2pybot-firmware/heartbeat.json"))
BUSY={"preparing","queued","running","recovering"}
HOLD=BUSY|{"recovery_required"}
BUILD_ID=re.compile(r"^[a-f0-9]{64}$")


def read_json(path,default=None):
    try:return json.loads(Path(path).read_text())
    except (OSError,ValueError):return default


def atomic_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+"."+uuid.uuid4().hex+".tmp")
    with temp.open("w",encoding="utf-8") as file:
        json.dump(value,file,indent=2);file.write("\n");file.flush();os.fsync(file.fileno())
    os.replace(temp,path)


def inspect_image(image):
    data=Path(image).read_bytes() if isinstance(image,(str,Path)) else bytes(image)
    if len(data)<288 or data[0]!=0xe9 or not 1<=data[1]<=16:
        raise ValueError("Not an ESP32 application image")
    if struct.unpack_from("<H",data,12)[0]!=0 or struct.unpack_from("<I",data,32)[0]!=0xabcd5432:
        raise ValueError("Expected an ESP32 application, not a bootloader/other chip image")
    offset=24;checksum=0xef
    for _ in range(data[1]):
        if offset+8>len(data):raise ValueError("Truncated segment header")
        _,length=struct.unpack_from("<II",data,offset);offset+=8
        if offset+length>len(data):raise ValueError("Truncated application segment")
        for byte in data[offset:offset+length]:checksum^=byte
        offset+=length
    checksum_at=offset|15
    if checksum_at>=len(data) or data[checksum_at]!=checksum:raise ValueError("Application checksum mismatch")
    end=checksum_at+1
    if data[23]:
        if end+32>len(data) or hashlib.sha256(data[:end]).digest()!=data[end:end+32]:
            raise ValueError("Application embedded SHA-256 mismatch")
        end+=32
    if len(data)>0x300000 or any(byte not in (0,255) for byte in data[end:]):
        raise ValueError("Unexpected application size/trailing image data")
    return {"sha256":hashlib.sha256(data).hexdigest(),"size_bytes":len(data),"image_bytes":end}


def load_build(root,identifier,verify=False):
    if not isinstance(identifier,str) or not BUILD_ID.fullmatch(identifier):raise ValueError("Invalid build ID")
    folder=Path(root)/"builds"/identifier
    manifest=read_json(folder/"manifest.json")
    if not manifest or manifest.get("build_id")!=identifier or manifest.get("sha256")!=identifier:
        raise ValueError("Build manifest is unavailable or inconsistent")
    image=folder/"application.bin"
    if image.is_symlink() or not image.is_file() or image.stat().st_size!=manifest.get("size_bytes"):
        raise ValueError("Build image is unavailable or changed")
    if manifest.get("app_offset")!=0x10000 or manifest.get("partition_bytes")!=0x300000:
        raise ValueError("Build must target the installed Huge APP application partition")
    if verify and inspect_image(image)["sha256"]!=identifier:raise ValueError("Build SHA-256 mismatch")
    return manifest,image


def register_build(root,incoming,installed_candidate=False):
    root=Path(root);incoming=Path(incoming)
    supplied=read_json(incoming/"manifest.json")
    if not supplied:raise ValueError("Missing compiled-build manifest")
    details=inspect_image(incoming/"application.bin");identifier=details["sha256"]
    if supplied.get("sha256")!=identifier or supplied.get("size_bytes")!=details["size_bytes"]:
        raise ValueError("Compiled build does not match its manifest")
    if supplied.get("app_offset")!=0x10000 or supplied.get("partition_bytes")!=0x300000:
        raise ValueError("Unsupported firmware partition target")
    if not re.fullmatch(r"\d+\.\d+\.\d+(?:[-.a-zA-Z0-9]*)",supplied.get("version","")):
        raise ValueError("Missing firmware version")
    validate_version(supplied["version"],allow_legacy=installed_candidate,allow_prerelease=True)
    manifest=dict(supplied,build_id=identifier,registered_at=time.time())
    target=root/"builds"/identifier;target.mkdir(parents=True,exist_ok=True)
    temporary=target/"application.bin.part";shutil.copy2(incoming/"application.bin",temporary)
    os.replace(temporary,target/"application.bin")
    atomic_json(target/"manifest.json",manifest)
    if installed_candidate:atomic_json(root/"rollback_candidate.json",{"build_id":identifier})
    else:atomic_json(root/"latest.json",{"build_id":identifier})
    return manifest


def public_build(manifest):
    return {key:manifest[key] for key in ("build_id","version","sha256","size_bytes","built_at","source_commit","source_dirty") if key in manifest}


class FirmwareManager:
    def __init__(self,link,snapshot,root=ROOT,heartbeat=HEARTBEAT,clock=time.time,host_ready=lambda:True):
        self.link=link;self.snapshot=snapshot;self.root=Path(root);self.heartbeat=Path(heartbeat);self.clock=clock
        self.lock=asyncio.Lock();self.preparing=False
        self.host_ready=host_ready
        job=read_json(self.root/"current.json",{})
        if job.get("status") in HOLD:self.link.hold_for_firmware()

    def job(self):return read_json(self.root/"current.json",{})
    def blocked(self):return self.job().get("status") in HOLD or self.link.maintenance
    def status(self):
        job=self.job();latest=None;installed=None;problem=""
        try:
            identifier=read_json(self.root/"latest.json",{}).get("build_id")
            if identifier:latest=public_build(load_build(self.root,identifier)[0])
        except ValueError as error:problem=str(error)
        current=read_json(self.root/"installed.json",{})
        try:
            if current.get("build_id"):installed=public_build(load_build(self.root,current["build_id"])[0])
        except ValueError:pass
        worker=read_json(self.heartbeat,{})
        worker_ready=self.clock()-worker.get("t",0)<8
        busy=job.get("status") in BUSY
        if job.get("status")=="recovery_required":problem="Controller recovery required; see update log"
        elif busy or self.link.maintenance:problem="Firmware update in progress"
        elif not worker_ready:problem="Firmware worker is unavailable"
        elif not latest:problem=problem or "No compiled firmware build is registered"
        elif not self.host_ready():problem="Start the camera stream and wait for host READY before reflashing"
        elif not self.link.connected or self.link.tel.get("stale",True):problem="Fresh controller telemetry is required"
        elif self.link.tel.get("balancing"):problem="Press local SELECT and wait for IDLE"
        elif not self.link.dump_complete or not self.link.tel.get("startup_ready"):problem="Wait for controller READY"
        elif self.link.derived.get("FLASHLOCK")!=1:problem="One-time FLASHLOCK firmware installation is required"
        elif self.link.tel.get("firmware_lock"):problem="Controller is reconnecting after maintenance"
        return {"latest":latest,"installed":installed,"job":job or None,"busy":busy or self.link.maintenance,
                "worker_ready":worker_ready,"can_flash":not bool(problem),"blocked_reason":problem}

    async def start(self,build_id):
        if self.lock.locked():raise ValueError("An update request is already being prepared")
        async with self.lock:
            status=self.status()
            if not status["can_flash"]:raise ValueError(status["blocked_reason"])
            if status["latest"]["build_id"]!=build_id:raise ValueError("Latest build changed; refresh before reflashing")
            manifest,image=await asyncio.to_thread(load_build,self.root,build_id,True)
            snapshot=self.snapshot()
            values={p["key"]:p["value"] for p in snapshot["params"]}
            if not values or any(not re.fullmatch(r"[A-Z0-9_]{1,20}",k) or not math.isfinite(v) for k,v in values.items()):
                raise ValueError("A complete valid parameter snapshot is required")
            identifier=uuid.uuid4().hex;folder=self.root/"jobs"/identifier;folder.mkdir(parents=True)
            shutil.copy2(image,folder/"selected.bin")
            atomic_json(folder/"request.json",{"id":identifier,"build":manifest,"snapshot":snapshot,"created_at":self.clock()})
            job={"id":identifier,"build_id":build_id,"version":manifest["version"],"status":"preparing", "stage":"locking",
                 "progress":1,"message":"Locking controller and saving live tuning","created_at":self.clock(),"updated_at":self.clock()}
            self.preparing=True;atomic_json(self.root/"current.json",job)
            try:
                await self.link.begin_firmware_maintenance()
                snapshot["params"]=self.link.params_list() # any raw diagnostic was stopped by FU,1
                atomic_json(folder/"request.json",{"id":identifier,"build":manifest,"snapshot":snapshot,"created_at":self.clock()})
                job.update(status="queued",stage="queued",progress=3,message="Queued on bot; phone may disconnect",updated_at=self.clock())
                atomic_json(self.root/"current.json",job)
            except BaseException as error:
                job.update(status="failed",stage="preparing",message=str(error),error=str(error),updated_at=self.clock())
                atomic_json(self.root/"current.json",job)
                if self.link._serial_paused:self.link.resume_after_firmware()
                else:self.link.maintenance=False
                raise
            finally:self.preparing=False
            return self.status()

    async def monitor(self):
        while True:
            job=self.job();hold=job.get("status") in HOLD
            if hold and not self.preparing and not self.link._serial_paused:self.link.hold_for_firmware()
            elif not hold and self.link.maintenance:self.link.resume_after_firmware()
            await asyncio.sleep(.20)
