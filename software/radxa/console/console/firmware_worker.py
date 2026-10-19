"""Independent, unprivileged firmware worker. API and camera remain online."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import struct
import subprocess
import threading
import time
import urllib.request

import serial

from .firmware import ROOT, HEARTBEAT, atomic_json, inspect_image, load_build, read_json, hardware_mismatch

API=os.environ.get("BOT_FIRMWARE_API","http://127.0.0.1:8080")
ESPTOOL=os.environ.get("BOT_ESPTOOL_PYTHON","/home/radxa/tools/2pybot-flasher/bin/python3")
PORT=os.environ.get("BOT_SERIAL","/dev/ttyUSB0")


def api(path):
    with urllib.request.urlopen(API+path,timeout=10) as response:return json.load(response)


def app_partition(prefix):
    apps=[]
    for offset in range(0x8000,0x9000,32):
        if prefix[offset:offset+2]!=b"\xaaP":break
        _,kind,subtype,address,size,label,flags=struct.unpack("<HBBII16sI",prefix[offset:offset+32])
        if kind==0:apps.append((address,size))
    if apps!=[(0x10000,0x300000)]:raise ValueError("Installed partition layout is not Huge APP at 0x10000")
    return apps[0]


class Controller:
    def __init__(self,log):
        self.log=log;self.buffer=b"";self.params={};self.derived={};self.expected=0
        self.complete=False;self.ready=False;self.locked=False;self.balancing=False;self.acks={}
        self.port=serial.Serial();self.port.port=PORT;self.port.baudrate=460800;self.port.timeout=.1
        self.port.exclusive=True;self.port.dtr=False;self.port.rts=False;self.port.open()
    def send(self,text):self.port.write((text+"\n").encode());self.port.flush()
    def close(self):self.port.close()
    def pump(self):
        self.buffer+=self.port.read(8192)
        while b"\n" in self.buffer:
            line,self.buffer=self.buffer.split(b"\n",1);text=line.decode(errors="replace").strip()
            if not text:continue
            with self.log.open("a",encoding="utf-8") as file:file.write(text+"\n")
            try:
                if text.startswith("P#,"):
                    self.expected=int(text[3:]);self.params={};self.complete=False
                elif text=="P.":self.complete=len(self.params)==self.expected and self.expected>0
                elif text.startswith("P,"):
                    fields=text.split(",",6)
                    if len(fields)==7:self.params[fields[1]]=float(fields[2])
                elif text.startswith("PR,"):
                    _,key,value=text.split(",",2);self.derived[key]=float(value)
                elif text.startswith("P!,"):
                    _,key,value=text.split(",",2);self.acks[key]=float(value)
                elif text=="FU!,1":self.locked=True
                elif text=="FU!,0":self.locked=False
                elif text=="BS,READY":self.ready=True
                elif text=="BS,ERROR":raise RuntimeError("Controller reported startup ERROR")
                elif text.startswith("D,"):
                    fields=text.split(",")
                    if len(fields)>=14:self.balancing=bool(int(float(fields[13])))
                if "-> BALANCING" in text or self.balancing:raise RuntimeError("Controller unexpectedly armed during maintenance")
            except (ValueError,IndexError):pass
        if len(self.buffer)>65536:self.buffer=b""
    def wait(self,predicate,timeout,message,retry=None):
        end=time.monotonic()+timeout;last=time.monotonic()
        while time.monotonic()<end:
            if retry and time.monotonic()-last>.6:retry();last=time.monotonic()
            self.pump()
            if predicate():return
        raise RuntimeError(message)
    def dump(self):
        self.complete=False;self.send("P?")
        self.wait(lambda:self.complete,10,"Complete parameter dump unavailable",lambda:self.send("P?"))
    def initialize_locked(self):
        self.dump()
        if self.derived.get("FLASHLOCK")!=1 or self.derived.get("STARTUPACK")!=1:
            raise RuntimeError("Selected image lacks FLASHLOCK/STARTUPACK support")
        self.send("FU,1")
        self.wait(lambda:self.locked,8,"Maintenance lock was not acknowledged",lambda:self.send("FU,1"))
        self.send("X")
        if not api("/api/ready").get("ready"):raise RuntimeError("Host camera/API/hotspot not ready for calibration")
        self.send("BR,1")
        self.wait(lambda:self.ready,30,"Controller initialization/calibration timed out",lambda:self.send("BR,1"))
        if not self.locked:raise RuntimeError("Arming lock was lost")
    def restore(self,snapshot):
        self.dump();wanted={p["key"]:p["value"] for p in snapshot["params"]}
        for key,value in wanted.items():
            if key not in self.params:raise RuntimeError("Firmware removed saved parameter "+key)
            if abs(self.params[key]-value)>max(1e-5,abs(value)*1e-5):
                self.acks.pop(key,None);self.send(f"P,{key},{value:.6g}")
                self.wait(lambda:key in self.acks,3,"Parameter acknowledgement missing: "+key)
        self.dump()
        if not all(abs(self.params[k]-v)<=max(1e-5,abs(v)*1e-5) for k,v in wanted.items()):
            raise RuntimeError("Live tuning restoration failed")
        debug=snapshot.get("debug",{})
        if self.derived.get("AUTOCLIMB")==1:
            self.send("AC,1" if debug.get("acEnabled",1) else "AC,0")
            policy=int(debug.get("acPolicy",0))
            if policy in (1,2,3):self.send({1:"H,0",2:"H,2",3:"H,1"}[policy])
        self.send("X");self.pump()
        if self.balancing or not self.locked:raise RuntimeError("Controller must remain locked and idle")
        return len(wanted)


class Worker:
    def __init__(self,root=ROOT,controller_factory=Controller):
        self.root=Path(root);self.controller_factory=controller_factory;self.lock=threading.RLock()
        self.state={};self.folder=None
    def progress(self,stage,percent,message,**extra):
        with self.lock:
            self.state.update(stage=stage,progress=percent,message=message,updated_at=time.time(),**extra)
            atomic_json(self.root/"current.json",self.state)
            atomic_json(self.folder/"result.json",self.state)
    def command(self,args,logname,writing=False):
        command=[ESPTOOL,"-m","esptool","--chip","esp32","--port",PORT,"--baud","230400",
                 "--after","no_reset"]+args
        with (self.folder/logname).open("w",encoding="utf-8") as log:
            process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors="replace")
            def reader():
                for line in process.stdout:
                    log.write(line);log.flush()
                    if writing:
                        match=re.search(r"Writing at.*?\(\s*(\d+)\s*%\)",line)
                        if match:self.progress("writing",40+int(match[1])*.25,f"Writing application: {match[1]}%")
            thread=threading.Thread(target=reader,daemon=True);thread.start()
            try:code=process.wait(timeout=90)
            except subprocess.TimeoutExpired:
                process.kill();process.wait();raise RuntimeError("Serial flash command timed out; see "+logname)
            finally:thread.join(timeout=3)
        if code:raise RuntimeError("Serial flash command failed; see "+logname)
    def boot(self,snapshot,rollback=False):
        self.progress("calibrating",85,"Restarting controller; keep robot still. Motors remain locked.")
        connection=self.controller_factory(self.folder/("rollback-serial.log" if rollback else "startup-serial.log"))
        try:
            # All native steps used no_reset. Explicit reset starts the application
            # only after prefix/image verification has finished.
            connection.port.rts=True;time.sleep(.12);connection.port.rts=False;time.sleep(.35)
            connection.initialize_locked()
            self.progress("restoring",92,"Restoring live tuning and verifying idle state")
            count=connection.restore(snapshot)
            return count
        finally:connection.close()
    def process(self,state,recover=False):
        identifier=state.get("id","")
        if not re.fullmatch(r"[a-f0-9]{32}",identifier):raise ValueError("Invalid update job ID")
        self.state=dict(state);self.folder=self.root/"jobs"/identifier
        request=read_json(self.folder/"request.json")
        if not request:raise ValueError("Missing update request")
        snapshot=request["snapshot"];rollback=self.folder/"rollback.bin";written=bool(state.get("write_started"))
        # At OS boot the hardware boot-gate owns RTS until this same host-ready
        # condition passes. Wait before touching the port, including recovery.
        deadline=time.monotonic()+90
        while True:
            try:
                if api("/api/ready").get("ready"):break
            except (OSError,ValueError):pass
            if time.monotonic()>=deadline:
                self.progress("waiting_host",0,"Host camera/API readiness unavailable; no serial operation attempted",
                              status="recovery_required" if written else "failed",error="Host not ready")
                return
            time.sleep(.5)
        time.sleep(.35) # allow the early boot-gate to close its serial handle
        if recover:
            self.recover(snapshot,rollback,written,"Worker restarted during update");return
        try:
            self.progress("checking",5,"Checking selected compiled image",status="running")
            if time.time()-request["created_at"]>120:raise RuntimeError("Queued firmware request expired before execution")
            image=self.folder/"selected.bin";build=request["build"]
            mismatch=hardware_mismatch(build,snapshot.get("derived",{}))
            if mismatch:raise RuntimeError(mismatch)
            if inspect_image(image)["sha256"]!=build["sha256"]:raise RuntimeError("Selected application changed")
            # Pause acknowledgement was obtained by the API before queuing.
            live=api("/api/status")
            if live["connected"] or live["telemetry"].get("balancing"):
                raise RuntimeError("Console did not release serial ownership")
            self.progress("backup",10,"Backing up boot/NVS prefix and verifying rollback image")
            prefix=self.folder/"boot-nvs-before.bin"
            self.command(["read_flash","0","0x10000",str(prefix)],"read-prefix.log")
            address,size=app_partition(prefix.read_bytes())
            if image.stat().st_size>size:raise RuntimeError("Application exceeds installed partition")
            installed=read_json(self.root/"installed.json") or read_json(self.root/"rollback_candidate.json",{})
            old,old_image=load_build(self.root,installed.get("build_id"),True)
            self.command(["verify_flash",hex(address),str(old_image)],"verify-current.log")
            shutil.copy2(old_image,rollback);atomic_json(self.folder/"rollback-build.json",old)
            atomic_json(self.folder/"backup.json",{"params":snapshot["params"],"debug":snapshot.get("debug",{}),
                "camera_profile":snapshot.get("camera_profile"),"calibration":snapshot.get("calibration",{}),
                "boot_nvs_sha256":hashlib.sha256(prefix.read_bytes()).hexdigest(),"rollback_sha256":old["sha256"]})
            self.progress("writing",40,"Writing application only at 0x10000",write_started=True)
            written=True;self.command(["write_flash","0x10000",str(image)],"write.log",True)
            self.progress("verifying",68,"Verifying complete application and unchanged boot/NVS")
            self.command(["verify_flash","0x10000",str(image)],"verify-new.log")
            after=self.folder/"boot-nvs-after.bin"
            self.command(["read_flash","0","0x10000",str(after)],"verify-prefix.log")
            if prefix.read_bytes()!=after.read_bytes():raise RuntimeError("Boot/NVS prefix changed unexpectedly")
            count=self.boot(snapshot)
            for filename,digest in snapshot.get("calibration",{}).items():
                if hashlib.sha256(Path(filename).read_bytes()).hexdigest()!=digest:raise RuntimeError("Camera calibration changed")
            if api("/api/camera/controls").get("colour_profile")!=snapshot.get("camera_profile"):
                raise RuntimeError("Camera profile changed")
            atomic_json(self.root/"installed.json",{"build_id":build["build_id"],"verified_at":time.time()})
            self.progress("complete",100,"Firmware verified; reconnecting controller. Local START is required.",
                          status="succeeded",params_preserved=count,application_only=True,boot_nvs_unchanged=True,
                          image_verified=True,rolled_back=False)
        except Exception as error:self.recover(snapshot,rollback,written,str(error))
    def recover(self,snapshot,rollback,written,error):
        try:
            if written:
                self.progress("rollback",75,"Update failed; restoring verified previous application",status="recovering",error=error)
                if not rollback.is_file():raise RuntimeError("Rollback image is unavailable")
                self.command(["write_flash","0x10000",str(rollback)],"rollback-write.log")
                self.command(["verify_flash","0x10000",str(rollback)],"rollback-verify.log")
                self.boot(snapshot,True)
                old=read_json(self.folder/"rollback-build.json",{})
                atomic_json(self.root/"installed.json",{"build_id":old["build_id"],"verified_at":time.time()})
            else:
                # A read/verify operation may have left the chip in its ROM stub.
                # Restart the unchanged application and restore volatile tuning.
                self.boot(snapshot,True)
            self.progress("failed",100,"Update failed; previous firmware restored" if written else "Update stopped before writing",
                          status="failed",error=error,rolled_back=written)
        except Exception as recovery_error:
            self.progress("recovery_required",100,"Serial recovery required; controller remains in maintenance",
                          status="recovery_required",error=error+"; rollback: "+str(recovery_error))


def main():
    ROOT.mkdir(parents=True,exist_ok=True);HEARTBEAT.parent.mkdir(parents=True,exist_ok=True)
    stop=threading.Event()
    def beat():
        while not stop.is_set():atomic_json(HEARTBEAT,{"t":time.time(),"pid":os.getpid()});stop.wait(1)
    threading.Thread(target=beat,daemon=True).start()
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    worker=Worker();first=True
    while not stop.is_set():
        state=read_json(ROOT/"current.json",{})
        if state.get("status")=="queued":worker.process(state)
        elif first and state.get("status") in ("running","recovering"):worker.process(state,True)
        elif state.get("status")=="preparing" and time.time()-state.get("updated_at",0)>15:
            state.update(status="failed",message="Preparation interrupted before flash",error="Preparation interrupted",updated_at=time.time())
            atomic_json(ROOT/"current.json",state)
        first=False;stop.wait(.25)


if __name__=="__main__":main()
