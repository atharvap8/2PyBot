"""Supervised start-relative route teaching/replay; no remote motor arming."""
import asyncio
import copy
import math
import os
from pathlib import Path
import secrets
import time
import uuid

from .firmware import atomic_json,read_json

ROOT=Path(os.environ.get("BOT_AUTONOMY_DIR","/var/lib/2pybot/autonomy"))
ENGAGED={"running"}


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))
def clip(value,lo,hi):return min(hi,max(lo,value))


def relative_pose(pose,origin):
    dx=pose[0]-origin[0];dy=pose[1]-origin[1];c=math.cos(origin[2]);s=math.sin(origin[2])
    return {"x":round(c*dx+s*dy,4),"y":round(-s*dx+c*dy,4),"heading":round(wrap(pose[2]-origin[2]),5)}


def world_points(points,origin):
    c=math.cos(origin[2]);s=math.sin(origin[2])
    return [{"x":origin[0]+c*p["x"]-s*p["y"],"y":origin[1]+s*p["x"]+c*p["y"],
             "heading":wrap(origin[2]+p["heading"])} for p in points]


def validate_points(points):
    if not isinstance(points,list) or len(points)<2:raise ValueError("A route needs at least two waypoints")
    clean=[];length=0
    for point in points:
        try:p={key:float(point[key]) for key in ("x","y","heading")}
        except (KeyError,TypeError,ValueError):raise ValueError("Invalid waypoint")
        if not all(math.isfinite(v) for v in p.values()):raise ValueError("Waypoints must be finite")
        p["heading"]=wrap(p["heading"])
        if clean:length+=math.hypot(p["x"]-clean[-1]["x"],p["y"]-clean[-1]["y"])
        clean.append(p)
    if math.hypot(clean[0]["x"],clean[0]["y"])>.01:raise ValueError("First waypoint must be the relative origin")
    return clean,length


def depth_clearance(info,turning=False,speed=0.0):
    if info.get("state")!="ranging" or info.get("stale",True) or info.get("age_ms") is None or info["age_ms"]>300:
        return {"clear":False,"reason":"depth_stale","nearest_m":None,"coverage":0}
    valid=info.get("valid",[]);ranges=info.get("corrected_distance_mm",info.get("distance_mm",[]))
    if len(valid)!=64 or len(ranges)!=64:return {"clear":False,"reason":"depth_unavailable","nearest_m":None,"coverage":0}
    roi=[r*8+c for r in range(2,6) for c in range(2,6)]
    observed=[i for i in roi if valid[i] and isinstance(ranges[i],(int,float)) and math.isfinite(ranges[i])]
    coverage=len(observed)/16
    if len(observed)<12 or any(sum(i in observed for i in [r*8+c for r in range(2,6)])<2 for c in range(2,6)):
        return {"clear":False,"reason":"depth_unknown","nearest_m":None,"coverage":coverage}
    nearest=min(ranges[i]/1000 for i in observed)
    if turning:
        for cols in ((0,1),(6,7)):
            zones=[r*8+c for r in range(2,6) for c in cols]
            if sum(bool(valid[i]) for i in zones)<6:
                return {"clear":False,"reason":"turn_depth_unknown","nearest_m":nearest,"coverage":coverage}
        nearest=min([nearest]+[ranges[i]/1000 for i in range(64) if valid[i] and math.isfinite(ranges[i])])
    stop=.35 if turning else .30
    speed=max(0.0,abs(speed));reaction=.40;deceleration=.35
    required=max(stop,.25+speed*reaction+speed*speed/(2*deceleration))
    clearance=max(0.0,nearest-.25)
    safe_speed=math.sqrt((deceleration*reaction)**2+2*deceleration*clearance)-deceleration*reaction
    reason="obstacle_near" if nearest<=stop else "braking_clearance" if nearest<=required else "clear"
    return {"clear":reason=="clear","reason":reason,"nearest_m":round(nearest,3),"coverage":coverage,
            "speed_cap_mps":max(0,round(safe_speed,3)),"stopping_clearance_m":round(required,3)}


def pursuit(pose,points,index,speed=.60):
    while index<len(points)-1 and math.hypot(points[index]["x"]-pose[0],points[index]["y"]-pose[1])<.12:index+=1
    point=points[index];dx=point["x"]-pose[0];dy=point["y"]-pose[1];distance=math.hypot(dx,dy)
    final=index==len(points)-1
    if final and distance<.12:
        heading=wrap(point["heading"]-pose[2])
        if abs(heading)<math.radians(12):return {"v":0,"w":0,"index":index,"distance":distance,"bearing":heading,"done":True}
        return {"v":0,"w":clip(1.2*heading,-.45,.45),"index":index,"distance":distance,"bearing":heading,"done":False}
    bearing=wrap(math.atan2(dy,dx)-pose[2])
    # Intermediate 0.20 m teaching samples must not cap cruise speed. Look
    # along the polyline for the next sharp corner/final stop instead.
    stop_distance=distance
    incoming=math.atan2(dy,dx)
    for j in range(index,len(points)-1):
        following=points[j+1];current=points[j]
        outgoing=math.atan2(following["y"]-current["y"],following["x"]-current["x"])
        if abs(wrap(outgoing-incoming))>.60:break
        stop_distance+=math.hypot(following["x"]-current["x"],following["y"]-current["y"])
        incoming=outgoing
    remaining=max(0,stop_distance-.12);brake=.25;reaction=.30
    approach=math.sqrt((brake*reaction)**2+2*brake*remaining)-brake*reaction
    v=0 if abs(bearing)>.65 else min(speed,max(.05,approach))*max(.25,math.cos(bearing))
    return {"v":v,"w":clip(1.3*bearing,-.45,.45),"index":index,"distance":distance,"bearing":bearing,"done":False}


class Autonomy:
    def __init__(self,link,depth,firmware_busy=lambda:False,root=ROOT,clock=time.monotonic):
        self.link=link;self.depth=depth;self.firmware_busy=firmware_busy;self.root=Path(root);self.clock=clock
        self.teach=None;self.selected=None;self.mission=None;self.lease_key=None
        self.last_operator=0;self.sequence=0;self.last_tick=clock();self.last_progress=clock();self.best_metric=math.inf
        self.last_v=self.last_w=0;self.events=[];self.lock=asyncio.Lock()
        self.action_epoch=0
    def pose(self):
        pose=(float(self.link.tel.get("x_m",0)),float(self.link.tel.get("y_m",0)),math.radians(float(self.link.tel.get("heading",0))))
        if not all(math.isfinite(v) for v in pose):raise ValueError("Odometry pose is invalid")
        return pose
    def event(self,text):
        self.events=(self.events+[{"t":time.time(),"text":text}])[-40:]
        self.link._log("[AUTONOMY] "+text)
    def tx(self,line):
        try:return self.link.send(line)
        except (OSError,RuntimeError):return False
    def routes(self):
        result=[]
        for path in sorted((self.root/"routes").glob("*.json")):
            data=read_json(path)
            if data:
                item={key:data[key] for key in ("id","name","length_m","points","created_at")}
                item.update(recorded=bool(data.get("samples")),duration_s=data.get("duration_s"),sample_count=len(data.get("samples",[])))
                result.append(item)
        return result
    def route(self,identifier):
        if not isinstance(identifier,str) or len(identifier)!=32 or any(c not in "0123456789abcdef" for c in identifier):
            raise ValueError("Invalid route ID")
        route=read_json(self.root/"routes"/(identifier+".json"))
        if not route:raise ValueError("Route unavailable")
        validate_points(route["points"]);return route
    def save(self,name,points,samples=None):
        name=str(name).strip()
        if not 1<=len(name)<=50:raise ValueError("Route name must contain 1–50 characters")
        points,length=validate_points(points)
        route={"schema":2,"id":uuid.uuid4().hex,"name":name,"points":points,"length_m":round(length,3),
                "created_at":time.time(),"frame":"start-relative wheel odometry; x forward, y left; heading CCW radians"}
        if samples:route.update(samples=samples,duration_s=samples[-1]["t"],replay_mode="original_timing_and_commands")
        atomic_json(self.root/"routes"/(route["id"]+".json"),route);self.selected=route["id"];return route
    def begin_teach(self,name):
        if self.mission and self.mission["state"] in ENGAGED:raise ValueError("Pause or stop replay before teaching")
        if not self.link.connected or self.link.tel.get("stale",True):raise ValueError("Fresh controller odometry is required")
        self.teach={"name":str(name),"origin":self.pose(),"epoch":self.link.pose_epoch,"points":[{"x":0,"y":0,"heading":0}],
                    "state":"recording","started":self.clock(),"samples":[]};self.record_sample();self.event("Teaching original speed, turns and timing")
    def record_sample(self):
        teach=self.teach;now=self.clock();point=relative_pose(self.pose(),teach["origin"])
        samples=teach["samples"];t=now-teach["started"]
        if samples and t<=samples[-1]["t"]:return
        v=float(self.link.debug.get("driveReq",self.link.tel.get("v_ms",0)))
        steps=self.link.derived.get("STEPSM",5092.96)
        track=self.link.params["TRACKW"].value if hasattr(self.link,"params") and "TRACKW" in self.link.params else .155
        w=2*float(self.link.debug.get("steer",0))/(steps*track)
        if not self.link.tel.get("balancing") and samples:
            previous=samples[-1];dt=t-previous["t"]
            v=((point["x"]-previous["x"])*math.cos(point["heading"])+(point["y"]-previous["y"])*math.sin(point["heading"]))/dt
            w=wrap(point["heading"]-previous["heading"])/dt
        if not math.isfinite(v) or not math.isfinite(w):raise ValueError("Invalid recorded drive reference")
        samples.append(dict(point,t=t,v=v,w=w))
    def add_teach_point(self):
        if not self.teach or self.teach["state"]!="recording":raise ValueError("No active route teaching")
        point=relative_pose(self.pose(),self.teach["origin"]);previous=self.teach["points"][-1]
        if math.hypot(point["x"]-previous["x"],point["y"]-previous["y"])<.05:return
        self.teach["points"].append(point)
    def finish_teach(self):
        if not self.teach:raise ValueError("No route teaching to save")
        if self.teach["state"]=="recording":self.record_sample();self.add_teach_point()
        if len(self.teach["points"])<2:self.teach["points"].append(relative_pose(self.pose(),self.teach["origin"]))
        route=self.save(self.teach["name"],self.teach["points"],self.teach["samples"]);self.teach=None
        self.event("Saved route "+route["name"]);return route
    def prerequisites(self):
        reasons=[];link=self.link;nav=link.nav
        if self.firmware_busy():reasons.append("Firmware maintenance is active")
        if link.derived.get("NAVIGATION")!=1:reasons.append("Install NAVIGATION-capable firmware")
        if not link.connected or link.tel.get("stale",True) or self.clock()-link.last_odom_monotonic>.30:reasons.append("Controller telemetry is stale")
        if not link.tel.get("startup_ready") or not link.tel.get("balancing"):reasons.append("Press local START and wait for balancing")
        if self.clock()-nav.get("received_monotonic",0)>.35:reasons.append("Navigation feedback is stale")
        if not nav.get("pad"):reasons.append("Connect the local gamepad")
        try:self.pose()
        except ValueError:reasons.append("Odometry pose is invalid")
        return reasons
    def depth_status(self):
        # Display-only: missing/close depth must never prevent replay or limit it.
        try:clearance=depth_clearance(self.depth())
        except Exception:clearance={"reason":"depth_unavailable","nearest_m":None,"coverage":0}
        return dict(clearance,advisory_only=True,gating=False)
    def status(self):
        reasons=self.prerequisites()
        teaching=None if not self.teach else {k:v for k,v in self.teach.items() if k not in ("origin","started","samples")}
        if teaching is not None:teaching["sample_count"]=len(self.teach["samples"])
        try:
            p=self.pose();pose={"x":p[0],"y":p[1],"heading":p[2],"epoch":self.link.pose_epoch}
        except ValueError:pose=None
        return {"supported":self.link.derived.get("NAVIGATION")==1,"pose":pose,
                "routes":self.routes(),"teaching":copy.deepcopy(teaching),"mission":None if not self.mission else copy.deepcopy({k:v for k,v in self.mission.items() if k!="samples"}),
                "can_start":not reasons,"blocked_reasons":reasons,"depth":self.depth_status(),"events":self.events,
                "controller":dict(self.link.nav),"limits":{"speed_mps":None,"default_speed_mps":None,"yaw_rps":None,
                "route_length_m":None,"forward_only":False,"supervision_timeout_s":3,"front_only_sensing":True,
                "replay_mode":"original_timing_and_commands","host_speed_shaping":False,
                "tof_gating":False,"payload_required":False}}
    async def grant(self):
        owner=secrets.randbelow(0x7ffffffe)+1
        if not self.tx(f"AN,{owner},1"):raise ValueError("Serial command unavailable")
        end=self.clock()+1.2
        while self.clock()<end:
            nav=self.link.nav
            if nav.get("token")==owner:
                if nav.get("state") in (1,2):self.sequence=0;return owner
                if nav.get("state")==9:raise ValueError("Controller refused: centre drive sticks and wait for stable balance")
            await asyncio.sleep(.02)
        self.tx(f"AN,{owner},0");raise ValueError("Navigation grant timed out")
    async def start(self,identifier,speed=None):
        async with self.lock:
            if self.mission and self.mission["state"] in ENGAGED:raise ValueError("A route is already engaged")
            if self.teach:raise ValueError("Save or cancel route teaching first")
            reasons=self.prerequisites()
            if reasons:raise ValueError("; ".join(reasons))
            if speed is not None and (not isinstance(speed,(int,float)) or not math.isfinite(speed) or speed<=0):raise ValueError("Replay speed must be finite and positive")
            route=self.route(identifier);origin=self.pose();generation=self.action_epoch;owner=await self.grant()
            if generation!=self.action_epoch:
                self.tx(f"AN,{owner},0");raise ValueError("Start was cancelled")
            self.mission={"id":uuid.uuid4().hex,"route_id":identifier,"name":route["name"],"state":"running","reason":"",
                "origin":origin,"points":world_points(route["points"],origin),"index":1,"speed_mps":speed or (self.link.params["DRIVEV"].value if hasattr(self.link,"params") and "DRIVEV" in self.link.params else .908784),"elapsed_s":0,
                "samples":route.get("samples",[]),"sample_index":0,"replay_mode":"original" if route.get("samples") else "legacy_waypoints",
                "distance_m":0,"command_v":0,"command_w":0,"epoch":self.link.pose_epoch,"token":owner}
            self.lease_key=secrets.token_hex(16);self.last_operator=self.clock();self.last_progress=self.clock();self.best_metric=math.inf
            self.last_v=self.last_w=0;self.event("Replay started: "+route["name"])
            return {"status":self.status(),"lease_key":self.lease_key,"mission_id":self.mission["id"]}
    def heartbeat(self,identifier,key):
        if not self.mission or self.mission["id"]!=identifier or not self.lease_key or not secrets.compare_digest(key,self.lease_key):
            raise ValueError("Supervision lease is not current; resume explicitly")
        self.last_operator=self.clock()
    def pause(self,reason="operator_pause"):
        self.action_epoch+=1
        if self.mission and self.mission["state"] in ENGAGED:
            self.tx(f"AN,{self.mission['token']},0");self.mission.update(state="paused",reason=reason,command_v=0,command_w=0)
            self.last_v=self.last_w=0;self.lease_key=None;self.event("Replay paused: "+reason)
    def stop(self,reason="operator_stop",state="stopped"):
        self.action_epoch+=1
        if self.mission:
            self.tx(f"AN,{self.mission['token']},0");self.mission.update(state=state,reason=reason,command_v=0,command_w=0)
            self.last_v=self.last_w=0;self.lease_key=None;self.event("Replay "+state+": "+reason)
    async def resume(self):
        async with self.lock:
            if not self.mission or self.mission["state"]!="paused":raise ValueError("No paused route to resume")
            if self.mission["epoch"]!=self.link.pose_epoch:raise ValueError("Odometry frame changed; restart at the marked route start")
            reasons=self.prerequisites()
            if reasons:raise ValueError("; ".join(reasons))
            generation=self.action_epoch;owner=await self.grant()
            if generation!=self.action_epoch:
                self.tx(f"AN,{owner},0");raise ValueError("Resume was cancelled")
            self.mission["token"]=owner;self.mission.update(state="running",reason="")
            self.lease_key=secrets.token_hex(16);self.last_operator=self.clock();self.last_progress=self.clock();self.best_metric=math.inf
            self.event("Replay explicitly resumed")
            return {"status":self.status(),"lease_key":self.lease_key,"mission_id":self.mission["id"]}
    def drive(self,v,w):
        if not self.mission:return
        self.sequence=(self.sequence+1)&0xffffffff
        if not self.tx(f"AU,{self.mission['token']},{self.sequence},{v:.4f},{w:.4f}"):
            self.pause("serial motion command failed");return
        self.mission.update(command_v=round(v,4),command_w=round(w,4))
    def tick(self):
        now=self.clock();dt=max(0,now-self.last_tick);self.last_tick=now
        if self.teach and self.teach["state"]=="recording":
            if self.link.pose_epoch!=self.teach["epoch"] or not self.link.connected or self.link.tel.get("stale",True):
                self.teach["state"]="interrupted";self.event("Teaching interrupted; save partial route or start again")
            else:
                self.record_sample();point=relative_pose(self.pose(),self.teach["origin"]);last=self.teach["points"][-1]
                if math.hypot(point["x"]-last["x"],point["y"]-last["y"])>=.20:self.add_teach_point()
        m=self.mission
        if not m or m["state"] not in ENGAGED:return
        if self.firmware_busy() or self.link.pose_epoch!=m["epoch"]:self.stop("odometry/maintenance change","aborted");return
        if not self.link.connected or self.link.tel.get("stale",True) or now-self.link.last_odom_monotonic>.30:self.pause("controller feedback lost");return
        if not self.link.tel.get("balancing"):self.stop("controller disarmed/fell","aborted");return
        nav=self.link.nav
        if nav.get("token")!=m["token"] or nav.get("state") not in (1,2) or now-nav.get("received_monotonic",0)>.35:
            self.pause("gamepad takeover / native lease lost");return
        if now-self.last_operator>3:self.pause("supervisor disconnected/backgrounded");return
        if m["samples"]:
            m["elapsed_s"]+=dt;samples=m["samples"];index=m["sample_index"]
            while index<len(samples)-1 and samples[index+1]["t"]<=m["elapsed_s"]:index+=1
            m["sample_index"]=index;m["index"]=min(len(m["points"])-1,int(index*len(m["points"])/len(samples)))
            m["distance_m"]=round(math.hypot(m["points"][-1]["x"]-self.pose()[0],m["points"][-1]["y"]-self.pose()[1]),3)
            if m["elapsed_s"]>=samples[-1]["t"]:
                self.drive(0,0);self.stop("recorded replay finished","completed");return
            self.drive(samples[index]["v"],samples[index]["w"]);return
        command=pursuit(self.pose(),m["points"],m["index"],m["speed_mps"])
        if command["index"]!=m["index"]:self.best_metric=math.inf;self.last_progress=now
        m.update(index=command["index"],distance_m=round(command["distance"],3))
        if command["done"]:
            self.drive(0,0)
            if abs(self.link.debug.get("v",0))<.08:self.stop("goal reached","completed")
            return
        metric=command["distance"]+.15*abs(command["bearing"])
        if metric<self.best_metric-.02:self.best_metric=metric;self.last_progress=now
        self.last_v=command["v"];self.last_w=command["w"]
        m["elapsed_s"]+=dt;self.drive(self.last_v,self.last_w)
    async def run(self):
        last_publish=0
        try:
            while True:
                try:self.tick()
                except Exception as error:self.pause("navigation error: "+str(error))
                if self.clock()-last_publish>=.5:
                    last_publish=self.clock()
                    mission=None if not self.mission else {k:v for k,v in self.mission.items() if k not in ("points","samples")}
                    self.link._publish({"type":"autonomy","mission":mission,
                                        "teaching":None if not self.teach else {"state":self.teach["state"],"points":len(self.teach["points"])}})
                await asyncio.sleep(.05)
        finally:self.stop("navigation process stopped")
