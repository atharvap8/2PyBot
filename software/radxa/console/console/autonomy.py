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
# A mission rides out short gaps in controller feedback instead of pausing. The
# controller is the safety net here, not the host: it drops the drive lease and
# stops by itself 0.5 s after the last command it received.
FEEDBACK_GRACE=2.0      # s without fresh odometry / lease feedback before a mission pauses
REGRANT_LIMIT=3         # times a mission may renew a lease the controller timed out
RESUME_IDLE=1.5         # s the gamepad must rest after a takeover before the autopilot drives again
NAV_TAKEOVER,NAV_TIMEOUT,NAV_DISABLED,NAV_REFUSED=3,4,6,9


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
        self.gap_since=None     # start of the current gap in controller feedback
        self.regrant=None       # lease renewal in flight: {"owner","until","why"}
        self.yielded=None       # when the operator took over with the gamepad (mission waits, then resumes)
        self.last_task=None     # arguments of the last lidar mission started, for the gamepad's GO key
        self.boost=0.0          # the gamepad's RT, 0 to 1, eased: how far above its chosen speed the mission drives
        self.next_grant=0.0
        self.slam=None          # SlamEngine (set by the server): map pose + lidar safety bubble
        self.planner=None       # RoamPlanner / GotoPlanner / ExplorePlanner for lidar missions
    # ---------------------------------------------------------------- lidar missions
    def slam_pose(self):
        """(pose, localized) from SLAM, or (None, False) when it is not running."""
        engine=self.slam
        if engine is None or engine.mode not in ("mapping","localizing"):return None,False
        with engine.lock:
            pose=tuple(float(v) for v in engine.pose)
            match=dict(engine.match)
        return pose,(match.get("ok") or match.get("lost",99)<6) and engine.robot_live()
    def nav_pose(self,kind):
        if kind in ("goto","explore"):
            pose,localized=self.slam_pose()
            if pose is None:raise ValueError("Load a map (or start mapping) first")
            return pose,localized
        pose,_=self.slam_pose()
        return (pose if pose is not None else self.pose()),True
    def bubble(self):
        if self.slam is None:return {"stale":True,"forward_cap":0.0,"reverse_cap":0.0}
        return self.slam.bubble.status()
    def mission_prerequisites(self,kind):
        reasons=self.prerequisites()
        if self.slam is None:reasons.append("Lidar mapping is not available")
        elif self.bubble().get("stale",True):reasons.append("Lidar safety bubble has no fresh scans")
        if kind in ("goto","explore") and self.slam is not None:
            if kind=="explore" and self.slam.mode!="mapping":reasons.append("Start mapping (or extend a map) before exploring")
            if kind=="goto" and self.slam.mode not in ("mapping","localizing"):reasons.append("Load a map first")
            elif self.slam_pose()[0] is not None and not self.slam_pose()[1]:reasons.append("Not localized on the map yet (press Find me)")
        return reasons
    def resolve_goal(self,goal):
        if not isinstance(goal,dict):raise ValueError("Goal required")
        if goal.get("label"):
            found=self.slam.label(goal["label"]) if self.slam else None
            if not found:raise ValueError("Unknown area label")
            return found
        try:x=float(goal["x"]);y=float(goal["y"])
        except (KeyError,TypeError,ValueError):raise ValueError("Goal needs x and y (metres) or a label")
        if not (math.isfinite(x) and math.isfinite(y)):raise ValueError("Goal must be finite")
        return {"name":str(goal.get("name") or "point")[:32],"x":x,"y":y}
    async def pad_key(self,key):
        """A / B / X / Y on the gamepad (5.4.0 firmware reports them as PK,<key>).

        GO resumes a paused mission, or repeats the last one. PAUSE holds it, END ends it,
        ROAM starts roaming. Starting from the gamepad needs the same conditions as from the
        Maps page; the gamepad is the operator's own hand on the robot, so nothing else has
        to be watching."""
        m=self.mission;state=m["state"] if m else None
        try:
            if key=="PAUSE":
                if state in ENGAGED:self.pause("operator_pause (gamepad)")
            elif key=="END":
                if state in ("running","paused"):self.stop("operator_stop (gamepad)")
            elif key=="GO":
                if state=="paused":await self.resume()
                elif state in ENGAGED:return
                elif self.last_task:await self.start_task(**self.last_task)
                else:self.event("Gamepad GO: choose where to go in Maps once; after that A repeats it");return
            elif key=="ROAM":
                if state in ENGAGED or state=="paused":return
                await self.start_task("roam")
            else:return
            if self.mission and self.mission["state"] in ENGAGED:self.mission["by_gamepad"]=True   # no page is expected to watch
        except ValueError as error:self.event("Gamepad "+key+" refused: "+str(error))
    async def start_task(self,kind,speed=None,duration_s=None,goal=None,goals=None):
        from .missions import ExplorePlanner,GotoPlanner,RoamPlanner,RoutePlanner
        async with self.lock:
            if self.mission and self.mission["state"] in ENGAGED:raise ValueError("A mission is already engaged")
            if self.teach:raise ValueError("Save or cancel route teaching first")
            if kind not in ("roam","goto","explore"):raise ValueError("Unknown mission")
            reasons=self.mission_prerequisites(kind)
            if reasons:raise ValueError("; ".join(reasons))
            for value in (speed,duration_s):
                if value is not None and (not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0):raise ValueError("Speed/duration must be positive")
            if kind=="roam":planner=RoamPlanner(speed or .25,duration_s or 600);name="Roam"
            elif kind=="explore":planner=ExplorePlanner(self.slam,speed or .25,duration_s or 1200);name="Explore and map"
            else:
                # one place, or several in the order given
                wanted=list(goals) if isinstance(goals,(list,tuple)) and goals else [goal]
                if len(wanted)>12:raise ValueError("A route can have at most 12 stops")
                targets=[self.resolve_goal(g) for g in wanted]
                if len(targets)==1:planner=GotoPlanner(self.slam,targets[0],speed or .30);name="Go to "+targets[0]["name"]
                else:
                    planner=RoutePlanner(self.slam,targets,speed or .30)
                    name="Go to "+", ".join(t["name"] for t in targets)
                    if len(name)>58:name=f"Go to {targets[0]['name']} and {len(targets)-1} more"
            generation=self.action_epoch;owner=await self.grant()
            if generation!=self.action_epoch:
                self.tx(f"AN,{owner},0");raise ValueError("Start was cancelled")
            self.planner=planner
            self.mission={"id":uuid.uuid4().hex,"kind":kind,"route_id":None,"name":name,"state":"running","reason":"",
                "origin":None,"points":[],"index":0,"speed_mps":planner.speed,"elapsed_s":0,"samples":[],"sample_index":0,
                "replay_mode":kind,"distance_m":0,"command_v":0,"command_w":0,"epoch":self.link.pose_epoch,"token":owner,"info":{}}
            self.lease_key=secrets.token_hex(16);self.last_operator=self.clock();self.last_progress=self.clock()
            self.gap_since=self.regrant=self.yielded=None
            self.last_task={"kind":kind,"speed":speed,"duration_s":duration_s,"goal":goal,"goals":goals}
            self.last_v=self.last_w=0;self.event("Mission started: "+name)
            return {"status":self.status(),"lease_key":self.lease_key,"mission_id":self.mission["id"]}
    def clear_plan(self):
        if self.slam is not None:
            with self.slam.lock:self.slam.plan=[];self.slam.goal=None
    def planner_tick(self,m,now,dt):
        kind=m["kind"]
        pose,localized=self.nav_pose(kind)
        bubble=self.bubble()
        if bubble.get("stale",True):
            self.drive(0,0);m["info"]=dict(m.get("info",{}),waiting="lidar bubble stale")
            if now-self.last_progress>2.0:self.pause("lidar safety bubble stale")
            return
        self.last_progress=now
        # Low obstacles under the 26.5 cm scan plane: the ToF corridor guard's
        # floor-aware cap, only when the operator has enabled the ToF guard.
        try:depth_cap=self.depth().get("forward_limit_mps")
        except Exception:depth_cap=None
        depth_cap=9.0 if not isinstance(depth_cap,(int,float)) else float(depth_cap)
        measured=float(self.link.debug.get("v",self.link.tel.get("v_ms",0)) or 0)
        # RT on the gamepad is a speed governor, not a takeover: the autopilot keeps the drive
        # and RT says how far above the chosen speed it may go. Eased over about half a second.
        nav=self.link.nav
        pull=nav.get("boost",0)/100 if nav.get("pad") else 0.0
        self.boost+=clip(pull-self.boost,-2*dt,2*dt)
        from .missions import TOP_SPEED,boosted
        top=min(TOP_SPEED,float(self.link.derived.get("VDRIVE") or TOP_SPEED))
        self.planner.speed=boosted(self.planner.base_speed,self.boost,top)
        m["speed_mps"]=round(self.planner.speed,3);m["boost"]=round(self.boost,2)
        args=(now,dt,pose,bubble,depth_cap,measured)
        step=self.planner.step(*args) if kind=="roam" else self.planner.step(*args,localized=localized)
        m["elapsed_s"]=round(m["elapsed_s"]+dt,2);m["info"]=step.info
        if step.done:
            self.drive(0,0);self.stop(step.reason,"completed" if step.reason in ("goal reached","roam time finished","explore time finished") or step.reason.startswith("exploration complete") else "aborted")
            return
        v=step.v;w=step.w
        # host-side bubble limits (firmware repeats the speed caps via BG)
        v=min(v,bubble["forward_cap"],depth_cap) if v>0 else max(v,-bubble["reverse_cap"])
        if w>0 and not bubble.get("turn_left_ok"):w=0.0
        if w<0 and not bubble.get("turn_right_ok"):w=0.0
        self.last_v=v;self.last_w=w;self.drive(v,w)
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
                if nav.get("state")==9:raise ValueError("Controller refused the drive lease: centre the sticks and wait for stable balance (firmware before 5.2.0 also needs the gamepad connected)")
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
            self.gap_since=self.regrant=self.yielded=None
            self.last_v=self.last_w=0;self.event("Replay started: "+route["name"])
            return {"status":self.status(),"lease_key":self.lease_key,"mission_id":self.mission["id"]}
    def heartbeat(self,identifier,key):
        if not self.mission or self.mission["id"]!=identifier or not self.lease_key or not secrets.compare_digest(key,self.lease_key):
            raise ValueError("Supervision lease is not current; resume explicitly")
        self.last_operator=self.clock()
    def pause(self,reason="operator_pause"):
        self.action_epoch+=1;self.gap_since=self.regrant=self.yielded=None
        if self.mission and self.mission["state"] in ENGAGED:
            self.tx(f"AN,{self.mission['token']},0");self.mission.update(state="paused",reason=reason,command_v=0,command_w=0)
            self.last_v=self.last_w=0;self.lease_key=None;self.event(self.mission["name"]+" paused: "+reason)
    def stop(self,reason="operator_stop",state="stopped"):
        self.action_epoch+=1;self.gap_since=self.regrant=self.yielded=None
        if self.mission:
            if self.mission["state"] in ENGAGED or self.mission["state"]=="paused":
                self.tx(f"AN,{self.mission['token']},0")
            was=self.mission["state"]
            self.mission.update(state=state,reason=reason,command_v=0,command_w=0)
            self.last_v=self.last_w=0;self.lease_key=None
            if was in ENGAGED or was=="paused":self.event(self.mission["name"]+" "+state+": "+reason)
            if self.mission.get("kind"):self.planner=None;self.clear_plan()
    async def resume(self):
        async with self.lock:
            if not self.mission or self.mission["state"]!="paused":raise ValueError("No paused route to resume")
            kind=self.mission.get("kind")
            if kind not in ("goto","explore") and self.mission["epoch"]!=self.link.pose_epoch:raise ValueError("Odometry frame changed; restart at the marked route start")
            if kind and self.planner is None:raise ValueError("Mission state was cleared; start it again")
            reasons=self.mission_prerequisites(kind) if kind else self.prerequisites()
            if reasons:raise ValueError("; ".join(reasons))
            generation=self.action_epoch;owner=await self.grant()
            if generation!=self.action_epoch:
                self.tx(f"AN,{owner},0");raise ValueError("Resume was cancelled")
            self.mission["token"]=owner;self.mission.update(state="running",reason="",regrants=0)
            self.gap_since=self.regrant=self.yielded=None
            self.lease_key=secrets.token_hex(16);self.last_operator=self.clock();self.last_progress=self.clock();self.best_metric=math.inf
            self.event("Replay explicitly resumed")
            return {"status":self.status(),"lease_key":self.lease_key,"mission_id":self.mission["id"]}
    def drive(self,v,w):
        if not self.mission:return
        self.sequence=(self.sequence+1)&0xffffffff
        if not self.tx(f"AU,{self.mission['token']},{self.sequence},{v:.4f},{w:.4f}"):
            self.pause("serial motion command failed");return
        self.mission.update(command_v=round(v,4),command_w=round(w,4))
    def hold(self,m,why):
        """Stand still but keep the drive lease fed while something sorts itself out."""
        self.sequence=(self.sequence+1)&0xffffffff
        self.tx(f"AU,{m['token']},{self.sequence},0.0000,0.0000")
        self.last_v=self.last_w=0;m.update(command_v=0,command_w=0)
        m["info"]=dict(m.get("info") or {},waiting=why)
    def lease_ok(self,m,now):
        """True when the controller still honours this mission's drive lease. A lease the
        controller dropped for a late command is renewed; one you took with the gamepad,
        or one it refuses to renew, pauses the mission."""
        nav=self.link.nav;token=nav.get("token");state=nav.get("state")
        if self.regrant:
            pending=self.regrant;handback=pending.get("handback")
            if token==pending["owner"] and state in (1,2):
                m["token"]=pending["owner"];self.sequence=0;self.regrant=None
                if handback:self.yielded=None;m["yielded"]=False;self.last_progress=now;self.event("Gamepad at rest: the autopilot is driving again")
                else:self.event("Drive lease renewed after "+pending["why"])
                return True
            if (token==pending["owner"] and state==NAV_REFUSED) or now>pending["until"]:
                self.tx(f"AN,{pending['owner']},0");self.regrant=None
                if not handback:self.pause("native lease lost: the controller would not renew it after "+pending["why"])
                return False                       # after a takeover: sticks not at rest yet, ask again shortly
            m["info"]=dict(m.get("info") or {},waiting="gamepad" if handback else "controller link");return False
        if self.yielded is None and token==m["token"] and state in (1,2):return True
        if self.yielded is not None or (token==m["token"] and state==NAV_TAKEOVER):
            # The operator took over with the gamepad. A lidar mission waits and takes the drive
            # back once the pad has rested; a recorded replay cannot pick up mid-way, so it pauses.
            if not m.get("kind"):self.pause("gamepad takeover");return False
            if self.yielded is None:
                self.yielded=now;self.next_grant=0.0;self.last_v=self.last_w=0
                m.update(yielded=True,command_v=0,command_w=0)
                self.event("Gamepad took over: the autopilot resumes when the sticks rest")
            idle=nav.get("idle_ms")              # 5.3.0 firmware reports it; older firmware: time since the takeover
            rested=idle>=RESUME_IDLE*1000 if isinstance(idle,int) else now-self.yielded>=RESUME_IDLE
            if rested and now>=self.next_grant:
                owner=secrets.randbelow(0x7ffffffe)+1;self.next_grant=now+1.0
                if self.tx(f"AN,{owner},1"):self.regrant={"owner":owner,"until":now+.8,"why":"the gamepad","handback":True}
            m["info"]=dict(m.get("info") or {},waiting="gamepad");return False
        if token==m["token"] and state in (NAV_TIMEOUT,NAV_DISABLED) and m.get("regrants",0)<REGRANT_LIMIT:
            owner=secrets.randbelow(0x7ffffffe)+1
            if self.tx(f"AN,{owner},1"):
                m["regrants"]=m.get("regrants",0)+1
                self.regrant={"owner":owner,"until":now+1.2,"why":"a late command" if state==NAV_TIMEOUT else "a controller hold"}
                m["info"]=dict(m.get("info") or {},waiting="controller link");return False
        self.pause("native lease lost");return False
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
        slam_frame=m.get("kind") in ("goto","explore")   # map pose: independent of odometry resets
        if self.firmware_busy() or (not slam_frame and self.link.pose_epoch!=m["epoch"]):self.stop("odometry/maintenance change","aborted");return
        nav=self.link.nav
        fresh=(self.link.connected and not self.link.tel.get("stale",True) and now-self.link.last_odom_monotonic<=.30
               and now-nav.get("received_monotonic",0)<=.35)
        if not fresh:
            if self.gap_since is None:self.gap_since=now
            if now-self.gap_since>FEEDBACK_GRACE:self.pause("controller feedback lost");return
            self.hold(m,"controller link");return
        if self.gap_since is not None:
            if now-self.gap_since>=.25:self.event(f"Rode out a {now-self.gap_since:.2f} s gap in controller feedback")
            self.gap_since=None
        if not self.link.tel.get("balancing"):self.stop("controller disarmed/fell","aborted");return
        if not self.lease_ok(m,now):return
        if now-self.last_operator>3:
            # A recorded replay drives blind, so it still needs someone watching. A lidar mission
            # plans and avoids by itself: losing the page (Wi-Fi out of range) must not strand it.
            if not m.get("kind"):self.pause("supervisor disconnected/backgrounded");return
            if not m.get("unsupervised"):
                m["unsupervised"]=True
                if not m.get("by_gamepad"):self.event("Lost the supervising page (Wi-Fi?): the mission carries on")
        elif m.get("unsupervised"):
            m["unsupervised"]=False
            if not m.get("by_gamepad"):self.event("Supervising page is back")
        if m.get("kind") in ("roam","goto","explore"):
            self.planner_tick(m,now,dt);return
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
        last_publish=0;last_loop=self.clock()
        try:
            while True:
                # Everything here shares one event loop with the serial reader. If something
                # held it up, say so: that is what a "controller feedback" gap usually is.
                late=self.clock()-last_loop-.05;last_loop=self.clock()
                if late>.25 and self.mission and self.mission["state"] in ENGAGED:self.event(f"Console was busy for {late:.2f} s")
                try:self.tick()
                except Exception as error:self.pause("navigation error: "+str(error))
                if self.clock()-last_publish>=.5:
                    last_publish=self.clock()
                    mission=None if not self.mission else {k:v for k,v in self.mission.items() if k not in ("points","samples")}
                    self.link._publish({"type":"autonomy","mission":mission,
                                        "teaching":None if not self.teach else {"state":self.teach["state"],"points":len(self.teach["points"])}})
                await asyncio.sleep(.05)
        finally:self.stop("navigation process stopped")
