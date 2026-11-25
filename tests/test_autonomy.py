import asyncio
import importlib
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"software/radxa/console"))
nav=importlib.import_module("console.autonomy")


class Clock:
    def __init__(self):self.t=100.0
    def __call__(self):return self.t


class Link:
    def __init__(self,clock):
        self.clock=clock;self.connected=True;self.pose_epoch=1;self.last_odom_monotonic=clock()
        self.tel={"x_m":0,"y_m":0,"heading":0,"stale":False,"startup_ready":True,"balancing":True}
        self.debug={"v":0,"pF":0,"w":0,"terrainState":0,"fpHazard":0}
        self.derived={"NAVIGATION":1};self.nav={"token":0,"state":0,"pad":True,"received_monotonic":clock()}
        self.sent=[];self.on_grant=None
    def send(self,line):
        self.sent.append(line)
        if line.startswith("AN,") and line.endswith(",1"):
            self.nav.update(token=int(line.split(",")[1]),state=1,received_monotonic=self.clock())
            if self.on_grant:self.on_grant()
        return True
    def _log(self,line):pass
    def _publish(self,message):pass


def depth(distance=1500):
    return {"state":"ranging","stale":False,"age_ms":30,"valid":[True]*64,"distance_mm":[distance]*64}


def setup(tmp_path):
    clock=Clock();link=Link(clock);current=[depth()]
    manager=nav.Autonomy(link,lambda:current[0],root=tmp_path,clock=clock)
    route=manager.save("test",[{"x":0,"y":0,"heading":0},{"x":.5,"y":0,"heading":0}])
    return manager,link,clock,current,route


def step(manager,link,clock,seconds=.05,heartbeat=True):
    clock.t+=seconds;link.last_odom_monotonic=clock();link.nav["received_monotonic"]=clock()
    if heartbeat and manager.lease_key:manager.last_operator=clock()
    manager.tick()


def test_transforms_and_invalid_routes():
    origin=(3,4,math.pi/2)
    point=nav.relative_pose((3,5,math.pi/2),origin)
    assert point["x"]==1 and abs(point["y"])<1e-6
    restored=nav.world_points([point],origin)[0]
    assert restored["x"]==pytest.approx(3) and restored["y"]==pytest.approx(5)
    with pytest.raises(ValueError):nav.validate_points([{"x":0,"y":0,"heading":0},{"x":float('nan'),"y":0,"heading":0}])


def test_depth_unknown_stale_partial_and_close_are_reported():
    assert nav.depth_clearance(depth())["clear"]
    unknown=depth();unknown["valid"]=[False]*64
    assert nav.depth_clearance(unknown)["reason"]=="depth_unknown"
    stale=depth();stale["age_ms"]=500
    assert not nav.depth_clearance(stale)["clear"]
    assert nav.depth_clearance(depth(200))["reason"]=="obstacle_near"
    turn=depth();turn["valid"]=[i%8 in (2,3,4,5) for i in range(64)]
    assert nav.depth_clearance(turn)["clear"] and not nav.depth_clearance(turn,True)["clear"]


def test_pursuit_reaches_square_in_an_ideal_not_hardware_simulation():
    points=[{"x":1,"y":0,"heading":0},{"x":1,"y":1,"heading":math.pi/2},
            {"x":0,"y":1,"heading":math.pi},{"x":0,"y":0,"heading":0}]
    pose=[0.,0.,0.];index=0
    for _ in range(1800):
        command=nav.pursuit(pose,points,index,.12);index=command["index"]
        assert 0<=command["v"]<=.12 and abs(command["w"])<=.45
        if command["done"]:break
        pose[0]+=.05*command["v"]*math.cos(pose[2]);pose[1]+=.05*command["v"]*math.sin(pose[2]);pose[2]=nav.wrap(pose[2]+.05*command["w"])
    assert command["done"] and index==3


def test_cruise_target_is_point_six_even_with_dense_teaching_samples():
    points=[{"x":i*.2,"y":0,"heading":0} for i in range(1,16)]
    command=nav.pursuit((0,0,0),points,0,.6)
    assert command["v"]==pytest.approx(.6)
    assert nav.depth_clearance(depth(700),speed=.6)["reason"]=="braking_clearance"
    good=nav.depth_clearance(depth(2000),speed=.6)
    assert good["clear"] and good["stopping_clearance_m"]>.95


def test_unknown_depth_is_advisory_and_does_not_block_start(tmp_path):
    m,link,clock,current,route=setup(tmp_path);current[0]["valid"]=[False]*64
    result=asyncio.run(m.start(route["id"]))
    assert result["lease_key"] and m.mission["state"]=="running"
    assert m.status()["depth"]["advisory_only"] and not m.status()["limits"]["tof_gating"]


def test_explicit_start_starts_lease_not_arming_and_depth_does_not_hold(tmp_path):
    m,link,clock,current,route=setup(tmp_path);result=asyncio.run(m.start(route["id"]))
    assert result["lease_key"] and link.sent[0].startswith("AN,") and "E" not in link.sent
    step(m,link,clock);assert m.mission["command_v"]>0
    current[0]=depth(200);step(m,link,clock)
    assert m.mission["state"]=="running" and m.mission["command_v"]>0
    current[0]=depth();step(m,link,clock)
    assert m.mission["state"]=="running"
    for _ in range(25):step(m,link,clock)
    assert m.mission["state"]=="running" and m.mission["command_v"]>0
    link.debug["terrainState"]=2;step(m,link,clock)
    assert m.mission["state"]=="running" # advisory airborne state does not revoke a native lease


@pytest.mark.parametrize("fault",["supervisor","takeover","pose_reset","disarm"])
def test_faults_pause_or_abort_without_remote_arm(tmp_path,fault):
    m,link,clock,current,route=setup(tmp_path);asyncio.run(m.start(route["id"]))
    if fault=="supervisor":step(m,link,clock,3.1,False)
    elif fault=="takeover":link.nav["state"]=3;step(m,link,clock)
    elif fault=="pose_reset":link.pose_epoch+=1;step(m,link,clock)
    elif fault=="disarm":link.tel["balancing"]=False;step(m,link,clock)
    assert m.mission["state"] in ("paused","aborted") and m.mission["command_v"]==0
    assert not any(line=="E" for line in link.sent)


def test_stop_during_grant_cancels_pending_start(tmp_path):
    m,link,clock,current,route=setup(tmp_path);link.on_grant=lambda:m.stop()
    with pytest.raises(ValueError,match="cancelled"):asyncio.run(m.start(route["id"]))
    assert m.mission is None and not any(line.startswith("AU,") for line in link.sent)


def test_teaching_records_relative_points_and_detects_reset(tmp_path):
    m,link,clock,current,route=setup(tmp_path);m.begin_teach("manual")
    link.tel["x_m"]=.25;step(m,link,clock)
    assert len(m.teach["points"])==2
    link.pose_epoch+=1;step(m,link,clock);assert m.teach["state"]=="interrupted"
    saved=m.finish_teach();assert saved["length_m"]==.25 and m.teach is None


# ---- 5.2.0: missions do not need the gamepad and ride out short controller gaps ----
def lapse(manager,link,clock,seconds,odometry=False):
    """Time passes with the supervisor present; controller feedback arrives only if asked."""
    clock.t+=seconds
    if odometry:link.last_odom_monotonic=clock();link.nav["received_monotonic"]=clock()
    if manager.lease_key:manager.last_operator=clock()
    manager.tick()


def test_a_mission_starts_and_keeps_driving_without_a_gamepad(tmp_path):
    m,link,clock,current,route=setup(tmp_path);link.nav["pad"]=False
    assert "gamepad" not in " ".join(m.prerequisites()).lower()
    asyncio.run(m.start(route["id"]));step(m,link,clock)
    assert m.mission["state"]=="running" and m.mission["command_v"]>0


def test_a_short_gap_in_controller_feedback_is_ridden_out(tmp_path):
    m,link,clock,current,route=setup(tmp_path);asyncio.run(m.start(route["id"]));step(m,link,clock)
    for _ in range(20):lapse(m,link,clock,.05)                      # 1 s with no odometry: hold still, keep the lease fed
    assert m.mission["state"]=="running" and m.mission["command_v"]==0
    assert link.sent[-1].endswith(",0.0000,0.0000") and link.sent[-1].startswith("AU,")
    step(m,link,clock)
    assert m.mission["state"]=="running" and m.mission["command_v"]>0
    assert any("gap in controller feedback" in e["text"] for e in m.events)


def test_a_long_gap_in_controller_feedback_pauses(tmp_path):
    m,link,clock,current,route=setup(tmp_path);asyncio.run(m.start(route["id"]));step(m,link,clock)
    for _ in range(50):lapse(m,link,clock,.05)
    assert m.mission["state"]=="paused" and m.mission["reason"]=="controller feedback lost"


def test_a_lease_the_controller_timed_out_is_renewed_a_few_times_only(tmp_path):
    m,link,clock,current,route=setup(tmp_path);asyncio.run(m.start(route["id"]));step(m,link,clock)
    for attempt in range(nav.REGRANT_LIMIT):
        first=m.mission["token"];link.nav["state"]=nav.NAV_TIMEOUT;step(m,link,clock)
        assert m.mission["state"]=="running" and link.sent[-1].startswith("AN,") and link.sent[-1].endswith(",1")
        step(m,link,clock);step(m,link,clock)
        assert m.mission["token"]!=first and m.mission["command_v"]>0
    link.nav["state"]=nav.NAV_TIMEOUT;step(m,link,clock)
    assert m.mission["state"]=="paused" and "lease lost" in m.mission["reason"]


def test_a_gamepad_takeover_is_never_renewed(tmp_path):
    m,link,clock,current,route=setup(tmp_path);asyncio.run(m.start(route["id"]));step(m,link,clock)
    grants=sum(line.startswith("AN,") and line.endswith(",1") for line in link.sent)
    link.nav["state"]=nav.NAV_TAKEOVER;step(m,link,clock)
    assert m.mission["state"]=="paused" and m.mission["reason"]=="gamepad takeover"
    assert sum(line.startswith("AN,") and line.endswith(",1") for line in link.sent)==grants


def test_a_refused_renewal_pauses(tmp_path):
    m,link,clock,current,route=setup(tmp_path);asyncio.run(m.start(route["id"]));step(m,link,clock)
    link.nav["state"]=nav.NAV_DISABLED
    link.on_grant=lambda:link.nav.update(state=nav.NAV_REFUSED)
    step(m,link,clock);step(m,link,clock)
    assert m.mission["state"]=="paused" and "would not renew" in m.mission["reason"]


# ---- 5.3.0: lidar missions survive a lost page and hand back after a gamepad takeover ----
class Bubble:
    def status(self,now=None):
        return {"stale":False,"forward_cap":.5,"reverse_cap":.3,"turn_left_ok":True,"turn_right_ok":True,
                "front_m":2.0,"free_by_heading":[3.0]*24,"state":"clear"}


class Slam:
    def __init__(self):
        import threading
        self.mode="localizing";self.lock=threading.Lock();self.pose=[0.0,0.0,0.0];self.match={"ok":True,"lost":0}
        self.bubble=Bubble();self.plan=[];self.goal=None
    def robot_live(self):return True


def roaming(tmp_path):
    m,link,clock,current,route=setup(tmp_path);m.slam=Slam();link.debug['v']=.2   # wheels turning: not stuck
    asyncio.run(m.start_task("roam"));step(m,link,clock);step(m,link,clock)
    assert m.mission["state"]=="running" and m.mission["command_v"]>0
    return m,link,clock


def grants(link):return sum(line.startswith("AN,") and line.endswith(",1") for line in link.sent)


def test_losing_the_supervising_page_does_not_stop_a_lidar_mission(tmp_path):
    m,link,clock=roaming(tmp_path)
    step(m,link,clock,3.5,heartbeat=False)                      # Wi-Fi out of range: no heartbeat
    for _ in range(40):step(m,link,clock,heartbeat=False)
    assert m.mission["state"]=="running" and m.mission["command_v"]>0 and m.mission["unsupervised"]
    assert any("carries on" in e["text"] for e in m.events)
    step(m,link,clock)                                          # the page is back
    assert m.mission["unsupervised"] is False


def test_a_recorded_replay_still_needs_its_supervisor(tmp_path):
    m,link,clock,current,route=setup(tmp_path);asyncio.run(m.start(route["id"]));step(m,link,clock)
    step(m,link,clock,3.5,heartbeat=False)
    assert m.mission["state"]=="paused" and "supervisor" in m.mission["reason"]


def test_after_a_takeover_the_autopilot_waits_for_the_pad_to_rest_then_drives_again(tmp_path):
    m,link,clock=roaming(tmp_path);before=grants(link);first=m.mission["token"]
    link.nav.update(state=nav.NAV_TAKEOVER,idle_ms=0);step(m,link,clock)
    assert m.mission["state"]=="running" and m.mission["yielded"] and m.mission["info"]["waiting"]=="gamepad"
    assert m.mission["command_v"]==0
    sent=len(link.sent)
    for _ in range(60):                                         # three seconds of the operator driving
        link.nav["idle_ms"]=40;step(m,link,clock)
    assert grants(link)==before and m.mission["state"]=="running", "it must not grab the drive while the sticks move"
    assert not any(line.startswith("AU,") for line in link.sent[sent:]), "and must not send drive commands either"
    link.nav["idle_ms"]=900;step(m,link,clock)
    assert grants(link)==before, "0.9 s of rest is not enough"
    link.nav["idle_ms"]=1600;step(m,link,clock)
    assert grants(link)==before+1
    step(m,link,clock);step(m,link,clock)
    assert m.mission["token"]!=first and m.mission["yielded"] is False and m.mission["command_v"]>0
    assert any("driving again" in e["text"] for e in m.events)


def test_handback_is_asked_again_when_the_controller_says_not_yet(tmp_path):
    m,link,clock=roaming(tmp_path)
    link.nav.update(state=nav.NAV_TAKEOVER,idle_ms=2000)
    held=m.mission["token"]
    link.on_grant=lambda:link.nav.update(token=held,state=nav.NAV_TAKEOVER)   # a stick is off-centre: refused
    for _ in range(50):step(m,link,clock)
    assert m.mission["state"]=="running" and m.mission["yielded"], "a refused hand-back is not a reason to pause"
    tries=grants(link)
    assert tries>=3
    link.on_grant=None;clock.t+=1.1;step(m,link,clock);step(m,link,clock);step(m,link,clock)
    assert m.mission["yielded"] is False and m.mission["command_v"]>0


def test_handback_works_on_firmware_that_does_not_report_pad_idle_time(tmp_path):
    m,link,clock=roaming(tmp_path);before=grants(link)
    link.nav.update(state=nav.NAV_TAKEOVER);link.nav.pop("idle_ms",None)
    for _ in range(20):step(m,link,clock)                       # 1 s
    assert grants(link)==before
    for _ in range(14):step(m,link,clock)                       # past 1.5 s since the takeover
    assert grants(link)==before+1 and m.mission["yielded"] is False


def test_select_during_a_takeover_still_ends_the_mission(tmp_path):
    m,link,clock=roaming(tmp_path)
    link.nav.update(state=nav.NAV_TAKEOVER,idle_ms=0);step(m,link,clock)
    link.tel["balancing"]=False;step(m,link,clock)
    assert m.mission["state"]=="aborted"
