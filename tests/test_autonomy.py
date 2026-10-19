import asyncio
import importlib
import math
from pathlib import Path
import sys
import time

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


@pytest.mark.parametrize("fault",["supervisor","takeover","pose_reset","disarm","airborne"])
def test_faults_pause_or_abort_without_remote_arm(tmp_path,fault):
    m,link,clock,current,route=setup(tmp_path);asyncio.run(m.start(route["id"]))
    if fault=="supervisor":step(m,link,clock,3.1,False)
    elif fault=="takeover":link.nav["state"]=3;step(m,link,clock)
    elif fault=="pose_reset":link.pose_epoch+=1;step(m,link,clock)
    elif fault=="disarm":link.tel["balancing"]=False;step(m,link,clock)
    else:link.debug["terrainState"]=2;step(m,link,clock)
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
