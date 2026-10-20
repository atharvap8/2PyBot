"""Real image/registry validation and update orchestration; hardware calls are stubbed."""
import asyncio
import hashlib
import importlib
from pathlib import Path
import struct
import sys
import time
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"software/radxa/console"))
fw=importlib.import_module("console.firmware")
worker=importlib.import_module("console.firmware_worker")


def image(seed=1):
    header=bytearray(24);header[0]=0xe9;header[1]=1;header[23]=1
    segment=struct.pack("<I",0xabcd5432)+bytes([seed])*252
    data=bytes(header)+struct.pack("<II",0x3f400020,len(segment))+segment
    checksum=0xef
    for byte in segment:checksum^=byte
    data+=bytes((len(data)|15)-len(data))+bytes([checksum])
    return data+hashlib.sha256(data).digest()


def register(root,tmp,seed=1,candidate=False):
    incoming=tmp/("incoming"+str(seed));incoming.mkdir()
    payload=image(seed);(incoming/"application.bin").write_bytes(payload)
    details=fw.inspect_image(payload)
    fw.atomic_json(incoming/"manifest.json",dict(details,version=f"3.{10+10*seed}.0",app_offset=0x10000,partition_bytes=0x300000,built_at="test"))
    return fw.register_build(root,incoming,candidate)


class Link:
    def __init__(self):
        self.connected=True;self.dump_complete=True;self.maintenance=False;self._serial_paused=False;self.began=False
        self.tel={"stale":False,"startup_ready":True,"balancing":False}
        self.derived={"FLASHLOCK":1}
    async def begin_firmware_maintenance(self):self.began=True;self.hold_for_firmware()
    def hold_for_firmware(self):self.maintenance=True;self._serial_paused=True;self.connected=False
    def resume_after_firmware(self):self.maintenance=False;self._serial_paused=False
    def params_list(self):return [{"key":"CURRENT","value":1500.0}]


def manager(root):
    heartbeat=root/"heartbeat.json";fw.atomic_json(heartbeat,{"t":time.time()})
    link=Link()
    snapshot=lambda:{"params":link.params_list(),"debug":{},"camera_profile":"rpi-imx708","calibration":{}}
    return fw.FirmwareManager(link,snapshot,root,heartbeat),link


def test_image_integrity_and_wrong_chip_or_bootloader():
    valid=image();assert fw.inspect_image(valid)["size_bytes"]==len(valid)
    corrupt=bytearray(valid);corrupt[100]^=1
    with pytest.raises(ValueError):fw.inspect_image(corrupt)
    with pytest.raises(ValueError):fw.inspect_image(valid[:-25])
    other=bytearray(valid);other[12]=5
    with pytest.raises(ValueError):fw.inspect_image(other)
    boot=bytearray(valid);boot[32:36]=bytes(4)
    with pytest.raises(ValueError):fw.inspect_image(boot)


def test_registry_pins_hash_and_rejects_changed_image(tmp_path):
    root=tmp_path/"state";build=register(root,tmp_path)
    assert fw.load_build(root,build["build_id"],True)[0]["version"]=="3.20.0"
    (root/"builds"/build["build_id"]/"application.bin").write_bytes(image(9))
    with pytest.raises(ValueError):fw.load_build(root,build["build_id"],True)
    with pytest.raises(ValueError):fw.load_build(root,"../outside")


@pytest.mark.parametrize("blocked",["balancing","stale","not_ready","no_flashlock"])
def test_preflight_rejects_without_locking_or_flashing(tmp_path,blocked):
    build=register(tmp_path/"state",tmp_path);m,link=manager(tmp_path/"state")
    if blocked=="balancing":link.tel["balancing"]=True
    elif blocked=="stale":link.tel["stale"]=True
    elif blocked=="not_ready":link.tel["startup_ready"]=False
    else:link.derived.clear()
    with pytest.raises(ValueError):asyncio.run(m.start(build["build_id"]))
    assert not link.began and not (m.root/"current.json").exists()


def test_queue_is_durable_and_rejects_stale_or_duplicate_request(tmp_path):
    root=tmp_path/"state";first=register(root,tmp_path,1);latest=register(root,tmp_path,2)
    m,link=manager(root)
    with pytest.raises(ValueError):asyncio.run(m.start(first["build_id"]))
    assert not link.began
    result=asyncio.run(m.start(latest["build_id"]))
    assert result["busy"] and result["job"]["status"]=="queued" and link._serial_paused
    with pytest.raises(ValueError):asyncio.run(m.start(latest["build_id"]))
    restarted,other=manager(root)
    assert other._serial_paused and restarted.blocked()
    assert len(list((root/"jobs").iterdir()))==1


def test_host_readiness_is_required_before_controller_lock(tmp_path):
    build=register(tmp_path/"state",tmp_path);m,link=manager(tmp_path/"state")
    m.host_ready=lambda:False
    assert not m.status()["can_flash"]
    with pytest.raises(ValueError,match="camera stream"):asyncio.run(m.start(build["build_id"]))
    assert not link.began


def test_manager_created_before_api_loop_serializes_competing_maintenance_requests(tmp_path):
    m,link=manager(tmp_path/'state')
    link.params_list=lambda:[{'key':'CURRENT','value':1500.0},{'key':'BOOTWAIT','value':1}]
    async def scenario():
        began=asyncio.Event();finish=asyncio.Event()
        async def hold():
            began.set();await finish.wait();link.hold_for_firmware()
        link.begin_firmware_maintenance=hold
        first=asyncio.create_task(m.acquire_deployment('a'*32,'b'*40))
        await began.wait()
        second=asyncio.create_task(m.acquire_deployment('c'*32,'b'*40))
        await asyncio.sleep(0)
        finish.set()
        assert (await first)['token']=='a'*32
        with pytest.raises(ValueError,match='Another deployment'):await second
        assert m.deployment_hold()['token']=='a'*32 and link._serial_paused
    asyncio.run(scenario())


class FakeController:
    def __init__(self,log):self.port=SimpleNamespace(rts=False)
    def initialize_locked(self):pass
    def restore(self,snapshot):return len(snapshot["params"])
    def close(self):pass


def prefix():
    data=bytearray([255])*65536
    data[0x8000:0x8020]=struct.pack("<HBBII16sI",0x50aa,0,0,0x10000,0x300000,b"app\0",0)
    return bytes(data)


@pytest.mark.parametrize("failure",[None,"new_write","all_writes"])
def test_worker_verifies_preserves_and_rolls_back(tmp_path,monkeypatch,failure):
    root=tmp_path/"state";register(root,tmp_path,1,True);new=register(root,tmp_path,2)
    m,link=manager(root);asyncio.run(m.start(new["build_id"]))
    w=worker.Worker(root,FakeController);calls=[]
    monkeypatch.setattr(worker.time,"sleep",lambda _:None)
    monkeypatch.setattr(worker,"api",lambda path:{"connected":False,"telemetry":{"balancing":False}} if path=="/api/status" else {"ready":True} if path=="/api/ready" else {"colour_profile":"rpi-imx708"})
    def run(args,logname,writing=False):
        calls.append(args)
        if args[0]=="read_flash":Path(args[-1]).write_bytes(prefix())
        if args[0]=="write_flash" and (failure=="all_writes" or failure=="new_write" and args[-1].endswith("selected.bin")):
            raise RuntimeError("injected serial error")
    monkeypatch.setattr(w,"command",run)
    w.process(m.job());result=fw.read_json(root/"current.json")
    writes=[call for call in calls if call[0]=="write_flash"]
    assert all(call[1]=="0x10000" for call in writes)
    if failure is None:
        assert result["status"]=="succeeded" and result["params_preserved"]==1
        assert result["boot_nvs_unchanged"] and result["image_verified"]
        assert fw.read_json(root/"installed.json")["build_id"]==new["build_id"]
    elif failure=="new_write":
        assert result["status"]=="failed" and result["rolled_back"]
        assert len(writes)==2 and writes[-1][-1].endswith("rollback.bin")
    else:
        assert result["status"]=="recovery_required"
        assert m.blocked() # no silent release of a partially programmed controller


def test_partition_layout_is_application_only():
    assert worker.app_partition(prefix())==(0x10000,0x300000)
    bad=bytearray(prefix());struct.pack_into("<I",bad,0x8004,0x20000)
    with pytest.raises(ValueError):worker.app_partition(bad)


def test_incremental_dump_finishes_before_retry_can_queue_another_dump(tmp_path,monkeypatch):
    clock=[0.0]
    class Port:
        def __init__(self):self.dtr=False;self.rts=False;self.queued=0;self.sent=[];self.next_line=0;self.index=0;self.active=False
        def open(self):pass
        def close(self):pass
        def flush(self):pass
        def write(self,data):
            self.sent.append(data)
            if data==b'P?\n':
                if self.active:self.queued+=1
                else:self.active=True;self.index=0;self.next_line=clock[0]+.005
        def read(self,size):
            clock[0]+=.1;out=[]
            while self.active and self.next_line<=clock[0]:
                if self.index==0:line='P#,120\n'
                elif self.index<=120:line=f'P,K{self.index},0,-1,1,MOTOR,test\n'
                elif self.index<136:line='PR,HUBSCHEMA,4\n'
                else:line='P.\n'
                out.append(line);self.index+=1;self.next_line+=.005
                if self.index>136:
                    if self.queued:self.queued-=1;self.index=0
                    else:self.active=False
            return ''.join(out).encode()
    port=Port();monkeypatch.setattr(worker.serial,'Serial',lambda:port)
    monkeypatch.setattr(worker.time,'monotonic',lambda:clock[0])
    controller=worker.Controller(tmp_path/'dump.log');controller.dump()
    assert controller.complete and len(controller.params)==120
    assert port.sent.count(b'P?\n')==1 and clock[0]<1.0


def test_deployment_maintenance_survives_host_restart_and_requires_its_token(tmp_path):
    root=tmp_path/'state';first,link=manager(root)
    link.params_list=lambda:[{'key':'CURRENT','value':1500.0},{'key':'BOOTWAIT','value':1}]
    token='a'*32
    asyncio.run(first.acquire_deployment(token,'b'*40))
    assert link._serial_paused and first.deployment_hold()['locked']
    second,new_link=manager(root)
    assert new_link._serial_paused and second.blocked()
    with pytest.raises(ValueError,match='token'):
        asyncio.run(second.release_deployment('c'*32))
    asyncio.run(second.release_deployment(token))
    assert not new_link._serial_paused and not second.deployment_hold()


def test_failed_deployment_lock_does_not_leave_a_phantom_hold(tmp_path):
    m,link=manager(tmp_path/'state')
    link.params_list=lambda:[{'key':'CURRENT','value':1500.0},{'key':'BOOTWAIT','value':1}]
    async def refuse():raise ValueError('local START is pending')
    link.begin_firmware_maintenance=refuse
    with pytest.raises(ValueError,match='pending'):asyncio.run(m.acquire_deployment('a'*32,'b'*40))
    assert not m.deployment_hold() and not link._serial_paused


def test_held_update_queues_once_and_cannot_unlock_an_active_flash(tmp_path):
    root=tmp_path/'state';build=register(root,tmp_path)
    m,link=manager(root);token='a'*32
    link.params_list=lambda:[{'key':'CURRENT','value':1500.0},{'key':'BOOTWAIT','value':1}]
    asyncio.run(m.acquire_deployment(token,'b'*40))
    job=asyncio.run(m.queue_deployment_build(token,build['build_id']))
    assert job['status']=='queued' and job['deployment_token']==token
    with pytest.raises(ValueError,match='active'):asyncio.run(m.queue_deployment_build(token,build['build_id']))
    with pytest.raises(ValueError,match='finish'):asyncio.run(m.release_deployment(token))
    fw.atomic_json(root/'current.json',dict(job,status='failed',rolled_back=True))
    asyncio.run(m.release_deployment(token));assert not link._serial_paused


def test_deployment_rejects_standalone_startup_before_acquiring_any_lock(tmp_path):
    m,link=manager(tmp_path/'state')
    with pytest.raises(ValueError,match='BOOTWAIT'):asyncio.run(m.acquire_deployment('a'*32,'b'*40))
    assert not link.began and not m.deployment_hold()


def test_unconfirmed_interrupted_deployment_requires_recovery_before_release(tmp_path):
    m,link=manager(tmp_path/'state');token='a'*32
    fw.atomic_json(m.deployment_hold_path,{'token':token,'commit':'b'*40,'locked':False})
    link.resume_after_firmware=lambda:pytest.fail('An unconfirmed hold must not hand back serial ownership')
    with pytest.raises(ValueError,match='recovery'):asyncio.run(m.release_deployment(token))
    assert m.deployment_hold()
