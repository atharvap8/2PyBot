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
    fw.atomic_json(incoming/"manifest.json",dict(details,version=f"3.12.{seed}",app_offset=0x10000,partition_bytes=0x300000,built_at="test"))
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
    assert fw.load_build(root,build["build_id"],True)[0]["version"]=="3.12.1"
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
