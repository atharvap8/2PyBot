"""Verify append-only firmware debug interoperability and passive capture fields."""
import importlib
import importlib.util
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def test_firmware_debug_and_host_schema_agree(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "software/radxa/console"))
    module = importlib.import_module("console.serial_link")
    link = module.SerialLink()
    # Actual printf field count catches format/parser mismatches on the MCU.
    sketch = (ROOT / "firmware/BaseLink/BaseLink.ino").read_text()
    debug_printf = sketch[sketch.index('Serial.printf("D,'):]
    format_strings = re.match(r'Serial.printf\(((?:"[^"\n]*"\s*)+),', debug_printf).group(1)
    format_text = "".join(re.findall(r'"([^"\n]*)"', format_strings))
    assert len(re.findall(r"%(?:\.\d+)?(?:ll|l)?[ufd]", format_text)) == 56
    values = [1000, 8, 150, 3.9, .4, .01, 8000, 300, 0, 1, 5100, 1, 1,
              -.5, .6, 1, .2, .55, 1, 0, .9, 26, 38, -.45, .3, .39, 1, 7700, 8300,
              1, 2, 0, 1, .2, 3, .02, 7, -.6, -.15, .25, 0, 0, 2,
              2,1.5,0,0,.25,.2,.3,1,.075,.2,2,120,-120]
    link._handle("D," + ",".join(map(str, values)))
    assert link.tel["balancing"]
    assert link.debug["wheelL"] == 7700 and link.debug["wheelR"] == 8300
    assert link.debug["fpRisk"] == .9 and link.debug["fpFlags"] == 38
    assert link.debug["driveRef"] == .3 and link.debug["balScale"] == 1
    assert link.debug["acEnabled"]==1 and link.debug["acState"]==2 and link.debug["acBlend"]==1
    assert link.debug["clZ"]==-.6 and link.debug["clRecover"]==0
    assert link.debug["acKind"]==2 and link.debug["dsActive"]==1 and link.debug["dsGoal"]==.075
    assert link.debug["drvMode"]==2 and link.debug["rpmCmdL"]==120 and link.debug["rpmCmdR"]==-120
    link._handle("D,"+",".join(map(str,values[:29])))
    assert "acEnabled" not in link.debug and link.debug["fpRisk"]==.9
    # Older firmware must remain usable; no stale override fields survive.
    link._handle("D," + ",".join(map(str, values[:20])))
    assert "fpRisk" not in link.debug and link.debug["soften"] == .55


def test_recorder_captures_override_and_explicit_wheel_targets(monkeypatch, tmp_path):
    source = ROOT / "scripts/diagnostics/record_drive.py"
    spec = importlib.util.spec_from_file_location("fall_recorder", source)
    recorder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(recorder)
    status = {"connected": True, "telemetry": {"stale": False, "ms": 20},
              "debug": {"ms": 10, "vCmd": 100, "steer": 10, "wheelL": 0, "wheelR": 0,
                        "fpRisk": .8, "fpPred": 25, "fpFlags": 4, "driveReq": -.9,
                        "driveRef": .5, "driveEff": .3, "balScale": 1}}

    class FakeReader:
        def __init__(self, url): pass
        def get(self, path):
            if path == "/api/status": return status
            if path == "/api/params": return {"params": []}
            return {"log": []}
        def close(self): pass

    ticks = iter([0, 0, 0, 1_000_000, 20_000_000, 20_000_000])
    monkeypatch.setattr(recorder, "Reader", FakeReader)
    monkeypatch.setattr(recorder.time, "monotonic_ns", lambda: next(ticks))
    result = recorder.capture(.01, "http://unused", tmp_path, "fixture")
    import csv
    with (Path(result["folder"]) / "telemetry.csv").open() as file:
        row = next(csv.DictReader(file))
    assert row["left_cmd_target_steps_s"] == "0" and row["right_cmd_target_steps_s"] == "0"
    assert row["fall_risk"] == "0.8" and row["fall_flags"] == "4"
    assert row["shaped_drive_mps"] == "0.5" and row["applied_balance_scale"] == "1"
    assert json.loads((Path(result["folder"]) / "baseline.json").read_text())["read_only"]
