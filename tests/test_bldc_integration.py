"""Host compatibility checks for the v4 hardware/telemetry boundary; no hardware IO."""
import importlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"software/radxa/console"))
firmware=importlib.import_module("console.firmware")
bridge=importlib.import_module("console.serial_link")


def test_hub_feedback_is_not_log_spam_and_bad_samples_do_not_replace_good_data():
    link=bridge.SerialLink()
    link._handle("HM,1000,-0.1,0.1,0.12,0.11,-10,10,25,26,0,0,0,1,1")
    assert link.hub["current_cmd_l"]==-.1 and link.tel["hub"]["link_ok"]==1
    assert not link.log
    link._handle("HM,1001,nan,0,0,0,0,0,0,0,0,0,0,0,0")
    assert link.hub["ms"]==1000


def test_supplied_hub_firmware_feedback_retains_faults_and_per_wheel_saturation():
    link=bridge.SerialLink()
    link._handle("HM,1200,2.7,0.8,-2.4,0.7,-110,28,26,27,2,0,1,0,1,1,3")
    assert link.hub["fault_l"]==2 and link.hub["fault_r"]==0
    assert link.hub["saturated_l"]==1 and link.hub["saturated_r"]==0 and link.hub["saturated"]==1
    assert link.hub["link_ok"]==1 and link.hub["mode_ok"]==1 and link.hub["self_righting_state"]==3
    assert link.tel["hub"]["current_cmd_l"]==2.7 and not link.log
    link._handle("HM,1300,0,0,0,0,0,0,25,25,0,0,0,1,1")
    assert "self_righting_state" not in link.hub and "saturated_l" not in link.hub


@pytest.mark.parametrize("derived",[{}, {"HUBMOTOR":1}, {"HUBSCHEMA":3}])
def test_v4_cannot_silently_import_old_hardware_tuning(derived):
    assert "manual commissioning" in firmware.hardware_mismatch({"parameter_schema":"bldc-v4"},derived)


def test_v4_accepts_its_schema_and_rejects_stepper_rollback():
    assert not firmware.hardware_mismatch({"parameter_schema":"bldc-v4"},{"HUBSCHEMA":4})
    assert "BLDC" in firmware.hardware_mismatch({}, {"HUBSCHEMA":4})
    assert not firmware.hardware_mismatch({}, {}) # historical updater compatibility
