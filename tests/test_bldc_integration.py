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


@pytest.mark.parametrize("derived",[{}, {"HUBMOTOR":1}, {"HUBSCHEMA":3}])
def test_v4_cannot_silently_import_old_hardware_tuning(derived):
    assert "manual commissioning" in firmware.hardware_mismatch({"parameter_schema":"bldc-v4"},derived)


def test_v4_accepts_its_schema_and_rejects_stepper_rollback():
    assert not firmware.hardware_mismatch({"parameter_schema":"bldc-v4"},{"HUBSCHEMA":4})
    assert "BLDC" in firmware.hardware_mismatch({}, {"HUBSCHEMA":4})
    assert not firmware.hardware_mismatch({}, {}) # historical updater compatibility
