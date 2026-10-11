import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def test_every_readback_parameter_matches_compiled_checkpoint():
    snapshot = json.loads((ROOT / "hardware/dynamics/nvs_rev3_1_0.json").read_text())
    header = (ROOT / "firmware/BaseLink/nvs_defaults_v3_1_0.h").read_text()
    values = dict((key, float(value)) for key, value in re.findall(r'\{"([A-Z0-9]+)",([-0-9.]+)f\}', header))
    assert len(values) == len(snapshot["params"]) == 107
    for param in snapshot["params"]:
        assert values[param["key"]] == param["value"]
        assert param["lo"] <= param["value"] <= param["hi"]
