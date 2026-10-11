import importlib.util
from pathlib import Path

import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / "scripts/lqr_design.py"
spec = importlib.util.spec_from_file_location("lqr_design", SOURCE)
design = importlib.util.module_from_spec(spec)
spec.loader.exec_module(design)


def test_reproduces_original_gain_sets_with_correct_units_and_signs():
    expected = {
        "normal": [-7.0547, -6.5334, -34.5881, -4.2854, -2.1082],
        "stiff": [-59.9721, -30.9190, -81.4373, -10.0906, -25.8199],
        "climb": [-21.5836, -18.3212, -66.5207, -8.3370, -8.1650],
    }
    result = design.design(0.15)
    for mode, target in expected.items():
        np.testing.assert_allclose(list(result[mode]["gains"].values()), target, atol=0.0001)
        assert result[mode]["nominal_linear_stable"]


def test_acceleration_plant_is_mass_invariant_at_fixed_geometry():
    a, b = design.plant(0.15)
    assert a[3, 2] == 9.81 / 0.15
    assert b[1, 0] == 1
    assert b[3, 0] == -1 / 0.15
    assert a[4, 0] == 1


def test_fixed_updated_gains_tolerate_geometry_filters_and_latency():
    profiles = design.design(0.15285841346692242, design.UPDATED_WEIGHTS)
    result = design.validate_profiles(profiles, [0.11, 0.15, 0.20])
    assert all(record["stable"] for record in result.values()), result
    assert result["normal"]["cases"] == 72


def test_acceleration_limited_shove_recovery_with_softening_and_delay():
    profile = design.design(0.15285841346692242, design.UPDATED_WEIGHTS)["normal"]
    result = design.simulate_hold(.20, list(profile["gains"].values()), tilt_deg=0, rate_deg_s=20, soften=.55)
    assert result["recovered"], result
