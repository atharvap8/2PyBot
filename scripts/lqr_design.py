"""Acceleration-input LQI design, CAD mass update and sampled-loop validation.

Requires numpy, scipy, trimesh. No serial access or automatic gain application.
Units: STL mm; model SI; state [x, v, theta, omega, integral(x)].
"""
import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
from scipy.linalg import expm, solve_continuous_are

WEIGHTS = {
    "normal": ([10, 4, 17.5, 0.3, 2], 0.45, "K"),
    "stiff": ([600, 40, 90, 1.5, 200], 0.30, "S"),
    "climb": ([50, 30, 160, 3, 20], 0.30, "C"),
}
UPDATED_WEIGHTS = {
    "normal": ([10, 4, 52.5, 8, 2], 0.45, "K"),
    "stiff": ([600, 40, 90, 1.5, 200], 0.30, "S"),
    "climb": ([50, 30, 160, 20, 20], 0.30, "C"),
}


def plant(length):
    if not np.isfinite(length) or length <= 0:
        raise ValueError("Effective pendulum length must be positive")
    a = np.zeros((5, 5))
    a[0, 1] = a[2, 3] = a[4, 0] = 1
    a[3, 2] = 9.81 / length
    b = np.array([[0], [1], [0], [-1 / length], [0]])
    return a, b


def poles(values):
    return [[round(float(v.real), 6), round(float(v.imag), 6)] for v in values]


def design(length, weights=None):
    a, b = plant(length)
    aug = np.zeros((6, 6))
    aug[:5, :5], aug[:5, 5:] = a, b
    sampled = expm(aug * 0.005)  # firmware is 200 Hz, continuous CARE gains held per cycle
    ad, bd = sampled[:5, :5], sampled[:5, 5:]
    result = {}
    for name, (q, r, prefix) in (weights or WEIGHTS).items():
        p = solve_continuous_are(a, b, np.diag(q), np.array([[r]]))
        k = b.T @ p / r
        continuous = np.linalg.eigvals(a - b @ k)
        discrete = np.linalg.eigvals(ad - bd @ k)
        velocity_ids = [1, 2, 3]
        av, bv = a[np.ix_(velocity_ids, velocity_ids)], b[velocity_ids]
        velocity = np.linalg.eigvals(av - bv @ k[:, velocity_ids])
        softened = k.copy()
        softened[:, 2:4] *= 0.55
        soft_poles = np.linalg.eigvals(a - b @ softened)
        soft_velocity = np.linalg.eigvals(av - bv @ softened[:, velocity_ids])
        result[name] = {
            "Q": q, "R": r,
            "gains": {f"{prefix}{i + 1}": round(float(v), 4) for i, v in enumerate(k[0])},
            "hold_poles": poles(continuous), "velocity_poles": poles(velocity),
            "sampled_max_pole_magnitude": float(max(abs(discrete))),
            "terrain_0.55_hold_poles": poles(soft_poles),
            "terrain_0.55_velocity_poles": poles(soft_velocity),
            "nominal_linear_stable": bool(max(continuous.real) < 0 and max(abs(discrete)) < 1),
        }
    return result


def sampled_closed_loop(length, gains, soften=1, lag=0, delay=False, velocity=False, dt=0.005):
    """Fixed gains with firmware EMAs, integral-update order and actuator uncertainty."""
    a, b = plant(length)
    a[4, 0] = 0  # firmware integrates position once, before evaluating feedback
    count = 6 if lag else 5
    if lag:
        physical_a, physical_b = np.zeros((count, count)), np.zeros((count, 1))
        physical_a[:5, :5] = a
        physical_a[1, 5], physical_a[3, 5] = 1, -1 / length
        physical_a[5, 5], physical_b[5, 0] = -1 / lag, 1 / lag
    else:
        physical_a, physical_b = a, b
    augmented = np.zeros((count + 1, count + 1))
    augmented[:count, :count], augmented[:count, count:] = physical_a, physical_b
    discretized = expm(augmented * dt)
    ad, bd = discretized[:count, :count], discretized[:count, count:]
    size = count + 3 + int(delay)
    update = np.eye(size)
    update[4, 0] = dt
    pitch_alpha, velocity_alpha = 1 - np.exp(-2 * np.pi * 50 * dt), 1 - np.exp(-2 * np.pi * 15 * dt)
    f1, f2, vf = count, count + 1, count + 2
    update[f1] = 0
    update[f1, 2], update[f1, f1] = pitch_alpha, 1 - pitch_alpha
    update[f2] = pitch_alpha * update[f1]
    update[f2, f2] += 1 - pitch_alpha
    update[vf] = 0
    update[vf, 1], update[vf, vf] = velocity_alpha, 1 - velocity_alpha
    k = np.asarray(gains, dtype=float).copy()
    k[2:4] *= soften
    if velocity:
        k[[0, 4]] = 0
    feedback = np.zeros(size)
    feedback[[0, vf, f2, 3, 4]] = k
    command = -feedback @ update
    closed = update.copy()
    closed[:count] = ad @ update[:count]
    if delay:
        closed[:count, -1:] += bd
        closed[-1] = command
    else:
        closed[:count] += bd @ command[None, :]
    if velocity:
        # Position and its unused integral are intentionally unregulated here.
        indices = [i for i in range(size) if i not in (0, 4)]
        closed = closed[np.ix_(indices, indices)]
    return closed


def validate_profiles(profiles, lengths):
    records = {}
    for mode, profile in profiles.items():
        gains = list(profile["gains"].values())
        scenarios = [(False, 1)] if mode == "stiff" else [(False, 1), (False, 0.55)]
        if mode == "normal":
            scenarios += [(True, 1), (True, 0.55)]
        worst, worst_case, cases = 0, None, 0
        for length in lengths:
            for lag in (0, 0.005, 0.015):
                for delay in (False, True):
                    for velocity, soften in scenarios:
                        matrix = sampled_closed_loop(length, gains, soften, lag, delay, velocity)
                        radius = float(max(abs(np.linalg.eigvals(matrix))))
                        cases += 1
                        if radius > worst:
                            worst, worst_case = radius, {"length_m": float(length), "lag_s": lag,
                                                       "delay_cycles": int(delay), "velocity": velocity,
                                                       "soften": soften}
        records[mode] = {"cases": cases, "stable": worst < 1,
                         "worst_sampled_pole_magnitude": worst, "worst_case": worst_case}
    return records


def simulate_hold(length, gains, tilt_deg=2, rate_deg_s=0, soften=1, integral_limit=0.5):
    """Nonlinear, acceleration/speed-limited disturbance recovery; no hardware I/O."""
    dt, lag = 0.005, 0.015
    state = np.array([0., 0., np.deg2rad(tilt_deg), np.deg2rad(rate_deg_s), 0.])
    pitch1 = pitch2 = state[2]
    velocity = integral = reference = command = previous_u = 0.
    pitch_alpha, velocity_alpha = 1 - np.exp(-2 * np.pi * 50 * dt), 1 - np.exp(-2 * np.pi * 15 * dt)
    k = np.asarray(gains, dtype=float).copy()
    k[2:4] *= soften
    peak_angle, peak_speed, saturation = abs(state[2]), 0., 0

    def derivative(s, u):
        return np.array([s[1], s[4], s[3], (9.81 * np.sin(s[2]) - s[4] * np.cos(s[2])) / length,
                         (u - s[4]) / lag])

    for _ in range(2400):
        pitch1 += pitch_alpha * (state[2] - pitch1)
        pitch2 += pitch_alpha * (pitch1 - pitch2)
        velocity += velocity_alpha * (state[1] - velocity)
        error = np.clip(state[0] - reference, -0.15, 0.15)
        integral = np.clip(integral + error * dt, -integral_limit, integral_limit)
        requested = -float(k @ [error, velocity, pitch2, state[3], integral])
        saturation += int(abs(requested) > 3.927)
        u = np.clip(requested, -3.927, 3.927)
        next_command = np.clip(command + u * dt, -1.669, 1.669)
        limited_u = (next_command - command) / dt
        command = next_command
        if abs(command) > .9 * 1.669:
            reference += (state[0] - reference) * 3 * dt
            integral -= integral * 2 * dt
        # Include a complete additional control-cycle delay.
        d1 = derivative(state, previous_u)
        d2 = derivative(state + d1 * dt / 2, previous_u)
        d3 = derivative(state + d2 * dt / 2, previous_u)
        d4 = derivative(state + d3 * dt, previous_u)
        state += dt / 6 * (d1 + 2 * d2 + 2 * d3 + d4)
        previous_u = limited_u
        peak_angle, peak_speed = max(peak_angle, abs(state[2])), max(peak_speed, abs(state[1]))
        if abs(state[2]) > np.deg2rad(55) or not np.isfinite(state).all():
            break
    return {"recovered": bool(abs(state[2]) < np.deg2rad(.5) and abs(state[1]) < .02 and peak_angle < np.deg2rad(55)),
            "peak_pitch_deg": float(np.rad2deg(peak_angle)), "peak_speed_mps": float(peak_speed),
            "final_pitch_deg": float(np.rad2deg(state[2])), "final_position_error_m": float(state[0] - reference),
            "acceleration_saturated_cycles": saturation}


def cad_update(models, masses, axle_y, axle_z, base_mass, base_height, base_length, scale=1):
    if base_mass <= 0 or base_height <= 0 or base_length < base_height:
        raise ValueError("Invalid baseline rigid-body mass/COM/effective length")
    first_moment = base_mass * base_height
    inertia = first_moment * base_length
    total_mass = base_mass
    parts = []
    for name, grams in masses.items():
        mesh = trimesh.load_mesh(models / name)
        if not mesh.is_watertight or mesh.volume <= 0:
            raise ValueError(f"{name}: invalid closed mesh")
        mass = grams / 1000 * scale
        centre = mesh.center_mass
        height, forward = (centre[2] - axle_z) / 1000, (centre[1] - axle_y) / 1000
        intrinsic = mesh.moment_inertia[0, 0] / mesh.volume * mass * 1e-6
        about_axle = intrinsic + mass * (height**2 + forward**2)
        inertia += about_axle
        first_moment += mass * height
        total_mass += mass
        parts.append({"name": name, "mass_g": mass * 1000,
                      "cad_com_mm": centre.tolist(), "height_above_assumed_axle_m": height,
                      "pitch_inertia_about_assumed_axle_kg_m2": about_axle})
    if first_moment <= 0:
        raise ValueError("Assembly COM must lie above axle")
    return {"parts": parts, "body_mass_kg": total_mass,
            "com_height_m": first_moment / total_mass,
            "pitch_inertia_about_axle_kg_m2": inertia,
            "effective_length_m": inertia / first_moment}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", type=Path, default=Path("models"))
    parser.add_argument("--masses", type=Path, default=Path("hardware/dynamics/printed_parts.json"))
    parser.add_argument("--effective-length", type=float, help="Measured/identified effective length, bypass CAD assumptions")
    parser.add_argument("--base-mass", type=float, default=1.34)
    parser.add_argument("--base-com-height", type=float, default=0.06)
    parser.add_argument("--base-effective-length", type=float, default=0.15)
    parser.add_argument("--axle-y-mm", type=float, default=45.0)
    parser.add_argument("--axle-z-mm", type=float, default=-64.15)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    mass_data = json.loads(args.masses.read_text(encoding="utf-8"))
    assembly = cad_update(args.models, mass_data["mass_g"], args.axle_y_mm, args.axle_z_mm,
                          args.base_mass, args.base_com_height, args.base_effective_length)
    length = args.effective_length if args.effective_length is not None else assembly["effective_length_m"]
    candidates = design(length, UPDATED_WEIGHTS)
    report = {
        "status": "identified-length design" if args.effective_length else "estimated CAD design; sampled-loop validated",
        "assumptions": {"base_rigid_body_mass_kg": args.base_mass,
                        "base_com_height_m": args.base_com_height,
                        "base_effective_length_m": args.base_effective_length,
                        "axle_y_mm": args.axle_y_mm, "axle_z_mm": args.axle_z_mm,
                         "baseline_all_up_mass_kg": 1.5, "estimated_wheel_mass_kg": 0.16,
                         "note": "Estimates, not measured mass properties. Subtract 160 g of wheels from the operator's 1.5 kg all-up baseline. Estimate 60 mm baseline body COM from heavy motors at the axle and battery/electronics above it. Current STL cavity dimensions match enclosure source: 40 mm standoffs, 3 mm bracket, 42.3 mm NEMA17 body imply axle Z=-64.15 mm. The 0.15 m baseline effective length reproduces the existing controller. Uncertainty is assessed with FIXED updated gains."},
        "mass_source": mass_data, "cad_assembly": assembly,
        "design_effective_length_m": length, "candidate": candidates,
        "old_length_reproduction": design(0.15),
        "sensitivity": [],
        "serial_candidate": [f"P,{key},{value:.4f}" for mode in candidates.values()
                             for key, value in mode["gains"].items()],
    }
    # Explicit geometry/mass uncertainty; does not prove robustness outside this model.
    for h in (0.04, 0.06, 0.10):
        for z in (-55.0, -63.75, -75.0):
            for scale in (0.9, 400 / sum(mass_data["mass_g"].values())):
                variant = cad_update(args.models, mass_data["mass_g"], args.axle_y_mm, z,
                                     args.base_mass, h, args.base_effective_length, scale)
                report["sensitivity"].append({"base_height_m": h, "axle_z_mm": z,
                                               "print_mass_scale": scale,
                                               "length_m": variant["effective_length_m"],
                                               "gains": design(variant["effective_length_m"])["normal"]["gains"]})
    report["validation"] = validate_profiles(candidates, np.linspace(0.11, 0.20, 10))
    report["validation_model"] = {
        "effective_length_range_m": [0.11, 0.20], "control_hz": 200,
        "pitch_filter": "two cascaded 50 Hz firmware EMAs", "velocity_filter": "15 Hz firmware EMA",
        "actuator_lag_s": [0, 0.005, 0.015], "additional_delay_cycles": [0, 1],
        "note": "Small-signal model checks include normal drive and terrain softening. Saturation, traction, slope and lost steps require physical balance testing; they are not established by eigenvalues."}
    report["disturbance_tests"] = []
    for mode, profile in candidates.items():
        for actual_length in (0.11, length, 0.20):
            for tilt, rate in ((2, 0), (-2, 0), (0, 20)):
                for soften in ([1] if mode == "stiff" else [1, .55]):
                    result = simulate_hold(actual_length, list(profile["gains"].values()), tilt, rate, soften,
                                           2 if mode == "climb" else .5)
                    report["disturbance_tests"].append({"mode": mode, "length_m": actual_length,
                                                         "initial_pitch_deg": tilt, "initial_rate_deg_s": rate,
                                                         "soften": soften, **result})
    if not all(value["stable"] for value in report["validation"].values()):
        raise RuntimeError("Gain validation failed: " + json.dumps(report["validation"]))
    if not all(value["recovered"] for value in report["disturbance_tests"]):
        raise RuntimeError("Disturbance recovery failed: " + json.dumps([v for v in report["disturbance_tests"] if not v["recovered"]]))
    encoded = json.dumps(report, indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(report["status"])
    print(f"Printed parts: {sum(mass_data['mass_g'].values()):.2f} g; assumed effective length: {length:.6f} m")
    for mode, values in candidates.items():
        print(mode, values["gains"], "linear stable:", values["nominal_linear_stable"])


if __name__ == "__main__":
    main()
