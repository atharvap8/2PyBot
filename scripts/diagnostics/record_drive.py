"""Passive timed capture of existing console telemetry. Never sends commands.

Run only when the operator says GO. Uses localhost HTTP keep-alive at a target
100 Hz and records actual MCU sample IDs/rates, not assumed sampling frequency.
"""
import argparse
import csv
from datetime import datetime, timezone
import http.client
import json
from pathlib import Path
import statistics
import time
import urllib.parse


class Reader:
    def __init__(self, url):
        parsed = urllib.parse.urlparse(url)
        if parsed.scheme != "http":raise ValueError("Use the existing HTTP console API")
        self.connection = http.client.HTTPConnection(parsed.hostname, parsed.port or 8080, timeout=2)

    def get(self, path):
        self.connection.request("GET", path)
        response = self.connection.getresponse()
        data = response.read()
        if response.status != 200:raise RuntimeError(f"HTTP {response.status}")
        return json.loads(data)

    def close(self):self.connection.close()


def capture(duration, url, directory, scenario, probe=False):
    reader = Reader(url)
    baseline = reader.get("/api/status")
    if not baseline["connected"] or baseline["telemetry"].get("stale", True):
        raise RuntimeError("Need fresh telemetry before the recording starts")
    if not baseline.get("debug"):
        raise RuntimeError("Debug stream is not available; recorder will not toggle firmware settings")
    params = None if probe else reader.get("/api/params")
    folder = None
    raw = None
    table = None
    fields = ["host_elapsed_ms", "host_wall_ns", "request_ms", "firmware_odom_ms", "firmware_debug_ms",
              "raw_pitch_deg", "forward_pitch_deg", "pitch_rate_dps", "u_mps2", "filtered_speed_mps",
              "encoder_speed_mps", "position_error_m", "base_cmd_steps_s", "steer_cmd_steps_s",
              "left_cmd_target_steps_s", "right_cmd_target_steps_s", "forward_input", "steer_input",
              "high_speed", "balancing", "sat_velocity", "sat_acceleration", "loop_max_us",
              "enc_left", "enc_right", "measured_left_mps", "measured_right_mps",
              "acceleration_g", "roughness", "softening", "terrain_state", "drops", "stale",
              "guard_enabled", "guard_forward_limit_mps", "fall_risk", "predicted_tilt_deg",
              "fall_flags", "requested_drive_mps", "shaped_drive_mps", "effective_drive_mps",
              "applied_balance_scale", "auto_climb_enabled", "auto_climb_state", "climb_policy", "climb_blend",
              "auto_tracking_error_mps", "auto_pitch_bias_deg", "auto_following_error_mps", "auto_climb_flags",
              "climb_integral_m_s", "climb_error_m", "climb_reference_mps", "immediate_hazard", "climb_bleed", "steady_request_s",
              "slope_kind", "exit_confirmation_s", "exit_block_bits", "mode_cooldown_s", "descent_limit_mps", "descent_excess_mps",
              "descent_request_scale", "descent_active", "descent_goal_mps", "directional_overspeed_mps"]
    if not probe:
        parent = Path(directory)
        parent.mkdir(parents=True, exist_ok=True)
        folder = parent / ("drive-"+datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S"))
        folder.mkdir()
        (folder/"baseline.json").write_text(json.dumps({"scenario":scenario,"duration_s":duration,
            "status":baseline,"params":params,"read_only":True,
            "wheel_commands":"Firmware wheelL/wheelR targets when available; otherwise reconstructed vCmd-steer/vCmd+steer. Not ISR-applied wheel rates.",
            "joystick":"Firmware-reported inputs after input arbitration/gesture/guard, not raw Bluetooth stick ADC.",
            "sampling":"Cached console state polled at target 100 Hz; actual distinct MCU rates recorded."},indent=2))
        raw = (folder/"events.jsonl").open("w")
        table = (folder/"telemetry.csv").open("w",newline="")
        writer = csv.DictWriter(table,fieldnames=fields);writer.writeheader()
    start = time.monotonic_ns()
    wall_start = time.time_ns()
    deadline = start+round(duration*1e9)
    next_poll = start
    debug_ids = set();odom_ids = set();latencies = [];errors = []
    last_debug = None;last_encoder = None;last_state = None;transitions = []
    count = 0
    print("PROBE" if probe else f"RECORDING STARTED: {duration:g} seconds",flush=True)
    try:
        while time.monotonic_ns() < deadline:
            request_at = time.monotonic_ns()
            try:status = reader.get("/api/status")
            except Exception as exc:
                errors.append({"elapsed_ms":(time.monotonic_ns()-start)/1e6,"error":str(exc)})
                reader.close();reader = Reader(url);continue
            received = time.monotonic_ns()
            if received >= deadline:break
            elapsed = (received-start)/1e6
            request_ms = (received-request_at)/1e6
            latencies.append(request_ms);count += 1
            tel=status["telemetry"];debug=status.get("debug",{});derived=status.get("derived",{});guard=status.get("tof",{})
            debug_ids.add(debug.get("ms"));odom_ids.add(tel.get("ms"))
            if raw:
                raw.write(json.dumps({"elapsed_ms":elapsed,"host_wall_ns":time.time_ns(),"request_ms":request_ms,
                    "telemetry":tel,"debug":debug,"derived":derived,
                    "guard":{"enabled":guard.get("guard_enabled"),"cap":guard.get("forward_limit_mps"),"reason":guard.get("guard_reason")}})+"\n")
            state=bool(debug.get("bal",tel.get("balancing")))
            if last_state is not None and state != last_state:transitions.append({"elapsed_ms":elapsed,"balancing":state,"pitch":tel.get("pitch"),"rate":debug.get("w")})
            last_state=state
            left_speed=right_speed=None
            current_encoder=(tel.get("ms",0),tel.get("encL",0),tel.get("encR",0))
            if last_encoder and current_encoder[0] > last_encoder[0]:
                dt=(current_encoder[0]-last_encoder[0])/1000
                counts_m=derived.get("COUNTSM")
                if counts_m:
                    left_speed=(current_encoder[1]-last_encoder[1])/counts_m/dt
                    right_speed=(current_encoder[2]-last_encoder[2])/counts_m/dt
            if current_encoder[0] != (last_encoder[0] if last_encoder else None):last_encoder=current_encoder
            if table and debug.get("ms") != last_debug:
                base=debug.get("vCmd",0);steer=debug.get("steer",0)
                writer.writerow(dict(zip(fields,[round(elapsed,3),time.time_ns(),round(request_ms,3),tel.get("ms"),debug.get("ms"),
                    tel.get("pitch"),debug.get("pF"),debug.get("w"),debug.get("u"),debug.get("v"),tel.get("v_ms"),debug.get("ex"),
                    base,steer,debug.get("wheelL",base-steer),debug.get("wheelR",base+steer),debug.get("fwd"),debug.get("str"),debug.get("spdHi"),state,
                    debug.get("satV"),debug.get("satA"),debug.get("loopMax"),tel.get("encL"),tel.get("encR"),left_speed,right_speed,
                    debug.get("accelMag"),debug.get("roughness"),debug.get("soften"),debug.get("terrainState"),debug.get("drops"),
                    tel.get("stale"),guard.get("guard_enabled"),guard.get("forward_limit_mps"),debug.get("fpRisk"),
                    debug.get("fpPred"),debug.get("fpFlags"),debug.get("driveReq"),debug.get("driveRef"),
                    debug.get("driveEff"),debug.get("balScale"),debug.get("acEnabled"),debug.get("acState"),
                    debug.get("acPolicy"),debug.get("acBlend"),debug.get("acError"),debug.get("acBias"),
                    debug.get("acFollow"),debug.get("acFlags"),debug.get("clZ"),debug.get("clEx"),
                    debug.get("clRef"),debug.get("fpHazard"),debug.get("clRecover"),debug.get("acStable"),
                    debug.get("acKind"),debug.get("acExit"),debug.get("acExitBlock"),debug.get("acCooldown"),
                    debug.get("dsLimit"),debug.get("dsExcess"),debug.get("dsScale"),debug.get("dsActive"),
                    debug.get("dsGoal"),debug.get("acOverspeed")])))
                last_debug=debug.get("ms")
            next_poll += 10_000_000
            wait=(next_poll-time.monotonic_ns())/1e9
            if wait>0:time.sleep(wait)
            elif wait < -.1:next_poll=time.monotonic_ns()
    finally:
        if raw:raw.close()
        if table:table.close()
    summary={"requested_duration_s":duration,"window_start_wall_ns":wall_start,"window_end_wall_ns":wall_start+round(duration*1e9),
        "polls":count,"distinct_debug_samples":len(debug_ids-{None}),"distinct_odom_samples":len(odom_ids-{None}),
        "debug_observed_hz":round(len(debug_ids-{None})/duration,2),"odom_observed_hz":round(len(odom_ids-{None})/duration,2),
        "median_request_ms":round(statistics.median(latencies),3) if latencies else None,"errors":errors,"balancing_transitions":transitions}
    if folder:
        try:
            logs=reader.get("/api/log?n=1000")
            logs["log"]=[row for row in logs["log"] if wall_start/1e9 <= row["t"] <= (wall_start+round(duration*1e9))/1e9]
            (folder/"logs.json").write_text(json.dumps(logs,indent=2))
        except Exception as exc:summary["log_error"]=str(exc)
        (folder/"summary.json").write_text(json.dumps(summary,indent=2))
        summary["folder"]=str(folder)
    reader.close()
    print(json.dumps(summary,indent=2),flush=True)
    return summary


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--duration",type=float,default=45)
    parser.add_argument("--api",default="http://127.0.0.1:8080")
    parser.add_argument("--directory",default="/home/radxa/archives/drive-tests")
    parser.add_argument("--scenario",default="Full joystick up, immediately down-right; bot front facing operator, moving towards operator; operator expects about three falls")
    parser.add_argument("--probe",action="store_true")
    args=parser.parse_args()
    if not 0<args.duration<=120:parser.error("Duration must be 0–120 seconds")
    capture(2 if args.probe else args.duration,args.api,args.directory,args.scenario,args.probe)
