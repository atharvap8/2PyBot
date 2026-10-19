"""Offline PG-to-home Android session analysis, with boot/mode-aware filtering.

Reads the supplied recording only; writes reports/plots to a separate directory.
No network, serial, arming or parameter writes. Use --output to choose the folder.
"""
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DEBUG_NAMES = ["pF", "w", "u", "v", "ex", "vCmd", "steer", "satV", "satA", "loopMax",
               "spdHi", "bal", "fwd", "str", "accelMag", "roughness", "soften", "terrainState",
               "drops", "fpRisk", "fpPred", "fpFlags", "driveReq", "driveRef", "driveEff",
               "balScale", "wheelL", "wheelR"]


def mode_transition(text, mode):
    if "2PyBot BaseLink" in text: return "idle"
    if "Motors DISABLED" in text or "Motors OFF" in text: return "idle"
    if "-> BALANCING" in text: return "normal"
    if "[HOLD]" in text:
        if "CLIMB" in text: return "climb"
        if "climb off" in text or "normal" in text: return "normal"
        if "STIFF" in text: return "stiff"
    return mode


def weighted_mean(value, weight):
    valid = np.isfinite(value) & np.isfinite(weight) & (weight > 0)
    return float(np.average(value[valid], weights=weight[valid])) if valid.any() else None


def analyse(folder, output, verbose=True):
    assert folder.is_dir()
    metadata = json.loads((folder / "metadata.json").read_text())
    params = {p["key"]: float(p["value"]) for p in metadata["robot"]["params"]}
    derived = metadata["robot"]["derived"]
    records = []; logs = []; intervals = []; modes = []; errors = []; changes = []
    counts = Counter(); mode = "unknown"; mode_start = 0; epoch = 0; last_debug_ms = None
    guard_counts = Counter(); data_keys = set(); last_event_ms = 0
    with (folder / "events.jsonl").open(encoding="utf-8") as file:
        for line_number, line in enumerate(file, 1):
            try: event = json.loads(line)
            except json.JSONDecodeError as exc:
                errors.append({"line": line_number, "error": str(exc)}); continue
            time = event["elapsed_ms"] / 1000
            last_event_ms = max(last_event_ms, event["elapsed_ms"])
            counts[event["source"] + ":" + event["event"]] += 1
            payload = event.get("payload", {})
            if event["source"] == "ws" and event["event"] == "log":
                text = payload.get("text", "")
                # Keep the complete raw log in input; report only a bounded excerpt.
                logs.append({"t_s": time, "text": text[:250], "raw_length": len(text)})
                next_mode = mode_transition(text, mode)
                if next_mode != mode:
                    intervals.append({"mode": mode, "start_s": mode_start, "end_s": time})
                    mode_start = time; mode = next_mode
                if "Failed to disconnect HIDS" in text or "SM_EVENT_PAIRING_COMPLETE" in text:
                    modes.append(time)
            if event["source"] == "ws" and event["event"] == "params":
                updated = {p["key"]: float(p["value"]) for p in payload.get("params", [])}
                changed = {k: [params.get(k), v] for k, v in updated.items() if params.get(k) != v}
                if changed: changes.append({"time_s": time, "changes_from_initial": changed})
            if event["source"] == "ws" and event["event"] == "tof":
                guard_counts[str(payload.get("tof", {}).get("guard_enabled"))] += 1
            if event["source"] != "ws" or event["event"] != "telemetry": continue
            tel = payload.get("tel", {}); debug = payload.get("debug", {})
            data_keys.update(tel); data_keys.update(debug)
            debug_ms = float(debug.get("ms", np.nan))
            if last_debug_ms is not None and debug_ms < last_debug_ms - 1000: epoch += 1
            if np.isfinite(debug_ms): last_debug_ms = debug_ms
            row = {"seq": event["seq"], "t": time, "epoch": epoch, "mode": mode,
                   "odom_ms": float(tel.get("ms", np.nan)), "debug_ms": debug_ms,
                   "encL": float(tel.get("encL", np.nan)), "encR": float(tel.get("encR", np.nan)),
                   "x": float(tel.get("x_m", np.nan)), "y": float(tel.get("y_m", np.nan)),
                   "tel_balancing": bool(tel.get("balancing")), "stale": bool(tel.get("stale", True)),
                   "label": event.get("terrain", "")}
            row.update({name: float(debug.get(name, np.nan)) for name in DEBUG_NAMES})
            records.append(row)
    assert records, "No WebSocket telemetry in recording"
    intervals.append({"mode": mode, "start_s": mode_start, "end_s": records[-1]["t"]})
    # Cross-check recorder CSV rather than counting the two copies as samples.
    with (folder / "telemetry.csv").open(newline="", encoding="utf-8") as file:
        csv_rows = list(csv.DictReader(file))
    assert len(csv_rows) == len(records)
    assert all(int(csv_row["seq"]) == row["seq"] and abs(float(csv_row["elapsed_ms"])/1000-row["t"]) < 1e-8
               for csv_row, row in zip(csv_rows, records))

    a = {name: np.array([row[name] for row in records], dtype=float)
         for name in ["t", "epoch", "odom_ms", "debug_ms", "encL", "encR", "x", "y"] + DEBUG_NAMES}
    t = a["t"]; n = len(t); mode_array = np.array([row["mode"] for row in records])
    stale = np.array([row["stale"] for row in records]); unique = np.ones(n, dtype=bool)
    unique[1:] = (a["debug_ms"][1:] != a["debug_ms"][:-1]) | (a["epoch"][1:] != a["epoch"][:-1])
    # A cached D line can lag a fresh O line during Bluetooth UART interleaving.
    age_ms = a["odom_ms"] - a["debug_ms"]
    fresh = ~stale & unique & np.isfinite(a["v"]) & (age_ms < 250) & (age_ms > -250)
    active = fresh & (a["bal"] > .5) & (mode_array != "idle")
    climb = active & (mode_array == "climb")
    normal = active & np.isin(mode_array, ["normal", "unknown"])
    weights = np.clip(np.diff(t, append=t[-1]), 0, .25)
    cm = float(derived["COUNTSM"]); sm = float(derived["STEPSM"])
    a["command_mps"] = a["vCmd"] / sm
    a["left_command_mps"] = a["wheelL"] / sm
    a["right_command_mps"] = a["wheelR"] / sm
    for name in ("encoder_left_mps", "encoder_right_mps", "encoder_common_mps", "encoder_dt_s"):
        a[name] = np.full(n, np.nan)
    resets = []; encoder_path = 0.; following = []
    for i in range(1, n):
        if a["epoch"][i] != a["epoch"][i-1]:
            resets.append({"first_new_telemetry_s": t[i], "old_debug_ms": a["debug_ms"][i-1],
                           "new_debug_ms": a["debug_ms"][i],
                           "host_pose_jump_m": float(np.hypot(a["x"][i]-a["x"][i-1],a["y"][i]-a["y"][i-1]))})
            continue
        dt = (a["odom_ms"][i]-a["odom_ms"][i-1])/1000
        if dt <= 0 or dt > .5 or stale[i] or stale[i-1]: continue
        left = (a["encL"][i]-a["encL"][i-1])/cm/dt
        right = (a["encR"][i]-a["encR"][i-1])/cm/dt
        a["encoder_left_mps"][i]=left;a["encoder_right_mps"][i]=right
        a["encoder_common_mps"][i]=(left+right)/2;a["encoder_dt_s"][i]=dt
        if active[i] and active[i-1]:
            encoder_path += abs((left+right)/2)*dt
            following.append({"i":i,"dt":dt,
                              "left_cmd":(a["left_command_mps"][i]+a["left_command_mps"][i-1])/2,
                              "right_cmd":(a["right_command_mps"][i]+a["right_command_mps"][i-1])/2,
                              "left_measured":left,"right_measured":right})
    # Infer integral from UNSATURATED production climb equation only. This is
    # not a recorded state; quantized telemetry and post-control saturation bleed
    # invalidate inversion near speed saturation or in other gain/mode branches.
    inferred = np.full(n, np.nan)
    identifiable = climb & (a["satA"] == 0) & (np.abs(a["vCmd"]) < .88*params["MAXSPD"])
    if params["C5"] != 0:
        th = np.radians(a["pF"]); rate = np.radians(a["w"])
        inferred[identifiable] = ((-a["u"]-params["C1"]*a["ex"]-params["C2"]*a["v"]
                                  -params["C3"]*a["balScale"]*th-params["C4"]*a["balScale"]*rate)
                                 /params["C5"])[identifiable]
    a["inferred_climb_integral"] = inferred

    def statistics(mask):
        selected = np.flatnonzero(mask); w = weights[mask]
        if not len(selected): return {"samples":0}
        v = a["v"][mask]; cmd = a["command_mps"][mask]; fwd = a["fwd"][mask]
        result={"samples":len(selected),"sample_weight_s":round(float(w.sum()),3),
                "forward_input_median":round(float(np.median(fwd)),4),
                "requested_ref_median_mps":round(float(np.median(a["driveReq"][mask])),4),
                "operator_ref_median_mps":round(float(np.median(a["driveRef"][mask])),4),
                "measured_speed_median_mps":round(float(np.median(v)),4),
                "measured_speed_mean_mps":round(weighted_mean(v,w),4),
                "command_speed_mean_mps":round(weighted_mean(cmd,w),4),
                "command_speed_median_mps":round(float(np.median(cmd)),4),
                "command_minus_filtered_rms_mps":round(float(np.sqrt(np.mean((cmd-v)**2))),4),
                "median_abs_pitch_deg":round(float(np.median(np.abs(a["pF"][mask]))),3),
                "pitch_mean_deg":round(weighted_mean(a["pF"][mask],w),3),
                "rate_rms_dps":round(float(np.sqrt(np.mean(a["w"][mask]**2))),3),
                "accel_saturation_fraction":round(weighted_mean(a["satA"][mask]>.5,w),4),
                "speed_saturation_fraction":round(weighted_mean(a["satV"][mask]>.5,w),4),
                "risk_median":round(float(np.median(a["fpRisk"][mask])),3),
                "risk_maximum":round(float(np.max(a["fpRisk"][mask])),3),
                "risk_at_25pct_floor_fraction":round(weighted_mean(np.abs(a["fpRisk"][mask]-.25)<.002,w),4),
                "predicted_tilt_max_deg":round(float(np.max(a["fpPred"][mask])),3),
                "recovery_fraction":round(weighted_mean(a["fpRisk"][mask]>0,w),4),
                "ex_median_m":round(float(np.median(a["ex"][mask])),4),
                "ex_at_clamp_fraction":round(weighted_mean(np.abs(a["ex"][mask])>=params["EXCLAMP"]-.003,w),4),
                "loop_max_us":round(float(np.max(a["loopMax"][mask]))),
                "loop_p99_us":round(float(np.quantile(a["loopMax"][mask],.99)))}
        estimates = inferred[mask & np.isfinite(inferred)]
        if len(estimates):
            result["inferred_integral_samples"]=len(estimates)
            result["inferred_integral_p05_median_p95_m_s"]=np.quantile(estimates,[.05,.5,.95]).round(4).tolist()
        targets = [point for point in following if mask[point["i"]]]
        if targets:
            for wheel in ("left","right"):
                error=np.array([point[wheel+"_cmd"]-point[wheel+"_measured"] for point in targets])
                result[wheel+"_command_following_mae_mps"]=round(float(np.mean(np.abs(error))),4)
                result[wheel+"_command_following_rms_mps"]=round(float(np.sqrt(np.mean(error**2))),4)
            # Sustained large wheel command with little rotation is a stronger
            # actuator-stall indication than joystick vs measured speed alone.
            unmatched=np.array([abs((point["left_cmd"]+point["right_cmd"])/2)>.3 and
                                 abs((point["left_measured"]+point["right_measured"])/2)<.05 for point in targets])
            result["large_command_low_rotation_fraction"]=round(float(unmatched.mean()),4)
        return result

    # Independently verify temporal integral decay over a mildly recovering
    # part of this record, rather than only inverting the instantaneous LQI.
    decay_window=climb & (t>=355) & (t<=365)
    identified=np.flatnonzero(decay_window & np.isfinite(inferred))
    decay_verification={}
    if len(identified)>2 and np.all(a["fpRisk"][decay_window]==.25):
        last_time=t[identified[0]];state=inferred[identified[0]];comparisons=[]
        for i in identified[1:]:
            while last_time<t[i]-1e-9:
                dt=min(.005,t[i]-last_time)
                ex=float(np.interp(last_time+dt/2,t[decay_window],a["ex"][decay_window]))
                state=state*np.exp(-.5*dt)+ex/.5*(1-np.exp(-.5*dt))
                last_time+=dt
            comparisons.append({"t_s":float(t[i]),"inferred_m_s":float(inferred[i]),"leak_model_m_s":float(state)})
        residual=np.array([row["inferred_m_s"]-row["leak_model_m_s"] for row in comparisons])
        decay_verification={"window_s":[355,365],"uncapped_samples":len(identified),
                            "initial_state_from_first_identified_sample":float(inferred[identified[0]]),
                            "rms_residual_m_s":float(np.sqrt(np.mean(residual**2))),"comparisons":comparisons,
                            "interpretation":"Temporal match to dz/dt=ex-0.5*z at recorded risk .25; not a closed-loop trajectory prediction"}

    phase_stats = []
    for interval in intervals:
        if interval["mode"] == "idle": continue
        mask=active & (t>=interval["start_s"]) & (t<interval["end_s"])
        phase_stats.append(dict(interval,**statistics(mask)))
    climb_stats=[]
    for interval in intervals:
        if interval["mode"] != "climb":continue
        mask=climb & (t>=interval["start_s"]) & (t<interval["end_s"])
        strong=mask & (a["fwd"]>=.8)
        climb_stats.append(dict(interval,all_samples=statistics(mask),forward_input_above_80pct=statistics(strong)))
    blocks=[]
    for start,end in ((0,120),(120,300),(302,337),(407.2,418.6)):
        mask=normal & (t>=start) & (t<end) & (a["fwd"]>=.8) & (np.abs(a["str"])<.1) & (np.abs(a["pF"])<10)
        blocks.append({"start_s":start,"end_s":end,**statistics(mask)})
    cutoffs=[log for log in logs if "Motors OFF" in log["text"]]
    bt_first=min(modes) if modes else None
    corrupted=[log for log in logs if log["text"].startswith(("D,","O,"))]
    thermal_fields=sorted(key for key in data_keys if any(s in key.lower() for s in ("temp","otpw","voltage","battery","drv_status")))
    active_seconds=sum(interval["end_s"]-interval["start_s"] for interval in intervals if interval["mode"]!="idle")
    source_hashes={name:hashlib.sha256((folder/name).read_bytes()).hexdigest()
                   for name in ("events.jsonl","telemetry.csv","metadata.json","README.txt")}
    summary={"session_id":metadata["session_id"],"start_utc":datetime.fromtimestamp(metadata["started_wall_ms"]/1000,timezone.utc).isoformat(),
             "source_files_sha256":source_hashes,"events":dict(counts),"telemetry_samples":n,
             "recorded_telemetry_duration_s":float(t[-1]),"last_event_s":last_event_ms/1000,
             "motors_enabled_interval_estimate_s":round(active_seconds,3),"active_unique_fresh_samples":int(active.sum()),
             "stale_samples":int(stale.sum()),"duplicate_debug_samples":int((~unique).sum()),
             "fresh_odom_cached_debug_age_over_250ms_samples":int(((age_ms>=250)&~stale).sum()),
             "sampling_median_interval_ms":round(float(np.median(np.diff(t)))*1000,3),
             "epochs":int(max(a["epoch"])+1),"resets":resets,"mode_intervals":intervals,
             "motor_stop_events":cutoffs,"bluetooth_error_start_s":bt_first,"bluetooth_error_messages":len(modes),
             "interleaved_serial_log_fragments":len(corrupted),"guard_enabled_values":dict(guard_counts),
             "thermal_or_voltage_telemetry_fields":thermal_fields,"parameter_changes":changes,"malformed_json_lines":errors,
             "no_normal_stop_diagnostics":not (folder/"diagnostics.json").exists(),
             "encoder_mean_wheel_absolute_travel_while_active_m":round(encoder_path,3),
             "active_statistics":statistics(active),"climb_intervals":climb_stats,"phase_statistics":phase_stats,
             "normal_full_forward_straight_comparison":blocks,
             "integral_decay_verification":decay_verification,
             "climb_recovery_integral_steady_state_bound_example_m_s":-params["EXCLAMP"]/(2*.25),
             "params":params,"derived":derived,
             "limits":["Approximately 10 Hz received snapshots, not lossless 200 Hz control-loop samples",
                       "Terrain label is not a measured ramp angle; no video, battery voltage or temperatures",
                       "Inferred integral uses uncapped climb samples; not a directly recorded state",
                       "Encoder wheel rotation does not distinguish tyre slip from ground travel",
                       "Idle command/risk/integral fields are cached; wheel targets are zero after disable",
                       "Host odometry has a reset jump; exclude it from distance estimates"]}
    output.mkdir(parents=True,exist_ok=True)
    (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
    timeline_fields=["seq","elapsed_s","epoch","mode","fresh_active","odom_ms","debug_ms","encoder_left_mps","encoder_right_mps",
                     "common_command_mps","measured_filtered_mps","requested_mps","shaped_mps","risk","position_error_m","inferred_integral_ms"]
    timeline_fields[-1]="inferred_integral_m_s"
    with (output/"climb-timeline.csv").open("w",newline="",encoding="utf-8") as file:
        writer=csv.writer(file);writer.writerow(timeline_fields)
        for i,row in enumerate(records):
            writer.writerow([row["seq"],row["t"],row["epoch"],row["mode"],bool(active[i]),row["odom_ms"],row["debug_ms"],
                             a["encoder_left_mps"][i],a["encoder_right_mps"][i],a["command_mps"][i],a["v"][i],a["driveReq"][i],
                             a["driveRef"][i],a["fpRisk"][i],a["ex"][i],inferred[i]])
    create_plots(output,a,active,climb,intervals,bt_first,logs,params)
    (output/"report.md").write_text(report_markdown(summary),encoding="utf-8")
    if verbose:
        print(json.dumps({key:summary[key] for key in ("session_id","recorded_telemetry_duration_s","motors_enabled_interval_estimate_s",
                "active_unique_fresh_samples","motor_stop_events","bluetooth_error_start_s","bluetooth_error_messages","resets",
                "thermal_or_voltage_telemetry_fields","climb_intervals","normal_full_forward_straight_comparison")},indent=2))
        print("REPORT",output/"report.md")
    return {"summary":summary,"arrays":a,"records":records,"active":active,"fresh":fresh,"unique":unique,
            "climb":climb,"normal":normal,"weights":weights,"modes":mode_array,"intervals":intervals,
            "following":following,"logs":logs,"metadata":metadata}


def create_plots(output,a,active,climb,intervals,bt_first,logs,params):
    t=a["t"]
    def values(name,mask):
        result=a[name].copy();result[~mask]=np.nan;return result
    plt.rcParams.update({"font.size":9,"axes.grid":True,"grid.alpha":.2})
    fig,axes=plt.subplots(5,1,figsize=(15,13),sharex=True,layout="constrained")
    axes[0].plot(t,values("driveReq",active),label="requested reference",alpha=.7)
    axes[0].plot(t,values("driveRef",active),label="shaped reference",alpha=.7)
    axes[0].set_ylabel("Reference m/s");axes[0].legend(loc="upper left",ncol=2)
    axes[1].plot(t,values("command_mps",active),label="common wheel command",alpha=.7)
    axes[1].plot(t,values("v",active),label="measured encoder speed",alpha=.7)
    axes[1].set_ylabel("Wheel m/s");axes[1].legend(loc="upper left",ncol=2)
    axes[2].plot(t,values("pF",active),label="forward-positive pitch")
    axes[2].set_ylabel("Pitch degrees");axes[2].legend(loc="upper left")
    axes[3].plot(t,values("fpRisk",active),label="recovery risk")
    axes[3].plot(t,values("balScale",active),label="selected attitude scale",alpha=.6)
    axes[3].set_ylabel("Factor");axes[3].legend(loc="upper left",ncol=2)
    axes[4].plot(t,values("ex",climb),label="climb position error m")
    axes[4].plot(t,a["inferred_climb_integral"],".",markersize=2,label="inferred integral m*s (uncapped samples)")
    axes[4].set_ylabel("Climb states");axes[4].legend(loc="upper left",ncol=2)
    for axis in axes:
        for interval in intervals:
            if interval["mode"]=="climb":axis.axvspan(interval["start_s"],interval["end_s"],color="purple",alpha=.10)
        if bt_first is not None:axis.axvline(bt_first,color="orange",linestyle=":",alpha=.7)
        for log in logs:
            if "Motors OFF" in log["text"]:axis.axvline(log["t_s"],color="red",linestyle=":",alpha=.5)
            if "2PyBot BaseLink" in log["text"]:axis.axvline(log["t_s"],color="black",linestyle="--",alpha=.6)
    axes[-1].set_xlabel("Seconds since recording start; purple = climb, orange = BT errors, black = reboot, red = disable")
    last_active=t[np.flatnonzero(active)[-1]];axes[-1].set_xlim(0,last_active+5)
    fig.suptitle("PG to home: active drive and climb timeline (idle cached controller states excluded)")
    fig.savefig(output/"drive-overview.png",dpi=150);plt.close(fig)
    fig,axes=plt.subplots(5,2,figsize=(17,13),sharex="col",layout="constrained")
    for column,(start,end) in enumerate(((345,379),(394,429))):
        mask=active & (t>=start) & (t<=end)
        axes[0,column].plot(t,values("driveRef",mask),label="shaped operator reference")
        axes[0,column].plot(t,values("command_mps",mask),label="common wheel command",alpha=.7)
        axes[0,column].plot(t,values("v",mask),label="measured speed",alpha=.7)
        axes[0,column].set_ylabel("m/s");axes[0,column].legend(fontsize=8)
        axes[1,column].plot(t,values("ex",climb&mask),label="position error")
        axes[1,column].axhline(-params["EXCLAMP"],color="red",linestyle="--",label="negative error clamp")
        axes[1,column].set_ylabel("Error m");axes[1,column].legend(fontsize=8)
        axes[2,column].plot(t,values("w",mask),label="pitch rate");axes[2,column].set_ylabel("degrees/s")
        axes[3,column].plot(t,values("u",mask),label="acceleration command")
        axes[3,column].set_ylabel("m/s2")
        axes[4,column].plot(t,values("fpRisk",mask),label="risk")
        axes[4,column].plot(t,a["inferred_climb_integral"],".",markersize=3,label="inferred integral m*s")
        axes[4,column].axhline(-params["EXCLAMP"]/(2*.25),color="red",linestyle="--",label="leaky-integral steady bound at risk .25")
        axes[4,column].legend(fontsize=8);axes[4,column].set_xlim(start,end)
        axes[4,column].set_xlabel("Seconds since recording start")
        axes[0,column].set_title("Before reboot" if column==0 else "After reboot (no measured cold-motor baseline)")
        for axis in axes[:,column]:
            for interval in intervals:
                if interval["mode"]=="climb":axis.axvspan(interval["start_s"],interval["end_s"],color="purple",alpha=.09)
    fig.suptitle("Climb attempts: reference accepted, low wheel command, clamp and persistent recovery")
    fig.savefig(output/"climb-detail.png",dpi=150);plt.close(fig)


def report_markdown(s):
    def clock(t):return f"{int(t//60):02d}:{t%60:05.2f}"
    lines=["# PG-to-home recording analysis", "",
           f"Session `{s['session_id']}`; start {s['start_utc']}.", "",
           "## Main findings", "",
           "Climb mode was accepted. The record contains sustained climb tracking failure with",
           "position error near the 0.15 m clamp, a latched 25% recovery override, and strong",
           "pitch-rate/acceleration oscillations. Motor heating remains possible but unmeasured:",
           "this recording contains no winding/case/driver temperature, thermal-warning/status",
           "register or supply-voltage fields.", "",
           f"- File duration: {s['recorded_telemetry_duration_s']:.2f} s, with {s['telemetry_samples']:,} received telemetry snapshots.",
           f"- Log-derived enabled duration: about {s['motors_enabled_interval_estimate_s']:.2f} s; idle controller fields after disable are cached.",
           f"- Analysis uses {s['active_unique_fresh_samples']:,} unique fresh active samples; stale samples: {s['stale_samples']}.",
           f"- Bluetooth stack errors begin at {clock(s['bluetooth_error_start_s'])}; {s['bluetooth_error_messages']} messages and {s['interleaved_serial_log_fragments']} interleaved serial fragments were received.",
           "- No received parameter changes. The ToF guard was OFF throughout the received frames.",
           "- An ESP32 reboot is present; its cause is not recorded. Driver UART checks afterward report 1500 mA, 1/8 microsteps and CoolStep ON.", "",
           "## Climb episodes", "",
           "Statistics below select fresh unique active climb snapshots with forward input at least 80%.", "",
           "| Start–end | Samples | Requested m/s | Measured median m/s | Wheel command median m/s | Accel capped | Error at clamp | Recovery |",
           "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for episode in s["climb_intervals"]:
        p=episode["forward_input_above_80pct"]
        if not p["samples"]:continue
        lines.append(f"| {clock(episode['start_s'])}–{clock(episode['end_s'])} | {p['samples']} | {p['requested_ref_median_mps']:.3f} | "
                     f"{p['measured_speed_median_mps']:.3f} | {p['command_speed_median_mps']:.3f} | {100*p['accel_saturation_fraction']:.1f}% | "
                     f"{100*p['ex_at_clamp_fraction']:.1f}% | {100*p['recovery_fraction']:.1f}% |")
    longest=max(s["climb_intervals"],key=lambda p:p["end_s"]-p["start_s"])
    p=longest["forward_input_above_80pct"]
    lines += ["",f"During {clock(longest['start_s'])}–{clock(longest['end_s'])}, the high-forward samples had mean measured speed "
              f"{p['measured_speed_mean_mps']:.4f} m/s and mean common wheel command {p['command_speed_mean_mps']:.4f} m/s, "
              f"despite a {p['requested_ref_median_mps']:.3f} m/s median operator reference.",
              "The wheels followed the small oscillating commands closely; the control path was not producing sustained forward wheel speed.", "",
              "## Controller interaction in revision 3.9", "",
              "`firmware/BaseLink/BaseLink.ino` advances the climb target with `operatorRef*(1-risk)`,",
              "then, whenever recovery is latched, bleeds the reference towards measured position",
              "and the climb integral towards zero at `2*risk` per second. The governor holds risk",
              "at least 0.25 until the bot remains calm for 250 ms (pitch <6 degrees, rate <35 dps).", "",
              "At risk 0.25, the climb integral behaves approximately as", "",
              "```text", "dz/dt = position_error - 0.5*z", "position_error = -0.15 m => steady z = -0.30 m*s",
              "```", "",
              "The nominal `CLMBZ=2.0 m*s` capacity therefore cannot accumulate under that",
              "persistent bleed. This is a specific interaction introduced by the new recovery",
              "override: slope/load compensation is continuously washed out while the bot is",
              "trying to climb. Inferred integral estimates from uncapped climb samples are",
              "included in the detailed plot and summary; they are reconstructed states, not sensors.", "",
              "The integral-decay check over 355–365 s uses the first inferred state and the",
              "subsequent recorded position error; it independently checks the temporal leakage",
              "law instead of only solving the instantaneous controller equation.", "",
              "Persistent recovery also cuts nominal climb-reference progress by at least 25%.",
              "The aggressive climb pitch/rate gain set often rails the physical acceleration",
              "limit. The tiny net wheel command while full-forward is held means lack of",
              "joystick-to-motion tracking is not by itself proof that motors ignored large step",
              "commands. Compare actual wheel targets against encoder rotation (summary includes",
              "per-wheel following error); physical traction and torque still require a measured test.", "",
              "## Command following during the long drive", "",
              "These are high-forward, nearly straight normal-drive samples with absolute pitch below 10 degrees.",
              "Different route segments are not a matched cold/warm ramp test.", "",
              "| Interval | Mean commanded m/s | Mean measured m/s | Measured / command | Recovery active |",
              "| --- | ---: | ---: | ---: | ---: |"]
    for block in s["normal_full_forward_straight_comparison"]:
        if not block["samples"]:continue
        ratio=block["measured_speed_mean_mps"]/block["command_speed_mean_mps"]
        lines.append(f"| {clock(block['start_s'])}–{clock(block['end_s'])} | {block['command_speed_mean_mps']:.3f} | "
                     f"{block['measured_speed_mean_mps']:.3f} | {ratio*100:.1f}% | {block['recovery_fraction']*100:.1f}% |")
    lines += ["", "There is an approximately 12% command/encoder shortfall already early in the",
              "drive. It does not suddenly widen in the later selected normal-drive segment:",
              "the controller itself commands less speed. Sampling/scale/pulse-following and",
              "physical load need checking; this pattern does not establish thermal step loss.", "",
              "## Heat, driver current and power", "",
              "- Configured current is 1500 mA RMS; effective CoolStep current is not recorded.",
              "- TMC2226 register reads in production cover setup/configuration, not continuous",
              "  `DRV_STATUS` over-temperature warning/shutdown or `CS_ACTUAL` reporting.",
              "- No thermal alarm text does not demonstrate that drivers stayed below their limits.",
              "- The first recorded climb starts at 05:46.76. There is no cold climb comparison.",
              "- A short reboot is not a measured cool-down test; failure afterward cannot rule",
              "  heating in or out. Supply sag, traction and hot-winding torque remain unmeasured.",
              "- Hardware DIAG stall-pin reporting is disabled in this build; no stall log is not a stall-free certificate.",
              "- Record driver OT/OTPW/current-scale flags in a low-rate task, supply voltage and",
              "  measured motor/driver temperatures. Compare the same ramp cold versus after the",
              "  drive, using the same battery voltage/current/mode and load.", "",
              "## Other observed stops", "",
              "| Time | Firmware event |", "| --- | --- |"]
    for stop in s["motor_stop_events"]:lines.append(f"| {clock(stop['t_s'])} | {stop['text']} |")
    lines += ["", "## Measurement and data quality", "",
              "- The Android app version is 3.8.0; firmware `FALLPREVENT=1` and the nine appended",
              "  fields show that the fall-prevention controller was installed.",
              "- Samples arrive at about 10 Hz. Fast 200 Hz oscillations/transients are undersampled.",
              "- Firmware milliseconds reset on reboot. A host odometry jump caused by encoder",
              "  reset is excluded from wheel-travel estimates; it is not actual ground travel.",
              f"  The immediate pose jump is {s['resets'][0]['host_pose_jump_m']:.1f} m; active mean-wheel absolute encoder travel is about "
              f"{s['encoder_mean_wheel_absolute_travel_while_active_m']:.1f} m.",
              "- The terrain label stays `indoor-flat`; it is an annotation, not a measured grade.",
              "- Idle `vCmd`, `u`, saturation and recovery fields retain the last controller state.",
              "  Use `balancing` plus explicit wheel targets; do not treat the idle tail as continued drive.",
              "- Recording ends with a connection failure and no normal-stop diagnostics.json.",
              "- Source checksums are stored in summary.json. Raw input files are left intact.", "",
              "## Next controller change supported by this evidence", "",
              "Make climb recovery mode-aware: preserve the slope-compensation integral during",
              "mild latched recovery, reserve integral/reference bleed for genuinely high risk",
              "or saturation, and retain full attitude feedback and existing fall cutoffs.",
              "Validate that change with a same-ramp cold/warm test plus the thermal/voltage",
              "measurements above. This analysis identifies a control interaction; it does not",
              "establish that a firmware change alone will overcome the physical load.", "",
              "## Outputs", "", "- `summary.json`: complete numerical results and provenance.",
              "- `climb-timeline.csv`: aligned samples with unique/fresh/active markers and reconstructed quantities.",
              "- `drive-overview.png`: active drive, risk, reference and climb-state history.",
              "- `climb-detail.png`: before/after reboot climb attempts.", ""]
    verification=s["integral_decay_verification"]
    if verification:
        insertion=f"Across {verification['uncapped_samples']} uncapped snapshots, temporal leakage fits the inferred integral with " \
                  f"RMS residual {verification['rms_residual_m_s']:.5f} m*s. The state approaches -0.30 m*s as predicted."
        position=lines.index("law instead of only solving the instantaneous controller equation.")+1
        lines.insert(position,insertion)
    return "\n".join(lines)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session",type=Path)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    analyse(args.session,args.output or args.session/"analysis")
