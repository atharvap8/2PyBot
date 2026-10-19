"""Publication figure pack and vector PDF for the PG-to-home recorder session.

Dependencies: numpy, matplotlib, ReportLab, PyMuPDF, pypdf, Pillow.
Outputs are separate from raw inputs. Figures: vector PDF/SVG + 350 dpi PNG.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import textwrap
import zipfile
from xml.sax.saxutils import escape

import pymupdf as fitz
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import findfont
from matplotlib.ticker import MaxNLocator
import numpy as np
from PIL import Image as PILImage, ImageDraw
from pypdf import PdfReader
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Flowable

from analyse_android_session import analyse, weighted_mean

BLUE="#0072B2"; ORANGE="#D55E00"; GREEN="#009E73"; PURPLE="#8E5BA6"; GRAY="#6E7884"
ROOT=Path(__file__).resolve().parents[2]


def build(session,out):
    assert session.is_dir()
    data=analyse(session,session/"analysis",verbose=False)
    a=data["arrays"];s=data["summary"];t=a["t"];active=data["active"];climb=data["climb"]
    normal=data["normal"];w=data["weights"];params=s["params"];intervals=data["intervals"]
    out.mkdir(parents=True,exist_ok=True);figdir=out/"figures";figdir.mkdir(exist_ok=True)
    plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10,"axes.titlesize":11,
                        "axes.labelsize":10,"legend.fontsize":8,"xtick.labelsize":8,"ytick.labelsize":8,
                        "axes.spines.top":False,"axes.spines.right":False,"axes.grid":True,"grid.alpha":.18,
                        "pdf.fonttype":42,"ps.fonttype":42,"svg.fonttype":"none","figure.dpi":120})
    catalog=[];checks=[];last_active=t[np.flatnonzero(active)[-1]]
    reboot_start=next(log["t_s"] for log in data["logs"] if "2PyBot BaseLink" in log["text"])

    def masked(name,mask=active):
        values=a[name].copy();values[~mask]=np.nan;return values
    def phase_bands(ax,start=0,end=None,events=False):
        end=last_active+3 if end is None else end
        for interval in intervals:
            if interval["mode"]=="climb":ax.axvspan(interval["start_s"],interval["end_s"],color=PURPLE,alpha=.08,zorder=0)
        if events:
            ax.axvline(reboot_start,color=GRAY,ls="--",lw=1)
            ax.axvline(s["bluetooth_error_start_s"],color=ORANGE,ls=":",lw=1)
            for event in s["motor_stop_events"]:ax.axvline(event["t_s"],color=ORANGE,ls=":",lw=.7,alpha=.5)
        ax.set_xlim(start,end);ax.set_xlabel("Elapsed recording time (s)")
    def save(fig,number,name,title,caption,kind="individual"):
        prefix=f"fig_{number:02d}_{name}"
        fig.canvas.draw();renderer=fig.canvas.get_renderer()
        # Catch labels/legends beyond the canvas before publishing.
        bbox=fig.bbox;off=[]
        ignored=set()
        for ax in fig.axes:
            for axis in (ax.xaxis,ax.yaxis):
                lo,hi=sorted(axis.get_view_interval())
                for tick in axis.get_major_ticks()+axis.get_minor_ticks():
                    if tick.get_loc()<lo-1e-9 or tick.get_loc()>hi+1e-9:
                        ignored.update((tick.label1,tick.label2))
        for artist in fig.findobj():
            if isinstance(artist,matplotlib.text.Text) and artist not in ignored and artist.get_visible() and artist.get_text():
                box=artist.get_window_extent(renderer)
                if box.width and box.height and (box.x0 < -2 or box.y0 < -2 or box.x1 > bbox.width+2 or box.y1 > bbox.height+2):
                    off.append(artist.get_text())
        assert not off,(prefix,"text outside figure",off)
        for extension in ("pdf","svg","png"):
            fig.savefig(figdir/(prefix+"."+extension),dpi=350,bbox_inches="tight",pad_inches=.08,
                        metadata={"Creator":"2PyBot offline session analysis"} if extension=="pdf" else None)
        checks.append({"figure":prefix,"text_within_canvas":True})
        catalog.append({"number":number,"id":prefix,"title":title,"caption":caption,"kind":kind})
        plt.close(fig)
    def single(title,size=(7.2,3.7)):
        fig,ax=plt.subplots(figsize=size,layout="constrained");ax.set_title(title);return fig,ax

    # 1. Acquisition quality: state timeline and observed transport intervals.
    fig,axes=plt.subplots(2,1,figsize=(7.2,5.0),layout="constrained",gridspec_kw={"height_ratios":[1,1.5]})
    for interval in intervals:
        color=PURPLE if interval["mode"]=="climb" else (GRAY if interval["mode"]=="idle" else BLUE)
        axes[0].broken_barh([(interval["start_s"],interval["end_s"]-interval["start_s"])],(.25,.5),facecolors=color)
    axes[0].plot(t[[row["stale"] for row in data["records"]]],np.full(s["stale_samples"],.9),"|",color=ORANGE,label="stale snapshots")
    axes[0].set(yticks=[],ylim=(0,1),xlabel="Elapsed time (s)",title="Normal/unknown (blue), climb (purple), idle (grey)")
    axes[0].legend(loc="lower right");axes[0].set_xlim(0,t[-1])
    dt=np.diff(t)*1000;assert np.all(dt>0)
    axes[1].hist(dt,bins=np.geomspace(dt.min(),dt.max(),55),color=BLUE,alpha=.8)
    axes[1].set_xscale("log");axes[1].set_yscale("log")
    axes[1].axvline(100,color=ORANGE,ls="--",label="nominal 10 Hz transport")
    axes[1].set(xlabel="Received snapshot interval (ms; logarithmic)",ylabel="Interval count (logarithmic)")
    axes[1].legend();fig.suptitle("Recording duration and sampling quality")
    save(fig,1,"acquisition_quality","Recording duration and sampling quality",
         "The 919.96 s telemetry trace contains 9,063 received snapshots, with median inter-arrival interval 101 ms. "
         "Colour denotes log-inferred operating mode. The 200 Hz controller is observed through approximately 10 Hz Android transport; "
         "this is not a lossless control-loop trace. Stale/cached boot-period controller states are excluded from active analyses.")

    # 2-6. Single-variable, individually reusable control traces.
    fig,ax=single("Operator reference and wheel-speed response")
    for field,label,color in (("driveReq","Operator reference",GRAY),("command_mps","Wheel command",ORANGE),("v","Encoder speed",BLUE)):
        ax.plot(t,masked(field),label=label,color=color,lw=.7,alpha=.85)
    phase_bands(ax,events=True);ax.set_ylabel("Speed (m s$^{-1}$)");ax.legend(ncol=3,loc="upper center")
    save(fig,2,"speed_tracking","Operator reference and wheel-speed response",
         "Fresh unique active samples. Operator reference is a requested control target, wheel command is the controller's common step-rate target divided by STEPSM, "
         "and measured speed is encoder-derived firmware velocity. Purple bands mark climb mode; vertical lines indicate reboot/BT errors/disable events. "
         "Wheel rotation is not externally measured ground speed.")
    for number,field,title,label,limits in (
        (3,"pF","Trim/sign-corrected body pitch","Pitch (degrees)",(-params["MAXTILT"],params["MAXTILT"])),
        (4,"w","Measured body pitch rate","Pitch rate (degrees s$^{-1}$)",(-params["MAXRATE"],params["MAXRATE"])),
        (5,"u","Common balance acceleration command","Acceleration command (m s$^{-2}$)",(-s["derived"]["AMAX"],s["derived"]["AMAX"]))):
        fig,ax=single(title);ax.plot(t,masked(field),color=BLUE,lw=.65)
        for limit in limits:ax.axhline(limit,color=ORANGE,ls="--",lw=.8)
        phase_bands(ax,events=True);ax.set_ylabel(label)
        if field=="u":caption="Acceleration-command samples and configured +/-3.927 m/s² actuator limit. This is an acceleration command, not measured ground acceleration or motor torque. Climb repeatedly reaches the clamp."
        else:caption=f"Fresh active {field} samples with configured {params['MAXTILT'] if field=='pF' else params['MAXRATE']:g} " \
                     f"{'degree fall' if field=='pF' else 'degree/s rate'} cutoff shown dashed. Recorded automatic cutoffs can occur between these approximately 10 Hz snapshots; threshold crossings may therefore be missed."
        save(fig,number,"pitch" if field=="pF" else "pitch_rate" if field=="w" else "acceleration_command",title,caption)
    fig,ax=single("Recovery governor and selected balance feedback")
    ax.plot(t,masked("fpRisk")*100,color=PURPLE,label="Recovery risk",lw=.9)
    ax.plot(t,masked("balScale")*100,color=BLUE,label="Selected attitude scale",lw=.7,alpha=.8)
    ax.axhline(25,color=GRAY,ls="--",lw=.8,label="Latched recovery floor")
    phase_bands(ax,events=True);ax.set_ylabel("Factor (%)");ax.set_ylim(-3,105);ax.legend(ncol=3,loc="upper center")
    save(fig,6,"recovery_feedback","Recovery governor and selected balance feedback",
         "Risk and selected pitch/rate feedback multiplier are different quantities. Mild recovery remains at 25% while full attitude feedback is selected during climb manoeuvres. "
         "Idle frozen risk/command values are excluded. Sampled risk does not measure temperature or physical failure probability.")

    longest=max(s["climb_intervals"],key=lambda r:r["end_s"]-r["start_s"])
    start,end=longest["start_s"],longest["end_s"]
    mask=active&(t>=start)&(t<end)
    fig,ax=single("Climb-mode reference versus actual wheel command")
    for field,label,color in (("driveRef","Shaped operator reference",GRAY),("command_mps","Wheel command",ORANGE),("v","Encoder speed",BLUE)):
        ax.plot(t,masked(field,mask),label=label,color=color,lw=1.0)
    phase_bands(ax,start,end);ax.set_ylabel("Speed (m s$^{-1}$)");ax.legend(loc="upper right")
    save(fig,7,"climb_wheel_response","Climb-mode reference versus actual wheel command",
         "Longest recorded climb episode, 352.301-377.283 s. Inputs vary, including a brief reversal. Among forward-input >=80% snapshots, the median operator reference is 0.250 m/s, "
         "mean commanded wheel speed 0.0100 m/s and mean measured speed 0.0095 m/s. The motor response follows the small commands; this does not prove that all physical torque/traction limits were absent.")
    fig,ax=single("Position tracking in the four climb episodes")
    for j,interval in enumerate(s["climb_intervals"]):
        mask=climb&(t>=interval["start_s"])&(t<interval["end_s"])
        ax.plot(t[mask]-interval["start_s"],a["ex"][mask]*100,label=f"Episode {j+1} ({interval['start_s']:.1f} s)",lw=1)
    ax.axhline(-params["EXCLAMP"]*100,color=GRAY,ls="--",label="Negative error clamp")
    ax.set(xlabel="Time since climb activation (s)",ylabel="Position error (cm)")
    ax.legend(loc="upper right",ncol=2,fontsize=7.5)
    save(fig,8,"climb_position_error","Position tracking in the four climb episodes",
         "Each episode is aligned to its logged climb activation, not to matched terrain or a repeated experimental protocol. Error is posM-xRef. "
         "Sustained negative error approaches the configured -15 cm clamp. Error approaching zero during input changes is not necessarily successful climbing.")
    check=s["integral_decay_verification"];rows=check["comparisons"]
    fig,axes=plt.subplots(2,1,figsize=(7.2,5),sharex=True,layout="constrained",gridspec_kw={"height_ratios":[2,1]})
    obs_t=np.array([r["t_s"] for r in rows]);obs=np.array([r["inferred_m_s"] for r in rows]);pred=np.array([r["leak_model_m_s"] for r in rows])
    axes[0].plot(obs_t,obs,"o",color=BLUE,ms=4,label="Reconstructed integral")
    axes[0].plot(obs_t,pred,color=ORANGE,label="Code-derived leakage law")
    axes[0].axhline(-.3,color=GRAY,ls="--",label="Steady-state bound at r=0.25, ex=-0.15 m")
    axes[0].set_ylabel("Integral z (m s)");axes[0].legend(loc="upper right",fontsize=7.5)
    axes[1].plot(obs_t,(obs-pred)*1000,"o-",ms=3,color=BLUE)
    axes[1].axhline(0,color=GRAY,lw=.7);axes[1].set(xlabel="Elapsed time (s)",ylabel="Residual (10$^{-3}$ m s)")
    fig.suptitle("Integral leakage: measured-data reconstruction and temporal consistency")
    save(fig,9,"integral_leakage_check","Integral leakage: reconstruction and temporal consistency",
         "Integral values are algebraically reconstructed only from uncapped climb samples and are not directly logged sensors. "
         "The model starts at the first inferred state and uses recorded error with dz/dt=ex-0.5z while risk=0.25. "
         f"There are {check['uncapped_samples']} identifiable snapshots; residual RMS is {check['rms_residual_m_s']:.5f} m s. "
         "This is a code-consistency check, not a fitted physical plant model or prediction of a modified controller's stability.")

    following=data["following"]
    fig,axes=plt.subplots(1,2,figsize=(7.2,3.7),layout="constrained",sharex=True,sharey=True)
    for ax,wheel in zip(axes,("left","right")):
        for label,selection,color in (("Normal",normal,BLUE),("Climb",climb,PURPLE)):
            selected=[r for r in following if selection[r["i"]]]
            x=np.array([r[wheel+"_cmd"] for r in selected]);y=np.array([r[wheel+"_measured"] for r in selected])
            ax.scatter(x,y,s=4,alpha=.35,color=color,label=f"{label} (n={len(x)})")
        ax.plot([-1.8,1.8],[-1.8,1.8],color=GRAY,ls="--",lw=.8)
        ax.set(xlim=(-1.8,1.8),ylim=(-1.8,1.8),title=wheel.capitalize()+" wheel",xlabel="Average target (m s$^{-1}$)")
        ax.legend(loc="upper left",fontsize=7)
    axes[0].set_ylabel("Encoder interval speed (m s$^{-1}$)")
    save(fig,10,"wheel_command_following","Per-wheel command following",
         "Encoder velocity is computed from adjacent same-boot odometry counts. The corresponding wheel target is approximated by the two received endpoint targets' average. "
         "Only fresh active pairs with positive MCU intervals <=0.5 s are included; asynchronous timestamps and approximately 10 Hz transport limit fast following-error accuracy. "
         "Dashed lines denote identity. Modes occur under different unmeasured terrain/load conditions, so this is descriptive, not a controlled mode comparison.")

    aggregate={}
    for name,mask in (("Normal",normal),("Climb",climb)):
        aggregate[name]={"n":int(mask.sum()),"recovery_pct":100*weighted_mean(a["fpRisk"][mask]>0,w[mask]),
                         "acceleration_cap_pct":100*weighted_mean(a["satA"][mask]>.5,w[mask]),
                         "speed_cap_pct":100*weighted_mean(a["satV"][mask]>.5,w[mask])}
    fig,ax=single("Sample-duration-weighted recovery and saturation")
    labels=["Recovery active","Acceleration capped","Speed capped"]
    for offset,name,color in ((-.18,"Normal",BLUE),(.18,"Climb",PURPLE)):
        vals=[aggregate[name][key] for key in ("recovery_pct","acceleration_cap_pct","speed_cap_pct")]
        bars=ax.bar(np.arange(3)+offset,vals,.34,color=color,label=f"{name} (n={aggregate[name]['n']})")
        ax.bar_label(bars,fmt="%.1f%%",padding=3,fontsize=8)
    ax.set(xticks=np.arange(3),xticklabels=labels,ylabel="Received active-sample duration (%)",ylim=(0,115))
    ax.legend(loc="upper right")
    save(fig,11,"recovery_saturation_summary","Sample-duration-weighted recovery and saturation",
         "Proportions are weighted by received sample intervals capped at 0.25 s, to avoid extending states across transport gaps. "
         "They describe observed snapshots, not exact 200 Hz controller occupancy. Modes have different references/load contexts. No confidence intervals or significance tests are asserted from autocorrelated samples.")

    fig,axes=plt.subplots(2,1,figsize=(7.2,5),sharex=True,layout="constrained")
    axes[0].plot(t,a["odom_ms"]/1000,color=BLUE,lw=.9);axes[0].set_ylabel("MCU odometry time (s)")
    axes[1].plot(t,a["x"],color=BLUE,label="Host odometry X",lw=.9)
    axes[1].plot(t,a["y"],color=GREEN,label="Host odometry Y",lw=.9)
    axes[1].set_ylabel("Host odometry coordinate (m)");axes[1].legend(loc="lower left")
    for ax in axes:ax.axvline(reboot_start,color=ORANGE,ls="--");ax.set_xlim(375,405)
    axes[1].set_xlabel("Elapsed recording time (s)");fig.suptitle("Reboot and odometry discontinuity")
    save(fig,12,"odometry_reset","Reboot and odometry discontinuity",
         f"The ESP32 reboot begins at {reboot_start:.3f} s. The immediate host-pose discontinuity at the first new sampled MCU epoch is "
         f"{s['resets'][0]['host_pose_jump_m']:.1f} m, an encoder-baseline/odometry artifact rather than measured ground travel. "
         "Epoch-boundary count differences are excluded from encoder travel/following calculations. No externally validated route or GPS ground truth was recorded.")

    # Combined overview: panels correspond to separately exported figures.
    fig,axes=plt.subplots(5,1,figsize=(7.2,8.4),sharex=True,layout="constrained")
    for name,label,color in (("driveReq","Operator",GRAY),("command_mps","Wheel target",ORANGE),("v","Encoder",BLUE)):
        axes[0].plot(t,masked(name),label=label,lw=.6,color=color)
    axes[0].set_ylabel("Speed\n(m/s)");axes[0].legend(ncol=3,loc="upper center",fontsize=7)
    for ax,name,label in zip(axes[1:4],("pF","w","u"),("Pitch\n(deg)","Rate\n(deg/s)","Command\n(m/s²)")):
        ax.plot(t,masked(name),color=BLUE,lw=.6);ax.set_ylabel(label)
    axes[4].plot(t,masked("fpRisk"),color=PURPLE,lw=.8);axes[4].set_ylabel("Recovery\nfactor")
    for ax in axes:phase_bands(ax,events=True);ax.label_outer()
    axes[-1].set_xlabel("Elapsed recording time (s)")
    for j,ax in enumerate(axes):ax.text(.005,.94,chr(97+j)+")",transform=ax.transAxes,va="top",fontweight="bold")
    fig.suptitle("Combined active-drive overview")
    save(fig,13,"combined_drive_overview","Combined active-drive overview",
         "(a) reference, commanded and encoder speed; (b) calibrated pitch; (c) pitch rate; (d) common acceleration command; (e) recovery state. "
         "Purple bands identify climb. Grey dashed line marks reboot, orange dotted lines mark Bluetooth-error onset and motor disables. "
         "Idle cached controller states are excluded. Use individual figures 2-6 for full-size axes and units.","combined")

    fig,axes=plt.subplots(5,2,figsize=(7.2,9.0),sharex="col",layout="constrained")
    for col,(lo,hi) in enumerate(((345,379),(394,429))):
        mask=active&(t>=lo)&(t<=hi)
        for name,label,color in (("driveRef","Reference",GRAY),("command_mps","Target",ORANGE),("v","Encoder",BLUE)):
            axes[0,col].plot(t,masked(name,mask),label=label,color=color,lw=.75)
        axes[0,col].set_title("Before reboot" if col==0 else "After reboot")
        axes[0,col].legend(ncol=1,loc="upper right",fontsize=6.5)
        axes[1,col].plot(t,masked("ex",climb&mask)*100,color=BLUE,lw=.75)
        axes[1,col].axhline(-15,color=ORANGE,ls="--",lw=.8)
        axes[2,col].plot(t,masked("w",mask),color=BLUE,lw=.6)
        axes[3,col].plot(t,masked("u",mask),color=BLUE,lw=.6)
        axes[4,col].plot(t,masked("inferred_climb_integral",mask),".",ms=2.5,color=BLUE)
        axes[4,col].axhline(-.3,color=ORANGE,ls="--",lw=.8)
        for row,ax in enumerate(axes[:,col]):
            phase_bands(ax,lo,hi);ax.label_outer();ax.text(.01,.94,f"{chr(97+row)}{col+1}",transform=ax.transAxes,va="top",fontsize=8)
        axes[4,col].set_xlabel("Elapsed time (s)")
    for row,label in enumerate(("Speed\n(m/s)","Error\n(cm)","Rate\n(deg/s)","Command\n(m/s²)","Inferred z\n(m s)")):axes[row,0].set_ylabel(label)
    fig.suptitle("Combined climb diagnosis: reference, motion and integral")
    save(fig,14,"combined_climb_diagnostics","Combined climb diagnosis before and after reboot",
         "The four climb activations are shaded purple. Columns are two chronological windows, not independent matched trials. "
         "Rows show operator/target/measured speeds, position error, pitch rate, acceleration command and inferred integral. "
         "The -0.30 m s dashed integral bound applies only while r=0.25 and ex=-0.15 m; it does not apply when recovery clears. "
         "A reboot is not a measured cold-motor comparison. All integral points are reconstructed from uncapped climb snapshots.","combined")

    fig,axes=plt.subplots(2,2,figsize=(7.2,6),layout="constrained")
    blocks=s["normal_full_forward_straight_comparison"];x=np.arange(len(blocks))
    times=[f"{b['start_s']:g}-{b['end_s']:g}" for b in blocks]
    axes[0,0].bar(x-.17,[b["command_speed_mean_mps"] for b in blocks],.32,color=ORANGE,label="Wheel command")
    axes[0,0].bar(x+.17,[b["measured_speed_mean_mps"] for b in blocks],.32,color=BLUE,label="Encoder")
    axes[0,0].set(xticks=x,xticklabels=times,ylabel="Mean speed (m/s)",xlabel="Normal-drive window (s)",title="a) Normal-drive means")
    axes[0,0].tick_params(axis="x",labelrotation=25);axes[0,0].legend(fontsize=7)
    ratios=[100*b["measured_speed_mean_mps"]/b["command_speed_mean_mps"] for b in blocks]
    bars=axes[0,1].bar(x,ratios,color=BLUE);axes[0,1].bar_label(bars,fmt="%.1f",fontsize=8,padding=2)
    axes[0,1].set(xticks=x,xticklabels=times,ylim=(0,110),ylabel="Measured / commanded mean (%)",title="b) Approximate mean following")
    axes[0,1].tick_params(axis="x",labelrotation=25)
    episodes=s["climb_intervals"]
    vals=[e["forward_input_above_80pct"]["accel_saturation_fraction"]*100 for e in episodes]
    bars=axes[1,0].bar(np.arange(4),vals,color=PURPLE);axes[1,0].bar_label(bars,fmt="%.1f%%",fontsize=8,padding=2)
    axes[1,0].set(xticks=np.arange(4),xticklabels=["1","2","3","4"],ylim=(0,105),xlabel="Recorded climb episode",ylabel="Acceleration cap occupancy (%)",title="c) High-forward climb samples")
    axes[1,1].axis("off")
    text=f"d) Data-quality context\n\n{s['telemetry_samples']:,} snapshots (~10 Hz)\n{s['active_unique_fresh_samples']:,} analysed active samples\n{s['stale_samples']} stale snapshots\n251 Bluetooth error messages\n1 MCU reboot\n4 automatic motor cutoffs\n\nTemperatures / voltage: unrecorded"
    axes[1,1].text(.02,.95,text,transform=axes[1,1].transAxes,va="top",fontsize=10)
    save(fig,15,"combined_analytics_dashboard","Combined descriptive analytics dashboard",
         "(a,b) selected high-forward, nearly straight normal-drive window means and measured/command ratio; (c) observed acceleration-cap proportion in high-forward climb snapshots; "
         "(d) data-quality context. Windows/episodes have different unmeasured terrain and load. Bars are descriptive summaries; no repeat-trial confidence intervals, thermal diagnosis or causal mode-performance comparison is claimed.","combined")

    # Machine-readable, traceable data tables for replotting and manuscript use.
    with (out/"processed_samples.csv").open("w",newline="",encoding="utf-8") as file:
        fieldnames=["seq","elapsed_s","mcu_epoch","log_inferred_mode","stale","unique_debug","fresh","active_analysis"]+list(a.keys())[2:]
        writer=csv.writer(file);writer.writerow(fieldnames)
        for i,row in enumerate(data["records"]):
            writer.writerow([row["seq"],row["t"],row["epoch"],row["mode"],row["stale"],bool(data["unique"][i]),bool(data["fresh"][i]),bool(active[i])]
                            +[a[key][i] for key in list(a.keys())[2:]])
    with (out/"climb_episode_metrics.csv").open("w",newline="",encoding="utf-8") as file:
        keys=["start_s","end_s","samples","requested_ref_median_mps","measured_speed_mean_mps","command_speed_mean_mps",
              "accel_saturation_fraction","recovery_fraction","ex_at_clamp_fraction"]
        writer=csv.DictWriter(file,fieldnames=keys);writer.writeheader()
        for episode in episodes:
            p=episode["forward_input_above_80pct"]
            writer.writerow({key:episode[key] if key in ("start_s","end_s") else p[key] for key in keys})
    with (out/"figure_index.csv").open("w",newline="",encoding="utf-8") as file:
        writer=csv.DictWriter(file,fieldnames=["number","id","title","caption","kind"]);writer.writeheader();writer.writerows(catalog)
    (out/"figure_captions.md").write_text("# Reusable figure captions\n\n"+"\n\n".join(f"## Figure {c['number']:02d}: {c['title']}\n\n{c['caption']}" for c in catalog)+"\n",encoding="utf-8")
    shutil.copy2(session/"analysis/summary.json",out/"analysis_summary.json")
    report=make_report(out,data,catalog,aggregate,reboot_start)
    validation=validate_report(out,report,catalog,checks)
    readme=f"""# 2PyBot PG-to-home publication figure pack

Main report: `{report.name}`. There are {len(catalog)} numbered figures, including
individual traces and combined panels. Each is supplied as vector PDF, editable
SVG with live text, and 350 dpi PNG. Fonts in PDF are embedded. Figures are
7.2 inches wide before cropping; layout and font sizes can be changed in the
generator for a venue's column-width requirements.

Use `figure_captions.md` and `figure_index.csv` for captions and selection.
`processed_samples.csv` includes all aligned samples plus explicit exclusion
flags; it is not a declaration that every row is independent/valid. Units match
recorded debug fields: pF degrees, w degrees/s, u m/s², v m/s, ex metres,
vCmd/steer/wheelL/wheelR steps/s, risks/scales dimensionless. *_mps fields are
converted m/s; inferred_climb_integral is m*s. Inferred state is not a sensor.

This is one observational session with unmeasured terrain/load, no thermal or
supply-voltage channels, and ~10 Hz transport of a 200 Hz control loop. The report
documents automatic cutoffs, resets and exclusions. Do not present mode groups
as matched repeat trials or claim overheating, an externally validated route,
or a before/after controller improvement from this data alone.

Reproduce from the repository root:
```
python scripts/diagnostics/publication_session_report.py "{session}" --output "{out}"
```

Dependencies: numpy matplotlib reportlab pymupdf pypdf Pillow.
Raw inputs are not included in the bundle; their SHA-256 digests are in the
analytics JSON. Reproducibility scripts are included in `scripts/`.
"""
    (out/"README.md").write_text(readme,encoding="utf-8")
    (out/"validation.json").write_text(json.dumps(validation,indent=2)+"\n",encoding="utf-8")
    bundle=out/"2PyBot-PG-to-home-publication-pack.zip"
    with zipfile.ZipFile(bundle,"w",compression=zipfile.ZIP_DEFLATED) as archive:
        for path in out.rglob("*"):
            if path.is_file() and path!=bundle and "previews" not in path.parts and not path.name.startswith("_report"):
                archive.write(path,path.relative_to(out))
        for name in ("publication_session_report.py","analyse_android_session.py"):
            archive.write(Path(__file__).parent/name,"scripts/"+name)
    with zipfile.ZipFile(bundle) as archive:assert archive.testzip() is None
    assert all(hashlib.sha256((session/name).read_bytes()).hexdigest()==value for name,value in s["source_files_sha256"].items())
    print(json.dumps({"report":str(report),"pages":validation["pages"],"figure_count":len(catalog),
                      "bundle":str(bundle),"raw_input_hashes_unchanged":True},indent=2))


class VectorFigure(Flowable):
    def __init__(self,path,placements,width,maxheight=505):
        super().__init__();self.path=path;self.placements=placements
        with fitz.open(path) as pdf:rect=pdf[0].rect
        scale=min(width/rect.width,maxheight/rect.height)
        self.width=rect.width*scale;self.height=rect.height*scale
    def draw(self):
        x,y=self.canv.absolutePosition(0,0)
        self.placements.append({"page":self.canv.getPageNumber()-1,"x":x,"y":y,"width":self.width,"height":self.height,"path":str(self.path)})


def make_report(out,data,catalog,aggregate,reboot_start):
    s=data["summary"];params=s["params"];placements=[];styles=getSampleStyleSheet()
    pdfmetrics.registerFont(TTFont("DV",findfont("DejaVu Sans")))
    pdfmetrics.registerFont(TTFont("DV-Bold",findfont(matplotlib.font_manager.FontProperties(family="DejaVu Sans",weight="bold"))))
    pdfmetrics.registerFontFamily("DV",normal="DV",bold="DV-Bold",italic="DV",boldItalic="DV-Bold")
    for name in ("Normal","BodyText","Heading1","Heading2","Heading3","Title"):
        styles[name].fontName="DV-Bold" if name.startswith("Heading") or name=="Title" else "DV"
    styles["BodyText"].fontSize=10;styles["BodyText"].leading=15;styles["BodyText"].spaceAfter=8
    styles["Heading1"].fontSize=18;styles["Heading1"].leading=23;styles["Heading1"].textColor=colors.HexColor("#16354a")
    styles["Heading2"].fontSize=13;styles["Heading2"].leading=18
    styles.add(ParagraphStyle("Caption",fontName="DV",fontSize=9,leading=13,spaceAfter=8))
    styles.add(ParagraphStyle("Small",fontName="DV",fontSize=8,leading=11))
    styles.add(ParagraphStyle("CoverSubtitle",fontName="DV",fontSize=13,leading=19,alignment=TA_CENTER))
    width,height=A4;margin=42;available=width-2*margin
    story=[]
    def para(text,style="BodyText"):story.append(Paragraph(text,styles[style]))
    def table(rows,widths=None):
        wrapped=[[Paragraph(escape(str(cell)),styles["Small"]) for cell in row] for row in rows]
        item=Table(wrapped,colWidths=widths or [available/len(rows[0])]*len(rows[0]),repeatRows=1,hAlign="LEFT")
        item.setStyle(TableStyle([("BACKGROUND",(0,0),(-1,0),colors.HexColor("#E7EEF3")),("VALIGN",(0,0),(-1,-1),"TOP"),
                                  ("TOPPADDING",(0,0),(-1,-1),6),("BOTTOMPADDING",(0,0),(-1,-1),6),
                                  ("LINEBELOW",(0,0),(-1,0),.7,colors.HexColor("#617586")),
                                  ("ROWBACKGROUNDS",(0,1),(-1,-1),[colors.white,colors.HexColor("#F5F7F9")])]))
        story.append(item);story.append(Spacer(1,10))
    para("2PyBot", "Title");story.append(Spacer(1,9))
    para("PG-to-home drive and climb analysis", "Title")
    para("Publication-oriented figures and descriptive analytics", "CoverSubtitle")
    story.append(Spacer(1,20))
    para(f"<b>Dataset:</b> {escape(s['session_id'])}<br/><b>Recording start:</b> {escape(s['start_utc'])}<br/>"
         f"<b>Scope:</b> one observational session of the existing encoder-equipped stepper platform.")
    table([["Telemetry span","Received samples","Analysed active samples","Enabled time"],
           ["15 min 19.96 s",f"{s['telemetry_samples']:,}",f"{s['active_unique_fresh_samples']:,}","6 min 49.34 s"]])
    para("Executive summary","Heading1")
    para("The recording shows sustained climb tracking failure while the firmware accepts climb mode and forward requests. "
         "The strongest identified controller interaction is a persistently latched mild recovery state that leaks climb's slope-compensation integral. "
         "Reconstructed state and its temporal decay agree with the implementation. The wheels follow the small oscillating commands during failed climbing; "
         "they are not receiving a sustained 0.25 m/s common wheel command in that interval.")
    para("Motor or driver heating remains possible, but no temperatures, thermal-status flags, actual current scale or battery voltage were recorded. "
         "The session also includes four automatic motor cutoffs, Bluetooth stack/serial interleaving faults and an ESP32 reboot with odometry discontinuity.")
    para("Evidence status","Heading2")
    para("Figures are prepared for reuse, with vector PDF/SVG and high-resolution PNG exports. This report does not supply a controlled cold/warm comparison, "
         "repeat-trial significance test, thermal diagnosis, or externally validated route. Use it as a documented case study and a basis for the next experiment.")
    story.append(PageBreak())

    para("1. Acquisition and analysis methods","Heading1")
    para("The raw Android recorder supplies ordered WebSocket/HTTP events, monotonic reception times, wall times, firmware milliseconds, "
         "encoder counts, controller debug fields and initial parameters. The CSV and JSONL telemetry sequences were cross-checked; they are two representations of the same samples.")
    table([["Processing item","Definition"],
           ["Observation rate","Median received interval 101 ms (~9.90 Hz). Firmware loop nominally 200 Hz; odometry 50 Hz and debug 100 Hz before transport."],
           ["Active sample","Fresh non-stale, unique debug timestamp within its MCU boot epoch, balancing flag set, and log-inferred mode not idle. O/D clock separation must stay within +/-250 ms."],
           ["Mode reconstruction","Accepted firmware HOLD/CLIMB/STIFF, balancing, disable and boot events. Initial mode is unknown; high-forward initial samples use the normal drive-reference scale."],
           ["Proportion weights","Following received duration capped at 0.25 s. These approximate observed occupancy, not exact loop occupancy."],
           ["Wheel following","Adjacent same-epoch encoder count differences divided by COUNTSM and MCU odometry interval; endpoint wheel targets averaged and divided by STEPSM."],
           ["Integral reconstruction","Invert uncapped climb LQI only when |vCmd| <0.88 MAXSPD. Quantized debug fields introduce reconstruction error."],
           ["Data independence","Snapshots are serially correlated. No p-values, independent-trial confidence intervals or causal mode comparisons are claimed."]],[125,available-125])
    para("Key formulas","Heading2")
    para("v<sub>L</sub> = ΔencL / (COUNTSM × Δt); v<sub>R</sub> = ΔencR / (COUNTSM × Δt).<br/>"
         "v<sub>target</sub> = wheelTarget / STEPSM.<br/>"
         "z = (-u - C1·ex - C2·v - C3·sf·θ - C4·sf·ω) / C5, using radians for θ and ω.<br/>"
         "For constant mild recovery r=0.25: dz/dt ≈ ex - 0.5z; ex=-0.15 m implies z<sub>ss</sub>=-0.30 m s.")
    story.append(PageBreak())

    para("2. Quantitative results","Heading1")
    para("Climb results below select unique fresh active snapshots with forward input ≥80%. The intervals were not matched by grade, traction, load or motor temperature.")
    rows=[["Episode / time (s)","n","Reference median (m/s)","Wheel mean (m/s)","Encoder mean (m/s)","Accel cap (%)","Recovery (%)"]]
    for j,e in enumerate(s["climb_intervals"]):
        p=e["forward_input_above_80pct"]
        rows.append([f"{j+1}: {e['start_s']:.1f}-{e['end_s']:.1f}",p["samples"],f"{p['requested_ref_median_mps']:.3f}",
                     f"{p['command_speed_mean_mps']:.4f}",f"{p['measured_speed_mean_mps']:.4f}",
                     f"{100*p['accel_saturation_fraction']:.1f}",f"{100*p['recovery_fraction']:.1f}"])
    table(rows,[88,30,77,77,77,76,available-425])
    para("The longest climb interval has approximately 0.0100 m/s mean commanded and 0.0095 m/s measured speed during high-forward snapshots, "
         "against a 0.250 m/s median operator reference. Its recovery state stays at 25%, and acceleration is capped about 69.8% of observed high-forward duration. "
         "The record therefore identifies both loss of slow tracking authority and substantial fast balance-control activity.")
    check=s["integral_decay_verification"]
    para(f"The integral leakage-law check contains {check['uncapped_samples']} identifiable snapshots in 355-365 s. With its initial state anchored to the first reconstructed value, "
         f"the code-derived temporal law gives residual RMS {check['rms_residual_m_s']:.5f} m s. The nominal CLMBZ limit is 2.0 m s, but persistent mild leakage "
         "limits steady compensation to approximately 0.30 m s at the recorded position clamp. This is evidence of the controller interaction, not proof of a revised controller's physical success.")
    para("Normal-drive following context","Heading2")
    rows=[["Selected window (s)","Command mean (m/s)","Encoder mean (m/s)","Encoder / command (%)","Recovery (%)"]]
    for b in s["normal_full_forward_straight_comparison"]:
        rows.append([f"{b['start_s']:g}-{b['end_s']:g}",f"{b['command_speed_mean_mps']:.3f}",f"{b['measured_speed_mean_mps']:.3f}",
                     f"{100*b['measured_speed_mean_mps']/b['command_speed_mean_mps']:.1f}",f"{100*b['recovery_fraction']:.1f}"])
    table(rows)
    para("An approximately 12% mean command/encoder shortfall is already present early and is similar later in selected normal-drive segments. "
         "Because both the commanded speed and physical route change, this is not a controlled test of thermal degradation. "
         "Pulse following, encoder scale and load remain candidates for the pre-existing discrepancy.")
    story.append(PageBreak())

    para("3. Hardware settings and event chronology","Heading1")
    table([["Recorded setting","Value"],["Configured motor current",f"{params['CURRENT']:g} mA RMS"],
           ["Driver mode","TMC2226; StealthChop; CoolStep enabled; 1/8 microsteps after reboot readback"],
           ["Conversions",f"STEPSM={s['derived']['STEPSM']:.2f}; COUNTSM={s['derived']['COUNTSM']:.2f}"],
           ["Controller limits",f"MAXSPD={params['MAXSPD']:g} steps/s; AMAX={s['derived']['AMAX']:.3f} m/s²"],
           ["Drive/climb reference",f"DRIVEV={params['DRIVEV']:.6f} m/s; CLMBV={params['CLMBV']:.2f} m/s"],
           ["Position/integral bounds",f"EXCLAMP={params['EXCLAMP']:.2f} m; CLMBZ={params['CLMBZ']:.1f} m s"],
           ["Firmware/app identification","Android app 3.8.0; FALLPREVENT capability and debug schema consistent with deployed 3.9 governor"],
           ["Recorded parameter changes","None received; ToF forward-reference guard OFF throughout"]],[155,available-155])
    rows=[["Elapsed time (s)","Recorded event"]]
    for event in s["motor_stop_events"]:rows.append([f"{event['t_s']:.3f}",event["text"]])
    rows.extend([[f"{s['bluetooth_error_start_s']:.3f}","Bluetooth error stream begins; 251 messages received"],
                 [f"{reboot_start:.3f}","ESP32 reboot begins; cause unrecorded"],
                 [f"{s['resets'][0]['first_new_telemetry_s']:.3f}","First sampled new firmware epoch; immediate host pose jump 212.6 m"],
                 [f"{s['last_event_s']:.3f}","Recording ends with connection failure; normal-stop diagnostics absent"]])
    table([rows[0]]+sorted(rows[1:],key=lambda r:float(r[0])),[100,available-100])
    para("Existing stepper-platform results should not be used as measured performance of the ordered M0601 hub motors. "
         "The recorder has no motor torque/current-scale or temperature channels, and hardware DIAG stall-pin logging is disabled in the examined build.")
    story.append(PageBreak())

    para("4. Interpretation, validity and next experiment","Heading1")
    para("<b>Supported interpretation:</b> mild recovery is persistently latched during the failed climbs, continuously bleeding the integral needed for slow slope/load compensation. "
         "The low net wheel command and temporal integral reconstruction support this mechanism. Fast acceleration saturation and strong pitch-rate oscillation coexist.")
    para("<b>Unresolved hypothesis:</b> motors or drivers could be heating, or the supply could sag. No temperature, thermal flags, actual current scale or battery voltage was recorded. "
         "Climb first appears after 346.758 s, so there is no cold-climb baseline; the short reboot is not a measured cool-down test.")
    table([["Potential claim","What this session supports"],["Controller/climb coupling","A reproducible code/data consistency mechanism; effect on modified-controller stability requires a new trial."],
           ["Heating caused the failure","Unresolved. Thermal/current/supply channels and matched cold/warm tests are needed."],
           ["Climb mode improves/worsens performance","Descriptive differences only. Modes have different references and unmeasured terrain/load."],
           ["191 m distance","Approximate mean-wheel absolute encoder travel while enabled, excluding reset edges; tyre slip and ground travel are unvalidated."],
           ["Trajectory/location accuracy","No external truth. A reboot injects an odometry discontinuity; the initial recorder terrain label does not measure slope."],
           ["High-frequency dynamic stability","The ~10 Hz transport undersamples the 200 Hz loop; apparent sampled oscillations cannot establish the true high-frequency spectrum."]],[170,available-170])
    para("Recommended repeat-test protocol","Heading2")
    para("Use a measured ramp angle and documented surface/load. Record a cold baseline and a post-drive repetition with matched battery voltage, current and speed mode. "
         "Add low-rate driver OT/OTPW/current-scale status, supply voltage and motor/driver temperature measurements outside the time-critical balance loop. "
         "Log explicit control mode and integral/reference state, and capture full-rate telemetry plus synchronized video/ground-motion reference. "
         "Test a climb-aware recovery change while retaining full attitude feedback, steering headroom and existing cutoff limits. Repeat independent trials and report trial-level variability.")
    para("Suggested manuscript use","Heading2")
    para("Figures 13-15 are overview/diagnosis panels; figures 1-12 are individually reusable. Describe this recording as an observational diagnostic case. "
         "For a performance paper, add an independently repeated baseline, revised controller, measured grade/load, thermal instrumentation and an explicit treatment of resets/dropouts.")
    story.append(PageBreak())
    for entry in catalog:
        para(f"Figure {entry['number']:02d}. {escape(entry['title'])}","Heading2")
        story.append(VectorFigure(out/"figures"/(entry["id"]+".pdf"),placements,available,maxheight=500))
        story.append(Spacer(1,10));para(escape(entry["caption"]),"Caption")
        para(f"Reusable files: {escape(entry['id'])}.pdf / .svg / .png","Small")
        story.append(PageBreak())
    para("Appendix: provenance and reproducibility","Heading1")
    para("Raw source-file SHA-256 hashes","Heading2")
    for name,digest in s["source_files_sha256"].items():
        para(escape(name),"Heading3");para(digest[:32]+"<br/>"+digest[32:],"Small")
    para("Analysis source references","Heading2")
    for name in ("firmware/BaseLink/BaseLink.ino","firmware/BaseLink/fall_prevention.h","firmware/BaseLink/stepper_control.cpp"):
        path=ROOT/name;digest=hashlib.sha256(path.read_bytes()).hexdigest()
        para(escape(name),"Small");para(digest[:32]+"<br/>"+digest[32:],"Small")
    para("The generator, raw analytic summary, aligned samples, episode metrics and caption index are included in the figure pack. "
         "Raw input files remain in the supplied recorder directory. The processed table retains flags so exclusions are reproducible; every row is not an independent experimental trial.")
    base=out/"_report_layout.pdf"
    def footer(canvas,doc):
        canvas.saveState();canvas.setFont("DV",8);canvas.setFillColor(colors.HexColor("#617586"))
        canvas.drawString(margin,22,"2PyBot | PG-to-home observational session")
        canvas.drawRightString(width-margin,22,str(doc.page));canvas.restoreState()
    doc=SimpleDocTemplate(str(base),pagesize=A4,leftMargin=margin,rightMargin=margin,topMargin=40,bottomMargin=40,
                          title="2PyBot PG-to-home: publication figures and analytics",author="2PyBot project",
                          subject="Single-session drive/climb diagnostic analysis; thermal contribution unmeasured")
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    result=out/"2PyBot-PG-to-home-analysis-and-figures.pdf"
    with fitz.open(base) as pdf:
        for placement in placements:
            page=pdf[placement["page"]];h=page.rect.height
            rect=fitz.Rect(placement["x"],h-placement["y"]-placement["height"],
                           placement["x"]+placement["width"],h-placement["y"])
            assert rect.x0>=0 and rect.y0>=0 and rect.x1<=page.rect.width and rect.y1<=page.rect.height
            with fitz.open(placement["path"]) as source:page.show_pdf_page(rect,source,0,keep_proportion=True)
        pdf.save(result,garbage=4,deflate=True)
    base.unlink()
    return result


def validate_report(out,report,catalog,checks):
    reader=PdfReader(report);assert len(reader.pages)>=len(catalog)+5
    with fitz.open(report) as pdf:
        text="\n".join(page.get_text() for page in pdf)
    assert all(f"Figure {entry['number']:02d}." in text for entry in catalog)
    assert "0.0095" in text and "0.00148" in text and "temperature" in text.lower()
    assert "nan" not in text.lower().split()
    previews=out/"previews";previews.mkdir(exist_ok=True)
    thumbs=[];bounds=[];font_count=0
    with fitz.open(report) as doc:
        for number,page in enumerate(doc):
            assert page.get_text().strip(),("empty page",number+1)
            font_count+=len(page.get_fonts())
            for block in page.get_text("dict")["blocks"]:
                for line in block.get("lines",[]):
                    for span in line["spans"]:
                        box=fitz.Rect(span["bbox"])
                        if box.x0<-.5 or box.y0<-.5 or box.x1>page.rect.width+.5 or box.y1>page.rect.height+.5:
                            bounds.append({"page":number+1,"text":span["text"],"bbox":list(box)})
            pix=page.get_pixmap(matrix=fitz.Matrix(.5,.5))
            image=PILImage.frombytes("RGB",[pix.width,pix.height],pix.samples)
            image.thumbnail((260,375));thumbs.append(image)
        for number in (0,4,17,len(doc)-1):
            page=doc[min(number,len(doc)-1)];page.get_pixmap(matrix=fitz.Matrix(1.6,1.6)).save(previews/f"page-{number+1:02d}.png")
    assert not bounds,bounds
    sheet=PILImage.new("RGB",(4*285,((len(thumbs)+3)//4)*410),"#dce1e6");draw=ImageDraw.Draw(sheet)
    for i,image in enumerate(thumbs):
        x=(i%4)*285+12;y=(i//4)*410+10;sheet.paste(image,(x,y));draw.text((x,y+380),f"Page {i+1}",fill="black")
    sheet.save(previews/"contact-sheet.png")
    figures=[]
    for entry in catalog:
        prefix=out/"figures"/entry["id"]
        with PILImage.open(prefix.with_suffix(".png")) as image:
            image.verify()
        assert PdfReader(prefix.with_suffix(".pdf")).pages
        assert "<svg" in prefix.with_suffix(".svg").read_text(encoding="utf-8")
        figures.append({"id":entry["id"],"formats":["pdf","svg","png"],
                        "pdf_sha256":hashlib.sha256(prefix.with_suffix(".pdf").read_bytes()).hexdigest()})
    return {"pages":len(reader.pages),"figure_count":len(catalog),"all_figures_in_report":True,
            "text_inside_page_bounds":True,"font_resources":font_count,"figure_canvas_checks":checks,
            "figures":figures,"report_sha256":hashlib.sha256(report.read_bytes()).hexdigest(),
            "vector_figures_embedded":True,"png_resolution_dpi":350}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session",type=Path)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args();build(args.session,args.output or args.session/"publication")
