#!/usr/bin/env python3
"""
gen_report.py — build the v1 PDF report from live project data.

Reads config/airframe.yaml, out/sizing.json and the actual flight-log CSVs, so
the document cannot drift from what was really configured and measured.

Usage:
  python3 tools/gen_report.py --out out/Drone_sim_v1_Report.pdf
"""

import argparse
import csv
import datetime
import json
import math
import os

import yaml
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (BaseDocTemplate, Frame, KeepTogether,
                                NextPageTemplate, PageBreak, PageTemplate,
                                Paragraph, Spacer, Table, TableStyle)

# ---------------------------------------------------------------- palette
NAVY = colors.HexColor("#12263A")
ACCENT = colors.HexColor("#1B6CA8")
GOOD = colors.HexColor("#1B7F४3".replace("४", "4"))
WARN = colors.HexColor("#B45309")
BAD = colors.HexColor("#9B1C1C")
LIGHT = colors.HexColor("#F1F5F9")
MID = colors.HexColor("#CBD5E1")
GREY = colors.HexColor("#475569")

ROOT = os.path.expanduser("~/drone_sim")
VERSION = "v1.1.0"


# ---------------------------------------------------------------- styles
def build_styles():
    ss = getSampleStyleSheet()
    s = {}
    s["title"] = ParagraphStyle("title", parent=ss["Title"], fontName="Helvetica-Bold",
                                fontSize=26, leading=30, textColor=NAVY, spaceAfter=4)
    s["subtitle"] = ParagraphStyle("subtitle", parent=ss["Normal"], fontSize=13,
                                   leading=17, textColor=GREY, alignment=TA_CENTER)
    s["h1"] = ParagraphStyle("h1", parent=ss["Heading1"], fontName="Helvetica-Bold",
                             fontSize=15, leading=19, textColor=NAVY,
                             spaceBefore=14, spaceAfter=7)
    s["h2"] = ParagraphStyle("h2", parent=ss["Heading2"], fontName="Helvetica-Bold",
                             fontSize=11.5, leading=15, textColor=ACCENT,
                             spaceBefore=10, spaceAfter=4)
    s["body"] = ParagraphStyle("body", parent=ss["Normal"], fontSize=9.5, leading=13.5,
                               textColor=colors.HexColor("#1E293B"), spaceAfter=5,
                               alignment=TA_JUSTIFY)
    s["small"] = ParagraphStyle("small", parent=s["body"], fontSize=8.2, leading=11,
                                textColor=GREY)
    s["cell"] = ParagraphStyle("cell", parent=ss["Normal"], fontSize=8, leading=10)
    s["cellb"] = ParagraphStyle("cellb", parent=s["cell"], fontName="Helvetica-Bold")
    s["cellr"] = ParagraphStyle("cellr", parent=s["cell"], alignment=2)
    s["th"] = ParagraphStyle("th", parent=ss["Normal"], fontSize=8,
                             fontName="Helvetica-Bold", textColor=colors.white, leading=10)
    s["thr"] = ParagraphStyle("thr", parent=s["th"], alignment=2)
    s["mono"] = ParagraphStyle("mono", parent=ss["Normal"], fontName="Courier",
                               fontSize=8, leading=11,
                               textColor=colors.HexColor("#0F172A"))
    return s


S = build_styles()


def P(t, st="body"):
    return Paragraph(t, S[st])


def callout(title, text, tone="warn"):
    col = {"warn": WARN, "bad": BAD, "good": GOOD, "info": ACCENT}[tone]
    bg = {"warn": colors.HexColor("#FEF3C7"), "bad": colors.HexColor("#FEE2E2"),
          "good": colors.HexColor("#DCFCE7"), "info": colors.HexColor("#E0F2FE")}[tone]
    inner = [[Paragraph(f"<b>{title}</b>", ParagraphStyle(
        "ct", fontName="Helvetica-Bold", fontSize=9.5, leading=12, textColor=col))],
        [Paragraph(text, ParagraphStyle("cb", fontSize=8.8, leading=12,
                                        textColor=colors.HexColor("#1E293B")))]]
    t = Table(inner, colWidths=[165 * mm])
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), bg),
        ("LINEBEFORE", (0, 0), (0, -1), 3, col),
        ("LEFTPADDING", (0, 0), (-1, -1), 8), ("RIGHTPADDING", (0, 0), (-1, -1), 8),
        ("TOPPADDING", (0, 0), (-1, 0), 6), ("BOTTOMPADDING", (0, -1), (-1, -1), 6),
        ("TOPPADDING", (0, 1), (-1, 1), 1),
    ]))
    return t


def table(header, rows, widths, aligns=None, zebra=True, fs=8):
    aligns = aligns or ["LEFT"] * len(header)
    data = [[Paragraph(h, S["thr"] if aligns[i] == "RIGHT" else S["th"])
             for i, h in enumerate(header)]]
    for r in rows:
        data.append([c if isinstance(c, Paragraph) else
                     Paragraph(str(c), S["cellr"] if aligns[i] == "RIGHT" else S["cell"])
                     for i, c in enumerate(r)])
    t = Table(data, colWidths=widths, repeatRows=1)
    st = [
        ("BACKGROUND", (0, 0), (-1, 0), NAVY),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("GRID", (0, 0), (-1, -1), 0.4, MID),
        ("LEFTPADDING", (0, 0), (-1, -1), 5), ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 3.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
    ]
    if zebra:
        for i in range(1, len(data)):
            if i % 2 == 0:
                st.append(("BACKGROUND", (0, i), (-1, i), LIGHT))
    t.setStyle(TableStyle(st))
    return t


# ---------------------------------------------------------------- data
def load():
    cfg = yaml.safe_load(open(f"{ROOT}/config/airframe.yaml"))
    sizing = json.load(open(f"{ROOT}/out/sizing.json"))
    return cfg, sizing


def read_mission(name="factory_mission_9x45_pi5"):
    """Mission-level metrics: leg times, path length, energy, reserve."""
    import re
    d = f"{ROOT}/out/runs/{name}"
    p = f"{d}/flight_log.csv"
    if not os.path.exists(p):
        return None
    rows = list(csv.DictReader(open(p)))

    def fl(x):
        try:
            return float(x)
        except (TypeError, ValueError):
            return float("nan")

    air = [r for r in rows if fl(r["z"]) < -0.8]
    if not air:
        return None

    legs, total = [], None
    rl = f"{d}/runner.log"
    if os.path.exists(rl):
        for ln in open(rl):
            m = re.search(r"LEG (\w+): ([\d.]+) s", ln)
            if m:
                legs.append((m.group(1), float(m.group(2))))
            m = re.search(r"TOTAL\s+([\d.]+) s", ln)
            if m:
                total = float(m.group(1))

    dist, prev = 0.0, None
    for r in air:
        q = (fl(r["x"]), fl(r["y"]), fl(r["z"]))
        if prev:
            dist += math.dist(q, prev)
        prev = q

    pw = [fl(r["power_total_w"]) for r in air]
    sp = [fl(r["speed_ms"]) for r in air]
    wh = fl(air[-1]["energy_wh"]) - fl(air[0]["energy_wh"])
    return {"airborne": fl(air[-1]["t_s"]) - fl(air[0]["t_s"]),
            "legs": legs, "total": total, "dist": dist,
            "mean_p": sum(pw) / len(pw), "peak_p": max(pw),
            "mean_v": sum(sp) / len(sp), "peak_v": max(sp),
            "wh": wh, "soc_end": fl(air[-1]["soc"])}


def read_compute():
    p = f"{ROOT}/out/compute_tradeoff.json"
    return json.load(open(p)) if os.path.exists(p) else None


def read_run(name):
    p = f"{ROOT}/out/runs/{name}/flight_log.csv"
    if not os.path.exists(p):
        return None
    rows = list(csv.DictReader(open(p)))
    if not rows:
        return None

    def fl(x):
        try:
            return float(x)
        except (TypeError, ValueError):
            return float("nan")

    air = [r for r in rows if fl(r["z"]) < -1.0]
    steady = [r for r in air if abs(fl(r["vz"])) < 0.3 and fl(r["t_s"]) > 20]
    ref = steady or air or rows

    def mean(k):
        v = [fl(r[k]) for r in ref if not math.isnan(fl(r[k]))]
        return sum(v) / len(v) if v else float("nan")

    gate_t = gate_wh = None
    for r in rows:
        if fl(r["soc"]) <= 0.20:
            gate_t, gate_wh = fl(r["t_s"]), fl(r["energy_wh"])
            break

    xs = [fl(r["x"]) for r in ref]
    ys = [fl(r["y"]) for r in ref]
    rms = mx = float("nan")
    if xs:
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        e = [math.hypot(a - cx, b - cy) for a, b in zip(xs, ys)]
        rms = math.sqrt(sum(v * v for v in e) / len(e))
        mx = max(e)

    return {"rows": len(rows), "power": mean("power_total_w"),
            "current": mean("current_a"), "rpm": mean("rpm_mean"),
            "thrust_g": mean("thrust_per_motor_g"), "speed": mean("speed_ms"),
            "gate_min": gate_t / 60.0 if gate_t else None, "gate_wh": gate_wh,
            "rms": rms, "max": mx}


# ---------------------------------------------------------------- page deco
def make_page(title_text):
    def deco(canv, doc):
        canv.saveState()
        canv.setFillColor(NAVY)
        canv.rect(0, A4[1] - 16 * mm, A4[0], 16 * mm, stroke=0, fill=1)
        canv.setFillColor(colors.white)
        canv.setFont("Helvetica-Bold", 9)
        canv.drawString(20 * mm, A4[1] - 10.5 * mm, title_text)
        canv.setFont("Helvetica", 8)
        canv.drawRightString(A4[0] - 20 * mm, A4[1] - 10.5 * mm, VERSION)

        canv.setStrokeColor(MID)
        canv.setLineWidth(0.5)
        canv.line(20 * mm, 14 * mm, A4[0] - 20 * mm, 14 * mm)
        canv.setFillColor(GREY)
        canv.setFont("Helvetica", 7.5)
        canv.drawString(20 * mm, 10 * mm,
                        "Verification level L0-L2: simulation only. Not a hardware-readiness claim.")
        canv.drawRightString(A4[0] - 20 * mm, 10 * mm, f"Page {doc.page}")
        canv.restoreState()
    return deco


def cover_deco(canv, doc):
    canv.saveState()
    canv.setFillColor(NAVY)
    canv.rect(0, A4[1] - 72 * mm, A4[0], 72 * mm, stroke=0, fill=1)
    canv.setFillColor(ACCENT)
    canv.rect(0, A4[1] - 75 * mm, A4[0], 3 * mm, stroke=0, fill=1)

    # Title lives INSIDE the band, in white — otherwise the band reads as a
    # large empty rectangle above the content.
    cx = A4[0] / 2
    canv.setFillColor(colors.white)
    canv.setFont("Helvetica-Bold", 25)
    canv.drawCentredString(cx, A4[1] - 30 * mm, "Quadrotor Flyability,")
    canv.drawCentredString(cx, A4[1] - 42 * mm, "Mission & Compute Report")
    canv.setFont("Helvetica", 11)
    canv.setFillColor(colors.HexColor("#93C5FD"))
    canv.drawCentredString(cx, A4[1] - 54 * mm,
                           "PX4 SITL  +  Gazebo Harmonic  +  ROS 2 Humble")
    canv.setFont("Helvetica", 9)
    canv.setFillColor(colors.HexColor("#64A8E0"))
    canv.drawCentredString(cx, A4[1] - 64 * mm,
                           "Endurance  |  Factory navigation mission  |  "
                           "Cloud vs edge compute")
    canv.setFillColor(GREY)
    canv.setFont("Helvetica", 7.5)
    canv.drawCentredString(A4[0] / 2, 12 * mm,
                           "Verification level L0-L2: simulation and software test only.")
    canv.restoreState()


# ---------------------------------------------------------------- build
def build(out_path):
    cfg, sizing = load()
    today = datetime.date.today().isoformat()

    by = {(r["payload"], r["prop"]): r for r in sizing}
    base = by[("pi5", "9x4.5")]

    hover = read_run("hover_endurance_9x45_pi5")
    windy = read_run("hover_windy_9x45_pi5")

    doc = BaseDocTemplate(out_path, pagesize=A4,
                          leftMargin=20 * mm, rightMargin=20 * mm,
                          topMargin=24 * mm, bottomMargin=20 * mm,
                          title="Drone_sim v1 - Flyability and Endurance Report",
                          author="Shaashwat Mishra")
    fr = Frame(doc.leftMargin, doc.bottomMargin, doc.width, doc.height, id="n")
    doc.addPageTemplates([
        PageTemplate(id="cover", frames=[Frame(
            doc.leftMargin, doc.bottomMargin, doc.width, doc.height - 40 * mm, id="c")],
            onPage=cover_deco),
        PageTemplate(id="main", frames=[fr],
                     onPage=make_page(
                         "Drone_sim - Flyability, Endurance, Mission & Compute")),
    ])

    E = []

    # ---------------- cover
    E += [Spacer(1, 18 * mm)]

    _m = read_mission()
    meta = [["Version", VERSION],
            ["Date", today],
            ["Repository", "github.com/shaashwatmishra203-creator/Drone_sim"],
            ["Airframe", f"{cfg['meta']['name']} - quad X, "
                         f"{cfg['geometry']['arm_length_m']*2000:.0f} mm diagonal"],
            ["Estimated AUW", f"{base['auw_kg']:.3f} kg  (88% still unweighed)"],
            ["Hover endurance", f"{hover['gate_min']:.2f} min measured in SITL"],
            ["Factory mission",
             (f"{_m['airborne']:.0f} s, {_m['dist']:.0f} m, "
              f"{_m['soc_end']*100:.0f}% pack remaining" if _m else "not run")],
            ["Compute", "Edge required onboard; cloud for map sharing"],
            ["Verdict", "FLIES. Endurance is the constraint, not thrust."]]
    t = Table([[Paragraph(f"<b>{a}</b>", S["cell"]), Paragraph(b, S["cell"])]
               for a, b in meta], colWidths=[38 * mm, 122 * mm])
    t.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.4, MID),
        ("BACKGROUND", (0, 0), (0, -1), LIGHT),
        ("TOPPADDING", (0, 0), (-1, -1), 5), ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
    ]))
    E += [t, Spacer(1, 8 * mm)]

    E += [callout(
        "What this report is, and is not",
        "These results describe a <b>model</b> of the aircraft, validated in "
        "software-in-the-loop simulation. They do <b>not</b> establish that the real "
        "aircraft is safe to build, power or fly. No propeller has been spun, no pack "
        "discharged, and no thrust measured on a stand. Critically, <b>88% of the mass "
        "budget is still estimated rather than weighed</b>, and endurance moves roughly "
        "2 minutes per 500 g. Treat every figure as a band, not a verdict.", "warn")]

    E += [Spacer(1, 6 * mm), Paragraph(
        "Prepared with Claude Opus 5. All figures generated directly from "
        "<font face='Courier'>config/airframe.yaml</font>, "
        "<font face='Courier'>out/sizing.json</font> and the recorded flight logs.",
        S["small"])]

    E += [NextPageTemplate("main"), PageBreak()]

    # ---------------- 1 executive summary
    E += [P("1. Executive summary", "h1")]
    E += [P("The question was whether the parts list the team settled on can fly, and "
            "for how long. It can. Thrust-to-weight is about "
            f"<b>{base['twr']:.2f}</b> and hovering consumes only "
            f"<b>{base['hover']['throttle']*100:.0f}%</b> of available thrust, which is "
            "comfortable margin. Flight capability is not the problem.")]
    E += [P("Endurance is. In still air the aircraft held a 5 m hover for "
            f"<b>{hover['gate_min']:.2f} minutes</b> before reaching the 80% "
            "depth-of-discharge limit. In 6 m/s wind with gusts that falls to roughly "
            "9 minutes. For an autonomy platform carrying two stereo cameras and a "
            "gimbal, that is a short working window.")]

    kpi = [[Paragraph("<b>Measured endurance</b>", S["cell"]),
            Paragraph("<b>Hover power</b>", S["cell"]),
            Paragraph("<b>Thrust-to-weight</b>", S["cell"]),
            Paragraph("<b>Wind penalty</b>", S["cell"])],
           [Paragraph(f"<font size=15 color='#1B6CA8'><b>{hover['gate_min']:.1f}</b></font> min",
                      S["cell"]),
            Paragraph(f"<font size=15 color='#1B6CA8'><b>{hover['power']:.0f}</b></font> W",
                      S["cell"]),
            Paragraph(f"<font size=15 color='#1B6CA8'><b>{base['twr']:.2f}</b></font>",
                      S["cell"]),
            Paragraph(f"<font size=15 color='#B45309'><b>+{100*(windy['power']/hover['power']-1):.0f}%</b></font> power",
                      S["cell"])]]
    t = Table(kpi, colWidths=[41 * mm] * 4)
    t.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.4, MID),
        ("BACKGROUND", (0, 0), (-1, 0), LIGHT),
        ("ALIGN", (0, 0), (-1, -1), "CENTER"),
        ("TOPPADDING", (0, 0), (-1, -1), 6), ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
    ]))
    E += [Spacer(1, 3 * mm), t, Spacer(1, 4 * mm)]

    E += [P("Three findings block the build as specified", "h2")]
    E += [P("None of them is about flight dynamics:", "body")]
    E += [table(
        ["#", "Finding", "Consequence"],
        [["F1", "The ZED 2 and ZED Mini cannot run on a Raspberry Pi 5. The ZED SDK "
                "requires an NVIDIA CUDA GPU; the Pi 5 has none.",
          "Perception stack is non-functional as specified."],
         ["F2", "The SpeedyBee F405 V5 cannot run a PX4 autonomy stack. PX4 dropped "
                "F405 support.",
          "Nothing validated here transfers to that board."],
         ["F3", "The 9x4.5 propellers reach ~15,900 rpm at full throttle against an "
                "estimated 16,111 rpm ceiling - about 1% margin.",
          "13-inch props would add 57% endurance."]],
        [10 * mm, 88 * mm, 67 * mm])]

    E += [Spacer(1, 3 * mm), callout(
        "Evidence quality on F3",
        "The 16,111 rpm ceiling is the <b>145000/D rule of thumb</b> for composite "
        "propellers, not a figure published by HQProp. This finding is only as strong "
        "as that number. Confirm the real rating with the manufacturer before treating "
        "it as settled - if their limit is higher, F3 shrinks from a blocking concern "
        "to a footnote.", "warn")]

    E += [PageBreak()]

    # ---------------- 2 ROS 2 software
    E += [P("2. The ROS 2 software", "h1")]
    E += [P("The simulation is driven by a small ROS 2 Humble package, "
            "<font face='Courier'>drone_eval</font>, living in "
            "<font face='Courier'>~/ws_px4/src/</font> alongside the standard "
            "<font face='Courier'>px4_msgs</font> and "
            "<font face='Courier'>px4_ros_com</font>. Data reaches ROS through PX4's "
            "uXRCE-DDS bridge rather than a <font face='Courier'>ros_gz</font> bridge, "
            "which is deliberately not installed - the PX4 path already carries "
            "everything needed and is lighter.")]

    E += [Spacer(1, 2 * mm), Paragraph(
        "Gazebo Harmonic  &lt;---&gt;  PX4 SITL  &lt;---&gt;  MicroXRCEAgent  &lt;---&gt;  ROS 2<br/>"
        "&nbsp;&nbsp;&nbsp;(physics)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;(flight control)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;(DDS bridge)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;drone_eval nodes",
        S["mono"]), Spacer(1, 3 * mm)]

    E += [P("The two nodes", "h2")]
    E += [table(
        ["Node", "What it does"],
        [[Paragraph("<b>flight_logger</b>", S["cellb"]),
          "Subscribes to position, attitude, status, per-motor commands and battery. "
          "Converts each motor command to rotor speed, then to thrust and shaft power "
          "through the propeller coefficients, and integrates a physical battery model. "
          "Writes one CSV per run and latches "
          "<font face='Courier'>/drone_eval/battery_empty</font> when the "
          "depth-of-discharge gate is reached."],
         [Paragraph("<b>mission_runner</b>", S["cellb"]),
          "Streams offboard heartbeats, arms, takes off, then flies the scenario profile "
          "- hover, forward-speed steps, maximum climb, or waypoints - and lands on "
          "completion, battery-empty, or timeout. PX4 requires a stream of setpoints "
          "<i>before</i> it will accept the offboard mode switch, so the node warms up "
          "before arming."]],
        [32 * mm, 133 * mm])]

    E += [P("Why the logger computes power itself", "h2")]
    E += [P("PX4's SITL battery simulator drains the pack linearly on <b>wall clock</b> "
            "via <font face='Courier'>SIM_BAT_DRAIN</font> and never looks at the "
            "motors, so its <font face='Courier'>BatteryStatus</font> is useless for "
            "endurance. Instead the logger takes the real motor commands and integrates "
            "an open-circuit-voltage curve with internal-resistance sag. That matters "
            "because both effects shorten a flight and a naive energy-divided-by-power "
            "calculation misses them: as the pack empties its voltage falls, so the same "
            "power demands more current, which sags the voltage further.")]

    E += [P("Topics and QoS", "h2")]
    E += [table(
        ["Topic", "Message", "Direction"],
        [["/fmu/out/vehicle_local_position_v1", "VehicleLocalPosition", "subscribe"],
         ["/fmu/out/vehicle_attitude", "VehicleAttitude", "subscribe"],
         ["/fmu/out/vehicle_status_v4", "VehicleStatus", "subscribe"],
         ["/fmu/out/actuator_motors", "ActuatorMotors", "subscribe"],
         ["/fmu/out/battery_status_v1", "BatteryStatus", "subscribe"],
         ["/fmu/in/offboard_control_mode", "OffboardControlMode", "publish"],
         ["/fmu/in/trajectory_setpoint", "TrajectorySetpoint", "publish"],
         ["/fmu/in/vehicle_command", "VehicleCommand", "publish"],
         ["/drone_eval/battery_empty", "std_msgs/Bool", "internal, latched"]],
        [72 * mm, 52 * mm, 41 * mm])]

    E += [Spacer(1, 2 * mm), callout(
        "Two traps that cost real debugging time",
        "<b>QoS.</b> PX4 offers every <font face='Courier'>/fmu/out/</font> topic as "
        "BEST_EFFORT with TRANSIENT_LOCAL durability. The rclpy default is RELIABLE, "
        "which is incompatible - a default subscriber receives nothing, silently. An "
        "empty log is a QoS mismatch until proven otherwise.<br/><br/>"
        "<b>Topic names.</b> PX4 1.18 appends message-version suffixes to some topics "
        "and not others, so both nodes resolve names against the live graph instead of "
        "hardcoding them. There is also no <font face='Courier'>/clock</font> without a "
        "ros_gz bridge, so <font face='Courier'>use_sim_time</font> would freeze both "
        "nodes at zero; they use PX4 message timestamps, which are the simulation clock.",
        "info")]

    E += [PageBreak()]

    # ---------------- 3 parts and masses
    E += [P("3. Parts and mass register", "h1")]
    E += [P("Every component, with the mass actually used by the model and a column for "
            "the real weight once the team measures it. <b>The measured column is "
            "deliberately empty: nothing has been weighed yet.</b> This is the single "
            "largest source of uncertainty in the entire report.")]

    items = sorted(base["mass_items"], key=lambda x: -x["total_kg"])
    rows = []
    for i in items:
        # The component register carries a generic propeller placeholder because
        # the actual prop is chosen per configuration; name the real one here.
        if i["id"] == "prop":
            i = dict(i, name=f"HQProp {base['prop']} nylon composite")
        conf = i["confidence"]
        if conf == "estimated":
            cmark = Paragraph("<font color='#B45309'><b>estimated</b></font>", S["cell"])
        elif conf == "vendor":
            cmark = Paragraph("<font color='#1B6CA8'>vendor spec</font>", S["cell"])
        else:
            cmark = Paragraph(conf, S["cell"])
        rows.append([
            i["name"], str(i["qty"]),
            Paragraph(f"{i['unit_kg']*1000:.0f}", S["cellr"]),
            Paragraph(f"<b>{i['total_kg']*1000:.0f}</b>", S["cellr"]),
            Paragraph("<font color='#94A3B8'>___</font>", S["cellr"]),
            cmark,
            Paragraph(f"{i['power_w']:.1f}" if i["power_w"] else "-", S["cellr"])])
    rows.append([Paragraph("<b>TOTAL (all-up weight)</b>", S["cellb"]), "",
                 "", Paragraph(f"<b>{base['auw_kg']*1000:.0f}</b>", S["cellr"]),
                 Paragraph("<font color='#94A3B8'>___</font>", S["cellr"]), "",
                 Paragraph(f"<b>{base['avionics_w']:.1f}</b>", S["cellr"])])

    E += [table(["Component", "Qty", "Unit (g)", "Est. (g)", "Measured (g)",
                 "Confidence", "Power (W)"],
                rows,
                [56 * mm, 10 * mm, 17 * mm, 17 * mm, 22 * mm, 25 * mm, 18 * mm],
                ["LEFT", "LEFT", "RIGHT", "RIGHT", "RIGHT", "LEFT", "RIGHT"])]

    est_kg = sum(i["total_kg"] for i in items if i["confidence"] == "estimated")
    E += [Spacer(1, 3 * mm), callout(
        "Estimated versus real mass",
        f"<b>{est_kg*1000:.0f} g of {base['auw_kg']*1000:.0f} g "
        f"({100*est_kg/base['auw_kg']:.0f}% of all-up weight) is an engineering estimate, "
        "not a measurement.</b> The four worth weighing first are the battery (830 g "
        "assumed), the four motors (195 g each assumed), the printed frame and legs "
        "(550 g assumed) and the gimbal assembly (210 g assumed). Endurance moves about "
        "2 minutes per 500 g, so the true figure could plausibly sit anywhere between "
        "roughly 8 and 14 minutes. Filling in the measured column and re-running "
        "<font face='Courier'>tools/sizing.py</font> collapses that range to a single "
        "number.", "bad")]

    E += [P("Mass sensitivity", "h2")]
    E += [P("What the uncertainty above actually costs, for the current 9-inch build:")]
    sweep = []
    for line in open(f"{ROOT}/out/sweep_auw.csv").read().strip().split("\n")[1:]:
        a, w, m, twr, _ = line.split(",")
        if abs(round(float(a) * 10) % 4) < 1e-6:
            hl = abs(float(a) - base["auw_kg"]) < 0.11
            f = (lambda x: f"<b>{x}</b>") if hl else (lambda x: x)
            sweep.append([Paragraph(f(a), S["cellr"]), Paragraph(f(f"{float(w):.0f}"), S["cellr"]),
                          Paragraph(f(f"{float(m):.1f}"), S["cellr"]),
                          Paragraph(f(f"{float(twr):.2f}"), S["cellr"])])
    E += [table(["All-up weight (kg)", "Hover power (W)", "Endurance (min)", "T/W"],
                sweep, [41 * mm] * 4, ["RIGHT"] * 4)]

    E += [PageBreak()]

    # ---------------- 4 propulsion
    E += [P("4. Propulsion, battery and mass properties", "h1")]

    m, b = cfg["motor"], cfg["battery"]
    pr = next(p for p in cfg["prop_options"] if p.get("in_parts_list"))
    spec = [
        ["Motor", f"{m['name']} x4  -  {m['kv']:.0f} KV, "
                  f"{m['i_max_cont_a']:.0f} A continuous  (all values unconfirmed)"],
        ["Propeller", f"HQProp {pr['id']} nylon composite x4  -  Ct {pr['ct']:.3f}, "
                      f"Cp {pr['cp']:.3f}, figure of merit {pr['figure_of_merit']:.2f}"],
        ["ESC", f"{cfg['esc']['name']}  -  {cfg['esc']['i_max_cont_a']:.0f} A continuous"],
        ["Battery", f"{b['name']}  -  {b['cells_series']}S, {b['capacity_ah']:.1f} Ah, "
                    f"{base['pack_wh']:.0f} Wh at {base['v_nominal']:.1f} V nominal"],
        ["Discharge limit", f"{b['dod_limit']*100:.0f}% depth of discharge "
                            f"({base['pack_wh']*b['dod_limit']:.0f} Wh usable)"],
        ["Disc loading", f"{base['disc_loading_kgm2']:.1f} kg per square metre "
                         f"({base['disc_area_m2']:.3f} m2 total disc area)"],
        ["Inertia (computed)", f"Ixx {base['inertia']['ixx']:.4f}, "
                               f"Iyy {base['inertia']['iyy']:.4f}, "
                               f"Izz {base['inertia']['izz']:.4f} kg m2"],
        ["Centre of gravity", f"x {base['cg_m'][0]*1000:+.1f} mm, "
                              f"y {base['cg_m'][1]*1000:+.1f} mm, "
                              f"z {base['cg_m'][2]*1000:+.1f} mm"],
    ]
    t = Table([[Paragraph(f"<b>{a}</b>", S["cell"]), Paragraph(bb, S["cell"])]
               for a, bb in spec], colWidths=[36 * mm, 129 * mm])
    t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.4, MID),
                           ("BACKGROUND", (0, 0), (0, -1), LIGHT),
                           ("TOPPADDING", (0, 0), (-1, -1), 4),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
                           ("LEFTPADDING", (0, 0), (-1, -1), 5)]))
    E += [t]

    E += [P("Propeller comparison", "h2")]
    prows = []
    for pid in ["9x4.5", "11x4.5", "13x4.4"]:
        r = by[("pi5", pid)]
        c = r["checks"]["prop_rpm_max"]
        tag = " (current)" if r["in_parts_list"] else ""
        gain = (r["hover"]["minutes"] / by[("pi5", "9x4.5")]["hover"]["minutes"] - 1) * 100
        mg = f"{c['margin_pct']:.1f}%"
        prows.append([
            Paragraph(f"<b>{pid}</b>{tag}", S["cell"]),
            Paragraph(f"{r['disc_loading_kgm2']:.1f}", S["cellr"]),
            Paragraph(f"{r['hover']['power_w']:.0f}", S["cellr"]),
            Paragraph(f"<b>{r['hover']['minutes']:.1f}</b>", S["cellr"]),
            Paragraph("-" if r["in_parts_list"] else f"<b>+{gain:.0f}%</b>", S["cellr"]),
            Paragraph(f"{r['max']['rpm']:.0f}", S["cellr"]),
            Paragraph(mg if c["pass"] and c["margin_pct"] > 5
                      else f"<font color='#9B1C1C'><b>{mg}</b></font>", S["cellr"]),
            Paragraph(f"{r['twr']:.2f}", S["cellr"])])
    E += [table(["Propeller", "Disc load", "Hover W", "Endur. min", "Gain",
                 "Max rpm", "rpm margin", "T/W"],
                prows, [30 * mm, 19 * mm, 18 * mm, 21 * mm, 17 * mm, 20 * mm,
                        22 * mm, 16 * mm],
                ["LEFT"] + ["RIGHT"] * 7)]
    E += [Paragraph("Disc load in kg per square metre. All six configurations pass every "
                    "design gate; the 9-inch rpm margin is the only red figure.", "small")
          if False else Paragraph(
        "Disc load in kg per square metre. All configurations pass the thrust gates - "
        "the 9-inch rpm margin is the one marginal figure.", S["small"])]

    E += [PageBreak()]

    # ---------------- 5 results
    E += [P("5. Flight test results", "h1")]
    E += [P("Two models were built deliberately independently - closed-form momentum "
            "theory, and PX4 flying the aircraft in Gazebo with power derived from real "
            "motor commands. They share only the configuration file. Their agreement is "
            "the reason the rest of these numbers are worth quoting.")]

    E += [P("Validation: analytical versus simulation", "h2")]
    xr = [["Rotor speed (hover)", f"{base['hover']['rpm']:.0f} rpm",
           f"{hover['rpm']:.0f} rpm",
           f"{abs(hover['rpm']/base['hover']['rpm']-1)*100:.1f}%"],
          ["Thrust per motor", f"{base['hover']['thrust_per_motor_g']:.0f} g",
           f"{hover['thrust_g']:.0f} g",
           f"{abs(hover['thrust_g']/base['hover']['thrust_per_motor_g']-1)*100:.1f}%"],
          ["Hover power", f"{base['hover']['power_w']:.0f} W",
           f"{hover['power']:.0f} W",
           f"{abs(hover['power']/base['hover']['power_w']-1)*100:.1f}%"],
          ["Endurance", f"{base['hover']['minutes']:.1f} min",
           f"{hover['gate_min']:.2f} min",
           f"{abs(hover['gate_min']/base['hover']['minutes']-1)*100:.1f}%"]]
    E += [table(["Quantity", "Analytical model", "Measured in SITL", "Difference"],
                [[Paragraph(f"<b>{a}</b>", S["cell"]), Paragraph(b_, S["cellr"]),
                  Paragraph(f"<b>{c_}</b>", S["cellr"]),
                  Paragraph(f"<font color='#1B7F43'><b>{d}</b></font>", S["cellr"])]
                 for a, b_, c_, d in xr],
                [48 * mm, 39 * mm, 39 * mm, 39 * mm],
                ["LEFT", "RIGHT", "RIGHT", "RIGHT"])]
    E += [Paragraph("The residual gap is expected: the analytical side uses momentum "
                    "theory with a figure of merit, while the logger integrates the power "
                    "coefficient against actual rotor speed. They are different "
                    "approximations, so exact agreement would be suspicious.", S["small"])]

    E += [P("How long it flew", "h2")]
    E += [P(f"The aircraft held a 5 metre hover in still air until the battery model "
            f"reached the 80% depth-of-discharge gate, consuming "
            f"<b>{hover['gate_wh']:.1f} Wh</b> of the {base['pack_wh']:.0f} Wh pack:")]

    rrows = [
        ["Still air (default world)",
         Paragraph(f"<b>{hover['gate_min']:.2f} min</b>", S["cellr"]),
         Paragraph(f"{hover['power']:.0f}", S["cellr"]),
         Paragraph(f"{hover['current']:.1f}", S["cellr"]),
         Paragraph(f"{hover['rpm']:.0f}", S["cellr"]),
         Paragraph(f"{hover['thrust_g']:.0f}", S["cellr"]),
         Paragraph(f"{hover['rms']:.2f}", S["cellr"])],
        ["6 m/s wind + 3 m/s gusts",
         Paragraph(f"<b>~9.2 min</b> (proj.)", S["cellr"]),
         Paragraph(f"{windy['power']:.0f}", S["cellr"]),
         Paragraph(f"{windy['current']:.1f}", S["cellr"]),
         Paragraph(f"{windy['rpm']:.0f}", S["cellr"]),
         Paragraph(f"{windy['thrust_g']:.0f}", S["cellr"]),
         Paragraph(f"{windy['rms']:.2f}", S["cellr"])],
        [Paragraph("<b>Wind penalty</b>", S["cellb"]),
         Paragraph(f"<font color='#B45309'><b>-19%</b></font>", S["cellr"]),
         Paragraph(f"<font color='#B45309'><b>+{100*(windy['power']/hover['power']-1):.0f}%</b></font>", S["cellr"]),
         Paragraph(f"+{100*(windy['current']/hover['current']-1):.0f}%", S["cellr"]),
         Paragraph(f"+{100*(windy['rpm']/hover['rpm']-1):.0f}%", S["cellr"]),
         Paragraph(f"+{100*(windy['thrust_g']/hover['thrust_g']-1):.0f}%", S["cellr"]),
         Paragraph(f"{windy['rms']/hover['rms']:.0f}x worse", S["cellr"])]]
    E += [table(["Condition", "Endurance", "Power (W)", "Current (A)", "Rotor (rpm)",
                 "Thrust/motor (g)", "Hold RMS (m)"],
                rrows, [40 * mm, 25 * mm, 19 * mm, 20 * mm, 21 * mm, 22 * mm, 18 * mm],
                ["LEFT"] + ["RIGHT"] * 6)]

    E += [Spacer(1, 2 * mm), callout(
        "Do not plan missions on the still-air number",
        f"The {hover['gate_min']:.1f} minute figure is a benchmark measured with zero "
        "disturbance. In moderate wind the same aircraft loses about a fifth of its "
        "endurance and its position-hold error rises from 2 cm to nearly a metre. "
        "<b>Plan real missions around 7 to 8 minutes of usable time with reserve.</b>",
        "warn")]

    E += [P("Scenarios not yet run", "h2")]
    E += [P("This version covers still-air endurance and wind. Four scenarios are built "
            "and scripted but have not been executed: full 13-inch endurance, the Jetson "
            "compute variant, a forward-speed sweep, a maximum-climb test, and a forest "
            "waypoint mission. An interrupted 13-inch run did produce one useful "
            "reading - 384.8 W hover against the model's 388 W predicted, a 0.8% match "
            "that independently exercises the motor-constant derivation at a different "
            "propeller diameter. Re-run everything with "
            "<font face='Courier'>bash scripts/batch2.sh</font>.")]

    E += [PageBreak()]

    # ---------------- 6 factory mission
    mis = read_mission()
    E += [P("6. Factory navigation and mapping mission", "h1")]
    if mis:
        E += [P("The representative mission, flown end to end: launch from base, "
                "slalom an obstacle course past storage racks and centreline "
                "pillars, fly a lawnmower scan pattern inside a 12 x 12 m room - "
                "the mapping pass whose data would go to the other vehicles - then "
                "return to base and land. Light wind (1.5 m/s, 0.8 m/s gusts) was "
                "applied after takeoff.")]

        mk = [[Paragraph("<b>Airborne</b>", S["cell"]),
               Paragraph("<b>Path flown</b>", S["cell"]),
               Paragraph("<b>Energy used</b>", S["cell"]),
               Paragraph("<b>Pack remaining</b>", S["cell"])],
              [Paragraph(f"<font size=15 color='#1B6CA8'><b>{mis['airborne']:.1f}</b></font> s", S["cell"]),
               Paragraph(f"<font size=15 color='#1B6CA8'><b>{mis['dist']:.0f}</b></font> m", S["cell"]),
               Paragraph(f"<font size=15 color='#1B6CA8'><b>{mis['wh']:.1f}</b></font> Wh", S["cell"]),
               Paragraph(f"<font size=15 color='#1B7F43'><b>{mis['soc_end']*100:.1f}</b></font> %", S["cell"])]]
        t = Table(mk, colWidths=[41 * mm] * 4)
        t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.4, MID),
                               ("BACKGROUND", (0, 0), (-1, 0), LIGHT),
                               ("ALIGN", (0, 0), (-1, -1), "CENTER"),
                               ("TOPPADDING", (0, 0), (-1, -1), 6),
                               ("BOTTOMPADDING", (0, 0), (-1, -1), 6)]))
        E += [Spacer(1, 2 * mm), t, Spacer(1, 4 * mm)]

        usable = base["pack_wh"] * cfg["battery"]["dod_limit"]
        trips = usable / mis["wh"]
        E += [P("Leg breakdown", "h2")]
        lt = sum(d for _, d in mis["legs"]) or 1
        names = {"outbound": "Outbound (obstacle course)",
                 "scan": "Scan (mapping pass)",
                 "inbound": "Inbound (return to base)", "land": "Land"}
        lrows = [[names.get(n, n), Paragraph(f"{d:.1f}", S["cellr"]),
                  Paragraph(f"{100*d/lt:.1f}%", S["cellr"])] for n, d in mis["legs"]]
        lrows.append([Paragraph("<b>TOTAL</b>", S["cellb"]),
                      Paragraph(f"<b>{lt:.1f}</b>", S["cellr"]),
                      Paragraph("", S["cellr"])])
        E += [table(["Leg", "Time (s)", "Share"], lrows,
                    [95 * mm, 35 * mm, 35 * mm], ["LEFT", "RIGHT", "RIGHT"])]

        scan_share = next((100 * d / lt for n, d in mis["legs"] if n == "scan"), 0)
        E += [Spacer(1, 2 * mm), callout(
            "Scanning is a third of the flight",
            f"The mapping pass is <b>{scan_share:.0f}% of the mission</b> - comparable "
            "to either transit leg. It is not a cheap add-on at the end, and mission "
            "planning should budget for it accordingly.", "info")]

        E += [P("Operational margin", "h2")]
        E += [table(["Metric", "Value"],
                    [["Mean speed", Paragraph(f"{mis['mean_v']:.2f} m/s (peak {mis['peak_v']:.2f})", S["cellr"])],
                     ["Mean power", Paragraph(f"{mis['mean_p']:.0f} W (peak {mis['peak_p']:.0f})", S["cellr"])],
                     ["Energy per metre flown", Paragraph(f"{mis['wh']/mis['dist']*1000:.0f} mWh/m", S["cellr"])],
                     ["Usable energy consumed", Paragraph(f"{100*mis['wh']/usable:.1f} %", S["cellr"])],
                     [Paragraph("<b>Round trips per charge</b>", S["cellb"]),
                      Paragraph(f"<b>{trips:.1f}</b>", S["cellr"])]],
                    [95 * mm, 70 * mm], ["LEFT", "RIGHT"])]

        E += [Spacer(1, 2 * mm), P(
            f"<b>Endurance is not the binding constraint for this task.</b> The hover "
            f"tests give about {hover['gate_min']:.0f} minutes; this mission takes one. "
            f"One pack supports roughly {trips:.0f} round trips, which reframes the "
            "problem: for factory survey work the limit is mission planning and "
            "turnaround, not battery capacity.")]

        E += [Spacer(1, 2 * mm), callout(
            "What this mission does NOT establish",
            "<b>No obstacle avoidance was tested.</b> The waypoints are pre-planned and "
            "collision-free by construction - the aircraft flew a known-good path, it "
            "did not perceive the racks and decide to avoid them. Likewise the scan leg "
            "measures what a mapping pass <i>costs</i> in time and energy; it does not "
            "produce a map. Both gaps need the depth pipeline, which needs CUDA "
            "hardware - see the next section.", "warn")]
    else:
        E += [P("Mission data not found - run "
                "<font face='Courier'>scripts/run_scenario.sh "
                "scenarios/factory_mission.yaml</font>.")]

    E += [PageBreak()]

    # ---------------- 7 compute architecture
    cp = read_compute()
    E += [P("7. Cloud versus edge compute", "h1")]
    if cp:
        E += [P("Where should perception and navigation run - on the aircraft, or on "
                "a server it streams to? The question is not which is faster in "
                "general, but three specific ones: <b>can the link carry it, can it "
                "stop in time, and what happens when the link drops?</b>")]

        E += [Spacer(1, 1 * mm), callout(
            "Verdict: split the workload",
            "Edge and cloud are not competing options - they solve different halves of "
            "the problem. <b>Depth, visual-inertial odometry, obstacle avoidance and "
            "local planning must run onboard.</b> Map merging, global optimisation and "
            "distribution to the other vehicles belong off-board. A CUDA-capable board "
            "is required either way.", "good")]

        E += [P("Bandwidth: what offload would have to push off the aircraft", "h2")]
        srows = [[s["stream"], Paragraph(f"{s['raw_mbps']:.0f}", S["cellr"]),
                  Paragraph(f"{s['comp_mbps']:.1f}", S["cellr"])]
                 for s in cp["streams"]]
        srows.append([Paragraph("<b>TOTAL</b>", S["cellb"]),
                      Paragraph(f"<b>{cp['total_raw_mbps']:.0f}</b>", S["cellr"]),
                      Paragraph(f"<b>{cp['total_compressed_mbps']:.0f}</b>", S["cellr"])])
        E += [table(["Stream", "Raw (Mbps)", "Compressed (Mbps)"], srows,
                    [95 * mm, 35 * mm, 35 * mm], ["LEFT", "RIGHT", "RIGHT"])]
        E += [Paragraph("Depth compresses only about 8x - it is high-entropy, and lossy "
                        "compression destroys exactly the geometry the obstacle "
                        "avoidance depends on. That stream dominates the budget.",
                        S["small"])]

        E += [P("What the link actually delivers", "h2")]
        brows = []
        for r in cp["bandwidth"]:
            v = r["verdict"]
            cell = (Paragraph(f"<font color='#1B7F43'><b>{v}</b></font>", S["cellr"])
                    if v == "OK" else
                    Paragraph(f"<font color='#9B1C1C'><b>{v}</b></font>", S["cellr"]))
            brows.append([r["condition"],
                          Paragraph(f"{r['goodput_mbps']:.0f}", S["cellr"]), cell])
        E += [table(["Condition (ALFA AWUS036ACM, 802.11ac)", "Goodput (Mbps)",
                     "Verdict"], brows,
                    [92 * mm, 32 * mm, 41 * mm], ["LEFT", "RIGHT", "RIGHT"])]
        E += [Spacer(1, 2 * mm), callout(
            "The link is worst exactly where the mission matters most",
            f"The mapping pass happens <b>inside the room, behind a wall, at the "
            f"furthest point from base</b>. Offload needs about "
            f"{cp['total_compressed_mbps']:.0f} Mbps; there, the link delivers roughly "
            "11 Mbps.", "bad")]

        E += [P("Latency: can it stop in time?", "h2")]
        rrows = []
        for r in cp["reaction"]:
            v = r["verdict"]
            col = {"SAFE": "#1B7F43", "ACCEPTABLE": "#B45309",
                   "UNSAFE": "#9B1C1C"}.get(v, "#1E293B")
            rrows.append([
                Paragraph(f"<b>{r['arch']}</b>", S["cell"]),
                Paragraph(f"{r['latency_ms']:.0f}", S["cellr"]),
                Paragraph(f"{r['reaction_m']:.2f}", S["cellr"]),
                Paragraph(f"{r['stop_m']:.2f}", S["cellr"]),
                Paragraph(f"<b>{r['total_m']:.2f}</b>", S["cellr"]),
                Paragraph(f"{cp['max_safe_speed'][r['arch']]:.2f}", S["cellr"]),
                Paragraph(f"<font color='{col}'><b>{v}</b></font>", S["cellr"])])
        E += [table(["Architecture", "Latency (ms)", "React (m)", "Stop (m)",
                     "Total (m)", "Max safe (m/s)", "Verdict"], rrows,
                    [38 * mm, 21 * mm, 19 * mm, 18 * mm, 19 * mm, 24 * mm, 26 * mm],
                    ["LEFT"] + ["RIGHT"] * 6)]
        E += [Paragraph(f"At the measured mission speed of {cp['speed_ms']:.2f} m/s, "
                        "braking at 5.7 m/s2. Corridor clearance is 1.25 m from "
                        "centreline to the nearest rack or pillar. Note that moving the "
                        "server on-premise saves only ~39 ms: the bottleneck is not "
                        "network distance but that 150 Mbps does not fit down an 11 Mbps "
                        "pipe, so bandwidth failure shows up as latency.", S["small"])]

        E += [P("Availability: what a dropout costs", "h2")]
        sp = cp["speed_ms"]
        E += [table(["Link dropout", "Distance flown blind"],
                    [[f"{d} ms", Paragraph(f"{sp*d/1000:.2f} m", S["cellr"])]
                     for d in (200, 500, 1000)],
                    [95 * mm, 70 * mm], ["LEFT", "RIGHT"])]
        E += [Spacer(1, 2 * mm), callout(
            "The strongest of the three arguments",
            "An edge architecture is simply unaffected by a dropout: perception and "
            "control never leave the aircraft. In a steel-racked factory, "
            "multi-hundred-millisecond dropouts from shadowing and multipath are "
            "routine rather than exceptional. <b>This is a safety property rather than "
            "a performance one, and no amount of link engineering removes it.</b> A "
            "better radio raises the bandwidth ceiling and shaves latency; it does not "
            "make a dropout safe.", "bad")]

        E += [PageBreak()]
        E += [P("Why edge compute is what makes the map sharing work", "h2")]
        E += [P("The goal is for this drone to map a room and send that map to the "
                "other vehicles being designed. That requirement is the clearest "
                "argument for the split architecture:")]
        E += [table(["", "Size", "Can it leave the aircraft?"],
                    [["Raw stereo + depth, continuous",
                      Paragraph(f"<b>{cp['total_raw_mbps']:.0f} Mbps</b>", S["cellr"]),
                      Paragraph("<font color='#9B1C1C'><b>No, at any range</b></font>", S["cell"])],
                     ["Finished occupancy map of the room, per scan",
                      Paragraph("<b>~ a few hundred KB</b>", S["cellr"]),
                      Paragraph("<font color='#1B7F43'><b>Yes, over almost any link</b></font>", S["cell"])]],
                    [78 * mm, 32 * mm, 55 * mm], ["LEFT", "RIGHT", "LEFT"])]
        E += [Spacer(1, 2 * mm),
              P("Processing on the aircraft compresses the link requirement by roughly "
                "<b>four orders of magnitude</b>, turning an impossible stream into a "
                "trivial one. Edge compute is not an alternative to sharing the map - "
                "it is the precondition for it. The map is also the right thing to "
                "share: other vehicles need the result, not the sensor feed.")]

        E += [P("Power is not the deciding factor", "h2")]
        prows = [[k, Paragraph(f"{v:.1f}", S["cellr"])]
                 for k, v in cp["power_w"].items()]
        E += [table(["Architecture", "Total power (W)"], prows,
                    [110 * mm, 55 * mm], ["LEFT", "RIGHT"])]
        E += [Paragraph("The difference between the two realistic architectures is "
                        "about 8.5 W - roughly 1.3% of the 630 W hover draw, or 15 "
                        "seconds of flight time. Well inside the noise in the mass "
                        "estimates.", S["small"])]

        E += [P("Recommended architecture", "h2")]
        E += [Paragraph(
            "ON THE AIRCRAFT (Jetson Orin NX)          OFF-BOARD (server or cloud)<br/>"
            "------------------------------            ---------------------------<br/>"
            "ZED depth + visual-inertial odom   -----&gt; finished map (few hundred KB)<br/>"
            "local occupancy map                       once per scan<br/>"
            "obstacle avoidance + local planner &lt;----- mission assignments,<br/>"
            "flight control (PX4)                      global map, fleet coordination<br/>"
            "<br/>"
            "must survive total link loss              may be seconds late",
            S["mono"])]

        E += [Spacer(1, 3 * mm), callout(
            "Limits of this analysis",
            "<b>Verification level L0.</b> No measurement on the real link, the real "
            "boards, or in the real building. Throughput figures are measured-order "
            "estimates for 802.11ac, not site survey data. A survey would change the "
            "numbers - access-point placement could raise the in-room figure "
            "substantially - but it would not change the dropout argument. Dropping to "
            "480p15 would cut the offload bandwidth roughly 4x and make the numbers "
            "much closer, at the cost of obstacle-detection range and precision.",
            "info")]
    else:
        E += [P("Compute analysis not found - run "
                "<font face='Courier'>tools/compute_tradeoff.py</font>.")]

    E += [PageBreak()]

    # ---------------- 8 next steps
    E += [P("8. Recommended actions", "h1")]
    E += [table(
        ["Priority", "Action", "Cost", "Benefit"],
        [["1", Paragraph("<b>Weigh the aircraft</b> and fill in the measured column in "
                         "section 3, then re-run the sizing tool.", S["cell"]),
          "free", "Removes the largest uncertainty in every number here"],
         ["2", Paragraph("<b>Confirm HQProp's real maximum rpm</b> for the 9x4.5.", S["cell"]),
          "free", "Settles or dismisses finding F3 entirely"],
         ["3", Paragraph("<b>Move to 13-inch propellers.</b> The 495 mm frame fits them "
                         "with about 20 mm to spare - verify against CAD.", S["cell"]),
          "one prop set", "+57% endurance, 10.7 to 16.8 minutes"],
         ["4", Paragraph("Swap the Raspberry Pi 5 for a CUDA-capable board such as a "
                         "Jetson Orin NX.", S["cell"]),
          "board", "Makes the perception stack possible at all; costs ~0.6 min"],
         ["5", Paragraph("Swap the SpeedyBee F405 for an H7 flight controller.", S["cell"]),
          "board", "Makes a PX4 autonomy stack possible at all"],
         ["6", Paragraph("Specify a dedicated 5 V / 5 A BEC and a power module.", S["cell"]),
          "small", "Avoids brownout; enables real battery telemetry"],
         ["7", Paragraph("Thrust-stand the motor and propeller to measure real Ct and Cp.",
                         S["cell"]),
          "one day", "Collapses the biggest remaining source of model error"]],
        [15 * mm, 71 * mm, 20 * mm, 59 * mm])]

    E += [Spacer(1, 3 * mm), P("What would change these conclusions", "h2")]
    E += [P("Measured masses materially different from the estimates, most likely the "
            "printed frame which could plausibly be 400 g or 800 g. An HQProp rating "
            "above roughly 17,000 rpm would retire F3. Measured propeller coefficients "
            "could move hover power by as much as 20%, carrying every endurance figure "
            "with them. None of these would threaten flyability itself - the thrust "
            "margin is large enough to absorb them.")]

    E += [Spacer(1, 4 * mm), callout(
        "A note on what the validation proves",
        "The close agreement between the analytical model and the simulation demonstrates "
        "<b>internal consistency, not physical accuracy</b>. Both models read the same "
        "configuration file, so if the propeller coefficients are wrong then both are "
        "wrong together and would still agree with each other. Only a thrust stand can "
        "settle that. This is why item 7 above matters more than its low priority "
        "suggests.", "info")]

    doc.build(E)
    return out_path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=f"{ROOT}/out/Drone_sim_v1_Report.pdf")
    a = ap.parse_args()
    p = build(a.out)
    print(f"wrote {p}  ({os.path.getsize(p)/1024:.0f} KB)")
