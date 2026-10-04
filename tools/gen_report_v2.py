#!/usr/bin/env python3
"""
gen_report_v2.py - PDF report for the factory hall v2 mission.

Numbers for the recorded run are read from its hall_report.json, so the PDF
cannot disagree with the data it describes. Runs 11 and 12 predate per-run
output folders; their figures are quoted from the run logs (see docs/10).

Usage: python3 tools/gen_report_v2.py --run out/runs/hall_mission_<time> \
                                      --out out/drone_sim_report_v2.pdf
"""

import argparse
import datetime
import json
import os

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (Image, PageBreak, Paragraph, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

VERSION = "v2.0.0"

ss = getSampleStyleSheet()
H1 = ParagraphStyle("h1", parent=ss["Heading1"], fontSize=15, spaceAfter=6)
H2 = ParagraphStyle("h2", parent=ss["Heading2"], fontSize=12, spaceBefore=8, spaceAfter=4)
P = ParagraphStyle("p", parent=ss["BodyText"], fontSize=9.2, leading=12.5, spaceAfter=5)
SMALL = ParagraphStyle("s", parent=P, fontSize=8, leading=10.5, textColor=colors.HexColor("#444444"))
NOTE = ParagraphStyle("n", parent=P, backColor=colors.HexColor("#fff4e5"),
                      borderColor=colors.HexColor("#e0a050"), borderWidth=0.6,
                      borderPadding=5, spaceBefore=4, spaceAfter=8)


def table(rows, widths, head=True):
    t = Table([[Paragraph(str(c), SMALL) for c in r] for r in rows], colWidths=widths)
    st = [("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#bbbbbb")),
          ("VALIGN", (0, 0), (-1, -1), "TOP"),
          ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3)]
    if head:
        st.append(("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e8eef7")))
    t.setStyle(TableStyle(st))
    return t


def ok(v):
    return "PASS" if v else "FAIL"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rep = json.load(open(os.path.join(a.run, "hall_report.json")))
    mis = rep.get("mission", {})
    legs = mis.get("legs_s", {})
    cv = rep.get("cv") or {}
    rtt = rep.get("link_round_trip_ms") or {}
    g = rep.get("gates", {})
    run_name = os.path.basename(os.path.normpath(a.run))

    doc = SimpleDocTemplate(a.out, pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm,
                            topMargin=14 * mm, bottomMargin=14 * mm,
                            title=f"drone_sim factory hall report {VERSION}")
    s = []
    s.append(Paragraph(f"Factory hall mission &mdash; camera navigation with cloud object detection ({VERSION})", H1))
    s.append(Paragraph(f"Generated {datetime.date.today().isoformat()} from <b>{run_name}</b>. "
                       "PX4 SITL v1.18 + Gazebo Harmonic + ROS 2 Humble.", SMALL))
    s.append(Paragraph(
        "<b>Verification level L1&ndash;L2: simulation only.</b> These results show what the simulated "
        "drone did in a simulated hall. They do not show that the real aircraft is safe to build or fly.", NOTE))

    s.append(Paragraph("Summary", H2))
    s.append(Paragraph(
        "The drone takes off from a pad, flies a 27.5 m factory aisle with five obstacles that each leave one "
        "gap (alternating sides), scans the room at the far end, flies back through the aisle and lands. The only "
        "obstacle information it has is what its camera sees: frames go over an emulated factory WiFi link to a "
        "cloud object detector, and the boxes that come back are turned into a map on the drone. "
        "<b>Of four clean runs, three completed the whole mission</b> with no contact, at least 0.52 m of "
        "clearance, and landings within 0.09 m of the pad centre. The fourth got stuck on the way back; a "
        "dead-end escape was added after it but has not yet been exercised in flight. "
        "<b>Detection recall fails its gate</b> (0.76&ndash;0.79 against 0.90).", P))

    s.append(Paragraph("Complete missions", H2))
    rows = [["Run", "Total", "Out / scan / back", "Min clearance", "Landing error", "Recall",
             "Range err. median / p90", "Round trip median / p95"],
            ["11 (headless)", "120.6 s", "47.5 / 13.0 / 49.8 s", "0.523 m", "0.089 m", "0.758",
             "2.1% / 8.7%", "117 / 174 ms"],
            ["12 (watched)", "142.0 s", "&mdash; / &mdash; / 74.6 s", "0.521 m", "0.083 m", "0.787",
             "1.9% / 8.3%", "118 / 166 ms"],
            ["14", "stuck on return", "51.6 / 12.8 / &mdash;", "0.556 m", "&mdash;", "0.837",
             "2.9% / 8.6%", "122 / 168 ms"]]
    rows.append([
        f"{run_name} (recorded)", f"{mis.get('total_s', 0):.1f} s",
        f"{legs.get('outbound', 0):.1f} / {legs.get('scan', 0):.1f} / {legs.get('inbound', 0):.1f} s",
        f"{rep.get('hull_clearance_min_m', float('nan')):.3f} m", f"{rep.get('landing_error_m', float('nan')):.3f} m",
        f"{cv.get('recall', 0):.3f}" if cv.get("recall") is not None else "&mdash;",
        f"{100 * cv['range_err_median']:.1f}% / {100 * cv['range_err_p90']:.1f}%" if cv.get("range_err_median") else "&mdash;",
        f"{rtt.get('median', 0):.0f} / {rtt.get('p95', 0):.0f} ms" if rtt.get("median") else "&mdash;"])
    s.append(table(rows, [24 * mm, 15 * mm, 27 * mm, 19 * mm, 18 * mm, 13 * mm, 24 * mm, 25 * mm]))
    s.append(Paragraph(
        f"Recorded run status: <b>{mis.get('status', '?')}</b>. All clearance, contact and landing figures are "
        "measured from Gazebo's true pose, not PX4's estimate. Runs 11 and 12 predate per-run output folders; "
        "their figures come from the run logs at the time. A third session was excluded because two run "
        "harnesses overlapped on one output folder.", SMALL))

    s.append(Paragraph("Gates (recorded run)", H2))
    gate_rows = [["Gate", "Threshold", "Result", "Why this threshold"],
                 ["Hull clearance", "&ge; 0.40 m every sample", ok(g.get("clearance >= 0.4 m")),
                  "about one prop radius beyond the 0.35 m airframe radius"],
                 ["Contact", "none", ok(g.get("no contact")), ""],
                 ["Landing", "&le; 0.50 m from pad centre", ok(g.get("landed within 0.5 m")),
                  "pad half-width less the airframe"],
                 ["Detection recall", "&ge; 0.90 (in view, &le; 8 m, &ge;50% visible)",
                  ok(g.get("CV recall >= 0.90")), "missing 1 in 10 nearby obstacles is too many"],
                 ["Range error", "median &le; 15%", ok(g.get("range error median <= 15%")),
                  "keeps the 1.2 m stop margin at 7 m look-ahead"]]
    s.append(table(gate_rows, [30 * mm, 45 * mm, 16 * mm, 87 * mm]))

    png = os.path.join(a.run, "hall_path.png")
    if os.path.exists(png):
        s.append(Paragraph("True flight path, obstacles and the camera-built map", H2))
        s.append(Image(png, width=178 * mm, height=178 * mm * 0.645))
        s.append(Paragraph("Black: the drone's true path from Gazebo. Red: cells the drone mapped from cloud "
                           "detections. The aisle is the only route from the pad (left) to the scan zone (right).",
                           SMALL))

    s.append(PageBreak())
    s.append(Paragraph("How it works", H2))
    for line in [
        "<b>Camera to cloud.</b> The Arducam (640x360, 15 fps) is read on the drone, JPEG-encoded at 10 Hz "
        "(~5&ndash;7 KB per frame) and sent over an emulated link: 11 Mbps, 35 &plusmn; 15 ms each way, 1% "
        "loss. The link is an assumption, not a measured network; drone and cloud both run on one laptop.",
        "<b>Cloud detection.</b> Classical computer vision: colour segmentation per object class, boxes, class "
        "labels, confidence, and the pixels where each object meets the floor. Chosen because the hall's objects "
        "are synthetic coloured shapes a pretrained detector would not recognise.",
        "<b>On the drone.</b> Each floor-contact pixel, with the camera height and the IMU attitude from PX4's "
        "EKF at capture time, gives a range and bearing. These build an occupancy map (0.25 m cells). Free "
        "space seen along each ray clears stale cells. A* plans over the map; floor never seen is passable "
        "but costs more. Local avoidance checks the next 2.5 m.",
        "<b>Safety stays on the drone.</b> It holds position if detections are older than 0.5 s (link lag or "
        "outage), and stops if anything is within 1.2 m ahead. It never depends on the cloud for safety.",
        "<b>Positioning</b> is PX4's EKF with simulated GPS (team decision for this version). Indoors there is "
        "no GPS; camera + IMU localisation is the next major step."]:
        s.append(Paragraph(line, P))

    s.append(Paragraph("Two bugs that invalidated earlier results", H2))
    s.append(table([
        ["Bug", "Effect", "Fix"],
        ["Waypoints in Gazebo's frame (East-North-Up) sent to PX4 unconverted (it reads North-East-Down)",
         "Every v1 mission flew the course rotated 90&deg;, beside the walls. All checks used the same wrong "
         "frame, so nothing flagged it.",
         "One conversion module with unit tests; all judging done against Gazebo's true pose."],
        ["Magnetic declination mismatch (world at California coordinates)",
         "PX4 heading 26&deg; wrong on the pad; camera detections rotated on the map.",
         "PX4's stock world origin; heading error now &minus;4.8&deg; on the pad, ~0&ndash;5&deg; in flight."],
    ], [52 * mm, 66 * mm, 60 * mm]))

    s.append(Paragraph("Retracted", H2))
    s.append(Paragraph(
        "All v1 factory-mission results: gates threaded, zero collisions, 71&ndash;74% camera-steered, 91&ndash;92 s "
        "mission time, the clearance figures and the camera+IMU fusion numbers. The v1 course was flown rotated "
        "90&deg;. This also explains the unresolved v1 contradiction (0.00 m clearance with no collision). Hover "
        "physics, thrust, power and endurance results are not affected. Also retracted: the claim that cut-off "
        "boxes give an upper bound on range (disproved in 45 of 70 cases).", P))

    s.append(Paragraph("Open items", H2))
    for line in [
        "Detection recall below gate (0.76&ndash;0.79 vs 0.90); shelf racks weakest (0.62&ndash;0.67). "
        "Suspected cause: adjacent same-colour racks merging into one box. Not yet confirmed.",
        "Too few runs to quote a spread with confidence (plan: N &ge; 5).",
        "Scripted link-outage test not yet run.",
        "Residual heading error ~0&ndash;5&deg; (&asymp;0.6 m at 7 m).",
        "Positioning is simulated GPS; camera + IMU localisation not yet ported to the hall."]:
        s.append(Paragraph("&bull; " + line, P))

    s.append(Paragraph("Reproduce", H2))
    s.append(Paragraph("<font face='Courier'>bash ~/drone_sim/scripts/hall_run.sh</font> (headless, judged "
                       "against truth) &nbsp;&middot;&nbsp; <font face='Courier'>MODE=frame_check bash "
                       "~/drone_sim/scripts/hall_run.sh</font> (position and heading check) &nbsp;&middot;&nbsp; "
                       "<font face='Courier'>scripts/run_sim.ps1</font> from Windows to watch with windows. "
                       "Full write-up: docs/10-factory-hall-v2.md.", P))
    doc.build(s)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
