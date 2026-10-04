#!/usr/bin/env python3
"""
build_replay.py - turn a recorded hall run into things GitHub can show.

GitHub cannot run Gazebo. This builds, from one run's REAL recorded data:
  docs/replay/data.json     hall geometry, the true flight path (Gazebo), the
                            camera map in the order cells were first seen,
                            mission legs, and the run's judged results
  docs/replay/frames/*.jpg  the cloud detector's annotated camera view, 2 fps,
                            each tagged with its capture time
  docs/media/camera.gif     the camera + cloud boxes, for the README
  docs/media/topdown.gif    the true path over the hall plan, for the README
The page docs/replay/index.html plays it all back in 3D in a browser.

Run with PYTHONNOUSERSITE=1 (OpenCV needs the system numpy).

  python3 tools/build_replay.py --run out/runs/hall_mission_<time>
"""

import argparse
import csv
import json
import math
import os
import shutil
import sys

import cv2
import numpy as np
import yaml
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_hall  # noqa: E402

CELL = 0.25


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--docs", default=os.path.join(HERE, "..", "docs"))
    a = ap.parse_args()
    run = os.path.abspath(a.run)
    cfg = yaml.safe_load(open(gen_hall.DEFAULT_CFG))
    out_dir = os.path.join(a.docs, "replay")
    media = os.path.join(a.docs, "media")
    frames_dir = os.path.join(out_dir, "frames")
    shutil.rmtree(frames_dir, ignore_errors=True)
    os.makedirs(frames_dir)
    os.makedirs(media, exist_ok=True)

    # ---- true path, 10 Hz -------------------------------------------------
    rows = list(csv.DictReader(open(os.path.join(run, "truth.csv"))))
    path, last = [], -1.0
    for r in rows:
        t = float(r["t"])
        if t - last < 0.1:
            continue
        last = t
        qw, qx, qy, qz = (float(r[k]) for k in ("qw", "qx", "qy", "qz"))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        path.append([round(t, 2), round(float(r["x"]), 3), round(float(r["y"]), 3),
                     round(float(r["z"]), 3), round(yaw, 3)])
    t0 = path[0][0]
    for p in path:
        p[0] = round(p[0] - t0, 2)

    # ---- camera map, in the order cells were first seen --------------------
    final = {(round(c["x"], 2), round(c["y"], 2)): c["class"]
             for c in json.load(open(os.path.join(run, "camera_map.json")))["cells"]}
    first = {}
    for line in open(os.path.join(run, "detections_edge.jsonl")):
        r = json.loads(line)
        px, py, pz, pyaw, _ = r["pose"]
        cx, cy = px + 0.09 * math.cos(pyaw), py + 0.09 * math.sin(pyaw)
        for cls, rng, b, _, how in r["dets"]:
            if how != "contact" or rng > 7.0:
                continue
            for dr in (0.0, 0.4, 0.8):
                wx = cx + (rng + dr) * math.cos(pyaw + b)
                wy = cy + (rng + dr) * math.sin(pyaw + b)
                key = (round((math.floor(wx / CELL) + 0.5) * CELL, 2),
                       round((math.floor(wy / CELL) + 0.5) * CELL, 2))
                if key in final and key not in first:
                    first[key] = round(r["t_cap"] - t0, 2)
    # cells whose first sighting cannot be reconstructed (rounding at a cell
    # edge) still belong to the final map: show them at the end, don't drop them
    t_end = float(rows[-1]["t"]) - t0
    for k in final:
        first.setdefault(k, round(t_end, 2))
    cells = sorted([[t, k[0], k[1], final[k]] for k, t in first.items()])

    # ---- legs: start of the outbound leg from the navigation log, then the
    # measured leg durations from mission_summary.json (the nav log is not
    # written while hovering for the scan, so it cannot mark that leg) -------
    legs = []
    t_out = None
    for line in open(os.path.join(run, "nav.jsonl")):
        r = json.loads(line)
        if r.get("leg_start"):                     # newer runs log leg changes
            legs.append([round(r["t"] - t0, 1), r["leg_start"]])
        elif t_out is None and r.get("leg") == "outbound":
            t_out = r["t"]
    if not legs and t_out is not None:
        summ = json.load(open(os.path.join(run, "mission_summary.json")))
        tt = t_out - t0
        for name, dur in summ["legs_s"].items():
            legs.append([round(tt, 1), name])
            tt += dur

    # ---- camera frames: video frame i <-> detection record i ----------------
    caps = [json.loads(l)["t_cap"] for l in open(os.path.join(run, "detections_cloud.jsonl"))]
    cap = cv2.VideoCapture(os.path.join(run, "detections.mp4"))
    frames, gif_cam, i, next_t = [], [], 0, -1.0
    while True:
        ok, img = cap.read()
        if not ok or i >= len(caps):
            break
        t = caps[i] - t0
        i += 1
        if t < next_t or t < 0:
            continue
        next_t = t + 0.5
        small = cv2.resize(img, (384, 216), interpolation=cv2.INTER_AREA)
        name = f"f{len(frames):04d}.jpg"
        cv2.imwrite(os.path.join(frames_dir, name), small, [cv2.IMWRITE_JPEG_QUALITY, 72])
        frames.append([round(t, 2), name])
        if len(frames) % 2 == 0:                 # GIF at 1 frame per second of flight
            gif_cam.append(Image.fromarray(cv2.cvtColor(
                cv2.resize(img, (320, 180), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)))
    if gif_cam:
        pal = [f.convert("P", palette=Image.ADAPTIVE, colors=96) for f in gif_cam]
        pal[0].save(os.path.join(media, "camera.gif"), save_all=True, append_images=pal[1:],
                    duration=250, loop=0, optimize=True)

    # ---- top-down GIF of the true path -------------------------------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    objs = gen_hall.objects(cfg)
    fig, ax = plt.subplots(figsize=(6.4, 4.0), dpi=80)
    gif_top = []
    T = path[-1][0]
    n_frames = 70
    for k in range(n_frames + 1):
        tk = T * k / n_frames
        ax.clear()
        for nm, cls, x, y, z, sx, sy, sz in objs:
            col = (0.75, 0.75, 0.77) if cls == "wall" else tuple(cfg["classes"][cls]["rgb"])
            ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color=col))
        seen = [c for c in cells if c[0] <= tk]
        if seen:
            ax.scatter([c[1] for c in seen], [c[2] for c in seen], s=1.5, c="r", marker="s")
        pts = [p for p in path if p[0] <= tk]
        ax.plot([p[1] for p in pts], [p[2] for p in pts], "k-", lw=1.4)
        if pts:
            p = pts[-1]
            ax.plot(p[1], p[2], "o", ms=6, mfc="w", mec="k")
            ax.arrow(p[1], p[2], 2.2 * math.cos(p[4]), 2.2 * math.sin(p[4]),
                     head_width=0.9, color="k")
        leg = [l[1] for l in legs if l[0] <= tk]
        ax.set_title(f"t = {tk:5.1f} s   {leg[-1] if leg else 'take-off'}   "
                     f"(true path from Gazebo; red = camera map)", fontsize=8)
        ax.set_xlim(-6.5, 44.5)
        ax.set_ylim(-15.5, 15.5)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        fig.tight_layout()
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(h, w, 3)
        gif_top.append(Image.fromarray(buf.copy()).convert("P", palette=Image.ADAPTIVE, colors=64))
    gif_top[0].save(os.path.join(media, "topdown.gif"), save_all=True,
                    append_images=gif_top[1:] + [gif_top[-1]] * 8, duration=120, loop=0,
                    optimize=True)

    # ---- data.json ----------------------------------------------------------
    rep = json.load(open(os.path.join(run, "hall_report.json")))
    data = {
        "run": os.path.basename(run),
        "note": "Recorded from the simulation (PX4 SITL + Gazebo). Path is Gazebo's true pose; "
                "the red cells are the drone's own map built from cloud detections.",
        "hall": cfg["hall"], "pad": cfg["pad"],
        "classes": {k: v["rgb"] for k, v in cfg["classes"].items()},
        "objects": [[nm, cls, x, y, z, sx, sy, sz] for nm, cls, x, y, z, sx, sy, sz in objs],
        "path": path, "cells": cells, "legs": legs, "frames": frames,
        "results": {
            "status": rep.get("mission", {}).get("status"),
            "legs_s": rep.get("mission", {}).get("legs_s"),
            "total_s": rep.get("mission", {}).get("total_s"),
            "min_clearance_m": rep.get("hull_clearance_min_m"),
            "landing_error_m": rep.get("landing_error_m"),
            "contact_samples": rep.get("contact_samples"),
            "recall": (rep.get("cv") or {}).get("recall"),
            "range_err_median": (rep.get("cv") or {}).get("range_err_median"),
            "round_trip_ms": rep.get("link_round_trip_ms"),
            "gates": rep.get("gates"),
        },
    }
    json.dump(data, open(os.path.join(out_dir, "data.json"), "w"), separators=(",", ":"))
    sz = lambda p: os.path.getsize(p) / 1e6
    print(f"path {len(path)} samples over {T:.1f} s, {len(cells)} map cells, {len(frames)} frames")
    print(f"data.json {sz(os.path.join(out_dir, 'data.json')):.2f} MB, "
          f"camera.gif {sz(os.path.join(media, 'camera.gif')):.2f} MB, "
          f"topdown.gif {sz(os.path.join(media, 'topdown.gif')):.2f} MB, "
          f"frames {sum(sz(os.path.join(frames_dir, f[1])) for f in frames):.2f} MB")


if __name__ == "__main__":
    main()
