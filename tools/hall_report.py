#!/usr/bin/env python3
"""
hall_report.py - judge a hall run against the TRUE geometry and TRUE pose.

Inputs (all in the run directory): truth.csv (Gazebo pose, truth_logger),
nav.jsonl / mission_summary.json (hall_mission), detections_cloud.jsonl
(cloud_detector), detections_edge.jsonl (hall_mission), link_stats.json.

Nothing here uses PX4's position estimate as truth. Clearance and contacts
come from Gazebo's pose against tools/gen_hall.objects(); the computer vision
is scored by projecting the same objects through the true camera pose.

Gates (provenance in docs; not relaxed when a run misses them):
  hull clearance >= 0.4 m from every object, every sample
  landing within 0.5 m of the pad (truth)
  CV recall >= 0.90 for objects in view, >= 50% unoccluded, <= 8 m away
  edge range error (floor method) median <= 15%
"""

import argparse
import bisect
import csv
import json
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_hall  # noqa: E402

CAM_FWD, CAM_DZ, CAM_PITCH = 0.09, -0.025, math.radians(15.0)


def load_truth(p):
    rows = []
    with open(p) as f:
        for r in csv.DictReader(f):
            rows.append(tuple(float(r[k]) for k in ("t", "x", "y", "z", "qw", "qx", "qy", "qz")))
    return rows


def at(truth, ts, t, tol=0.15):
    i = bisect.bisect_left(ts, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(ts) and (best is None or abs(ts[j] - t) < abs(ts[best] - t)):
            best = j
    return truth[best] if best is not None and abs(ts[best] - t) <= tol else None


def qrot(q):
    w, x, y, z = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def aabb_dist(px, py, pz, o):
    _, _, cx, cy, cz, sx, sy, sz = o
    dx = max(abs(px - cx) - sx / 2, 0.0)
    dy = max(abs(py - cy) - sy / 2, 0.0)
    dz = max(abs(pz - cz) - sz / 2, 0.0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def footprint_dist(px, py, o):
    _, _, cx, cy, _, sx, sy, _ = o
    dx = max(abs(px - cx) - sx / 2, 0.0)
    dy = max(abs(py - cy) - sy / 2, 0.0)
    return math.hypot(dx, dy)


def ray_footprint(px, py, ang, o):
    """Distance along a horizontal ray to an object's footprint (slab method)."""
    _, _, cx, cy, _, sx, sy, _ = o
    dx, dy = math.cos(ang), math.sin(ang)
    t0, t1 = -1e9, 1e9
    for p, d, lo, hi in ((px, dx, cx - sx / 2, cx + sx / 2), (py, dy, cy - sy / 2, cy + sy / 2)):
        if abs(d) < 1e-9:
            if p < lo or p > hi:
                return None
            continue
        a, b = (lo - p) / d, (hi - p) / d
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
    return t0 if t1 >= t0 and t0 > 0 else None


class TrueCamera:
    def __init__(self, cfg):
        c = cfg["camera"]
        self.w, self.h = c["width"], c["height"]
        self.f = (self.w / 2.0) / math.tan(math.radians(c["hfov_deg"]) / 2.0)
        cp, sp = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
        self.Rmount = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]   # +pitch about y = nose down

    def pose(self, tr):
        _, x, y, z, qw, qx, qy, qz = tr
        Rb = qrot((qw, qx, qy, qz))
        C = [x + v for v, x in zip(mv(Rb, [CAM_FWD, 0, CAM_DZ]), (x, y, z))]
        R = mm(Rb, self.Rmount)
        return C, R

    def project_box(self, C, R, o):
        _, _, cx, cy, cz, sx, sy, sz = o
        us, vs, depths = [], [], []
        for ix in (-1, 1):
            for iy in (-1, 1):
                for iz in (-1, 1):
                    P = (cx + ix * sx / 2 - C[0], cy + iy * sy / 2 - C[1], cz + iz * sz / 2 - C[2])
                    pc = [R[0][k] * P[0] + R[1][k] * P[1] + R[2][k] * P[2] for k in range(3)]
                    if pc[0] < 0.1:
                        continue
                    us.append(self.w / 2 - self.f * pc[1] / pc[0])
                    vs.append(self.h / 2 - self.f * pc[2] / pc[0])
                    depths.append(pc[0])
        if len(us) < 4:
            return None
        u0, u1 = max(min(us), 0), min(max(us), self.w - 1)
        v0, v1 = max(min(vs), 0), min(max(vs), self.h - 1)
        if u1 <= u0 or v1 <= v0:
            return None
        return (u0, v0, u1, v1, min(depths))


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def score_cv(run, cfg, objs, truth, ts):
    p = os.path.join(run, "detections_cloud.jsonl")
    if not os.path.exists(p):
        return None
    cam = TrueCamera(cfg)
    targets = [o for o in objs if o[1] != "wall"]
    gt_n = gt_hit = det_n = det_hit = frames = 0
    per_class = {}
    for line in open(p):
        r = json.loads(line)
        tr = at(truth, ts, r["t_cap"])
        if tr is None or tr[3] < 1.0:          # only score frames in flight
            continue
        frames += 1
        C, R = cam.pose(tr)
        proj = []
        for o in targets:
            pb = cam.project_box(C, R, o)
            if pb:
                proj.append((o, pb))
        dets = r["dets"]
        det_n += len(dets)
        matched_det = set()
        for o, (u0, v0, u1, v1, dep) in proj:
            # simple occlusion: area of this box covered by a nearer box
            area = (u1 - u0) * (v1 - v0)
            covered = 0.0
            for o2, (a0, b0, a1, b1, d2) in proj:
                if o2 is o or d2 >= dep - 0.3:
                    continue
                covered += max(0, min(u1, a1) - max(u0, a0)) * max(0, min(v1, b1) - max(v0, b0))
            for k, d in enumerate(dets):
                if d[0] == o[1] and iou((u0, v0, u1, v1), d[1:5]) >= 0.3:
                    matched_det.add(k)
            visible = 1.0 - min(1.0, covered / area) if area > 0 else 0.0
            dist = footprint_dist(C[0], C[1], o)
            if visible < 0.5 or area < 400 or dist > 8.0:
                continue
            gt_n += 1
            hit = any(d[0] == o[1] and iou((u0, v0, u1, v1), d[1:5]) >= 0.3 for d in dets)
            gt_hit += hit
            pc = per_class.setdefault(o[1], [0, 0])
            pc[0] += 1
            pc[1] += hit
        det_hit += len(matched_det)
    # edge range error (floor method only: the other methods are bounds)
    errs, bound_ok, bound_n = [], 0, 0
    pe = os.path.join(run, "detections_edge.jsonl")
    if os.path.exists(pe):
        for line in open(pe):
            r = json.loads(line)
            tr = at(truth, ts, r["t_cap"])
            if tr is None or tr[3] < 1.0:
                continue
            C, Rm = cam.pose(tr)
            yaw = math.atan2(Rm[1][0], Rm[0][0])
            for cls, rng, bl, br, how in r["dets"]:
                ang = yaw + (bl + br) / 2.0
                cands = [ray_footprint(C[0], C[1], ang, o) for o in targets if o[1] == cls]
                cands = [c for c in cands if c is not None]
                if not cands:
                    continue
                true = min(cands)
                if how in ("floor", "contact") and true <= 9.0:
                    errs.append(abs(rng - true) / true)
                elif how not in ("floor", "contact"):
                    # what the drone actually uses for these: an obstacle at
                    # half the returned range. Safe only if nearer than truth.
                    bound_n += 1
                    bound_ok += 0.5 * rng <= true
    errs.sort()
    return {
        "frames_scored": frames,
        "recall": gt_hit / gt_n if gt_n else None, "gt_instances": gt_n,
        "precision": det_hit / det_n if det_n else None, "detections": det_n,
        "recall_by_class": {k: round(v[1] / v[0], 3) for k, v in per_class.items() if v[0]},
        "range_err_median": errs[len(errs) // 2] if errs else None,
        "range_err_p90": errs[int(len(errs) * 0.9)] if errs else None,
        "range_samples": len(errs),
        "close_flags_placed_nearer_than_truth": f"{bound_ok}/{bound_n}",
    }


def plot(run, cfg, objs, truth, nav):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except Exception as e:
        return f"plot skipped: {e}"
    fig, ax = plt.subplots(figsize=(13, 8))
    for nm, cls, cx, cy, cz, sx, sy, sz in objs:
        col = (0.7, 0.7, 0.72) if cls == "wall" else tuple(cfg["classes"][cls]["rgb"])
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color=col))
        if cls != "wall":
            ax.text(cx, cy, nm, ha="center", va="center", fontsize=7)
    p = cfg["pad"]
    ax.add_patch(Rectangle((p["xy"][0] - p["size"] / 2, p["xy"][1] - p["size"] / 2),
                           p["size"], p["size"], fill=False, ls="--", color="k"))
    if truth:
        ax.plot([r[1] for r in truth], [r[2] for r in truth], "k-", lw=1.6,
                label="TRUE path (Gazebo)")
    if nav:
        ax.plot([n["pos"][0] for n in nav if "pos" in n], [n["pos"][1] for n in nav if "pos" in n],
                "m:", lw=1.0, label="PX4 estimate (converted to world)")
    mp = os.path.join(run, "camera_map.json")
    if os.path.exists(mp):
        cells = json.load(open(mp))["cells"]
        ax.scatter([c["x"] for c in cells], [c["y"] for c in cells], s=6, c="r",
                   marker="s", label="camera map (from cloud detections)")
    ax.set_aspect("equal")
    ax.set_xlim(cfg["hall"]["x"][0] - 1, cfg["hall"]["x"][1] + 1)
    ax.set_ylim(cfg["hall"]["y"][0] - 1, cfg["hall"]["y"][1] + 1)
    ax.set_xlabel("x East (m)")
    ax.set_ylabel("y North (m)")
    ax.set_title("Factory hall - true flight path, obstacles and camera map")
    ax.legend(loc="upper left", fontsize=8)
    out = os.path.join(run, "hall_path.png")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--config", default=gen_hall.DEFAULT_CFG)
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    objs = gen_hall.objects(cfg)
    M = cfg["mission"]
    tp = os.path.join(a.run, "truth.csv")
    truth = load_truth(tp) if os.path.exists(tp) else []
    ts = [r[0] for r in truth]
    nav = [json.loads(l) for l in open(os.path.join(a.run, "nav.jsonl"))] \
        if os.path.exists(os.path.join(a.run, "nav.jsonl")) else []
    rep = {"truth_samples": len(truth)}
    if not truth:
        print("NO TRUTH DATA - cannot judge this run")
        return
    airborne = [r for r in truth if r[3] > 0.5]
    radius = float(M["airframe_radius_m"])
    worst = (float("inf"), None, None)
    contacts = 0
    for r in airborne:
        for o in objs:
            d = aabb_dist(r[1], r[2], r[3], o) - radius
            if d < worst[0]:
                worst = (d, o[0], r)
        if min(aabb_dist(r[1], r[2], r[3], o) for o in objs) - radius <= 0:
            contacts += 1
    xs = [r[1] for r in airborne] or [0]
    rep.update({
        "furthest_east_m": round(max(xs), 2),
        "hull_clearance_min_m": round(worst[0], 3),
        "closest_object": worst[1],
        "contact_samples": contacts,
        "landing_error_m": round(math.hypot(truth[-1][1] - cfg["pad"]["xy"][0],
                                            truth[-1][2] - cfg["pad"]["xy"][1]), 3),
        "final_height_m": round(truth[-1][3], 2),
    })
    # PX4-vs-truth agreement: the frame check, on every navigation sample
    gaps = []
    for n in nav:
        if "pos" in n:
            tr = at(truth, ts, n["t"], 0.2)
            if tr:
                gaps.append(math.hypot(n["pos"][0] - tr[1], n["pos"][1] - tr[2]))
    if gaps:
        gaps.sort()
        rep["px4_vs_truth_median_m"] = round(gaps[len(gaps) // 2], 3)
        rep["px4_vs_truth_max_m"] = round(gaps[-1], 3)
    fc = [n for n in nav if "frame_check" in n]
    if fc:
        rep["frame_check"] = []
        for n in fc:
            tr = at(truth, ts, n["t"], 0.3)
            rep["frame_check"].append({"target_enu": n["target"],
                                       "truth": [round(tr[1], 2), round(tr[2], 2)] if tr else None})
    s = os.path.join(a.run, "mission_summary.json")
    if os.path.exists(s):
        rep["mission"] = json.load(open(s))
    lk = os.path.join(a.run, "link_stats.json")
    if os.path.exists(lk):
        rep["link_round_trip_ms"] = json.load(open(lk)).get("round_trip_ms")
    rep["cv"] = score_cv(a.run, cfg, objs, truth, ts)
    rep["plot"] = plot(a.run, cfg, objs, truth, nav)

    g = {}
    g["clearance >= 0.4 m"] = rep["hull_clearance_min_m"] >= 0.4
    g["no contact"] = contacts == 0
    if rep.get("mission", {}).get("status", "").startswith("complete"):
        g["landed within 0.5 m"] = rep["landing_error_m"] <= 0.5
    cv = rep["cv"] or {}
    if cv.get("recall") is not None:
        g["CV recall >= 0.90"] = cv["recall"] >= 0.90
    if cv.get("range_err_median") is not None:
        g["range error median <= 15%"] = cv["range_err_median"] <= 0.15
    rep["gates"] = g
    json.dump(rep, open(os.path.join(a.run, "hall_report.json"), "w"), indent=2)

    print(f"TRUTH: furthest east {rep['furthest_east_m']} m | min hull clearance "
          f"{rep['hull_clearance_min_m']} m ({rep['closest_object']}) | contact samples {contacts}")
    print(f"       landing error {rep['landing_error_m']} m, final height {rep['final_height_m']} m")
    if gaps:
        print(f"       PX4 estimate vs truth: median {rep['px4_vs_truth_median_m']} m, "
              f"max {rep['px4_vs_truth_max_m']} m")
    if fc:
        for f_ in rep["frame_check"]:
            print(f"       frame check: target {f_['target_enu']} -> truth {f_['truth']}")
    if cv:
        print(f"CV:    recall {cv['recall'] if cv['recall'] is None else round(cv['recall'], 3)} "
              f"({cv['gt_instances']} instances, {cv['frames_scored']} frames) | precision "
              f"{cv['precision'] if cv['precision'] is None else round(cv['precision'], 3)} | "
              f"by class {cv['recall_by_class']}")
        if cv["range_err_median"] is not None:
            print(f"       range error (floor) median {100 * cv['range_err_median']:.1f}% "
                  f"p90 {100 * cv['range_err_p90']:.1f}% over {cv['range_samples']} | "
                  f"close-flag placed nearer than truth {cv['close_flags_placed_nearer_than_truth']}")
    print("GATES: " + ", ".join(f"{k}: {'PASS' if v else 'FAIL'}" for k, v in g.items()))
    print(f"plot:  {rep['plot']}")


if __name__ == "__main__":
    main()
