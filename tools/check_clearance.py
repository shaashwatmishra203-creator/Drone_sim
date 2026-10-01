#!/usr/bin/env python3
"""
check_clearance.py — how close did the aircraft actually get to anything?

Measures the flown trajectory against the same obstacle geometry the world is
generated from, so "it navigated an obstacle course" is a measurement rather
than an impression.

Reports, per obstacle, the closest approach in 3D and whether the path ever
passed BETWEEN two obstacles (which is what makes a course a course, as opposed
to a gentle curve around things standing off to the side).

Usage: python3 tools/check_clearance.py [run_dir]
"""

import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from runio import read_rows  # noqa: E402
from gen_world import obstacle_list          # noqa: E402

ROOT = os.path.expanduser("~/drone_sim")
AIRFRAME_RADIUS = 0.35      # half the 0.5 m frame plus prop overhang


def box_distance(p, box):
    """Distance from point p to the surface of an axis-aligned box.
    Zero if inside."""
    _, cx, cy, cz, sx, sy, sz = box
    dx = max(abs(p[0] - cx) - sx / 2.0, 0.0)
    dy = max(abs(p[1] - cy) - sy / 2.0, 0.0)
    dz = max(abs(p[2] - cz) - sz / 2.0, 0.0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def main(run_dir):
    p = os.path.join(run_dir, "flight_log.csv")
    rows = read_rows(p, verbose=True)

    def fl(x):
        try:
            return float(x)
        except (TypeError, ValueError):
            return float("nan")

    # ENU: log z is NED-down, so height above ground is -z
    path = [(fl(r["x"]), fl(r["y"]), -fl(r["z"])) for r in rows
            if fl(r["z"]) < -0.8]
    if not path:
        print("no airborne data")
        return

    obs = obstacle_list()
    print("=" * 72)
    print("OBSTACLE CLEARANCE — measured against the flown path")
    print("=" * 72)
    print(f"  path points: {len(path)}   airframe radius assumed "
          f"{AIRFRAME_RADIUS:.2f} m\n")

    results = []
    for b in obs:
        d = min(box_distance(q, b) for q in path)
        results.append((b[0], d, d - AIRFRAME_RADIUS))

    results.sort(key=lambda r: r[1])
    print(f"  {'obstacle':<20} {'closest (m)':>12} {'gap to hull':>12}  assessment")
    print("  " + "-" * 68)
    for name, d, gap in results:
        if gap < 0.3:
            a = "TIGHT — real avoidance needed"
        elif gap < 1.0:
            a = "close"
        elif gap < 2.0:
            a = "comfortable"
        else:
            a = "never a factor"
        print(f"  {name:<20} {d:12.2f} {gap:12.2f}  {a}")

    tight = [r for r in results if r[2] < 1.0]
    never = [r for r in results if r[2] > 2.0]

    print("\n  SUMMARY")
    print(f"    obstacles within 1.0 m of the hull : {len(tight)} of {len(obs)}")
    print(f"    obstacles never closer than 2.0 m  : {len(never)} of {len(obs)}")
    print(f"    absolute closest approach          : {results[0][1]:.2f} m "
          f"({results[0][0]})")

    # Did the path ever pass BETWEEN two obstacles?
    print("\n  SQUEEZES (path passing between two obstacles at once)")
    squeezes = 0
    for q in path:
        near = sorted(((box_distance(q, b), b[0]) for b in obs))[:2]
        if len(near) == 2 and near[0][0] < 2.5 and near[1][0] < 2.5:
            squeezes += 1
    if squeezes:
        print(f"    {squeezes} path points had two obstacles within 2.5 m")
    else:
        print("    NONE — the aircraft was never between two obstacles at once.")
        print("    The course is a curve around scenery, not a slalom.")

    print("\n" + "=" * 72)
    if not tight:
        print("  VERDICT: the course does NOT constrain the flight path.")
        print("  Nothing came within 1 m of the airframe. A drone with no")
        print("  perception at all would fly this successfully, which means it")
        print("  tests waypoint following, not obstacle navigation.")
    else:
        print(f"  VERDICT: {len(tight)} obstacle(s) genuinely constrained the path.")
    print("=" * 72)


if __name__ == "__main__":
    d = sys.argv[1] if len(sys.argv) > 1 else \
        f"{ROOT}/out/runs/factory_mission_9x45_pi5"
    main(d)
