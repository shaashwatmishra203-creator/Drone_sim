#!/usr/bin/env python3
"""
analyze_mission.py — mission-level analysis of the factory navigation run.

Answers the operational questions the hover tests cannot:
  - how long each leg actually took
  - how much energy the mapping pass costs versus the transit
  - how much reserve is left on touchdown
  - how many round trips one pack supports

Usage: python3 tools/analyze_mission.py [run_dir]
"""

import csv
import math
import os
import re
import sys

import yaml

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from runio import read_rows

ROOT = os.path.expanduser("~/drone_sim")


def f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def leg_times(runner_log):
    """Pull the leg durations the mission runner reported."""
    legs, total = [], None
    if not os.path.exists(runner_log):
        return legs, total
    for ln in open(runner_log):
        m = re.search(r"LEG (\w+): ([\d.]+) s", ln)
        if m:
            legs.append((m.group(1), float(m.group(2))))
        m = re.search(r"TOTAL\s+([\d.]+) s", ln)
        if m:
            total = float(m.group(1))
    return legs, total


def main(run_dir):
    csv_path = os.path.join(run_dir, "flight_log.csv")
    rows = read_rows(csv_path, verbose=True)
    legs, total = leg_times(os.path.join(run_dir, "runner.log"))

    cfg = yaml.safe_load(open(f"{ROOT}/config/airframe.yaml"))
    b = cfg["battery"]
    pack_wh = b["capacity_ah"] * b["nominal_v_per_cell"] * b["cells_series"]
    usable_wh = pack_wh * b["dod_limit"]

    air = [r for r in rows if f(r["z"]) < -0.8]
    if not air:
        print("never left the ground")
        return

    t0, t1 = f(air[0]["t_s"]), f(air[-1]["t_s"])
    wh0, wh1 = f(air[0]["energy_wh"]), f(air[-1]["energy_wh"])
    soc_end = f(air[-1]["soc"])

    # path length actually flown
    dist = 0.0
    prev = None
    for r in air:
        p = (f(r["x"]), f(r["y"]), f(r["z"]))
        if prev:
            dist += math.dist(p, prev)
        prev = p

    powers = [f(r["power_total_w"]) for r in air]
    speeds = [f(r["speed_ms"]) for r in air]
    mean_p = sum(powers) / len(powers)
    peak_p = max(powers)
    mean_v = sum(speeds) / len(speeds)
    peak_v = max(speeds)

    airborne = t1 - t0
    wh_used = wh1 - wh0

    print("=" * 66)
    print("FACTORY NAVIGATION MISSION")
    print("=" * 66)
    print(f"  airborne time      {airborne:8.1f} s   ({airborne/60:.2f} min)")
    if total:
        print(f"  mission legs       {total:8.1f} s   ({total/60:.2f} min)")
    print(f"  path flown         {dist:8.1f} m")
    print(f"  mean speed         {mean_v:8.2f} m/s  (peak {peak_v:.2f})")
    print(f"  mean power         {mean_p:8.1f} W    (peak {peak_p:.0f})")
    print(f"  energy used        {wh_used:8.2f} Wh   of {usable_wh:.0f} Wh usable")
    print(f"  pack remaining     {soc_end*100:8.1f} %")

    if legs:
        print("\n  LEG BREAKDOWN")
        print(f"    {'leg':10s} {'time (s)':>9s} {'share':>7s}")
        lt = sum(d for _, d in legs)
        for n, d in legs:
            print(f"    {n:10s} {d:9.1f} {100*d/lt:6.1f}%")

    # energy headroom
    reserve_frac = (soc_end - (1.0 - b["dod_limit"])) / b["dod_limit"]
    trips = usable_wh / wh_used if wh_used > 0 else 0
    print("\n  OPERATIONAL")
    print(f"    usable energy consumed   {100*wh_used/usable_wh:5.1f} %")
    print(f"    reserve above DoD gate   {100*reserve_frac:5.1f} %")
    print(f"    round trips per charge   {trips:5.1f}")
    print(f"    energy per metre flown   {wh_used/dist*1000:5.1f} mWh/m")

    verdict = ("COMFORTABLE" if trips >= 2.5 else
               "ADEQUATE" if trips >= 1.8 else
               "MARGINAL" if trips >= 1.2 else "INSUFFICIENT")
    print(f"\n  VERDICT: {verdict} — one pack supports {trips:.1f} of these missions")
    print("=" * 66)


if __name__ == "__main__":
    d = sys.argv[1] if len(sys.argv) > 1 else \
        f"{ROOT}/out/runs/factory_mission_9x45_pi5"
    main(d)
