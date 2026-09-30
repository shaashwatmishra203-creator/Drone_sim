#!/usr/bin/env python3
"""summarize_run.py — condense one flight_log.csv into the numbers that matter.

Endurance is reported two ways and they mean different things:
  measured   the run actually reached the DoD gate; this is elapsed time
  projected  the run was cut short, so steady-state power is extrapolated
             through the battery model. Labelled as a projection, not a result.
"""

import csv
import math
import os
import sys

import yaml

CONFIG = os.path.expanduser("~/drone_sim/config/airframe.yaml")


def project(power_w, cfg_path=CONFIG, dt=1.0, cap_s=7200):
    """Integrate the real battery model at a constant power draw."""
    b = yaml.safe_load(open(cfg_path))["battery"]
    cells, cap, r = b["cells_series"], b["capacity_ah"], b["internal_resistance_ohm"]
    floor = 1.0 - b["dod_limit"]
    curve = sorted([(float(s), float(v)) for s, v in b["ocv_curve"]])

    def ocv(soc):
        soc = max(0.0, min(1.0, soc))
        for i in range(len(curve) - 1):
            s0, v0 = curve[i]
            s1, v1 = curve[i + 1]
            if s0 <= soc <= s1:
                fr = 0.0 if s1 == s0 else (soc - s0) / (s1 - s0)
                return (v0 + fr * (v1 - v0)) * cells
        return curve[-1][1] * cells

    soc, t = 1.0, 0.0
    while soc > floor and t < cap_s:
        vo = ocv(soc)
        disc = vo * vo - 4.0 * r * power_w
        if disc <= 0:
            break
        i = (vo - math.sqrt(disc)) / (2.0 * r)
        soc -= (i * dt / 3600.0) / cap
        t += dt
    return t / 60.0


def f(x, d=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return d


def main(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        print("  (empty)")
        return

    # Steady state = airborne, after settling, low vertical rate.
    air = [r for r in rows if f(r["z"]) < -1.0]
    steady = [r for r in air if abs(f(r["vz"])) < 0.3 and f(r["t_s"]) > 20]
    ref = steady or air or rows

    def col(rs, k):
        return [f(r[k]) for r in rs if not math.isnan(f(r[k]))]

    def mean(v):
        return sum(v) / len(v) if v else float("nan")

    t_end = f(rows[-1]["t_s"])
    soc_end = f(rows[-1]["soc"])
    wh = f(rows[-1]["energy_wh"])

    # True measured endurance is when SoC first crossed the DoD gate, NOT the
    # end of the file: after the gate the vehicle lands and then idles on the
    # ground, which would inflate the number.
    t_gate = wh_gate = None
    for r in rows:
        if f(r["soc"]) <= 0.20:
            t_gate = f(r["t_s"])
            wh_gate = f(r["energy_wh"])
            break
    reached = t_gate is not None

    p = mean(col(ref, "power_total_w"))
    cur = mean(col(ref, "current_a"))
    rpm = mean(col(ref, "rpm_mean"))
    thr = mean(col(ref, "thrust_per_motor_g"))
    spd = mean(col(ref, "speed_ms"))

    print(f"  duration          {t_end:7.1f} s  ({t_end/60:.2f} min)")
    print(f"  steady-state pts  {len(ref):7d}")
    print(f"  mean power        {p:7.1f} W")
    print(f"  mean current      {cur:7.1f} A")
    print(f"  mean rotor speed  {rpm:7.0f} rpm")
    print(f"  thrust per motor  {thr:7.0f} g")
    print(f"  mean speed        {spd:7.2f} m/s")
    print(f"  energy used       {wh:7.2f} Wh")
    print(f"  SoC at end        {soc_end:7.3f}")

    if reached:
        print(f"  ENDURANCE         {t_gate/60:7.2f} min  (MEASURED to DoD gate, "
              f"{wh_gate:.1f} Wh)")
    else:
        # Project from the STEADY-STATE power, not from the last row: after a
        # landing the motors are idle and the last row's remaining-time figure
        # is wildly optimistic. Re-run the battery model at the measured
        # steady power so voltage sag and the falling OCV are accounted for.
        print(f"  ENDURANCE         {project(p):7.2f} min  "
              f"(PROJECTED at {p:.0f} W steady draw; run did not reach the gate)")

    # position hold quality, meaningful for the wind scenarios
    if ref and abs(spd) < 2.0:
        xs, ys = col(ref, "x"), col(ref, "y")
        if xs and ys:
            mx, my = mean(xs), mean(ys)
            err = [math.hypot(x - mx, y - my) for x, y in zip(xs, ys)]
            print(f"  pos-hold RMS      {math.sqrt(mean([e*e for e in err])):7.2f} m"
                  f"   max {max(err):.2f} m")


if __name__ == "__main__":
    main(sys.argv[1])
