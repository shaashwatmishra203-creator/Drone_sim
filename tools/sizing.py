#!/usr/bin/env python3
"""
sizing.py — analytical flyability / endurance model for the drone team's quad.

Physics, in order:
  1. Mass rollup + CG + inertia tensor from the component register.
  2. BLDC motor + propeller coefficient model  -> max thrust, T/W, hover RPM.
  3. Momentum theory with figure of merit      -> hover shaft power.
  4. Battery OCV + internal-resistance model   -> endurance by time-stepped
                                                  discharge to the DoD gate.
  5. Forward flight (induced + profile + parasite) -> best-endurance and
                                                  best-range cruise speeds.

Verification level: L0/L1 (analytical model + software test). This does NOT
establish that the aircraft is safe to build or fly.

Usage:
  python3 sizing.py --config config/airframe.yaml
  python3 sizing.py --config config/airframe.yaml --sweep-auw
  python3 sizing.py --config config/airframe.yaml --json out/sizing.json
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass, field, asdict

try:
    import yaml
except ImportError:
    sys.exit("PyYAML required:  pip3 install pyyaml")

IN2M = 0.0254
RHO_SL = 1.225


# ---------------------------------------------------------------------------
# Atmosphere
# ---------------------------------------------------------------------------
def air_density(alt_m: float, temp_c: float) -> float:
    """ISA-ish density with a temperature override. Density altitude matters
    for endurance; do not silently assume sea-level standard."""
    p = 101325.0 * (1.0 - 2.25577e-5 * alt_m) ** 5.25588
    t = temp_c + 273.15
    return p / (287.058 * t)


# ---------------------------------------------------------------------------
# Mass / inertia
# ---------------------------------------------------------------------------
@dataclass
class MassItem:
    id: str
    name: str
    qty: int
    unit_kg: float
    total_kg: float
    confidence: str
    source: str
    power_w: float
    positions: list = field(default_factory=list)


def arm_positions(pos, arm_len):
    """Mirror a single |x|,|y| magnitude onto the four quad-X arms."""
    _, _, z = pos
    d = arm_len / math.sqrt(2.0)
    return [(d, d, z), (-d, -d, z), (d, -d, z), (-d, d, z)]


def build_mass_register(cfg, payload_key, prop):
    """Returns (items, total_kg). Prop mass comes from the chosen prop option,
    not the placeholder in the component list."""
    arm = cfg["geometry"]["arm_length_m"]
    items = []

    for c in cfg["components"]:
        unit = float(c["mass_kg"])
        if c["id"] == "prop":
            unit = float(prop["mass_kg"])
        qty = int(c.get("qty", 1))
        if c.get("mirror") == "arms":
            positions = arm_positions(c["pos"], arm)
        else:
            positions = [tuple(c["pos"])] * qty
        items.append(MassItem(
            id=c["id"], name=c["name"], qty=qty, unit_kg=unit,
            total_kg=unit * qty, confidence=c.get("confidence", "estimated"),
            source=c.get("source", ""), power_w=float(c.get("power_w", 0.0)),
            positions=positions))

    pv = cfg["payload_variants"][payload_key]
    items.append(MassItem(
        id=f"compute:{payload_key}", name=pv["name"], qty=1,
        unit_kg=float(pv["mass_kg"]), total_kg=float(pv["mass_kg"]),
        confidence=pv.get("confidence", "estimated"), source=pv.get("source", ""),
        power_w=float(pv.get("power_w", 0.0)), positions=[tuple(pv["pos"])]))

    return items, sum(i.total_kg for i in items)


def cg_and_inertia(items, body):
    """CG from component positions; inertia as point masses about the CG plus
    a solid-box term for the distributed body mass. Computed, never hand-typed,
    so it cannot go stale when a mass changes."""
    m_tot = sum(i.total_kg for i in items)
    cg = [0.0, 0.0, 0.0]
    for it in items:
        mpp = it.total_kg / len(it.positions)
        for p in it.positions:
            for k in range(3):
                cg[k] += mpp * p[k]
    cg = [c / m_tot for c in cg]

    ixx = iyy = izz = 0.0
    for it in items:
        mpp = it.total_kg / len(it.positions)
        for p in it.positions:
            dx, dy, dz = p[0] - cg[0], p[1] - cg[1], p[2] - cg[2]
            ixx += mpp * (dy * dy + dz * dz)
            iyy += mpp * (dx * dx + dz * dz)
            izz += mpp * (dx * dx + dy * dy)

    # Distributed-body term so a point-mass model does not under-predict inertia.
    mb = m_tot * 0.25
    bx, by, bz = body["x_m"], body["y_m"], body["z_m"]
    ixx += mb * (by * by + bz * bz) / 12.0
    iyy += mb * (bx * bx + bz * bz) / 12.0
    izz += mb * (bx * bx + by * by) / 12.0

    return cg, (ixx, iyy, izz)


# ---------------------------------------------------------------------------
# Propeller + motor
# ---------------------------------------------------------------------------
def prop_thrust(n_rps, ct, dia_m, rho):
    return ct * rho * n_rps ** 2 * dia_m ** 4


def prop_shaft_power(n_rps, cp, dia_m, rho):
    return cp * rho * n_rps ** 3 * dia_m ** 5


def prop_torque(n_rps, cp, dia_m, rho):
    return cp * rho * n_rps ** 2 * dia_m ** 5 / (2.0 * math.pi)


def rps_for_thrust(thrust_n, ct, dia_m, rho):
    if thrust_n <= 0:
        return 0.0
    return math.sqrt(thrust_n / (ct * rho * dia_m ** 4))


def motor_max_operating_point(motor, prop, v_bat, rho):
    """Equilibrium of the BLDC motor and the propeller at the current limit,
    then check whether voltage binds first. Returns the binding limit."""
    kv = motor["kv"]
    ke = 60.0 / (2.0 * math.pi * kv)          # V per rad/s
    kt = ke                                    # N.m per A (SI identity)
    i_max = min(float(motor["i_max_cont_a"]), float(motor.get("_esc_i_max", 1e9)))
    rm = motor["rm_ohm"]
    i0 = motor["i0_a"]
    d_m = prop["diameter_in"] * IN2M

    # --- current-limited operating point -----------------------------------
    q_avail = kt * max(i_max - i0, 0.0)
    n_i = math.sqrt(2.0 * math.pi * q_avail / (prop["cp"] * rho * d_m ** 5))
    v_needed = ke * (2.0 * math.pi * n_i) + i_max * rm

    if v_needed <= v_bat:
        n, i, limit = n_i, i_max, "current"
    else:
        # --- voltage-limited: solve motor torque == prop torque at full duty
        lo, hi = 0.0, kv * v_bat / 60.0
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            omega = 2.0 * math.pi * mid
            i_draw = (v_bat - ke * omega) / rm
            q_motor = kt * max(i_draw - i0, 0.0)
            if q_motor > prop_torque(mid, prop["cp"], d_m, rho):
                lo = mid
            else:
                hi = mid
        n = 0.5 * (lo + hi)
        i = (v_bat - ke * 2.0 * math.pi * n) / rm
        limit = "voltage"

    return {
        "rpm": n * 60.0,
        "thrust_n": prop_thrust(n, prop["ct"], d_m, rho),
        "current_a": i,
        "shaft_w": prop_shaft_power(n, prop["cp"], d_m, rho),
        "limited_by": limit,
    }


# ---------------------------------------------------------------------------
# Battery
# ---------------------------------------------------------------------------
class Battery:
    def __init__(self, cfg):
        self.cells = int(cfg["cells_series"])
        self.cap_ah = float(cfg["capacity_ah"])
        self.r = float(cfg["internal_resistance_ohm"])
        self.dod = float(cfg["dod_limit"])
        self.curve = sorted([(float(s), float(v)) for s, v in cfg["ocv_curve"]])

    def ocv(self, soc):
        soc = max(0.0, min(1.0, soc))
        pts = self.curve
        for i in range(len(pts) - 1):
            s0, v0 = pts[i]
            s1, v1 = pts[i + 1]
            if s0 <= soc <= s1:
                f = 0.0 if s1 == s0 else (soc - s0) / (s1 - s0)
                return (v0 + f * (v1 - v0)) * self.cells
        return pts[-1][1] * self.cells

    def draw(self, power_w, soc):
        """Terminal voltage and current for a constant-power draw, including
        internal-resistance sag. Solves R*I^2 - Vocv*I + P = 0."""
        vo = self.ocv(soc)
        disc = vo * vo - 4.0 * self.r * power_w
        if disc <= 0:
            return None, None          # power exceeds what the pack can deliver
        i = (vo - math.sqrt(disc)) / (2.0 * self.r)
        return vo - i * self.r, i

    def nominal_wh(self):
        return self.ocv(0.5) * self.cap_ah


# ---------------------------------------------------------------------------
# Flight power
# ---------------------------------------------------------------------------
def induced_velocity(thrust_n, rho, area_m2, v_free=0.0):
    """Glauert induced velocity; reduces to hover when v_free = 0."""
    vh = math.sqrt(thrust_n / (2.0 * rho * area_m2))
    if v_free <= 1e-6:
        return vh
    vi = vh
    for _ in range(100):
        new = thrust_n / (2.0 * rho * area_m2 * math.sqrt(v_free ** 2 + vi ** 2))
        if abs(new - vi) < 1e-9:
            break
        vi = 0.5 * (vi + new)
    return vi


def flight_power(mass_kg, v_ms, rho, disc_area, fom, eta_motor, cd_a, g):
    """Total electrical power at a given airspeed.

    Shaft power decomposes as induced + profile + parasite. Profile power is
    calibrated from the hover figure of merit and held roughly constant, which
    is the standard first-order treatment.
    """
    weight = mass_kg * g
    drag = 0.5 * rho * cd_a * v_ms ** 2
    thrust = math.sqrt(weight ** 2 + drag ** 2)     # tilted to beat drag

    vh = induced_velocity(weight, rho, disc_area, 0.0)
    p_profile = weight * vh * (1.0 / fom - 1.0)     # from hover FoM

    vi = induced_velocity(thrust, rho, disc_area, v_ms)
    p_induced = thrust * vi
    p_parasite = drag * v_ms

    shaft = p_induced + p_profile + p_parasite
    return shaft / eta_motor, shaft, thrust


def endurance(batt: Battery, mass_kg, v_ms, rho, disc_area, fom, eta_motor,
              cd_a, avionics_w, g, dt=1.0, max_s=7200):
    """Time-stepped discharge to the DoD gate. Not Wh / W — the pack sags and
    the OCV falls as it empties, and both shorten the flight."""
    soc = 1.0
    soc_floor = 1.0 - batt.dod
    t = 0.0
    wh = 0.0
    p_prop, _, _ = flight_power(mass_kg, v_ms, rho, disc_area, fom,
                                eta_motor, cd_a, g)
    p_tot = p_prop + avionics_w

    peak_i = 0.0
    while t < max_s and soc > soc_floor:
        v_term, i = batt.draw(p_tot, soc)
        if v_term is None:
            return {"ok": False, "reason": "power demand exceeds pack capability",
                    "minutes": 0.0, "power_w": p_tot}
        peak_i = max(peak_i, i)
        soc -= (i * dt / 3600.0) / batt.cap_ah
        wh += p_tot * dt / 3600.0
        t += dt

    return {"ok": True, "minutes": t / 60.0, "power_w": p_tot,
            "prop_power_w": p_prop, "current_a": peak_i, "wh_used": wh,
            "v_end": batt.ocv(soc)}


# ---------------------------------------------------------------------------
# Full analysis for one configuration
# ---------------------------------------------------------------------------
def analyse(cfg, payload_key, prop, auw_override=None):
    env = cfg["environment"]
    g = env["gravity"]
    rho = air_density(env["altitude_m"], env["temperature_c"])

    items, mass = build_mass_register(cfg, payload_key, prop)
    if auw_override is not None:
        mass = auw_override
    cg, inertia = cg_and_inertia(items, cfg["geometry"]["body"])

    avionics_w = sum(i.power_w for i in items)
    d_m = prop["diameter_in"] * IN2M
    disc_single = math.pi * (d_m / 2.0) ** 2
    disc_total = 4.0 * disc_single

    batt = Battery(cfg["battery"])
    v_nom = batt.ocv(0.5)

    motor = dict(cfg["motor"])
    motor["_esc_i_max"] = cfg["esc"]["i_max_cont_a"]
    mx = motor_max_operating_point(motor, prop, v_nom, rho)

    weight_n = mass * g
    thrust_per_motor = weight_n / 4.0
    hover_rps = rps_for_thrust(thrust_per_motor, prop["ct"], d_m, rho)
    hover_rpm = hover_rps * 60.0

    fom = prop["figure_of_merit"]
    eta = cfg["assumptions"]["motor_esc_efficiency"]
    cd_a = cfg["assumptions"]["drag"]["cd_a_m2"]

    hov = endurance(batt, mass, 0.0, rho, disc_total, fom, eta, cd_a,
                    avionics_w, g)

    twr = (4.0 * mx["thrust_n"]) / weight_n
    # Two different "throttle" numbers, routinely conflated:
    #   thrust fraction = hover thrust / max thrust  (what the "hover under
    #     50-60% throttle" design rule actually refers to)
    #   rpm fraction    = hover rpm / max rpm        (closer to stick position,
    #     since duty maps roughly linearly to rpm)
    # Gate on the thrust fraction; report both.
    hover_thrust_frac = 1.0 / twr
    hover_rpm_frac = hover_rpm / max(mx["rpm"], 1e-9)

    # cruise sweep
    cruise = []
    for v in [x * 0.5 for x in range(0, 41)]:
        e = endurance(batt, mass, v, rho, disc_total, fom, eta, cd_a,
                      avionics_w, g, dt=2.0)
        if not e["ok"]:
            break
        cruise.append({"v_ms": v, "power_w": e["power_w"],
                       "minutes": e["minutes"],
                       "range_km": v * e["minutes"] * 60.0 / 1000.0})
    best_end = max(cruise, key=lambda r: r["minutes"]) if cruise else None
    best_rng = max(cruise, key=lambda r: r["range_km"]) if cruise else None

    gates = cfg["gates"]
    checks = {
        "prop_rpm_hover": {
            "value": hover_rpm, "limit": prop["max_rpm"],
            "pass": hover_rpm <= prop["max_rpm"],
            "margin_pct": 100.0 * (prop["max_rpm"] - hover_rpm) / prop["max_rpm"]},
        "prop_rpm_max": {
            "value": mx["rpm"], "limit": prop["max_rpm"],
            "pass": mx["rpm"] <= prop["max_rpm"],
            "margin_pct": 100.0 * (prop["max_rpm"] - mx["rpm"]) / prop["max_rpm"]},
        "thrust_to_weight": {
            "value": twr, "limit": gates["thrust_to_weight_min"]["value"],
            "pass": twr >= gates["thrust_to_weight_min"]["value"]},
        "hover_thrust_fraction": {
            "value": hover_thrust_frac,
            "limit": gates["hover_thrust_fraction_max"]["value"],
            "pass": hover_thrust_frac <= gates["hover_thrust_fraction_max"]["value"]},
    }

    tip_speed = (hover_rps * 2 * math.pi) * (d_m / 2.0)
    max_tip = (mx["rpm"] / 60.0 * 2 * math.pi) * (d_m / 2.0)

    return {
        "payload": payload_key,
        "prop": prop["id"],
        "in_parts_list": prop.get("in_parts_list", False),
        "rho": rho,
        "auw_kg": mass,
        "cg_m": cg,
        "inertia": {"ixx": inertia[0], "iyy": inertia[1], "izz": inertia[2]},
        "avionics_w": avionics_w,
        "disc_area_m2": disc_total,
        "disc_loading_kgm2": mass / disc_total,
        "pack_wh": batt.nominal_wh(),
        "v_nominal": v_nom,
        "hover": {
            "rpm": hover_rpm,
            "throttle": hover_thrust_frac,
            "rpm_fraction": hover_rpm_frac,
            "thrust_per_motor_n": thrust_per_motor,
            "thrust_per_motor_g": thrust_per_motor / g * 1000.0,
            "power_w": hov["power_w"],
            "prop_power_w": hov.get("prop_power_w"),
            "current_a": hov.get("current_a"),
            "minutes": hov["minutes"],
            "ok": hov["ok"],
            "tip_speed_ms": tip_speed,
            "tip_mach": tip_speed / 340.3,
        },
        "max": {
            "rpm": mx["rpm"], "thrust_per_motor_n": mx["thrust_n"],
            "thrust_total_n": 4 * mx["thrust_n"],
            "thrust_total_kgf": 4 * mx["thrust_n"] / g,
            "current_per_motor_a": mx["current_a"],
            "limited_by": mx["limited_by"],
            "tip_speed_ms": max_tip, "tip_mach": max_tip / 340.3,
        },
        "twr": twr,
        "best_endurance": best_end,
        "best_range": best_rng,
        "max_speed_ms": cruise[-1]["v_ms"] if cruise else 0.0,
        "cruise": cruise,
        "checks": checks,
        "mass_items": [asdict(i) for i in items],
    }


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def fmt_check(c):
    return "PASS" if c["pass"] else "FAIL"


def print_report(results, cfg):
    print("=" * 78)
    print("FLYABILITY & ENDURANCE — ANALYTICAL MODEL")
    print("Verification level: L0/L1 (analytical). NOT a hardware-readiness claim.")
    print("=" * 78)

    r0 = results[0]
    print(f"\nAir density {r0['rho']:.4f} kg/m3  |  pack {r0['pack_wh']:.0f} Wh "
          f"@ {r0['v_nominal']:.1f} V nominal  |  DoD gate "
          f"{cfg['battery']['dod_limit']*100:.0f}%")

    prov = [i for i in r0["mass_items"] if i["confidence"] == "estimated"]
    est_kg = sum(i["total_kg"] for i in prov)
    print(f"Mass provisional: {est_kg:.3f} kg of {r0['auw_kg']:.3f} kg "
          f"({100*est_kg/r0['auw_kg']:.0f}%) is still ESTIMATED.")

    print("\n" + "-" * 78)
    hdr = (f"{'payload':9s} {'prop':8s} {'AUW':>6s} {'disc':>6s} {'hover':>7s} "
           f"{'I_hov':>6s} {'thr':>5s} {'T/W':>5s} {'endur':>7s} {'vbest':>6s}")
    print(hdr)
    print(f"{'':9s} {'':8s} {'kg':>6s} {'kg/m2':>6s} {'W':>7s} {'A':>6s} "
          f"{'%':>5s} {'':>5s} {'min':>7s} {'m/s':>6s}")
    print("-" * 78)
    for r in results:
        tag = "*" if r["in_parts_list"] else " "
        be = r["best_endurance"]["v_ms"] if r["best_endurance"] else 0
        print(f"{r['payload']:9s} {r['prop']+tag:8s} {r['auw_kg']:6.2f} "
              f"{r['disc_loading_kgm2']:6.1f} {r['hover']['power_w']:7.0f} "
              f"{r['hover']['current_a']:6.1f} {100*r['hover']['throttle']:5.1f} "
              f"{r['twr']:5.2f} {r['hover']['minutes']:7.1f} {be:6.1f}")
    print("-" * 78)
    print("* = propeller currently in the parts list")

    print("\nGATE CHECKS")
    print("-" * 78)
    for r in results:
        bad = [k for k, v in r["checks"].items() if not v["pass"]]
        status = "OK" if not bad else "FAILS: " + ", ".join(bad)
        print(f"  {r['payload']:9s} {r['prop']:8s}  {status}")
        for k, c in r["checks"].items():
            if not c["pass"]:
                print(f"      {k:18s} {c['value']:.3f} vs limit {c['limit']:.3f}"
                      f"   [{fmt_check(c)}]")

    print("\nPROP RPM DETAIL (the 9-inch question)")
    print("-" * 78)
    print(f"  {'prop':8s} {'hover rpm':>10s} {'max rpm':>10s} {'rated':>8s} "
          f"{'margin':>8s} {'tip Mach':>9s}")
    seen = set()
    for r in results:
        if r["prop"] in seen:
            continue
        seen.add(r["prop"])
        c = r["checks"]["prop_rpm_max"]
        print(f"  {r['prop']:8s} {r['hover']['rpm']:10.0f} {r['max']['rpm']:10.0f} "
              f"{c['limit']:8.0f} {c['margin_pct']:7.1f}% "
              f"{r['max']['tip_mach']:9.2f}")


def write_csv(results, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cols = ["payload", "prop", "in_parts_list", "auw_kg", "disc_loading_kgm2",
            "hover_power_w", "hover_current_a", "hover_throttle", "hover_rpm",
            "hover_minutes", "twr", "max_thrust_kgf", "max_rpm", "limited_by",
            "best_endurance_v_ms", "best_endurance_min", "best_range_v_ms",
            "best_range_km", "max_speed_ms", "gates_failed"]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in results:
            bad = ";".join(k for k, v in r["checks"].items() if not v["pass"])
            be, br = r["best_endurance"], r["best_range"]
            w.writerow([
                r["payload"], r["prop"], r["in_parts_list"],
                f"{r['auw_kg']:.4f}", f"{r['disc_loading_kgm2']:.2f}",
                f"{r['hover']['power_w']:.1f}", f"{r['hover']['current_a']:.2f}",
                f"{r['hover']['throttle']:.4f}", f"{r['hover']['rpm']:.0f}",
                f"{r['hover']['minutes']:.2f}", f"{r['twr']:.3f}",
                f"{r['max']['thrust_total_kgf']:.2f}", f"{r['max']['rpm']:.0f}",
                r["max"]["limited_by"],
                f"{be['v_ms']:.1f}" if be else "", f"{be['minutes']:.2f}" if be else "",
                f"{br['v_ms']:.1f}" if br else "", f"{br['range_km']:.2f}" if br else "",
                f"{r['max_speed_ms']:.1f}", bad])


def sweep_auw(cfg, payload_key, prop, lo=2.3, hi=4.2, step=0.1):
    out = []
    v = lo
    while v <= hi + 1e-9:
        r = analyse(cfg, payload_key, prop, auw_override=v)
        out.append({"auw_kg": v, "hover_power_w": r["hover"]["power_w"],
                    "hover_minutes": r["hover"]["minutes"], "twr": r["twr"],
                    "hover_throttle": r["hover"]["throttle"]})
        v += step
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--outdir", default="out")
    ap.add_argument("--json")
    ap.add_argument("--sweep-auw", action="store_true")
    ap.add_argument("--plots", action="store_true")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    results = []
    for payload in cfg["payload_variants"]:
        for prop in cfg["prop_options"]:
            results.append(analyse(cfg, payload, prop))

    print_report(results, cfg)

    os.makedirs(args.outdir, exist_ok=True)
    write_csv(results, os.path.join(args.outdir, "sizing.csv"))
    print(f"\nwrote {os.path.join(args.outdir, 'sizing.csv')}")

    payload = json.dumps(results, indent=2)
    jpath = args.json or os.path.join(args.outdir, "sizing.json")
    with open(jpath, "w") as f:
        f.write(payload)
    print(f"wrote {jpath}")

    if args.sweep_auw:
        base = next(p for p in cfg["prop_options"] if p.get("in_parts_list"))
        sw = sweep_auw(cfg, "pi5", base)
        p = os.path.join(args.outdir, "sweep_auw.csv")
        with open(p, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["auw_kg", "hover_power_w", "hover_minutes", "twr",
                        "hover_throttle"])
            for row in sw:
                w.writerow([f"{row['auw_kg']:.2f}", f"{row['hover_power_w']:.1f}",
                            f"{row['hover_minutes']:.2f}", f"{row['twr']:.3f}",
                            f"{row['hover_throttle']:.4f}"])
        print(f"wrote {p}")
        print("\nAUW SENSITIVITY (pi5, parts-list prop)")
        print(f"  {'AUW kg':>7s} {'hover W':>8s} {'endur min':>10s} {'T/W':>6s}")
        for row in sw:
            if abs(round(row["auw_kg"], 2) * 10 % 2) < 1e-6:
                print(f"  {row['auw_kg']:7.1f} {row['hover_power_w']:8.0f} "
                      f"{row['hover_minutes']:10.1f} {row['twr']:6.2f}")


if __name__ == "__main__":
    main()
