#!/usr/bin/env python3
"""
gen_docs.py — regenerate the data-driven documentation from the config and the
sizing results, so the docs can never disagree with the model that ran.

Generates:
  docs/01-parts-and-masses.md   the parts + mass register
  docs/02-specifications.md     the derived spec sheet

The narrative docs (00, 03-07) are written by hand and are NOT touched here.

Usage:
  python3 tools/gen_docs.py --config config/airframe.yaml --sizing out/sizing.json
"""

from __future__ import annotations

import argparse
import datetime
import json
import os

import yaml

CONF_MARK = {
    "measured": "measured",
    "vendor": "vendor spec",
    "estimated": "**ESTIMATED**",
}

BANNER = (
    "> **Verification level: L0/L1 — analytical model and software test only.**\n"
    "> These figures describe a *model* of the aircraft. They do not establish\n"
    "> that the aircraft is safe to build, power, or fly.\n"
)


def load(path):
    with open(path) as f:
        return yaml.safe_load(f) if path.endswith((".yaml", ".yml")) else json.load(f)


# ---------------------------------------------------------------------------
def gen_parts(cfg, results, out):
    r = results[0]
    items = r["mass_items"]
    now = datetime.date.today().isoformat()

    est = [i for i in items if i["confidence"] == "estimated"]
    est_kg = sum(i["total_kg"] for i in est)

    L = []
    L.append("# 01 — Parts and Mass Register\n")
    L.append(f"*Generated from `config/airframe.yaml` on {now}. "
             f"Do not edit by hand — edit the config and re-run "
             f"`tools/gen_docs.py`.*\n")
    L.append(BANNER)

    L.append("\n## Mass status\n")
    L.append(f"- **All-up weight (AUW, pi5 variant, parts-list prop): "
             f"{r['auw_kg']:.3f} kg**")
    L.append(f"- Still estimated: **{est_kg:.3f} kg ({100*est_kg/r['auw_kg']:.0f}% "
             f"of AUW)** across {len(est)} line items")
    L.append(f"- Centre of gravity: x={r['cg_m'][0]*1000:+.1f} mm, "
             f"y={r['cg_m'][1]*1000:+.1f} mm, z={r['cg_m'][2]*1000:+.1f} mm "
             f"(FLU, from frame-plate centre)")
    L.append("\nEndurance moves about **2 minutes per 500 g**, so every "
             "estimated row below is directly a source of error in the flight "
             "time. Weigh the bold ones first.\n")

    L.append("\n## Register\n")
    L.append("| Component | Qty | Unit (g) | Total (g) | Confidence | Power (W) | Source |")
    L.append("|---|---:|---:|---:|---|---:|---|")
    for i in sorted(items, key=lambda x: -x["total_kg"]):
        src = " ".join(str(i["source"]).split())
        pw = f"{i['power_w']:.1f}" if i["power_w"] else "—"
        L.append(f"| {i['name']} | {i['qty']} | {i['unit_kg']*1000:.0f} | "
                 f"{i['total_kg']*1000:.0f} | {CONF_MARK.get(i['confidence'], i['confidence'])} "
                 f"| {pw} | {src} |")
    L.append(f"| **TOTAL** | | | **{r['auw_kg']*1000:.0f}** | | "
             f"**{r['avionics_w']:.1f}** | |")

    L.append("\n\n## Compute variants\n")
    L.append("| Variant | Mass (g) | Power (W) | CUDA | Can run ZED cameras |")
    L.append("|---|---:|---:|---|---|")
    for k, v in cfg["payload_variants"].items():
        cuda = "yes" if v.get("cuda_capable") else "**no**"
        zed = "yes" if v.get("cuda_capable") else "**NO — see findings**"
        L.append(f"| {v['name']} | {v['mass_kg']*1000:.0f} | {v['power_w']:.1f} "
                 f"| {cuda} | {zed} |")

    L.append("\n\n## Propeller options\n")
    L.append("| Prop | Dia (in) | Pitch | Mass (g) | Ct | Cp | Max RPM (rated) | FoM | In parts list |")
    L.append("|---|---:|---:|---:|---:|---:|---:|---:|---|")
    for p in cfg["prop_options"]:
        L.append(f"| {p['id']} | {p['diameter_in']:.1f} | {p['pitch_in']:.1f} | "
                 f"{p['mass_kg']*1000:.0f} | {p['ct']:.3f} | {p['cp']:.3f} | "
                 f"{p['max_rpm']:.0f} | {p['figure_of_merit']:.2f} | "
                 f"{'**yes**' if p.get('in_parts_list') else 'no'} |")

    L.append("\n\n## Battery\n")
    b = cfg["battery"]
    L.append(f"- {b['name']}")
    L.append(f"- {b['cells_series']}S, {b['capacity_ah']:.1f} Ah, "
             f"{b['c_rating']}C, **{r['pack_wh']:.0f} Wh** at "
             f"{r['v_nominal']:.1f} V nominal")
    L.append(f"- Internal resistance {b['internal_resistance_ohm']*1000:.0f} mΩ "
             f"({CONF_MARK.get(b.get('confidence'), '')})")
    L.append(f"- Depth-of-discharge gate: {b['dod_limit']*100:.0f}%")

    L.append("\n\n## What still needs weighing\n")
    for i in sorted(est, key=lambda x: -x["total_kg"]):
        L.append(f"- **{i['name']}** — assumed {i['total_kg']*1000:.0f} g. "
                 f"{' '.join(str(i['source']).split())}")

    write(out, "\n".join(L))


# ---------------------------------------------------------------------------
def gen_specs(cfg, results, out):
    now = datetime.date.today().isoformat()
    L = []
    L.append("# 02 — Derived Specifications\n")
    L.append(f"*Generated from `out/sizing.json` on {now}. "
             f"Do not edit by hand.*\n")
    L.append(BANNER)

    r0 = results[0]
    L.append(f"\nAir density **{r0['rho']:.4f} kg/m³** "
             f"({cfg['environment']['altitude_m']:.0f} m, "
             f"{cfg['environment']['temperature_c']:.0f} °C). "
             f"Arm length {cfg['geometry']['arm_length_m']*1000:.0f} mm "
             f"({cfg['geometry']['arm_length_m']*2000:.0f} mm diagonal).\n")

    L.append("\n## Performance summary\n")
    L.append("| Compute | Prop | AUW (kg) | Disc load (kg/m²) | Hover (W) | "
             "Hover (A) | Hover thrust frac | T/W | **Endurance (min)** | "
             "Best cruise (m/s) | Max speed (m/s) |")
    L.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in results:
        star = " ⭐" if r["in_parts_list"] else ""
        be = r["best_endurance"]["v_ms"] if r["best_endurance"] else 0
        L.append(f"| {r['payload']} | {r['prop']}{star} | {r['auw_kg']:.2f} | "
                 f"{r['disc_loading_kgm2']:.1f} | {r['hover']['power_w']:.0f} | "
                 f"{r['hover']['current_a']:.1f} | {r['hover']['throttle']*100:.0f}% | "
                 f"{r['twr']:.2f} | **{r['hover']['minutes']:.1f}** | "
                 f"{be:.1f} | {r['max_speed_ms']:.1f} |")
    L.append("\n⭐ = propeller currently in the parts list.\n")

    L.append("\n## Propulsion detail\n")
    L.append("| Prop | Hover RPM | Max RPM | Rated max RPM | Margin | "
             "Tip speed (m/s) | Tip Mach | Max thrust (kgf) | Limited by |")
    L.append("|---|---:|---:|---:|---:|---:|---:|---:|---|")
    seen = set()
    for r in results:
        if r["prop"] in seen:
            continue
        seen.add(r["prop"])
        c = r["checks"]["prop_rpm_max"]
        flag = "" if c["pass"] else " ⚠️"
        L.append(f"| {r['prop']} | {r['hover']['rpm']:.0f} | {r['max']['rpm']:.0f} | "
                 f"{c['limit']:.0f} | {c['margin_pct']:.1f}%{flag} | "
                 f"{r['max']['tip_speed_ms']:.0f} | {r['max']['tip_mach']:.2f} | "
                 f"{r['max']['thrust_total_kgf']:.2f} | {r['max']['limited_by']} |")

    L.append("\n\n## Mass properties\n")
    L.append("| Compute | Prop | AUW (kg) | Ixx | Iyy | Izz | CG x/y/z (mm) |")
    L.append("|---|---|---:|---:|---:|---:|---|")
    for r in results:
        i = r["inertia"]
        cg = r["cg_m"]
        L.append(f"| {r['payload']} | {r['prop']} | {r['auw_kg']:.3f} | "
                 f"{i['ixx']:.4f} | {i['iyy']:.4f} | {i['izz']:.4f} | "
                 f"{cg[0]*1000:+.0f} / {cg[1]*1000:+.0f} / {cg[2]*1000:+.0f} |")
    L.append("\nInertia in kg·m², computed from the component layout — not "
             "hand-typed, so it tracks any mass change automatically.\n")

    L.append("\n## Gate checks\n")
    L.append("| Compute | Prop | " + " | ".join(results[0]["checks"].keys()) + " |")
    L.append("|---|---|" + "---|" * len(results[0]["checks"]))
    for r in results:
        cells = []
        for k, c in r["checks"].items():
            cells.append(("PASS " if c["pass"] else "**FAIL** ") +
                         f"({c['value']:.2f})")
        L.append(f"| {r['payload']} | {r['prop']} | " + " | ".join(cells) + " |")

    L.append("\nGate definitions, sources and uncertainties are in "
             "`docs/03-methodology.md`. Gates are not relaxed to make a "
             "configuration pass.\n")

    L.append("\n## Cruise performance (pi5, parts-list prop)\n")
    r = next((x for x in results if x["payload"] == "pi5" and x["in_parts_list"]),
             results[0])
    L.append("| Airspeed (m/s) | Power (W) | Endurance (min) | Range (km) |")
    L.append("|---:|---:|---:|---:|")
    for row in r["cruise"]:
        if abs(row["v_ms"] % 2.0) < 1e-6:
            L.append(f"| {row['v_ms']:.0f} | {row['power_w']:.0f} | "
                     f"{row['minutes']:.1f} | {row['range_km']:.2f} |")
    if r["best_range"]:
        L.append(f"\n**Best range** {r['best_range']['range_km']:.2f} km at "
                 f"{r['best_range']['v_ms']:.1f} m/s. "
                 f"**Best endurance** {r['best_endurance']['minutes']:.1f} min at "
                 f"{r['best_endurance']['v_ms']:.1f} m/s.\n")

    write(out, "\n".join(L))


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/airframe.yaml")
    ap.add_argument("--sizing", default="out/sizing.json")
    ap.add_argument("--docdir", default="docs")
    a = ap.parse_args()

    cfg = load(a.config)
    with open(a.sizing) as f:
        results = json.load(f)

    gen_parts(cfg, results, os.path.join(a.docdir, "01-parts-and-masses.md"))
    gen_specs(cfg, results, os.path.join(a.docdir, "02-specifications.md"))


if __name__ == "__main__":
    main()
