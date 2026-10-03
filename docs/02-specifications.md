# 02 — Derived Specifications

*Generated from `out/sizing.json` on 2026-10-03. Do not edit by hand.*

> **Verification level: L0/L1 — analytical model and software test only.**
> These figures describe a *model* of the aircraft. They do not establish
> that the aircraft is safe to build, power, or fly.


Air density **1.2250 kg/m³** (0 m, 15 °C). Arm length 248 mm (495 mm diagonal).


## Performance summary

| Compute | Prop | AUW (kg) | Disc load (kg/m²) | Hover (W) | Hover (A) | Hover thrust frac | T/W | **Endurance (min)** | Best cruise (m/s) | Max speed (m/s) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| pi5 | 9x4.5 ⭐ | 2.82 | 17.2 | 518 | 24.5 | 26% | 3.81 | **12.5** | 9.0 | 20.0 |
| pi5 | 11x4.5 | 2.84 | 11.6 | 405 | 19.0 | 30% | 3.32 | **16.1** | 8.5 | 20.0 |
| pi5 | 13x4.4 | 2.86 | 8.4 | 335 | 15.7 | 33% | 3.04 | **19.5** | 8.5 | 20.0 |
| orin_nx | 9x4.5 ⭐ | 2.90 | 17.7 | 552 | 26.2 | 27% | 3.71 | **11.7** | 8.5 | 20.0 |
| orin_nx | 11x4.5 | 2.92 | 11.9 | 434 | 20.5 | 31% | 3.23 | **15.0** | 9.0 | 20.0 |
| orin_nx | 13x4.4 | 2.94 | 8.6 | 361 | 16.9 | 34% | 2.95 | **18.1** | 8.5 | 20.0 |

⭐ = propeller currently in the parts list.


## Propulsion detail

| Prop | Hover RPM | Max RPM | Rated max RPM | Margin | Tip speed (m/s) | Tip Mach | Max thrust (kgf) | Limited by |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 9x4.5 | 8158 | 15927 | 16111 | 1.1% | 191 | 0.56 | 10.77 | current |
| 11x4.5 | 5656 | 10310 | 13182 | 21.8% | 151 | 0.44 | 9.44 | current |
| 13x4.4 | 4209 | 7334 | 11154 | 34.2% | 127 | 0.37 | 8.70 | current |


## Mass properties

| Compute | Prop | AUW (kg) | Ixx | Iyy | Izz | CG x/y/z (mm) |
|---|---|---:|---:|---:|---:|---|
| pi5 | 9x4.5 | 2.825 | 0.0322 | 0.0343 | 0.0554 | -3 / +0 / +20 |
| pi5 | 11x4.5 | 2.841 | 0.0327 | 0.0348 | 0.0564 | -3 / +0 / +20 |
| pi5 | 13x4.4 | 2.865 | 0.0335 | 0.0356 | 0.0579 | -3 / +0 / +20 |
| orin_nx | 9x4.5 | 2.905 | 0.0323 | 0.0345 | 0.0555 | -3 / +0 / +21 |
| orin_nx | 11x4.5 | 2.921 | 0.0329 | 0.0351 | 0.0565 | -3 / +0 / +21 |
| orin_nx | 13x4.4 | 2.945 | 0.0336 | 0.0358 | 0.0580 | -3 / +0 / +21 |

Inertia in kg·m², computed from the component layout — not hand-typed, so it tracks any mass change automatically.


## Gate checks

| Compute | Prop | prop_rpm_hover | prop_rpm_max | thrust_to_weight | hover_thrust_fraction |
|---|---|---|---|---|---|
| pi5 | 9x4.5 | PASS (8157.65) | PASS (15927.45) | PASS (3.81) | PASS (0.26) |
| pi5 | 11x4.5 | PASS (5655.95) | PASS (10310.19) | PASS (3.32) | PASS (0.30) |
| pi5 | 13x4.4 | PASS (4209.33) | PASS (7334.38) | PASS (3.04) | PASS (0.33) |
| orin_nx | 9x4.5 | PASS (8272.35) | PASS (15927.45) | PASS (3.71) | PASS (0.27) |
| orin_nx | 11x4.5 | PASS (5735.03) | PASS (10310.19) | PASS (3.23) | PASS (0.31) |
| orin_nx | 13x4.4 | PASS (4267.69) | PASS (7334.38) | PASS (2.95) | PASS (0.34) |

Gate definitions, sources and uncertainties are in `docs/03-methodology.md`. Gates are not relaxed to make a configuration pass.


## Cruise performance (pi5, parts-list prop)

| Airspeed (m/s) | Power (W) | Endurance (min) | Range (km) |
|---:|---:|---:|---:|
| 0 | 518 | 12.5 | 0.00 |
| 2 | 514 | 12.6 | 1.51 |
| 4 | 505 | 12.8 | 3.08 |
| 6 | 494 | 13.1 | 4.73 |
| 8 | 485 | 13.4 | 6.42 |
| 10 | 485 | 13.4 | 8.04 |
| 12 | 498 | 13.0 | 9.38 |
| 14 | 529 | 12.2 | 10.28 |
| 16 | 579 | 11.1 | 10.69 |
| 18 | 652 | 9.9 | 10.66 |
| 20 | 748 | 8.5 | 10.24 |

**Best range** 10.74 km at 17.0 m/s. **Best endurance** 13.4 min at 9.0 m/s.

