# 02 — Derived Specifications

*Generated from `out/sizing.json` on 2026-09-30. Do not edit by hand.*

> **Verification level: L0/L1 — analytical model and software test only.**
> These figures describe a *model* of the aircraft. They do not establish
> that the aircraft is safe to build, power, or fly.


Air density **1.2250 kg/m³** (0 m, 15 °C). Arm length 248 mm (495 mm diagonal).


## Performance summary

| Compute | Prop | AUW (kg) | Disc load (kg/m²) | Hover (W) | Hover (A) | Hover thrust frac | T/W | **Endurance (min)** | Best cruise (m/s) | Max speed (m/s) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| pi5 | 9x4.5 ⭐ | 3.12 | 19.0 | 601 | 28.6 | 29% | 3.45 | **10.7** | 9.5 | 20.0 |
| pi5 | 11x4.5 | 3.13 | 12.8 | 469 | 22.2 | 33% | 3.01 | **13.8** | 9.5 | 20.0 |
| pi5 | 13x4.4 | 3.16 | 9.2 | 388 | 18.2 | 36% | 2.76 | **16.8** | 9.0 | 20.0 |
| orin_nx | 9x4.5 ⭐ | 3.20 | 19.5 | 636 | 30.4 | 30% | 3.37 | **10.1** | 9.0 | 20.0 |
| orin_nx | 11x4.5 | 3.21 | 13.1 | 500 | 23.6 | 34% | 2.94 | **13.0** | 9.5 | 20.0 |
| orin_nx | 13x4.4 | 3.24 | 9.5 | 415 | 19.5 | 37% | 2.69 | **15.7** | 9.0 | 20.0 |

⭐ = propeller currently in the parts list.


## Propulsion detail

| Prop | Hover RPM | Max RPM | Rated max RPM | Margin | Tip speed (m/s) | Tip Mach | Max thrust (kgf) | Limited by |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 9x4.5 | 8569 | 15927 | 16111 | 1.1% | 191 | 0.56 | 10.77 | current |
| 11x4.5 | 5940 | 10310 | 13182 | 21.8% | 151 | 0.44 | 9.44 | current |
| 13x4.4 | 4419 | 7334 | 11154 | 34.2% | 127 | 0.37 | 8.70 | current |


## Mass properties

| Compute | Prop | AUW (kg) | Ixx | Iyy | Izz | CG x/y/z (mm) |
|---|---|---:|---:|---:|---:|---|
| pi5 | 9x4.5 | 3.117 | 0.0342 | 0.0380 | 0.0582 | +3 / -2 / +19 |
| pi5 | 11x4.5 | 3.133 | 0.0347 | 0.0385 | 0.0592 | +3 / -2 / +19 |
| pi5 | 13x4.4 | 3.157 | 0.0355 | 0.0393 | 0.0607 | +3 / -2 / +19 |
| orin_nx | 9x4.5 | 3.197 | 0.0343 | 0.0383 | 0.0584 | +2 / -2 / +19 |
| orin_nx | 11x4.5 | 3.213 | 0.0348 | 0.0388 | 0.0594 | +2 / -2 / +20 |
| orin_nx | 13x4.4 | 3.237 | 0.0356 | 0.0396 | 0.0609 | +2 / -2 / +20 |

Inertia in kg·m², computed from the component layout — not hand-typed, so it tracks any mass change automatically.


## Gate checks

| Compute | Prop | prop_rpm_hover | prop_rpm_max | thrust_to_weight | hover_thrust_fraction |
|---|---|---|---|---|---|
| pi5 | 9x4.5 | PASS (8568.89) | PASS (15927.45) | PASS (3.45) | PASS (0.29) |
| pi5 | 11x4.5 | PASS (5939.50) | PASS (10310.19) | PASS (3.01) | PASS (0.33) |
| pi5 | 13x4.4 | PASS (4418.63) | PASS (7334.38) | PASS (2.76) | PASS (0.36) |
| orin_nx | 9x4.5 | PASS (8678.15) | PASS (15927.45) | PASS (3.37) | PASS (0.30) |
| orin_nx | 11x4.5 | PASS (6014.86) | PASS (10310.19) | PASS (2.94) | PASS (0.34) |
| orin_nx | 13x4.4 | PASS (4474.26) | PASS (7334.38) | PASS (2.69) | PASS (0.37) |

Gate definitions, sources and uncertainties are in `docs/03-methodology.md`. Gates are not relaxed to make a configuration pass.


## Cruise performance (pi5, parts-list prop)

| Airspeed (m/s) | Power (W) | Endurance (min) | Range (km) |
|---:|---:|---:|---:|
| 0 | 601 | 10.7 | 0.00 |
| 2 | 597 | 10.8 | 1.30 |
| 4 | 587 | 11.0 | 2.64 |
| 6 | 574 | 11.2 | 4.04 |
| 8 | 564 | 11.5 | 5.50 |
| 10 | 561 | 11.5 | 6.90 |
| 12 | 572 | 11.3 | 8.14 |
| 14 | 599 | 10.8 | 9.04 |
| 16 | 646 | 10.0 | 9.57 |
| 18 | 716 | 8.9 | 9.65 |
| 20 | 810 | 7.9 | 9.44 |

**Best range** 9.66 km at 17.5 m/s. **Best endurance** 11.5 min at 9.5 m/s.

