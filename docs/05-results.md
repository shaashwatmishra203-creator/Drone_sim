# 05 — Results

> **Verification level: L0–L2.** Analytical model and software-in-the-loop only.
> No hardware has been tested. 88% of the mass budget is still estimated.

All runs: estimated AUW 3.12 kg (pi5) / 3.20 kg (orin_nx), 6S 6000 mAh,
sea-level ISA, DoD gate 80%.

---

## The cross-check that matters

Two independent models — closed-form momentum theory, and PX4 flying the
aircraft in Gazebo with power derived from actual motor commands. They share
only `config/airframe.yaml`.

**Still-air hover, 9×4.5, pi5:**

| Quantity | Analytical | SITL measured | Δ |
|---|---:|---:|---:|
| Rotor speed | 8569 rpm | 8619 rpm | **0.6%** |
| Thrust per motor | 779 g | 788 g | **1.2%** |
| Hover power | 601 W | 570.6 W | **5.1%** |
| Endurance | 10.7 min | **11.39 min** | **6.4%** |
| Hover motor command | 0.492 predicted | 0.496 observed | 0.8% |

Agreement to ~1% on rotor speed and ~5–6% on power and endurance. The residual
is expected and explainable: the analytical side uses momentum theory with a
figure of merit, while the logger integrates `Cp` against the actual rpm. They
are not the same approximation, so exact agreement would be suspicious.

**This is the single most important result in the project.** It is what makes
the rest of the numbers worth quoting.

---

## Scenario results

### hover_endurance — 9×4.5, pi5, still air ✅ complete

Held 5 m until the battery model reached the 80% DoD gate.

| | |
|---|---:|
| **Endurance (measured to DoD gate)** | **11.39 min** |
| Energy used to gate | 107.5 Wh |
| Steady hover power | 570.6 W |
| Steady current | 25.5 A |
| Rotor speed | 8619 rpm |
| Thrust per motor | 788 g |
| Position-hold RMS | 0.02 m (max 0.05 m) |

Position hold in still air is essentially perfect, as expected with no
disturbance and a 3.4 thrust-to-weight ratio.

### hover_windy — 9×4.5, pi5, 6 m/s + 3 m/s gusts ✅ complete

Wind ramped in over ~12 s after reaching altitude, then gusts applied as
perturbations around the mean.

| | Still air | 6 m/s + gusts | Penalty |
|---|---:|---:|---:|
| Hover power | 570.6 W | **698.7 W** | **+22.5%** |
| Current | 25.5 A | 30.0 A | +17.6% |
| Rotor speed | 8619 rpm | 9237 rpm | +7.2% |
| Thrust per motor | 788 g | 907 g | +15.1% |
| Endurance | 11.39 min | **~9.2 min** (projected) | **−19%** |
| Position-hold RMS | 0.02 m | **0.93 m** (max 9.75 m) | 46× worse |

The 9.75 m peak excursion happened during the wind ramp, before the controller
settled against the new steady disturbance.

**Plan real missions around 7–8 minutes of usable time with reserve.** The
"11 minute" figure is a still-air number and should not be used for mission
planning.

### hover_endurance — 13×4.4, pi5 ⏳ partial

Started but interrupted. Early steady-state reading at t = 113 s:

| | Analytical | SITL (partial) |
|---|---:|---:|
| Hover power | 388 W | **384.8 W** |

**0.8% agreement** — an independent confirmation of the prop recommendation
using a completely different propeller diameter, which exercises the
`motorConstant` derivation rather than just reproducing one tuned point.

### Pending

| Scenario | Purpose | Status |
|---|---|---|
| hover_endurance 13×4.4 pi5 | Full endurance on the recommended prop | ⏳ interrupted, needs re-run |
| hover_endurance 9×4.5 orin_nx | Cost of the CUDA compute variant | ⏳ not yet run |
| cruise_sweep | Max speed, best-endurance and best-range cruise | ⏳ not yet run |
| climb_test | Vertical thrust margin | ⏳ not yet run |
| forest_mission | Realistic waypoint mission with wind | ⏳ not yet run |

Re-run with `bash scripts/batch2.sh`, then regenerate docs with
`python3 tools/gen_docs.py`.

---

## Analytical matrix (all six variants)

From `tools/sizing.py`. SITL has confirmed the 9×4.5/pi5 row end-to-end and the
13×4.4/pi5 hover power.

| Compute | Prop | AUW (kg) | Disc load (kg/m²) | Hover (W) | Hover (A) | Thrust frac | T/W | **Endurance (min)** | Best cruise (m/s) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pi5 | **9×4.5** ⭐ | 3.12 | 19.0 | 601 | 28.6 | 29% | 3.45 | **10.7** | 9.5 |
| pi5 | 11×4.5 | 3.13 | 12.8 | 469 | 22.2 | 33% | 3.01 | **13.8** | 9.5 |
| pi5 | 13×4.4 | 3.16 | 9.2 | 388 | 18.2 | 36% | 2.76 | **16.8** | 9.0 |
| orin_nx | **9×4.5** ⭐ | 3.20 | 19.5 | 636 | 30.4 | 30% | 3.37 | **10.1** | 9.0 |
| orin_nx | 11×4.5 | 3.21 | 13.1 | 500 | 23.6 | 34% | 2.94 | **13.0** | 9.5 |
| orin_nx | 13×4.4 | 3.24 | 9.5 | 415 | 19.5 | 37% | 2.69 | **15.7** | 9.0 |

⭐ = propeller currently in the parts list. **All gates pass in all six
configurations** — flyability is not the constraint.

### Propeller rpm margin

| Prop | Hover rpm | Max rpm | Rated max | Margin | Tip Mach |
|---|---:|---:|---:|---:|---:|
| **9×4.5** | 8569 | 15,927 | 16,111 | **1.1%** ⚠️ | 0.56 |
| 11×4.5 | 5940 | 10,310 | 13,182 | 21.8% | 0.44 |
| 13×4.4 | 4419 | 7,334 | 11,154 | 34.2% | 0.37 |

The 9″ prop reaches its rated ceiling at full throttle. See
[06-findings.md](06-findings.md) — and note the rated figure is a rule of
thumb, not an HQProp specification.

### Mass sensitivity (pi5, 9×4.5)

| AUW (kg) | Hover (W) | Endurance (min) | T/W |
|---:|---:|---:|---:|
| 2.4 | 415 | 15.7 | 4.49 |
| 2.8 | 515 | 12.6 | 3.85 |
| **3.2** | **624** | **10.3** | **3.37** |
| 3.6 | 739 | 8.6 | 2.99 |
| 4.0 | 861 | 7.4 | 2.69 |

**Roughly 2 minutes per 500 g.** With 88% of the mass still estimated, this
table is the honest expression of how much the endurance figure could move.
Weighing the aircraft collapses it to a single row.

---

## Reading the numbers responsibly

- **Endurance is to the 80% DoD gate**, not to a flat pack. Flying past it
  trades pack life for minutes.
- **"Measured" means measured in simulation.** It inherits every modelling
  assumption in [03-methodology.md](03-methodology.md), above all the
  unmeasured `Ct`/`Cp`.
- **Still-air endurance is a benchmark, not a mission planning figure.** Use
  the windy result.
- **The analytical/SITL agreement validates internal consistency, not physical
  accuracy.** Both models share `config/airframe.yaml`; if `Ct` is wrong, they
  are both wrong together and would still agree. Only a thrust stand settles that.
