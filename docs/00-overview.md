# 00 — Overview

## What this is

A physics model and a PX4/Gazebo simulation of the drone team's quadrotor,
built to answer one question before anyone prints a frame or solders an XT90:

> **Will it fly, and for how long?**

It answers that two independent ways — an analytical model and a full
software-in-the-loop flight — and cross-checks them against each other.

---

## ⚠️ What this can and cannot tell you

> **Verification level: L0–L2 — analytical model and software-in-the-loop only.**

This work establishes that **a model of this aircraft flies in simulation**. It
does **not** establish that the real aircraft is safe to build, power, or fly.
Nothing here has touched hardware. No propeller has been spun, no pack
discharged, no thrust measured on a stand.

Concretely:

| The simulation CAN tell you | The simulation CANNOT tell you |
|---|---|
| Whether the thrust budget closes at all | Whether these specific motors survive this prop |
| Roughly how long it hovers | Actual pack capacity or sag under real load |
| How much wind costs in power and endurance | Whether the printed frame is stiff enough |
| Which prop choice is better, and by how much | Whether the ESCs overheat |
| Where the design is marginal | Whether the flight controller can run the autonomy stack |

Every number here inherits the uncertainty of its inputs, and **88% of the
mass budget is still estimated rather than measured**. Treat the results as a
band, not a verdict, until the team weighs the aircraft.

---

## Headline results

See `05-results.md` for the full matrix. In short, with the current parts list
(9×4.5 props, Raspberry Pi 5, estimated 3.12 kg AUW):

- **It flies.** Thrust-to-weight ≈ 3.4, hover at ~29% of available thrust.
- **Hover endurance ≈ 10–11 minutes** in still air.
- **Wind costs about 23%** — 6 m/s wind with 3 m/s gusts pushed hover power
  from 570 W to 699 W, cutting projected endurance to ~9.2 min.
- **The 9-inch propeller is the weak point**, not the thrust budget. At full
  throttle it turns ~15,900 rpm against a rated ceiling of ~16,100 — about 1%
  margin. Larger props would buy ~6 more minutes.
- **The ZED cameras cannot run on the Raspberry Pi 5** at all. Unrelated to
  flight dynamics, but it invalidates the perception stack as specified.

---

## How to reproduce everything

Everything runs in **WSL Ubuntu 22.04**. Nothing runs on Windows directly.

```bash
cd ~/drone_sim && python3 tools/sizing.py --config config/airframe.yaml --sweep-auw
```

```bash
cd ~/drone_sim && python3 tools/gen_model.py --config config/airframe.yaml --install
```

```bash
cd ~/PX4-Autopilot && make px4_sitl_default
```

```bash
bash ~/drone_sim/scripts/run_scenario.sh ~/drone_sim/scenarios/hover_endurance.yaml 9x4.5 pi5
```

```bash
bash ~/drone_sim/scripts/batch.sh
```

```bash
cd ~/drone_sim && python3 tools/gen_docs.py
```

---

## Document map

| Doc | Contents |
|---|---|
| `01-parts-and-masses.md` | Parts register, every mass with its confidence. **Generated** |
| `02-specifications.md` | Derived spec sheet, all variants. **Generated** |
| `03-methodology.md` | The physics, the assumptions, and the gates |
| `04-simulation-setup.md` | How the PX4/Gazebo/ROS 2 stack is wired |
| `05-results.md` | Per-scenario results and the analytical-vs-SITL cross-check |
| `06-findings.md` | Verdict and prioritized part changes |
| `07-worklog.md` | What was run, what broke, what was decided |

`01` and `02` are regenerated from `config/airframe.yaml` — edit the config,
never those files.

## The one thing to change first

Weigh the aircraft. The frame, gimbal, battery and motors are all estimates,
and endurance moves roughly **2 minutes per 500 g**. Every number here
tightens the moment real masses land in `config/airframe.yaml`.
