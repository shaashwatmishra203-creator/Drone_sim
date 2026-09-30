# 01 — Parts and Mass Register

*Generated from `config/airframe.yaml` on 2026-09-30. Do not edit by hand — edit the config and re-run `tools/gen_docs.py`.*

> **Verification level: L0/L1 — analytical model and software test only.**
> These figures describe a *model* of the aircraft. They do not establish
> that the aircraft is safe to build, power, or fly.


## Mass status

- **All-up weight (AUW, pi5 variant, parts-list prop): 3.117 kg**
- Still estimated: **2.740 kg (88% of AUW)** across 8 line items
- Centre of gravity: x=+2.6 mm, y=-1.6 mm, z=+18.7 mm (FLU, from frame-plate centre)

Endurance moves about **2 minutes per 500 g**, so every estimated row below is directly a source of error in the flight time. Weigh the bold ones first.


## Register

| Component | Qty | Unit (g) | Total (g) | Confidence | Power (W) | Source |
|---|---:|---:|---:|---|---:|---|
| Ovonic 22.2V 6S 50C 6000mAh (XT90 female) | 1 | 830 | 830 | **ESTIMATED** | — | typical 6S 6000mAh LiPo; WEIGH THIS — largest single mass |
| HobbyWing XRotor 3115 900KV | 4 | 195 | 780 | **ESTIMATED** | — | typical 31xx-class outrunner; CONFIRM from vendor datasheet |
| Frame, PETG-CF printed, + nylon legs | 1 | 550 | 550 | **ESTIMATED** | — | 500 mm class printed frame estimate — DERIVE FROM CAD VOLUME |
| 3-DoF gimbal + gimbal camera | 1 | 210 | 210 | **ESTIMATED** | 5.0 | unidentified lab unit — WEIGH AND IDENTIFY |
| Stereolabs ZED 2 | 1 | 180 | 180 | vendor spec | 2.0 | ZED 2 = 166 g body, +cable |
| ALFA AWUS036ACM dual-band USB adapter + antennas | 1 | 170 | 170 | **ESTIMATED** | 3.0 | adapter ~130 g + 2 antennas; CONFIRM |
| Wiring, XT90, standoffs, fasteners, straps | 1 | 120 | 120 | **ESTIMATED** | — | build allowance |
| Raspberry Pi 5 (8 GB) + active cooler + 5V regulator | 1 | 100 | 100 | vendor spec | 12.0 | Pi 5 = 46 g, cooler 25 g, regulator ~30 g |
| Stereolabs ZED Mini | 1 | 62 | 62 | vendor spec | 2.0 | ZED Mini = 62.9 g |
| propeller (see prop_options) | 4 | 10 | 40 | **ESTIMATED** | — | HQProp 9x4.5 nylon composite, typical |
| GPS / compass module + mast | 1 | 40 | 40 | **ESTIMATED** | 0.5 | typical M8N/M9N puck plus mast |
| SpeedyBee F405 V5 stack (FC + 4-in-1 ESC, 30x30) | 1 | 35 | 35 | vendor spec | 2.0 | SpeedyBee F405 V5 product page |
| **TOTAL** | | | **3117** | | **26.5** | |


## Compute variants

| Variant | Mass (g) | Power (W) | CUDA | Can run ZED cameras |
|---|---:|---:|---|---|
| Raspberry Pi 5 (8 GB) + active cooler + 5V regulator | 100 | 12.0 | **no** | **NO — see findings** |
| Jetson Orin NX + carrier + cooler | 180 | 25.0 | yes | yes |


## Propeller options

| Prop | Dia (in) | Pitch | Mass (g) | Ct | Cp | Max RPM (rated) | FoM | In parts list |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 9x4.5 | 9.0 | 4.5 | 10 | 0.112 | 0.048 | 16111 | 0.58 | **yes** |
| 11x4.5 | 11.0 | 4.5 | 14 | 0.105 | 0.042 | 13182 | 0.62 | no |
| 13x4.4 | 13.0 | 4.4 | 20 | 0.098 | 0.036 | 11154 | 0.65 | no |


## Battery

- Ovonic 22.2V 6S1P 50C 6000mAh
- 6S, 6.0 Ah, 50C, **136 Wh** at 22.7 V nominal
- Internal resistance 30 mΩ (**ESTIMATED**)
- Depth-of-discharge gate: 80%


## What still needs weighing

- **Ovonic 22.2V 6S 50C 6000mAh (XT90 female)** — assumed 830 g. typical 6S 6000mAh LiPo; WEIGH THIS — largest single mass
- **HobbyWing XRotor 3115 900KV** — assumed 780 g. typical 31xx-class outrunner; CONFIRM from vendor datasheet
- **Frame, PETG-CF printed, + nylon legs** — assumed 550 g. 500 mm class printed frame estimate — DERIVE FROM CAD VOLUME
- **3-DoF gimbal + gimbal camera** — assumed 210 g. unidentified lab unit — WEIGH AND IDENTIFY
- **ALFA AWUS036ACM dual-band USB adapter + antennas** — assumed 170 g. adapter ~130 g + 2 antennas; CONFIRM
- **Wiring, XT90, standoffs, fasteners, straps** — assumed 120 g. build allowance
- **propeller (see prop_options)** — assumed 40 g. HQProp 9x4.5 nylon composite, typical
- **GPS / compass module + mast** — assumed 40 g. typical M8N/M9N puck plus mast
