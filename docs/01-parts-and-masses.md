# 01 — Parts and Mass Register

*Generated from `config/airframe.yaml` on 2026-10-03. Do not edit by hand — edit the config and re-run `tools/gen_docs.py`.*

> **Verification level: L0/L1 — analytical model and software test only.**
> These figures describe a *model* of the aircraft. They do not establish
> that the aircraft is safe to build, power, or fly.


## Mass status

- **All-up weight (AUW, pi5 variant, parts-list prop): 2.825 kg**
- Still estimated: **2.680 kg (95% of AUW)** across 8 line items
- Centre of gravity: x=-2.6 mm, y=+0.0 mm, z=+20.1 mm (FLU, from frame-plate centre)

Endurance moves about **2 minutes per 500 g**, so every estimated row below is directly a source of error in the flight time. Weigh the bold ones first.


## Register

| Component | Qty | Unit (g) | Total (g) | Confidence | Power (W) | Source |
|---|---:|---:|---:|---|---:|---|
| Ovonic 22.2V 6S 50C 6000mAh (XT90 female) | 1 | 830 | 830 | **ESTIMATED** | — | typical 6S 6000mAh LiPo; WEIGH THIS — largest single mass |
| HobbyWing XRotor 3115 900KV | 4 | 195 | 780 | **ESTIMATED** | — | typical 31xx-class outrunner; CONFIRM from vendor datasheet |
| Frame, PETG-CF printed, + nylon legs | 1 | 550 | 550 | **ESTIMATED** | — | 500 mm class printed frame estimate — DERIVE FROM CAD VOLUME |
| 3-DoF gimbal + gimbal camera (lab unit, unidentified) | 1 | 210 | 210 | **ESTIMATED** | 5.0 | unidentified lab unit — WEIGH AND IDENTIFY. Carried as mass and power only; it is not the navigation sensor. |
| ALFA AWUS036ACM AC1200 USB 3.0 + 2x 5dBi antennas | 1 | 145 | 145 | **ESTIMATED** | 3.0 | adapter ~105 g + 2x 5dBi antennas ~20 g each; CONFIRM on a scale |
| Wiring, XT90, standoffs, fasteners, straps | 1 | 120 | 120 | **ESTIMATED** | — | build allowance |
| Raspberry Pi 5 (8 GB) + active cooler + 5V regulator | 1 | 100 | 100 | vendor spec | 12.0 | Pi 5 = 46 g, cooler 25 g, regulator ~30 g. SPECIFIED BRAIN. |
| HQProp 9x4.5 nylon composite | 4 | 10 | 40 | **ESTIMATED** | — | HQProp 9x4.5 nylon composite, typical |
| SpeedyBee F405 V5 stack (FC + 4-in-1 ESC, 30x30, 2-6S) | 1 | 35 | 35 | vendor spec | 2.0 | SpeedyBee F405 V5 Standard, Betaflight-configurable |
| Arducam Camera Module 3 (IMX708, 12MP, 75 deg DFOV, autofocus) | 1 | 10 | 10 | vendor spec | 0.5 | Pi Camera Module 3 board ~4 g + 15-22 pin FFC cable. THE mapping / detection / navigation sensor. CSI to the Pi 5. |
| Standard IMU breakout (ICM-42688-P class) + cable | 1 | 5 | 5 | **ESTIMATED** | 0.1 | 6-axis MEMS IMU breakout; the FC carries its own, this is the dedicated one for the navigation stack on the compute board |
| **TOTAL** | | | **2825** | | **22.6** | |


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
- **3-DoF gimbal + gimbal camera (lab unit, unidentified)** — assumed 210 g. unidentified lab unit — WEIGH AND IDENTIFY. Carried as mass and power only; it is not the navigation sensor.
- **ALFA AWUS036ACM AC1200 USB 3.0 + 2x 5dBi antennas** — assumed 145 g. adapter ~105 g + 2x 5dBi antennas ~20 g each; CONFIRM on a scale
- **Wiring, XT90, standoffs, fasteners, straps** — assumed 120 g. build allowance
- **HQProp 9x4.5 nylon composite** — assumed 40 g. HQProp 9x4.5 nylon composite, typical
- **Standard IMU breakout (ICM-42688-P class) + cable** — assumed 5 g. 6-axis MEMS IMU breakout; the FC carries its own, this is the dedicated one for the navigation stack on the compute board
