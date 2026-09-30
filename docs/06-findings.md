# 06 — Findings and Recommendations

> **Verification level: L0–L2.** Simulation and software test only. Nothing
> here has been measured on hardware. These are findings about a *model*.

---

## Verdict

**Yes — the aircraft flies, with comfortable thrust margin.** Thrust-to-weight
is about 3.4 and hover consumes roughly 29% of available thrust, well inside
the design gates. Flight *capability* is not the problem with this parts list.

**Endurance is the problem, and one part choice causes most of it.** About
10–11 minutes of hover in still air, ~9 minutes in moderate wind. For an
autonomy platform carrying two stereo cameras and a gimbal, that is a short
working window.

**Two findings block the build as specified**, and neither is about flight
dynamics.

---

## Blocking

### F1 — The ZED cameras cannot run on a Raspberry Pi 5

**Finding:** The parts list pairs a ZED 2 and a ZED Mini with a Raspberry Pi 5.
The Stereolabs ZED SDK requires an NVIDIA CUDA GPU. The Pi 5 has none. Neither
camera will produce depth, and the SDK will not install.

**Evidence:** Stereolabs publishes CUDA as a hard requirement. The Pi 5's GPU
is a VideoCore VII with no CUDA support. The reference photograph the team is
working from shows a Jetson-class board in the "On-board Computer" position.

**Impact:** The entire perception stack is non-functional as specified. This is
independent of whether the aircraft flies.

**Options:**

| Option | Mass Δ | Power Δ | Endurance cost |
|---|---:|---:|---|
| Jetson Orin NX + carrier | +80 g | +13 W | ~0.6 min |
| Drop to one ZED on a Jetson | +18 g | +11 W | ~0.4 min |
| Keep the Pi 5, replace the ZEDs with Pi-native stereo | 0 | 0 | none, but loses the ZED SDK's depth and VIO |

**Recommendation:** move to a CUDA-capable board. The endurance cost is under a
minute — trivial next to losing the perception stack entirely.

**Verification:** L0. Established from vendor documentation, not tested here.

---

### F2 — The flight controller cannot run a PX4 autonomy stack

**Finding:** The SpeedyBee F405 V5 is a 30×30 racing stack built for Betaflight.
PX4 has dropped F405 support — the target does not exist in current PX4. The
team's stated goal is ROS 2 autonomy, and this simulation is PX4-based.

**Impact:** Everything validated here runs on PX4. None of it transfers to an
F405 running Betaflight. ArduPilot on F405 is possible but heavily
feature-limited on a 1 MB flash target, and is not what was simulated.

**Options:**
- An F7 or H7 flight controller with PX4 support (Pixhawk 6C, Kakute H7, Matek H743)
- ArduPilot on F405, accepting reduced features and a different autonomy stack

**Recommendation:** move to an H7 board. This is also the cheaper fix now than
after the frame is printed and wired.

**Verification:** L0. From PX4's supported-target list, not tested.

---

## Significant

### F3 — The 9-inch propeller has essentially no rpm margin

**Finding:** At full throttle on 6S the 9×4.5 turns approximately **15,900 rpm**
against an estimated rated ceiling of **16,100 rpm** — about **1% margin**. Tip
speed reaches ~191 m/s (Mach 0.56). The 11″ and 13″ options have 22% and 34%
margin respectively.

**Cause:** 900 KV on 6S is a fast combination. A 3115-size motor is built to
swing 11–13″; a 9″ prop under-loads the motor while over-revving the propeller.
The pairing is mismatched in both directions at once.

**⚠️ Evidence quality — read this before acting.** The 16,111 rpm ceiling is the
`145000 / D` rule of thumb for composite propellers, **not** a figure from
HQProp. The finding is only as good as that number. **Confirm the real rated
rpm with HQProp before treating this as settled.** If their actual limit is
higher, this drops from a blocking concern to a note.

**Verification:** L1. Analytical, resting on an unconfirmed rating.

---

### F4 — Larger propellers buy ~6 minutes for almost nothing

**Finding:** Disc loading with 9″ props is 19.0 kg/m². Going to 13″ drops it to
9.1 kg/m² and cuts hover power from 601 W to 388 W.

| Prop | Disc loading | Hover power | Endurance | Gain |
|---|---:|---:|---:|---|
| 9×4.5 (current) | 19.0 kg/m² | 601 W | 10.7 min | — |
| 11×4.5 | 12.8 kg/m² | 469 W | 13.8 min | **+29%** |
| 13×4.4 | 9.2 kg/m² | 388 W | 16.8 min | **+57%** |

The frame already accommodates 13″: a 495 mm diagonal gives 350 mm between
adjacent motors against the 330 mm a 13″ prop needs. Tight, but it fits — and
it should be checked against the actual CAD before ordering.

T/W stays healthy at 2.76 even on 13″, and all gates still pass.

**Recommendation:** **this is the single highest-value change on the list.** Move
to 13″ props. It costs a set of propellers and buys over 50% more flight time.

**Verification:** L1 analytical; the 13″ configuration has also been flown in
SITL (see `05-results.md`).

---

### F5 — Wind costs about a quarter of the endurance

**Finding:** 6 m/s wind with 3 m/s gusts raised hover power from 570 W to
699 W — **+23%** — and cut projected endurance to ~9.2 min. Position hold
degraded from 0.02 m RMS to 0.93 m RMS, with 9.75 m peak excursion during the
ramp.

**Impact:** A "10 minute" endurance figure is a still-air number. Plan real
missions around **7–8 minutes** of usable time with reserve, in wind.

**Verification:** L2, SITL measured.

---

## Worth fixing before the build

### F6 — Power distribution is not specified and will not work as-is

The Pi 5 (or Jetson), gimbal, two cameras and the ALFA adapter together need a
dedicated **5 V / 5 A minimum** BEC. The SpeedyBee stack's onboard BEC will not
carry that load. Not simulated — flagged for the electrical team.

### F7 — Missing from the parts list entirely

- No RC receiver
- No telemetry radio
- No power module / current sensor (without which the aircraft cannot measure
  its own battery state in flight — and every endurance number here becomes
  unverifiable on the real vehicle)

---

## Priority order

| # | Action | Cost | Benefit |
|---|---|---|---|
| 1 | **Weigh the aircraft** and update `config/airframe.yaml` | free | Removes the largest uncertainty in every number |
| 2 | Confirm HQProp's real max rpm | free | Settles or dismisses F3 |
| 3 | **Move to 13″ propellers** | one prop set | **+57% endurance** |
| 4 | Swap the Pi 5 for a CUDA board | board cost | Makes the perception stack possible at all |
| 5 | Swap the F405 for an H7 | board cost | Makes PX4 autonomy possible at all |
| 6 | Specify a 5 V/5 A BEC and a power module | small | Avoids a brownout, enables battery telemetry |
| 7 | Thrust-stand the motor/prop to get real Ct/Cp | a day | Collapses the biggest remaining model error |

Items 1 and 2 cost nothing and should happen first.

---

## What would change these conclusions

- **Measured masses** materially different from the estimates — most likely the
  frame, which could plausibly be 400 g or 800 g.
- **HQProp rating above ~17,000 rpm** would retire F3.
- **Measured Ct/Cp** could move hover power by ±20%, which moves every endurance
  figure with it.
- **A confirmed 3115 datasheet** could change the T/W and max-thrust picture,
  though not enough to threaten flyability.
