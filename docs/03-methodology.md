# 03 — Methodology

> **Verification level: L0–L2.** Analytical model and software-in-the-loop.
> No hardware measurement informs any number below.

Two independent models, deliberately kept separate so they can check each other:

1. **`tools/sizing.py`** — closed-form momentum theory plus a BLDC motor model.
2. **PX4 SITL + Gazebo** — the aircraft actually flown by real flight-control
   software, with power derived from the motor commands PX4 produces.

They share only `config/airframe.yaml`. If they agree, the agreement means
something. They currently agree within ~5% on hover power (see `05-results.md`).

---

## 1. Mass and inertia

Masses come from the component register in the config. The inertia tensor is
**computed**, never hand-typed:

- Each component contributes a point mass at its position, about the computed CG.
- A solid-box term adds the distributed body mass that point masses miss
  (25% of AUW, over the body dimensions).

Hand-typed inertia goes stale the instant a mass changes. This does not.

**Uncertainty:** component *positions* are estimates. Inertia errors mostly
affect attitude dynamics and gust response, not hover power or endurance.

---

## 2. Propeller model

Standard non-dimensional coefficients, with `n` in revolutions per second:

```
T = Ct · ρ · n² · D⁴          thrust
P = Cp · ρ · n³ · D⁵          shaft power
Q = Cp · ρ · n² · D⁵ / 2π     torque
```

**This is the weakest link in the whole model.** Ct and Cp are generic
thin-electric-propeller values, not measurements of an HQProp 9×4.5. A 15%
error in Ct moves hover rpm by ~7% and power by ~20%.

*To fix:* put a prop on a thrust stand, sweep rpm, fit Ct and Cp. That single
measurement would do more for confidence than anything else in this model.

---

## 3. Motor model

A standard BLDC model, solved against the propeller's torque demand:

```
Ke = Kt = 60 / (2π · Kv)
Q_motor = Kt · (I − I₀)
ω such that Q_motor(ω) = Q_prop(ω)
```

Maximum thrust is whichever binds first:

- **current-limited** — motor or ESC continuous current ceiling, or
- **voltage-limited** — back-EMF reaches pack voltage.

The tool reports which one bound. For this airframe the 9″ prop is
current-limited; the 13″ becomes voltage-limited.

**Uncertainty:** Kv, Rm, I₀ and I_max are all marked `estimated` in the config.
They need confirming against the XRotor 3115 datasheet.

---

## 4. Hover power — momentum theory with figure of merit

```
v_h = √( T / (2ρA) )              induced velocity in hover
P_ideal = T · v_h
P_shaft = P_ideal / FoM
P_elec  = P_shaft / η_motor+esc  +  P_avionics
```

Figure of merit bundles profile drag, tip losses and swirl. Values used: 0.58
(9″), 0.62 (11″), 0.65 (13″) — larger, slower discs are more efficient.

For forward flight, power decomposes into three terms:

```
P_shaft(V) = T·v_i(V)  +  P_profile  +  D·V
```

with `v_i` from Glauert's relation solved iteratively, `P_profile` calibrated
from the hover FoM and held constant, and `D = ½ρ·CdA·V²`.

**Uncertainty:** `CdA = 0.060 m²` is an engineering estimate for an airframe
carrying two cameras, a slung gimbal, tall legs, a GPS mast and two large
antennas. Best-endurance cruise speed is sensitive to it.

---

## 5. Battery — why not Wh ÷ W

Endurance is a **time-stepped discharge**, not a division. At each step:

```
V_oc  = f(SoC)                        from the per-cell OCV curve
R·I² − V_oc·I + P = 0    →    I = (V_oc − √(V_oc² − 4RP)) / 2R
SoC  -= I·dt / (3600·capacity)
```

This matters because both effects shorten the flight and a naive `Wh / W`
misses them: open-circuit voltage falls as the pack empties, so the same power
demands more current, which sags the terminal voltage further, which demands
more current again.

### Why PX4's own battery is not used

PX4's SITL battery simulator
(`src/modules/simulation/battery_simulator/BatterySimulator.cpp`) drains the
pack linearly on **wall clock** via `SIM_BAT_DRAIN`:

```c
_battery_percentage -= (now_us - _last_integration_us) / discharge_interval_us;
```

It never looks at the motors. `BatteryStatus` from SITL is therefore useless
for endurance, and `flight_logger` integrates the model above instead, driven
by the actual motor commands. `SIM_BAT_DRAIN` is still set to a sensible value
so PX4's own low-battery failsafes fire at a plausible time.

---

## 6. Linking the analytical model to Gazebo

Gazebo's `MulticopterMotorModel` uses `thrust = motorConstant · ω²` and
`moment = momentConstant · thrust`. Equating to the coefficient form:

```
motorConstant  = Ct · ρ · D⁴ / (4π²)
momentConstant = Cp · D / (2π · Ct)
```

`tools/gen_model.py` derives both from the same config the analytical model
reads, so the two cannot silently diverge.

**Independent check:** for the 9×4.5 this gives `momentConstant = 0.0156`
against the 0.016 PX4 ships for its own x500. Two different routes to
essentially the same number.

PX4 maps a normalized motor command `u` to rotor speed as

```
ω = EC_MIN + u · (EC_MAX − EC_MIN)
```

so `SIM_GZ_EC_MAX` must equal `maxRotVelocity` in the SDF, and `MPC_THR_HOVER`
must account for the `EC_MIN` offset — it is **not** simply
`hover_rpm / max_rpm`. `flight_logger` inverts exactly this mapping to recover
rotor speed from the logged commands.

---

## 7. Gates

Gates are engineering decisions with provenance, recorded in the config. **None
is relaxed to make a configuration pass.**

| Gate | Value | Rationale | Uncertainty |
|---|---|---|---|
| `dod_limit` | 0.80 | LiPo cycle-life limit; deeper discharge degrades the pack | ±0.05 depending on how much pack life the team will trade |
| `thrust_to_weight_min` | 2.0 | Below ~2:1 there is no authority for wind rejection or attitude correction | 1.8 is flyable but marginal |
| `hover_thrust_fraction_max` | 0.50 | Hover must use at most half of available thrust | Deliberately restates T/W ≥ 2 in the units a pilot sees |
| `prop_rpm_margin` | 1.00 | Loaded rpm must not exceed the prop's rated maximum | **Weak.** `max_rpm` is the 145000/D rule of thumb, *not* an HQProp figure |

The prop-rpm gate carries the most consequence and the least evidence. It is
what flags the 9″ propeller, and it rests on a rule of thumb. **Confirm the
real rated rpm with HQProp before treating that finding as settled.**

### A note on hover "throttle"

Two different quantities get called throttle and routinely conflated:

- **thrust fraction** = hover thrust ÷ max thrust — what the "hover under 50–60%"
  design rule actually means. This is what the gate checks.
- **rpm / stick fraction** = hover rpm ÷ max rpm — closer to stick position.

They differ by roughly a square root. An early version of this model gated on
the rpm fraction and produced a spurious failure for the 13″ prop.

---

## 8. Known limitations

1. **Ct/Cp are not measured.** Biggest single source of error.
2. **Motor parameters are unconfirmed.** Kv, Rm, I₀, I_max all need the datasheet.
3. **88% of mass is estimated.** ~2 min endurance per 500 g.
4. **No thermal model.** Motor and ESC heating is not simulated; a long hover at
   high current is exactly where a real build fails first.
5. **Profile power held constant with airspeed.** Fine at low advance ratio,
   optimistic at high speed.
6. **Wind is applied as a uniform field**, ramped after takeoff, with random
   gusts — not a spatially correlated Dryden turbulence field.
7. **Rigid airframe.** A printed PETG-CF frame has real modes; none are modelled.
8. **Visual meshes are cosmetic.** They are the x500's 10″ geometry regardless of
   the prop actually simulated. All physics comes from the plugin constants,
   not the meshes.
