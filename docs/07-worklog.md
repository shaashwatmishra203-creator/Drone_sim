# 07 — Work Log

Append-only. What was run, what it produced, what was decided, what was wrong.

---

## 2026-09-30 — Build

### Environment survey
Found WSL Ubuntu 22.04 already carrying ROS 2 Humble, PX4 v1.18.0-beta1 with
SITL **already built**, Gazebo Harmonic 8.15.0, a built `~/ws_px4` with
`px4_msgs`/`px4_ros_com`, and `MicroXRCEAgent`. Stock worlds include `windy`,
`forest`, `baylands`. No `ros_gz` — decided against adding it, since
PX4 → uXRCE-DDS → `px4_msgs` already works and is the lighter path.

### First-pass feasibility
Momentum theory on an estimated 3.12 kg AUW: 598 W hover, 10.7 min. Flagged
the 9″ prop at ~16,000 rpm on 6S as the likely weak point. This set the
direction for everything after.

### Analytical model
`tools/sizing.py` reproduced the hand calculation at **601 W / 10.7 min** —
close enough to confirm no unit errors.

**Two bugs found and fixed:**

1. **Gate semantics.** `hover_throttle` compared an *rpm* fraction against a
   limit that means a *thrust* fraction. The two differ by roughly a square
   root, and the error produced a spurious FAIL for the 13″ prop (0.602 vs a
   0.600 limit) on a configuration with a perfectly healthy T/W of 2.76.
   Gate now checks thrust fraction; rpm fraction reported separately.
2. **Drag area too low.** `CdA` of 0.035 m² put best-endurance cruise at
   11.5 m/s, implausibly fast. Raised to 0.060 m² to reflect two cameras, a
   slung gimbal, tall legs, a mast and two antennas. Best cruise moved to a
   sensible 9–9.5 m/s.

### Model generation
Decided to **patch PX4's `x500_base/model.sdf`** rather than hand-write SDF —
reuses their sensor noise models and preserves rotor numbering and spin
directions.

**Independent validation:** derived `momentConstant = 0.0156` for the 9×4.5
against the 0.016 PX4 ships for its own x500. Two different routes, essentially
the same number. Good evidence the coefficient derivation is right.

**Two bugs found and fixed:**

1. **Invalid rotor inertia.** Set `ixx = izz/100, iyy = izz/2, izz = izz`,
   which violates the tensor triangle inequality `Ixx + Iyy ≥ Izz`. All six
   models were rejected by `gz sdf -k`. Rewrote using the lamina relation
   (`Izz = Ixx + Iyy`), which satisfies it by construction.
2. **Airframe IDs outside the reserved range.** Used 4100–4105, which PX4 could
   later claim. Moved to 22000–22005, inside PX4's documented custom range.

### Bringing the stack up — five real failures

1. **Preflight fail: no GCS.** Airframe inherited `NAV_DLL_ACT 2` from x500;
   headless unattended runs have no GCS, so the vehicle never reached "Ready
   for takeoff". Disabled datalink/RC failsafes **for simulation only**, with a
   warning in the airframe file not to copy that to hardware.

2. **uXRCE-DDS agent never ran.** `MicroXRCEAgent` failed to load
   `libmicroxrcedds_agent.so.2.4`; PX4 sat "disconnected" with zero `/fmu`
   topics. The libraries are in `~/.local/lib`, not on the loader path. Fixed
   with `LD_LIBRARY_PATH` in the run script, plus an explicit check that the
   agent is alive before continuing.

3. **`actuator_motors` not published outward.** PX4 ships it as an `/fmu/in/`
   topic only. It is the only accurate source of per-motor command and hence of
   real propulsive power, so added it to `dds_topics.yaml` publications. Second
   and last PX4 edit.

4. **Topic names carry version suffixes.** PX4 1.18 publishes
   `vehicle_local_position_v1`, `vehicle_status_v4`, `battery_status_v1`, while
   `vehicle_attitude` and `actuator_motors` carry none. Hardcoded names silently
   matched nothing. Replaced with `resolve_topic()`, which matches the live
   graph and takes the highest `_vN`.
   *Retracted:* an intermediate fix changed `vehicle_status_v1` to the
   unversioned `vehicle_status` — also wrong; the live topic is `_v4`.

5. **No `/clock`.** Launched both nodes with `use_sim_time:=true`, which froze
   them at t=0 because nothing publishes `/clock` without a `ros_gz` bridge.
   Switched both to PX4 message timestamps, which *are* the simulation clock.

### Wind flipped the aircraft — and why that was not the aircraft's fault
First windy run: armed, "Takeoff detected", then
`Preflight Fail: Attitude failure (roll)` → failsafe → disarm, within ~1 s.

Isolated it by running the identical offboard profile in the `default` world
with no wind: **flew perfectly, 0.02 m position-hold RMS.** So the airframe and
control allocation were fine; the wind application was not. A 6 m/s step onto a
grounded vehicle with a settling estimator is an impulse.

Fixed by ramping wind in over ~12 s *after* the vehicle reaches altitude, then
applying gusts as perturbations. Re-ran: held position through 6 m/s with 3 m/s
gusts at 0.93 m RMS.

**Worth recording as a method note:** the first instinct on seeing a flip is to
suspect motor ordering or the moment constant. The isolation run ruled both out
in one step and pointed at the test harness instead.

### Cross-check: analytical vs SITL
Still-air hover, 9×4.5, pi5:

| Quantity | Analytical | SITL | Δ |
|---|---|---|---|
| Rotor speed | 8569 rpm | 8616 rpm | 0.5% |
| Thrust/motor | 779 g | 788 g | 1.2% |
| Hover power | 601 W | 570 W | 5.2% |

Two independent models agreeing to ~5% on power and ~1% on rpm. The residual
power gap is expected: the analytical side uses momentum theory with a figure
of merit, while the logger integrates `Cp` against the actual rpm.

`MPC_THR_HOVER` also needed correcting — PX4 maps commands as
`ω = EC_MIN + u·(EC_MAX − EC_MIN)`, so the hover command is **not**
`hover_rpm / max_rpm`. Corrected 0.538 → 0.492 against an observed 0.496.

### Wind penalty
6 m/s with 3 m/s gusts: hover power 570 → 699 W (**+23%**), projected endurance
down to ~9.2 min, position-hold RMS 0.93 m (max 9.75 m during the ramp).

---

## Retracted: "camera + IMU fusion reduces error to 0.12 m (3.24x better RMS)"

Claimed in commit `cc3938f`. **It does not reproduce.** A replay of the same
mission against the same world gave:

| | first run | replay |
|---|---:|---:|
| IMU-only RMS | 54.6 m | 41.9 m |
| camera+IMU RMS | 16.8 m | **43.7 m** |
| ratio | 3.24x better | **0.96x - worse than no fusion** |
| landmark fixes | 454 | **252, then frozen** |

The cause is a defect, not variance. Fix count climbs 42 -> 240 through the
outbound leg, where the fused error is genuinely excellent (0.08 m against
0.58 m for IMU alone). It then freezes at 252 for the final ~55 s while the
mapper is still detecting gates - 19 detections, the last one at the very end
of the run. The measurements were arriving; the estimator was rejecting them.

`state_estimator.py` associates a measurement with a gate like this:

    pred_x = self.fus_p[0] + rel_fwd
    gate = min(self.gates, key=lambda g: abs(g[0] - pred_x))
    if abs(gate[0] - pred_x) > 2.5: return

The validation gate is keyed on the estimate it exists to correct. Once drift
passes 2.5 m every measurement is rejected, which guarantees further drift:
positive feedback with no path back. The fused track then ends up *worse* than
raw IMU, because it carries corrupted velocity on top of the same bias.

So the earlier number was a run that happened to hold lock the whole way. One
passing run is not a result. What is actually established is narrower:

**Established (L1):** with lock held, camera landmark fixes bound IMU drift to
well under a metre (0.08 m observed at 42 fixes vs 0.58 m IMU-only).
**Not established:** that the filter holds lock over a full mission, or that
fusion improves end-to-end accuracy. On the evidence it currently does not.

Needed before the claim can be restated: a recovery path for lost association
(widen the gate with accumulated uncertainty rather than a fixed 2.5 m, and
allow re-acquisition from a known gate when several consecutive measurements
are rejected), then N>=5 runs reported as a spread, not a best case.

---

## Open items

- **Measured masses.** 88% of AUW is still estimated. ~2 min endurance per 500 g.
- **Ct/Cp are generic**, not HQProp measurements. Largest source of model error.
- **Motor parameters unconfirmed** — Kv, Rm, I₀, I_max need the 3115 datasheet.
- **Prop max rpm is a rule of thumb** (145000/D), not an HQProp figure. This is
  the evidence behind the headline 9″ finding and it needs confirming.
- No thermal model; motor/ESC heating is where a real build fails first.
