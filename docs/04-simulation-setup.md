# 04 — Simulation Setup

Everything runs in **WSL Ubuntu 22.04**. Nothing runs on Windows directly.

## Stack

| Component | Version / location |
|---|---|
| ROS 2 | Humble, `/opt/ros/humble` |
| PX4-Autopilot | `v1.18.0-beta1-496-g8f43cf7268`, `~/PX4-Autopilot` |
| Gazebo | Harmonic, `gz sim 8.15.0` |
| uXRCE-DDS agent | `~/.local/bin/MicroXRCEAgent` (libs in `~/.local/lib`) |
| ROS 2 workspace | `~/ws_px4` — `px4_msgs`, `px4_ros_com`, `drone_eval` |
| Project | `~/drone_sim` |

`ros_gz` is **not** installed and is **not** needed. The data path is
PX4 → uXRCE-DDS → `px4_msgs`. Do not add a `ros_gz` dependency.

```
Gazebo Harmonic  <--->  PX4 SITL  <--->  MicroXRCEAgent  <--->  ROS 2
   (physics)          (flight ctrl)        (DDS bridge)      drone_eval
```

---

## Generated artefacts

`tools/gen_model.py` writes, from `config/airframe.yaml`:

- `~/PX4-Autopilot/Tools/simulation/gz/models/n360_quad_<prop>_<payload>/`
- `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/220xx_gz_n360_quad_<prop>_<payload>`

| Autostart | Model |
|---|---|
| 22000 | `n360_quad_9x45_pi5` |
| 22001 | `n360_quad_11x45_pi5` |
| 22002 | `n360_quad_13x44_pi5` |
| 22003 | `n360_quad_9x45_orin_nx` |
| 22004 | `n360_quad_11x45_orin_nx` |
| 22005 | `n360_quad_13x44_orin_nx` |

IDs sit in PX4's reserved `[22000, 22999]` custom range so a PX4 update cannot
collide with them.

The models are produced by **patching PX4's own `x500_base/model.sdf`** rather
than hand-writing SDF. That reuses PX4's tuned sensor noise models and, more
importantly, preserves their rotor numbering and spin directions — get those
wrong and the aircraft flips on takeoff.

---

## Changes made to the PX4 tree

Two, both minimal and both necessary:

1. **`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`** — six lines
   registering the generated airframes.
2. **`src/modules/uxrce_dds_client/dds_topics.yaml`** — publishes
   `/fmu/out/actuator_motors`. PX4 ships this as an *input* topic only, but the
   per-motor command is the only accurate source of propulsive power. Original
   saved as `dds_topics.yaml.bak_drone_sim`.

Both survive a `make px4_sitl_default`. Re-apply after a PX4 update.

---

## Topics and QoS

**PX4 offers `/fmu/out/*` as BEST_EFFORT with TRANSIENT_LOCAL durability.**
The rclpy default is RELIABLE, which is incompatible — a default subscriber
receives **nothing, silently**. Both nodes declare:

```python
QoSProfile(reliability=BEST_EFFORT, durability=TRANSIENT_LOCAL,
           history=KEEP_LAST, depth=5)
```

**An empty CSV is a QoS mismatch until proven otherwise.** Check that before
suspecting the model.

### Topic names carry version suffixes

PX4 1.18 appends a message-version suffix to some topics and not others:

| Logical topic | Actual name |
|---|---|
| `vehicle_local_position` | `/fmu/out/vehicle_local_position_v1` |
| `vehicle_status` | `/fmu/out/vehicle_status_v4` |
| `battery_status` | `/fmu/out/battery_status_v1` |
| `vehicle_attitude` | `/fmu/out/vehicle_attitude` (none) |
| `actuator_motors` | `/fmu/out/actuator_motors` (none) |

Hardcoding either form breaks on upgrade, so both nodes resolve names against
the live graph via `resolve_topic()`, taking the highest `_vN` they find.

### No `/clock`

Without `ros_gz` nothing publishes `/clock`, so **`use_sim_time:=true` would
freeze both nodes at t=0**. They use PX4 message timestamps instead, which are
the simulation clock in microseconds. Do not "fix" this by adding
`use_sim_time`.

---

## The `drone_eval` package

`~/ws_px4/src/drone_eval`

- **`flight_logger`** — subscribes to position, attitude, status, motors and
  battery; converts motor commands to rotor speed, then to thrust and shaft
  power via the propeller coefficients; integrates the battery model; writes a
  CSV per run. Publishes `/drone_eval/battery_empty` (latched) when the DoD
  gate is reached.
- **`mission_runner`** — streams offboard heartbeats, arms, takes off, flies
  the scenario profile, and lands on completion, battery-empty, or timeout.

PX4 requires a stream of offboard setpoints *before* it will accept the mode
switch, hence the warm-up in `mission_runner` before arming.

---

## Airframe parameters worth knowing

```
SIM_GZ_EC_MIN1..4   150            motor command floor
SIM_GZ_EC_MAX1..4   = maxRotVelocity in the SDF
MPC_THR_HOVER       from the sizing model, EC_MIN-corrected
NAV_DLL_ACT         0              datalink-loss failsafe DISABLED
NAV_RCL_ACT         0              RC-loss failsafe DISABLED
COM_RC_IN_MODE      4              no RC expected
```

> **The failsafes are disabled because these runs are unattended and headless
> with no GCS and no RC link. Without that, preflight fails and the vehicle
> never reaches "Ready for takeoff". Do NOT copy these onto real hardware** —
> disabling the datalink failsafe on a real aircraft is exactly the wrong trade.

---

## Wind

Applied over gz transport so it is scriptable per scenario:

```
gz topic -t /world/<world>/wind -m gz.msgs.Wind \
  -p "linear_velocity: {x: .., y: .., z: ..}, enable_wind: true"
```

The airframe's `base_link` carries `<enable_wind>true</enable_wind>`, without
which the world's wind does not act on it at all.

**Wind is ramped in over ~12 s after the vehicle reaches altitude, then gusts
are applied as perturbations around the mean.** Stepping wind onto a grounded
vehicle whose estimator is still settling is an impulse: PX4 fails it out with
`Preflight Fail: Attitude failure (roll)` before it ever takes off. That is an
artefact of how wind is applied, not a property of the aircraft.

Worlds used: `default` (still air), `windy`, `forest`.

---

## Running

```bash
bash ~/drone_sim/scripts/run_scenario.sh ~/drone_sim/scenarios/hover_endurance.yaml 9x4.5 pi5
```

```bash
bash ~/drone_sim/scripts/batch.sh
```

Outputs land in `~/drone_sim/out/runs/<scenario>_<prop>_<payload>/` —
`flight_log.csv`, `px4.log`, `logger.log`, `runner.log`, `agent.log`.

## Troubleshooting

| Symptom | Cause |
|---|---|
| Empty CSV | QoS mismatch, or a topic name whose `_vN` suffix changed |
| No `/fmu` topics at all | Agent died — `LD_LIBRARY_PATH` must include `~/.local/lib` |
| Nodes log nothing, timestamps frozen | `use_sim_time:=true` with no `/clock` |
| `Attitude failure (roll)` right after arming | Wind stepped instead of ramped |
| Never reaches "Ready for takeoff" | Datalink/RC failsafe with no GCS attached |
| `invalid inertia` on model load | Rotor tensor violates Ixx + Iyy ≥ Izz |
