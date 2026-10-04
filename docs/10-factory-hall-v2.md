# 10 — Factory hall v2: camera navigation with cloud object detection

**Verification level: L1–L2 (software-in-the-loop simulation only).** Nothing
here shows that the aircraft is safe to build or fly. "The simulated drone
returned to the pad" is the claim; "the drone can navigate a factory" is not.

## 1. What changed and why

The v1 factory mission was redone from scratch after the team watched it and
saw the drone roaming open ground outside the course. Two bugs explained
that, and both invalidate earlier mission-level results (see §7):

| Bug | Effect | Fix |
|---|---|---|
| **World-frame waypoints sent to PX4 unconverted.** Gazebo is ENU (x = East); PX4 reads setpoints as NED (x = North). | Every earlier mission flew the course **rotated 90°**, beside the walls. The sensor model, clearance check and estimator all used the same wrong frame, so they agreed with each other and nothing flagged it. | `drone_eval/frames.py` converts in one place, with unit tests. Position is now checked against Gazebo's **true pose** (`truth_logger`), never against PX4's own estimate. |
| **Magnetic declination mismatch.** The world sat at California coordinates; PX4 looks up declination for that position, but Gazebo's magnetometer uses a fixed field vector. | PX4's heading was **26° wrong** on the pad, settling to ~6° in flight. Every camera detection was rotated on the map. | World moved to PX4's stock (Zurich) origin, set in `config/factory_hall.yaml`. Heading error now −4.8° on the pad and ~0–5° in flight, measured against truth. |

## 2. The hall

One enclosed 50 × 30 m hall (`config/factory_hall.yaml` → `tools/gen_hall.py`).
The launch pad is the world origin. A 27.5 m aisle formed by two shelving rows
leads to a scan zone at the far end; storage blocks close the floor beside the
aisle at both ends, so **the aisle is the only route** (earlier layouts let the
drone fly round the outside, which tests nothing).

Inside the aisle, five obstacles each leave one usable gap, alternating side to
side, with pillars in the wide lanes: `rack_1` (gap south) → `machine_1`
(north) → `rack_2` (north) → `pallets_1` (south) → `machine_2` (north). Every
obstacle is taller than the 2 m cruise altitude, so it must be flown round.
`tools/check_course.py` checks before any flight that the course is passable
at the planner's own clearance (narrowest free band after inflation: 0.8 m).

Each object class has its own colour, defined once and read by both the world
generator and the detector: shelf racks blue, machines green, pallet stacks
red, pillars yellow. The pad is pale grey on purpose; a coloured pad was
detected as an obstacle.

## 3. Pipeline

```
Arducam (Gazebo, 640x360 @ 15 fps)
  -> cloud_link (on the drone): JPEG q70, 10 Hz, ~5-7 KB/frame
  -> EMULATED factory WiFi: 11 Mbps, 35 +/- 15 ms one way, 1 % loss
  -> cloud_detector ("cloud"): HSV colour segmentation -> boxes, class,
     confidence, ground-contact points (where each object meets the floor)
  -> back over the emulated link -> /drone/detections
  -> hall_mission (on the drone):
       contact pixel + camera height + IMU attitude (PX4 EKF) -> range, bearing
       -> occupancy map (0.25 m cells; free space cleared along each ray)
       -> global A* path over the map (unseen floor passable but costed)
       -> local avoidance on the next 2.5 m, edge safety rules
       -> PX4 offboard setpoints (converted ENU -> NED)
```

**Object detection runs in the cloud, navigation and safety on the drone**, as
the team asked. The drone never relies on the link for safety. It holds position if
detections are older than 0.5 s, and stops if anything is within 1.2 m ahead.
Both the drone and the "cloud" run on one laptop; **the link is emulated** and
its parameters are assumptions, not a network measurement.

The detector is classical computer vision, by choice: the hall's objects are
synthetic coloured shapes that a pretrained detector would not recognise.
Positioning uses PX4's EKF with simulated GPS; the camera is used for obstacle
detection and mapping, not for localisation (team decision for this version).

## 4. Results

Gates and their provenance:

| Gate | Value | Why |
|---|---|---|
| Hull clearance, every sample | ≥ 0.40 m | ~one prop radius of margin beyond the airframe radius (0.35 m) |
| Contact | none | |
| Landing | ≤ 0.50 m from pad centre | about the pad's half-width (1.2 m) less the airframe |
| Detection recall (in view, ≥ 50 % unoccluded, ≤ 8 m) | ≥ 0.90 | missing 1 in 10 nearby obstacles is too many for a camera-only avoider |
| Range error (floor contact), median | ≤ 15 % | keeps the 1.2 m stop margin intact at 7 m look-ahead |

Clean complete missions (truth from Gazebo):

| Run | Total | Out / scan / back | Min clearance | Landing error | Recall | Range err. median / p90 | Link round trip median / p95 |
|---|---|---|---|---|---|---|---|
| 11 (headless) | 120.6 s | 47.5 / 13.0 / 49.8 s | 0.523 m | 0.089 m | 0.758 | 2.1 % / 8.7 % | 117 / 174 ms |
| 12 (watched with the window) | 142.0 s | — / — / 74.6 s | 0.521 m | 0.083 m | 0.787 | 1.9 % / 8.3 % | 118 / 166 ms |
| 15 (recorded; replay and raw data in repo) | 140.2 s | 67.7 / 12.0 / 50.5 s | 0.540 m | 0.077 m | 0.786 | 2.2 % / 9.3 % | 117 / 169 ms |

**Clean runs since the final course layout: 4. Completed the whole mission: 3
(runs 11, 12, 15).** Run 14 flew the aisle and the scan (51.6 s outbound,
0.556 m closest approach, no contact), then got stuck on the way back in the
corner at the south end of the aisle exit: its plan led into an unseen lane
that turned out blocked, and it had no way to abandon that route. Dead-end
escape was added after it (§5); run 15 did not need to use it, so that
mechanism is still **untested in flight**.

In every completed run: **clearance PASS, no contact PASS, landing PASS,
range error PASS, detection recall FAIL.**

Run 13 is excluded: the launcher was started twice and two run harnesses
overlapped on one output folder, so duplicate controllers cannot be ruled out.
`hall_run.sh` now refuses to start a second run, and every run writes to its
own folder. Runs 11 and 12 wrote to a shared folder before that fix, so their
raw files were overwritten. Their figures above come from the run logs
recorded at the time. Runs 14 and 15 have their raw data committed.

Other measurements (run 12): 1,417 detection messages over the flight; the
global planner found a path every time it replanned (113 plans, 89 ms median,
147 ms max); decisions were 70 % moving, 9 % turning to look first, 18 % holding
because no safe heading was visible yet, 2 % stopping for something close, 1 %
holding for stale detections.

Positive control (frame and heading), against Gazebo truth: commanded 3 m East
→ true (2.65, −0.03); then 3 m North → (3.09, 2.73); back → (0.18, 0.19).
Heading error −4.8° on the pad, −3.1° median in flight.

## 5. What failed, and what fixed it

The mission did not work until the twelfth flight. The failures are kept
because each changed the design:

| Run | What happened | Cause | Change |
|---|---|---|---|
| 1 | Completed, but flew **round** the obstacles; 0.26 m clearance (fail), landed 0.56 m out (fail) | Open hall offers a way round; face-only map; NAV_LAND too early | Aisle; map object depth; descend centred before landing |
| 2 | Flew round the aisle; stuck after the scan | Open sides; each box mapped at one range (its nearest point) | Storage blocks; ground-contact points per column |
| 3 | Node crashed at take-off | Two methods named `mark` | Renamed; pyflakes before every run |
| 3b, 4 | Stuck at the aisle entrance | 3 m gaps closed on the 0.4 m map; then a pure local planner dithered between two dead ends | 4 m gaps (checked by `check_course.py`); global A* planner |
| 5 | 71 of 98 plans found no path | **26° heading error** rotated detections into the empty gap | Stock world origin (declination) |
| 6–8 | Stuck at the entrance | Inflation double-counted grid rounding; "too close to range" flags blocked known walls; local check over the full 7 m left a 4° window | Inflate cell centres only; drop flags the map already explains; local check over 2.5 m when following a path |
| 9, 10 | Outbound and scan worked; return went round the outside of the aisle | Unseen floor counted as free, so an unseen detour looked shorter | Storage blocks at the exit; unseen floor costs extra on the planner |
| 11, 12 | **Complete** | | |
| 14 | Outbound and scan worked; stuck on the return at the aisle exit | Plan led into an unseen lane that was blocked; no way to give up on a route | Dead-end escape: after 20 s without progress, the surrounding floor is marked costly and the route is replanned (up to 3 times) |
| 15 | **Complete** (escape not needed) | | |

## 6. Open items

- **Detection recall fails the gate (0.76–0.79 against 0.90).** Shelf racks are
  the weak class (0.62–0.67). Adjacent racks share a colour and likely merge
  into one blob that the scorer cannot match to either object. Suspected, not
  yet confirmed. Navigation is less affected because it uses ground-contact
  points rather than boxes, but the gate is not relaxed.
- **Too few runs** (4 clean, 3 complete) to quote a success rate or a spread
  with confidence; the plan calls for N ≥ 5 complete runs, and the dead-end
  escape has not yet been exercised in flight.
- **Link outage test not run.** The hold-on-stale-detections rule exists but
  has not been exercised by a scripted outage.
- **Residual heading error ~0–5°** (≈0.6 m at 7 m) from the remaining
  declination mismatch.
- **Positioning is simulated GPS.** Indoors there is none; camera + IMU
  localisation is the next major step, and the v1 estimator was not ported to
  the hall.
- **Close-flag rule:** the drone placed its "too close to range" obstacles nearer
  than the truth in 93 of 97 cases (run 12). The 4 that were not are unexamined.

## 7. Retracted

- **All v1 factory-mission results** — gates threaded, 0 collisions, 71–74 %
  camera-steered, 91–92 s mission time, clearance figures, and the camera+IMU
  fusion numbers. The course was flown rotated 90°, and every check shared the
  same frame error. This also resolves the long-open "0.00 m clearance with no
  collision" contradiction: the clearance was measured in the wrong frame.
  Hover physics, thrust, power and endurance results are unaffected (they do
  not depend on horizontal position).
- **"Cut-off boxes give an upper bound on range"** (an early claim in
  `avoid.py`). Flight data disproved it: the true range exceeded the
  "bound" in 45 of 70 cases. Cut-off boxes are now treated as "close, range
  unknown" and placed at half the returned range; the report checks that this
  placement is nearer than the truth.

## 8. See it on GitHub

GitHub cannot run Gazebo, so run 15 is published as a replay of its recorded
data: `docs/replay/index.html` plays the true flight path in 3D in the
browser, with the drone's map filling in as the camera sees things and the
cloud detector's annotated camera view in sync. With GitHub Pages enabled
(Settings → Pages → Deploy from branch → `main` / `docs`) it is served at
`https://shaashwatmishra203-creator.github.io/Drone_sim/replay/`. The README
shows two animated GIFs made from the same run.

## 9. Reproduce

```bash
# headless, judged against truth; output in out/runs/hall_mission_<time>/
bash ~/drone_sim/scripts/hall_run.sh

# positive control: position and heading against Gazebo truth
MODE=frame_check bash ~/drone_sim/scripts/hall_run.sh

# is the course passable at the planner's clearance?
python3 ~/drone_sim/tools/check_course.py
```

With windows, from Windows PowerShell (restarts WSL until the display bridge is
healthy, opens Gazebo and the cloud-CV view, then flies):

```bash
powershell -ExecutionPolicy Bypass -File \\wsl$\Ubuntu-22.04\home\shaash\drone_sim\scripts\run_sim.ps1
```

Each run writes `hall_report.json`, `hall_path.png` (true path, obstacles and
camera map), `truth.csv`, `camera_map.json`, `link_stats.json`,
`thrust_power.json` and `detections.mp4` (the cloud's annotated view).
