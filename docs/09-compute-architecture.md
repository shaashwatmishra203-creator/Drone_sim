# 09 — Cloud versus Edge Compute

> **Verification level: L0.** Analytical, from published hardware
> specifications and measured-order networking figures. Nothing here was tested
> on the real link or the real boards.

Where should perception and navigation actually run — on the aircraft, or on a
server it streams to? Reproduce with `python3 tools/compute_tradeoff.py`.

The question is not "which is faster in general". It is three specific ones:
**can the link carry it, can it stop in time, and what happens when the link
drops?**

---

## Verdict first

**Split the workload. Edge and cloud are not competing options — they solve
different halves of the problem.**

| | Runs where | Why |
|---|---|---|
| Depth, VIO, obstacle avoidance, local planning | **Onboard (edge), mandatory** | Dropouts must not blind the aircraft; bandwidth and latency both fail otherwise |
| Map merging, global optimisation, fleet distribution | **Cloud / off-board** | Latency-tolerant, low data rate, needs cross-vehicle state |

**And the consequence for the parts list: a CUDA-capable board is required
either way.** The Raspberry Pi 5 cannot run either ZED camera, and cloud
offload is *not* a way to rescue it — offload fails independently on bandwidth,
latency and availability.

---

## 1. Bandwidth — what offload would have to push off the aircraft

| Stream | Raw | Compressed |
|---|---:|---:|
| ZED 2 stereo (720p30, YUV422) | 885 Mbps | 19.7 Mbps |
| ZED Mini stereo (720p30, YUV422) | 885 Mbps | 19.7 Mbps |
| Depth map (720p30, float32) | 885 Mbps | 110.6 Mbps |
| **Total** | **2654 Mbps** | **149.9 Mbps** |

Depth compresses only about 8× because it is high-entropy, and lossy
compression destroys exactly the geometry the obstacle avoidance depends on.
That single stream dominates the budget.

**What the link actually delivers** — ALFA AWUS036ACM, 802.11ac, against
distance, one interior wall and steel racking:

| Condition | Goodput | Verdict |
|---|---:|---|
| At base, line of sight (5 m) | 390 Mbps | OK |
| Mid obstacle course (16 m), racking | 105 Mbps | **insufficient** |
| Room doorway (30 m), racking | 60 Mbps | **insufficient** |
| **Inside the room (32 m), 1 wall + racking** | **11 Mbps** | **insufficient** |
| Far corner of the room, 2 walls | 2 Mbps | **insufficient** |

> **The mapping pass happens inside the room, behind a wall, at the furthest
> point from base — precisely where the link is worst.** Offload needs ~150 Mbps
> and gets about 11. The architecture fails hardest exactly where the mission
> matters most.

## 2. Latency — can it stop in time?

| Architecture | Total latency | Dominant term |
|---|---:|---|
| **Edge (onboard)** | **69 ms** | sensor capture 33 ms |
| On-premise server | 539 ms | uplink serialisation 442 ms |
| Cloud (regional datacentre) | 578 ms | uplink serialisation 442 ms |

The uplink term dominates both offload cases. It is not network distance that
hurts — moving the server into the building saves only 39 ms — it is that
150 Mbps of data does not fit down an 11 Mbps pipe. Bandwidth failure shows up
*as* latency: the frame arrives late because it arrives slowly.

At the measured mission speed of 1.76 m/s, braking at 5.7 m/s² (30° tilt limit):

| Architecture | Latency | Reaction | Stopping | Total | Verdict |
|---|---:|---:|---:|---:|---|
| Edge (onboard) | 69 ms | 0.12 m | 0.27 m | **0.39 m** | safe |
| On-premise server | 539 ms | 0.95 m | 0.27 m | 1.22 m | marginal |
| Cloud (regional DC) | 578 ms | 1.02 m | 0.27 m | **1.29 m** | **unsafe** |

Corridor clearance is 1.25 m from centreline to the nearest rack or pillar. The
cloud architecture consumes more than the whole clearance before it stops.

**Maximum safe speed** — where reaction plus stopping fills the clearance:

| Architecture | Max safe speed |
|---|---:|
| Edge (onboard) | **3.39 m/s** |
| On-premise server | 1.79 m/s |
| Cloud (regional DC) | 1.71 m/s |

Edge roughly doubles the speed at which the aircraft can safely work — which
translates directly into more area surveyed per battery.

## 3. Availability — what a dropout costs

| Dropout | Distance flown blind at 1.76 m/s |
|---|---:|
| 200 ms | 0.35 m |
| 500 ms | 0.88 m |
| 1000 ms | 1.76 m |

An edge architecture is simply unaffected: perception and control never leave
the aircraft. In a steel-racked factory, multi-hundred-millisecond dropouts
from shadowing and multipath are routine rather than exceptional.

**This is the strongest argument of the three, because it is a safety property
rather than a performance one, and no amount of link engineering removes it.**
A better radio raises the bandwidth ceiling and shaves latency; it does not
make a dropout safe.

## 4. Platform capability

| Platform | Mass | Power | CUDA | ZED depth | Verdict |
|---|---:|---:|---|---:|---|
| Raspberry Pi 5 (8 GB) | 100 g | 12 W | **no** | — | **cannot run ZED** |
| Jetson Orin NX 16 GB | 180 g | 25 W | yes | 60 fps | capable |

The ZED SDK requires an NVIDIA CUDA GPU. The Pi 5's VideoCore VII is not one,
so neither the ZED 2 nor the ZED Mini produces depth on the listed board under
any configuration — local or offloaded.

## 5. Power — not the deciding factor

| Architecture | Compute | Radio | Total |
|---|---:|---:|---:|
| Edge, Orin NX (map upload only) | 25 W | 1.5 W | 26.5 W |
| Cloud offload, Pi 5 + full streaming | 12 W | 6.0 W | 18.0 W |
| Cloud offload, Orin NX + streaming | 25 W | 6.0 W | 31.0 W |

Cloud offload does save compute power, but spends most of it back on sustained
transmit. The difference between the two realistic architectures is **8.5 W —
about 1.3% of the ~630 W hover draw**, well inside the noise in the mass
estimates. Roughly 15 seconds of flight time. Power does not decide this.

---

## Why edge compute is what makes the map sharing work

Your stated goal is for this drone to map a room and send that map to the other
vehicles being designed. That requirement is the clearest argument for the
split architecture:

- **Raw stereo is 2654 Mbps.** It cannot leave the aircraft, at any range.
- **An occupancy or voxel map of a 12 × 12 m room is a few hundred kilobytes.**
  It can be sent over almost any link, including the 2 Mbps available in the
  far corner of the room.

Processing on the aircraft compresses the link requirement by roughly **four
orders of magnitude**, turning an impossible stream into a trivial one. Edge
compute is not an alternative to sharing the map — it is the precondition for it.

The map is also the right thing to share: other vehicles need the *result*, not
the sensor feed. Distributing a finished map is latency-tolerant, happens once
per scan rather than 30 times a second, and is exactly the kind of work that
belongs off-board.

## Recommended architecture

```
  ON THE AIRCRAFT (Jetson Orin NX)          OFF-BOARD (server or cloud)
  ------------------------------            ---------------------------
  ZED depth + visual-inertial odom   -----> finished map (few hundred KB,
  local occupancy map                       once per scan)
  obstacle avoidance + local planner  <---- mission assignments, global map
  flight control (PX4)                      updates from other vehicles
                                            fleet coordination
  ^ must survive a total link loss           ^ may be seconds late
```

The aircraft must be able to complete or safely abort its mission with the link
down. Everything that survives being seconds late belongs off-board.

## Limits of this analysis

- **Verification level L0.** No measurement on the real link, the real boards,
  or in the real building. The throughput figures are measured-order estimates
  for 802.11ac, not site survey data.
- **A site survey would change the numbers.** Factory RF is highly specific;
  access-point placement could raise the in-room figure substantially. It would
  not change the dropout argument.
- **Compression ratios are assumptions.** A lossless or learned depth codec
  could improve the depth budget meaningfully.
- **Latency figures assume 720p30.** Dropping to 480p15 would cut the offload
  bandwidth by roughly 4× and make the numbers much closer — at the cost of the
  range and precision of the obstacle detection.
