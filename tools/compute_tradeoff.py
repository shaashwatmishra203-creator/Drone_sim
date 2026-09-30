#!/usr/bin/env python3
"""
compute_tradeoff.py — cloud offload versus onboard (edge) compute for the
factory navigation and mapping mission.

The question is not "which is faster in general" but three specific ones:

  1. BANDWIDTH   Can the link carry what cloud offload requires, at the range
                 and through the walls this mission actually flies?
  2. LATENCY     Is the perception-to-action delay short enough that the drone
                 can stop before hitting something, at mission speed?
  3. AVAILABILITY What happens when the link drops? A factory full of steel
                 racking is a hostile RF environment.

Power is modelled too, but it is not the deciding factor - the difference is
small either way.

Usage: python3 tools/compute_tradeoff.py [--speed 1.76] [--json out/compute.json]
"""

import argparse
import json
import math
import os

ROOT = os.path.expanduser("~/drone_sim")

# ---------------------------------------------------------------- sensors
# ZED 2 / ZED Mini at 720p30. Raw stereo is two 1280x720 frames; depth is a
# float32 map of the same size. These are the streams an offload architecture
# would have to push off the aircraft.
SENSORS = {
    "zed2_stereo_720p30": dict(w=1280, h=720, fps=30, chans=2, bytes_px=2,
                               label="ZED 2 stereo (720p30, YUV422)"),
    "zed_mini_stereo_720p30": dict(w=1280, h=720, fps=30, chans=2, bytes_px=2,
                                   label="ZED Mini stereo (720p30, YUV422)"),
    "depth_720p30": dict(w=1280, h=720, fps=30, chans=1, bytes_px=4,
                         label="Depth map (720p30, float32)"),
}
# Realistic compression ratios. Stereo compresses like video; depth does not -
# it is high-entropy and lossy compression destroys the geometry you need.
COMPRESSION = {"stereo": 45.0, "depth": 8.0}

# ---------------------------------------------------------------- link
# ALFA AWUS036ACM: 802.11ac, 2x2 MIMO, 867 Mbps PHY ceiling at 5 GHz / 80 MHz.
# PHY rate is not throughput. These are measured-order TCP goodput figures.
LINK = {
    "phy_max_mbps": 867.0,
    "goodput_fraction": 0.45,      # MAC overhead, retries, half duplex
    "tx_power_w": 6.0,             # sustained transmit draw
    "range_curve": [               # (metres, usable goodput Mbps, line of sight)
        (5, 390), (10, 330), (20, 220), (30, 150), (50, 85), (80, 40), (120, 15),
    ],
    "wall_penetration_db": 14.0,   # one interior wall at 5 GHz
    "rack_clutter_db": 8.0,        # steel racking, multipath and shadowing
}

# ---------------------------------------------------------------- compute
PLATFORMS = {
    "pi5": dict(
        label="Raspberry Pi 5 (8 GB)", mass_g=100, power_w=12.0, cuda=False,
        depth_fps=0.0,       # cannot run the ZED SDK at all
        vio_fps=0.0,
        note="No NVIDIA GPU. The ZED SDK requires CUDA, so neither ZED "
             "camera produces depth on this board under any configuration."),
    "orin_nx": dict(
        label="Jetson Orin NX 16 GB", mass_g=180, power_w=25.0, cuda=True,
        depth_fps=60.0,
        vio_fps=60.0,
        note="Runs the ZED SDK natively: NEURAL depth and positional tracking "
             "at 30-60 Hz within its power envelope."),
}

# ---------------------------------------------------------------- latency
# Each stage in milliseconds. Cloud RTT assumes a regional datacentre; an
# on-premise server would cut the network term to single digits.
LAT_EDGE = [("sensor capture + transfer", 33.0),
            ("depth + VIO on device", 22.0),
            ("local planner", 8.0),
            ("control + actuator", 6.0)]

LAT_CLOUD = [("sensor capture + transfer", 33.0),
             ("encode for transmission", 18.0),
             ("uplink serialisation", None),      # computed from bandwidth
             ("network round trip", 45.0),
             ("remote inference", 25.0),
             ("downlink + decode", 9.0),
             ("control + actuator", 6.0)]

LAT_ONPREM = [("sensor capture + transfer", 33.0),
              ("encode for transmission", 18.0),
              ("uplink serialisation", None),
              ("network round trip", 6.0),        # same building
              ("remote inference", 25.0),
              ("downlink + decode", 9.0),
              ("control + actuator", 6.0)]

# Quadrotor emergency deceleration. A multirotor stops by pitching back; at a
# 30 degree limit that is g*tan(30).
MAX_TILT_DEG = 30.0
G = 9.80665


def raw_mbps(s):
    d = SENSORS[s]
    return d["w"] * d["h"] * d["fps"] * d["chans"] * d["bytes_px"] * 8 / 1e6


def link_goodput(range_m, walls=0, clutter=True):
    """Interpolate the range curve, then apply obstruction losses.
    Each 6 dB of loss roughly halves usable throughput in this regime."""
    pts = LINK["range_curve"]
    if range_m <= pts[0][0]:
        base = pts[0][1]
    elif range_m >= pts[-1][0]:
        base = pts[-1][1]
    else:
        base = pts[-1][1]
        for i in range(len(pts) - 1):
            r0, g0 = pts[i]
            r1, g1 = pts[i + 1]
            if r0 <= range_m <= r1:
                f = (range_m - r0) / (r1 - r0)
                base = g0 + f * (g1 - g0)
                break
    db = walls * LINK["wall_penetration_db"] + (LINK["rack_clutter_db"] if clutter else 0)
    return base * (2 ** (-db / 6.0))


def stopping_distance(v):
    a = G * math.tan(math.radians(MAX_TILT_DEG))
    return v * v / (2 * a), a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=1.76,
                    help="mission speed m/s (measured mission mean)")
    ap.add_argument("--max-range", type=float, default=32.0,
                    help="furthest point from base, metres")
    ap.add_argument("--json", default=f"{ROOT}/out/compute_tradeoff.json")
    a = ap.parse_args()

    out = {"speed_ms": a.speed, "max_range_m": a.max_range}

    print("=" * 74)
    print("CLOUD OFFLOAD vs ONBOARD (EDGE) COMPUTE")
    print("for the factory navigation + mapping mission")
    print("=" * 74)

    # ---------------- 1 bandwidth
    print("\n1. BANDWIDTH — what offload would have to push off the aircraft")
    print("-" * 74)
    print(f"  {'stream':<38} {'raw Mbps':>10} {'compressed':>12}")
    tot_raw = tot_comp = 0.0
    streams = []
    for k, d in SENSORS.items():
        raw = raw_mbps(k)
        ratio = COMPRESSION["depth"] if "depth" in k else COMPRESSION["stereo"]
        comp = raw / ratio
        tot_raw += raw
        tot_comp += comp
        streams.append({"stream": d["label"], "raw_mbps": raw, "comp_mbps": comp})
        print(f"  {d['label']:<38} {raw:10.0f} {comp:11.1f}")
    print(f"  {'TOTAL':<38} {tot_raw:10.0f} {tot_comp:11.1f}")
    out["streams"] = streams
    out["total_raw_mbps"] = tot_raw
    out["total_compressed_mbps"] = tot_comp

    print(f"\n  Offload needs roughly {tot_comp:.0f} Mbps sustained, with low jitter.")
    print("  Note depth compresses only ~8x: lossy compression destroys the very")
    print("  geometry the obstacle avoidance depends on.")

    print("\n  Link capability (ALFA AWUS036ACM, 802.11ac):")
    print(f"  {'condition':<44} {'goodput':>10} {'verdict':>14}")
    conds = [
        ("At base, line of sight (5 m)", 5, 0, False),
        ("Mid obstacle course (16 m), steel racking", 16, 0, True),
        ("Room doorway (30 m), racking", 30, 0, True),
        ("Inside room (32 m), 1 wall + racking", 32, 1, True),
        ("Inside room, 2 walls (far corner)", 34, 2, True),
    ]
    bw_rows = []
    for label, rng, walls, clutter in conds:
        g = link_goodput(rng, walls, clutter)
        ok = g > tot_comp * 1.5          # 50% headroom for jitter and retries
        marg = g > tot_comp
        v = "OK" if ok else ("MARGINAL" if marg else "INSUFFICIENT")
        bw_rows.append({"condition": label, "goodput_mbps": g, "verdict": v})
        print(f"  {label:<44} {g:8.0f} Mb {v:>14}")
    out["bandwidth"] = bw_rows

    print("\n  The mapping pass happens INSIDE the room, behind a wall, at the")
    print("  furthest point from base. That is precisely where the link is worst.")

    # ---------------- 2 latency
    print("\n\n2. LATENCY — can it stop in time?")
    print("-" * 74)

    def chain(stages, up_mbps):
        tot = 0.0
        det = []
        for name, ms in stages:
            if ms is None:
                # serialisation delay for one frame's worth of compressed data
                frame_mbits = tot_comp / 30.0
                ms = frame_mbits / max(up_mbps, 0.1) * 1000.0
            det.append((name, ms))
            tot += ms
        return tot, det

    g_room = link_goodput(32, 1, True)
    arch = {}
    for name, stages, up in [("Edge (onboard)", LAT_EDGE, None),
                             ("Cloud (regional DC)", LAT_CLOUD, g_room),
                             ("On-premise server", LAT_ONPREM, g_room)]:
        tot, det = chain(stages, up if up else 1e9)
        arch[name] = tot
        print(f"\n  {name}: {tot:.0f} ms total")
        for n, ms in det:
            print(f"      {n:<30} {ms:7.1f} ms")
    out["latency_ms"] = arch

    stop_d, decel = stopping_distance(a.speed)
    print(f"\n  At {a.speed:.2f} m/s (measured mission mean), braking at "
          f"{decel:.1f} m/s2:")
    print(f"  {'architecture':<24} {'latency':>9} {'react':>8} {'stop':>8} "
          f"{'total':>8} {'verdict':>13}")
    CLEAR = 1.25   # clearance from corridor centreline to nearest obstacle
    react_rows = []
    for name, lat in arch.items():
        react = a.speed * lat / 1000.0
        total = react + stop_d
        v = "SAFE" if total < CLEAR * 0.5 else \
            "ACCEPTABLE" if total < CLEAR else "UNSAFE"
        react_rows.append({"arch": name, "latency_ms": lat,
                           "reaction_m": react, "stop_m": stop_d,
                           "total_m": total, "verdict": v})
        print(f"  {name:<24} {lat:7.0f}ms {react:7.2f}m {stop_d:7.2f}m "
              f"{total:7.2f}m {v:>13}")
    print(f"\n  Corridor clearance is {CLEAR:.2f} m from centreline to the "
          f"nearest rack or pillar.")
    out["reaction"] = react_rows

    print("\n  Speed at which each architecture consumes the full clearance:")
    for name, lat in arch.items():
        lo, hi = 0.1, 30.0
        for _ in range(80):
            mid = (lo + hi) / 2
            sd, _ = stopping_distance(mid)
            if mid * lat / 1000.0 + sd < CLEAR:
                lo = mid
            else:
                hi = mid
        print(f"      {name:<24} max safe speed {lo:5.2f} m/s")
        out.setdefault("max_safe_speed", {})[name] = lo

    # ---------------- 3 availability
    print("\n\n3. AVAILABILITY — what a dropout costs")
    print("-" * 74)
    for d_ms in (200, 500, 1000):
        blind = a.speed * d_ms / 1000.0
        print(f"  A {d_ms:4d} ms link dropout leaves a cloud-dependent drone "
              f"blind for {blind:.2f} m of travel.")
    print("\n  An edge architecture is unaffected: perception and control never")
    print("  leave the aircraft. In a steel-racked factory, multi-hundred-ms")
    print("  dropouts from shadowing and multipath are routine, not exceptional.")
    print("  This is a safety argument, and it does not soften with a better link.")

    # ---------------- 4 platform capability
    print("\n\n4. PLATFORM CAPABILITY — can the listed board even do the job?")
    print("-" * 74)
    print(f"  {'platform':<26} {'mass':>7} {'power':>7} {'CUDA':>6} "
          f"{'depth':>9} {'verdict':>16}")
    for k, p in PLATFORMS.items():
        v = "CANNOT RUN ZED" if not p["cuda"] else "capable"
        print(f"  {p['label']:<26} {p['mass_g']:5d} g {p['power_w']:5.0f} W "
              f"{('yes' if p['cuda'] else 'NO'):>6} "
              f"{(str(int(p['depth_fps'])) + ' fps' if p['depth_fps'] else '-'):>9} "
              f"{v:>16}")
    print()
    for k, p in PLATFORMS.items():
        print(f"  {p['label']}: {p['note']}")

    # ---------------- 5 power
    print("\n\n5. POWER — smaller than you would expect")
    print("-" * 74)
    print(f"  {'architecture':<34} {'compute':>9} {'radio':>8} {'total':>8}")
    radio_cloud = LINK["tx_power_w"]
    radio_edge = 1.5    # telemetry and map upload only, low duty cycle
    combos = [("Edge, Orin NX (map upload only)", PLATFORMS["orin_nx"]["power_w"], radio_edge),
              ("Cloud offload, Pi 5 + full streaming", PLATFORMS["pi5"]["power_w"], radio_cloud),
              ("Cloud offload, Orin NX + streaming", PLATFORMS["orin_nx"]["power_w"], radio_cloud)]
    pw = {}
    for label, c, r in combos:
        pw[label] = c + r
        print(f"  {label:<34} {c:7.0f} W {r:6.1f} W {c+r:6.1f} W")
    delta = pw["Cloud offload, Pi 5 + full streaming"] - pw["Edge, Orin NX (map upload only)"]
    print(f"\n  Difference between the two realistic architectures: "
          f"{abs(delta):.1f} W")
    print(f"  Against a ~630 W hover draw that is {abs(delta)/630*100:.1f}% of total "
          f"power - well under the noise in the mass estimates.")
    print("  Power does not decide this question.")
    out["power_w"] = pw

    # ---------------- verdict
    print("\n\n" + "=" * 74)
    print("VERDICT")
    print("=" * 74)
    print("""
  Split the workload. They are not competing options; they solve different
  halves of the problem.

  EDGE (onboard, mandatory) - the control loop:
      depth, visual-inertial odometry, obstacle avoidance, local planning.
      Reasons, in order of weight:
        1. A dropout must not blind the aircraft. This is a safety property,
           and no amount of link engineering removes it.
        2. Offload needs ~{comp:.0f} Mbps sustained; inside the room, behind a
           wall, the link delivers about {groom:.0f} Mbps.
        3. Cloud latency roughly {ratio:.1f}x the edge figure, which caps safe
           speed at {cs:.2f} m/s against {es:.2f} m/s.

  CLOUD / OFF-BOARD (appropriate and useful) - everything latency-tolerant:
      map merging across vehicles, global pose-graph optimisation, long-term
      storage, fleet coordination, and distributing the finished map to the
      other vehicles being designed.

  The elegant part: edge compute is what makes the map sharing FEASIBLE.
  Raw stereo is {raw:.0f} Mbps and cannot leave the aircraft. An occupancy or
  voxel map of a 12 x 12 m room is a few hundred kilobytes. Processing on the
  aircraft compresses the link requirement by roughly four orders of magnitude,
  turning an impossible stream into a trivial one.

  CONSEQUENCE FOR THE PARTS LIST:
      The Raspberry Pi 5 cannot run either ZED camera, so "edge" is not an
      option with the currently listed board - and cloud offload is not a way
      to rescue it, because offload still fails on bandwidth, latency and
      dropout. A CUDA-capable board is required either way.
""".format(comp=tot_comp, groom=g_room,
           ratio=arch["Cloud (regional DC)"] / arch["Edge (onboard)"],
           cs=out["max_safe_speed"]["Cloud (regional DC)"],
           es=out["max_safe_speed"]["Edge (onboard)"],
           raw=tot_raw))

    os.makedirs(os.path.dirname(a.json), exist_ok=True)
    with open(a.json, "w") as f:
        json.dump(out, f, indent=2)
    print(f"wrote {a.json}")


if __name__ == "__main__":
    main()
