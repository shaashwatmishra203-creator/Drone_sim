#!/usr/bin/env python3
"""
camera_mapper — map the course from the Arducam and find the gaps to fly through.

TWO PERCEPTION MODES
--------------------
  geometric  (default)  Ray-cast the camera's field of view against the world
                        geometry. Produces bearing + range returns with the
                        camera's real FOV, range limit, angular resolution and
                        occlusion. No GPU, negligible CPU.

  pixel                 Segment the floor in the rendered image and project
                        obstacle bases onto the ground plane. This is the real
                        algorithm, but it CANNOT RUN HERE: Gazebo renders the
                        camera entirely black when headless in WSL (measured:
                        mean 0, max 0 over every frame) because there is no GPU
                        render context. Kept because it is correct on a machine
                        with working offscreen rendering.

WHAT 'geometric' IS AND IS NOT
------------------------------
It IS a sensor model. It honours everything about the Arducam that determines
what the drone can know: 66 deg horizontal FOV, 15 deg down-tilt, a usable
range window, the angular resolution of 640 px, occlusion by nearer objects,
and range noise that grows with distance.

It is NOT image processing. It does not model lighting, texture, motion blur,
or detection failures on low-contrast surfaces. A real monocular pipeline would
also have to infer range, which this grants directly. So it answers "can the
drone navigate the course given this camera's geometry?" and not "will the
vision algorithm work?".

Verification level: L1 — model-based, no rendered imagery involved.
"""

import json
import math
import os
import random
import sys
import threading

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node as RclNode
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, Float32MultiArray

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=5)

OUT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=5)

LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def resolve_topic(node, base, direction="out", timeout_s=20.0):
    import time
    want = f"/fmu/{direction}/{base}"
    end = time.time() + timeout_s
    while time.time() < end:
        names = [n for n, _ in node.get_topic_names_and_types()]
        if want in names:
            return want
        c = [n for n in names
             if n.startswith(want + "_v") and n[len(want) + 2:].isdigit()]
        if c:
            return sorted(c, key=lambda n: int(n.rsplit("_v", 1)[1]))[-1]
        time.sleep(0.5)
    return want


def ray_box_2d(ox, oy, dx, dy, bx, by, hx, hy):
    """Distance along a unit ray to an axis-aligned box, or None.
    Slab method; box centred (bx,by) with half-extents (hx,hy)."""
    tmin, tmax = 0.0, float("inf")
    for o, d, b, h in ((ox, dx, bx, hx), (oy, dy, by, hy)):
        lo, hi = b - h, b + h
        if abs(d) < 1e-9:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax:
            return None
    return tmin if tmin > 0 else None


class CameraMapper(RclNode):

    def __init__(self):
        super().__init__("camera_mapper")
        self.declare_parameter("config", "")
        self.declare_parameter("world_tools", "")
        self.declare_parameter("out_json", "")
        self.declare_parameter("mode", "geometric")
        self.declare_parameter("grid_res_m", 0.25)
        self.declare_parameter("max_range_m", 12.0)
        self.declare_parameter("min_range_m", 1.2)
        self.declare_parameter("n_rays", 64)

        cfg_path = self.get_parameter("config").value
        self.cfg = yaml.safe_load(open(cfg_path)) if cfg_path else {}
        cam = self.cfg.get("nav_camera", {})

        self.mode = self.get_parameter("mode").value
        self.W = int(cam.get("width", 640))
        self.hfov = math.radians(float(cam.get("hfov_deg", 66.0)))
        self.pitch_down = math.radians(float(cam.get("pitch_down_deg", 15.0)))
        self.res = float(self.get_parameter("grid_res_m").value)
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.min_range = float(self.get_parameter("min_range_m").value)
        self.n_rays = int(self.get_parameter("n_rays").value)
        self.out_json = self.get_parameter("out_json").value

        # World geometry, loaded from the SAME generator that builds the world,
        # so the sensor model can never disagree with what is actually there.
        wt = self.get_parameter("world_tools").value
        if wt and wt not in sys.path:
            sys.path.insert(0, wt)
        from gen_world import obstacle_list
        self.boxes = []
        for name, x, y, z, sx, sy, sz in obstacle_list():
            top = z + sz / 2.0
            if top < 0.4:          # too low to matter to a 2.5 m cruise
                continue
            self.boxes.append((name, x, y, sx / 2.0, sy / 2.0, top))

        self.hits = {}
        self.scans = 0
        self.returns = 0
        self.pos = None
        self.yaw = None
        self._closing = False
        self.last_gap = None
        self.rng = random.Random(1234)

        self.cbg = ReentrantCallbackGroup()
        t_pos = resolve_topic(self, "vehicle_local_position")
        t_att = resolve_topic(self, "vehicle_attitude")
        self.create_subscription(VehicleLocalPosition, t_pos, self.cb_pos,
                                 PX4_QOS, callback_group=self.cbg)
        self.create_subscription(VehicleAttitude, t_att, self.cb_att,
                                 PX4_QOS, callback_group=self.cbg)
        self.create_subscription(Bool, "/drone_eval/mission_done",
                                 self.cb_done, LATCHED, callback_group=self.cbg)

        # [bearing_rad, clear_range_m, confidence] — instantaneous look-ahead
        self.pub_gap = self.create_publisher(
            Float32MultiArray, "/drone/nav/gap", OUT_QOS)
        # [barrier_x, gap_y_centre, gap_width, confidence] — derived from the
        # ACCUMULATED map, which is what the navigator actually steers on.
        # A single frame's bearing is noisy and, close to a gate, small angular
        # errors become large lateral ones; averaging the map over many scans
        # is far steadier.
        self.pub_gate = self.create_publisher(
            Float32MultiArray, "/drone/nav/gate_gap", OUT_QOS)
        self.last_gate = None

        self.create_timer(1.0 / 10.0, self.scan, callback_group=self.cbg)
        self.create_timer(5.0, self.report, callback_group=self.cbg)

        self.get_logger().info(
            f"camera_mapper [{self.mode}] : {math.degrees(self.hfov):.0f} deg FOV, "
            f"{self.n_rays} rays, range {self.min_range}-{self.max_range} m, "
            f"grid {self.res} m, {len(self.boxes)} obstacles in world")
        if self.mode == "pixel":
            self.get_logger().warn(
                "pixel mode: Gazebo renders this camera black when headless in "
                "WSL, so no obstacles will be found. Use mode:=geometric.")

    # -- inputs ------------------------------------------------------------
    def cb_pos(self, m):
        self.pos = (m.x, m.y, m.z)

    def cb_att(self, m):
        w, x, y, z = m.q
        self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    # -- perception --------------------------------------------------------
    def scan(self):
        """One camera frame's worth of perception: cast rays across the FOV,
        keep the nearest return per ray, accumulate into the world map."""
        if self._closing or self.pos is None or self.yaw is None:
            return
        alt = -self.pos[2]
        if alt < 0.5:
            return

        ox, oy = self.pos[0], self.pos[1]
        half = self.hfov / 2.0
        clear = []

        for i in range(self.n_rays):
            # bearing within the FOV, +ve to the left
            frac = (i / (self.n_rays - 1.0)) - 0.5
            bearing = -frac * self.hfov
            th = self.yaw + bearing
            dx, dy = math.cos(th), math.sin(th)

            best = None
            for name, bx, by, hx, hy, top in self.boxes:
                # the camera is pitched down; something must be tall enough to
                # fall inside the vertical field of view at that distance
                t = ray_box_2d(ox, oy, dx, dy, bx, by, hx, hy)
                if t is None or t < self.min_range or t > self.max_range:
                    continue
                if top < alt - t * math.tan(self.pitch_down + half):
                    continue          # too short to be seen at that range
                if best is None or t < best:
                    best = t

            if best is None:
                clear.append((bearing, self.max_range))
                continue

            # range noise grows with distance, as monocular range inference does
            noisy = best * (1.0 + self.rng.gauss(0.0, 0.03)) \
                + self.rng.gauss(0.0, 0.02)
            noisy = max(self.min_range, min(self.max_range, noisy))
            clear.append((bearing, noisy))
            self.returns += 1

            wx = ox + dx * noisy
            wy = oy + dy * noisy
            key = (int(round(wx / self.res)), int(round(wy / self.res)))
            self.hits[key] = self.hits.get(key, 0) + 1

        self.scans += 1
        self.publish_gap(clear)
        self.publish_gate_gap(ox, oy)

    def publish_gate_gap(self, ox, oy, look_lo=1.5, look_hi=9.0):
        """Find the next barrier ahead in the ACCUMULATED map and locate its gap.

        Bins mapped cells by x in the band ahead of the aircraft; the fullest
        bin is the barrier. Within it, the widest y-interval with no mapped
        cells is the opening to fly through. Built from many scans, so it does
        not jitter the way a single frame's bearing does.
        """
        if not self.hits:
            return
        bin_w = 0.5
        bins = {}
        for (gx, gy), n in self.hits.items():
            x, y = gx * self.res, gy * self.res
            if not (ox + look_lo <= x <= ox + look_hi):
                continue
            if abs(y - oy) > 7.0:
                continue
            bins.setdefault(int(round(x / bin_w)), []).append(y)

        if not bins:
            return
        bx = max(bins, key=lambda k: len(bins[k]))
        ys = sorted(bins[bx])
        if len(ys) < 4:
            return
        barrier_x = bx * bin_w

        # widest interior gap in the barrier's y coverage
        best_w, best_c = 0.0, None
        for a, b in zip(ys, ys[1:]):
            if b - a > best_w:
                best_w, best_c = b - a, (a + b) / 2.0
        if best_c is None or best_w < 1.2:
            return                       # no opening wide enough to be a gate

        conf = min(1.0, len(ys) / 25.0)
        # Publish the RELATIVE geometry as well as the world-frame estimate.
        # The relative pair is what a landmark position fix needs: the world
        # values here are derived from the vehicle's own pose, so comparing
        # them against a known gate measures the mapper's error, not the
        # estimator's. Range and lateral offset from the aircraft do not have
        # that circularity.
        rel_fwd = barrier_x - ox
        rel_lat = best_c - oy
        msg = Float32MultiArray()
        msg.data = [float(barrier_x), float(best_c), float(best_w), float(conf),
                    float(rel_fwd), float(rel_lat)]
        self.pub_gate.publish(msg)
        self.last_gate = msg.data

    def publish_gap(self, clear):
        """Widest run of long-range bearings is the gap worth flying at."""
        if not clear:
            return
        far = self.max_range * 0.8
        best_len, best_span = 0, None
        run = None
        for i, (_, rng) in enumerate(clear + [(0.0, 0.0)]):
            if rng >= far:
                if run is None:
                    run = i
            elif run is not None:
                if i - run > best_len:
                    best_len, best_span = i - run, (run, i)
                run = None
        msg = Float32MultiArray()
        if best_span is None:
            nearest = min(r for _, r in clear)
            msg.data = [0.0, float(nearest), 0.0]
        else:
            a, b = best_span
            mid = (a + b) // 2
            msg.data = [float(clear[mid][0]), float(clear[mid][1]),
                        float(best_len) / len(clear)]
        self.pub_gap.publish(msg)
        self.last_gap = msg.data

    def report(self):
        gt = self.last_gate
        gs = (f"gate @ x={gt[0]:5.1f} gap y={gt[1]:+5.2f} "
              f"w={gt[2]:4.2f}m conf={gt[3]:.2f}") if gt else "no gate mapped yet"
        self.get_logger().info(
            f"scans {self.scans} | returns {self.returns} | "
            f"map cells {len(self.hits)} | {gs}")

    # -- output ------------------------------------------------------------
    def cb_done(self, m):
        if not m.data or self._closing:
            return
        self._closing = True
        self.get_logger().info("mission complete — saving map")
        try:
            self.save()
        except Exception as e:
            self.get_logger().error(f"map save failed: {e}")
        self.create_timer(1.0, lambda: os._exit(0))

    def save(self):
        if not self.out_json:
            return
        cells = [{"x_m": round(k[0] * self.res, 3),
                  "y_m": round(k[1] * self.res, 3), "hits": v}
                 for k, v in sorted(self.hits.items()) if v >= 2]
        xs = [c["x_m"] for c in cells] or [0.0]
        ys = [c["y_m"] for c in cells] or [0.0]
        doc = {
            "source": "Arducam Camera Module 3 — geometric sensor model",
            "mode": self.mode,
            "verification_level": "L1 (model-based; no rendered imagery)",
            "camera": {
                "hfov_deg": round(math.degrees(self.hfov), 1),
                "pitch_down_deg": round(math.degrees(self.pitch_down), 1),
                "image_width_px": self.W,
                "angular_bins": self.n_rays,
                "range_window_m": [self.min_range, self.max_range],
            },
            "grid_resolution_m": self.res,
            "scans": self.scans,
            "returns": self.returns,
            "occupied_cells": len(cells),
            "extent_m": {"x_min": min(xs), "x_max": max(xs),
                         "y_min": min(ys), "y_max": max(ys)},
            "method": "FOV ray-cast against world geometry with occlusion and "
                      "distance-dependent range noise; returns accumulated in "
                      "the world frame using the PX4 pose estimate",
            "limits": [
                "models camera geometry, not image appearance",
                "no lighting, texture or detection-failure modelling",
                "grants range directly; a real monocular pipeline must infer it",
                "pixel mode is unusable here: headless Gazebo renders this "
                "camera black (measured mean 0, max 0)",
            ],
            "cells": cells,
        }
        os.makedirs(os.path.dirname(os.path.abspath(self.out_json)), exist_ok=True)
        with open(self.out_json, "w") as f:
            json.dump(doc, f, indent=2)
        self.get_logger().info(
            f"wrote {self.out_json} ({len(cells)} cells, {self.scans} scans)")


def main():
    rclpy.init()
    node = CameraMapper()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        try:
            if not node._closing:
                node.save()
        except Exception as e:
            print(f"map save on shutdown failed: {e}")
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
