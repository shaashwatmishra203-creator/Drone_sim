#!/usr/bin/env python3
"""
state_estimator — camera + IMU fusion, and the drift it prevents.

WHY THIS EXISTS
  The parts list has no GPS. Indoors there would be none anyway. That leaves
  the IMU and the camera, and an IMU alone cannot hold a position: accelerometer
  bias is integrated twice, so error grows with the square of time. This node
  runs both estimators side by side so the difference is a measurement rather
  than an assertion:

    imu_only   strapdown dead reckoning from the IMU alone
    fused      the same prediction, corrected whenever the camera recognises a
               mapped barrier (a gate) and can fix position against it

  PX4's own EKF2 estimate is logged alongside as the reference. It is not
  ground truth, but it is the best available and it is what the vehicle flies on.

HOW THE CAMERA CORRECTS
  The mapper reports the barrier ahead as (x, gap_centre_y, width). Gate
  positions are fixed features of the course, so observing one gives a position
  fix: the measured barrier range pins x, and the gap centre pins y. That is a
  landmark update, which is what bounds the drift.

LIMITS
  - Correction depends on the mapper's geometric sensor model, so it inherits
    all of its assumptions.
  - Landmark association is by nearest mapped barrier; a wrong association
    would inject error rather than remove it.
  - This estimates position, not attitude. Attitude comes from PX4.
"""

import json
import math
import os
import sys

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node as RclNode
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, Float32MultiArray

from px4_msgs.msg import SensorCombined, VehicleAttitude, VehicleLocalPosition

PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=5)
LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

G = 9.80665


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


def quat_rotate(q, v):
    """Rotate vector v by quaternion q = (w,x,y,z)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


class StateEstimator(RclNode):

    def __init__(self):
        super().__init__("state_estimator")
        self.declare_parameter("world_tools", "")
        self.declare_parameter("out_json", "")
        # A calibrated ICM-42688-P class part sits around 0.01-0.02 m/s^2 of
        # residual accelerometer bias. 0.02 is used here: double-integrated over
        # a 90 s mission that alone is tens of metres, which is the whole point
        # of carrying a camera.
        self.declare_parameter("accel_bias", 0.02)     # m/s^2, modelled bias
        self.declare_parameter("gyro_bias", 0.002)     # rad/s
        self.declare_parameter("fix_gain", 0.6)        # correction strength

        wt = self.get_parameter("world_tools").value
        if wt and wt not in sys.path:
            sys.path.insert(0, wt)
        from gen_world import GATES
        self.gates = [(float(x), float(y)) for x, y in GATES]

        self.out_json = self.get_parameter("out_json").value
        self.ab = float(self.get_parameter("accel_bias").value)
        self.gb = float(self.get_parameter("gyro_bias").value)
        self.k = float(self.get_parameter("fix_gain").value)

        # two estimators, same prediction, different corrections
        self.imu_p = None      # [x, y, z] dead reckoning only
        self.imu_v = [0.0, 0.0, 0.0]
        self.fus_p = None      # [x, y, z] IMU + camera landmark fixes
        self.fus_v = [0.0, 0.0, 0.0]

        self.q = None
        self.ref = None        # PX4 EKF2 position, the reference
        self.last_t = None
        self.samples = []
        self.fixes = 0
        self.imu_updates = 0
        self._closing = False

        self.cbg = ReentrantCallbackGroup()
        t_imu = resolve_topic(self, "sensor_combined")
        t_att = resolve_topic(self, "vehicle_attitude")
        t_pos = resolve_topic(self, "vehicle_local_position")
        self.create_subscription(SensorCombined, t_imu, self.cb_imu,
                                 PX4_QOS, callback_group=self.cbg)
        self.create_subscription(VehicleAttitude, t_att, self.cb_att,
                                 PX4_QOS, callback_group=self.cbg)
        self.create_subscription(VehicleLocalPosition, t_pos, self.cb_ref,
                                 PX4_QOS, callback_group=self.cbg)
        self.create_subscription(Float32MultiArray, "/drone/nav/gate_gap",
                                 self.cb_gate, 10, callback_group=self.cbg)
        self.create_subscription(Bool, "/drone_eval/mission_done",
                                 self.cb_done, LATCHED, callback_group=self.cbg)

        self.create_timer(0.05, self.record, callback_group=self.cbg)
        self.create_timer(5.0, self.report, callback_group=self.cbg)
        self.get_logger().info(
            f"camera+IMU fusion: accel bias {self.ab} m/s2, gyro bias {self.gb} "
            f"rad/s, {len(self.gates)} gate landmarks")

    # -- inputs ------------------------------------------------------------
    def cb_att(self, m):
        self.q = tuple(float(x) for x in m.q)

    def cb_ref(self, m):
        self.ref = (float(m.x), float(m.y), float(m.z))
        if self.imu_p is None and abs(m.z) > 0.1:
            # initialise both estimators from the first good fix
            self.imu_p = [float(m.x), float(m.y), float(m.z)]
            self.fus_p = [float(m.x), float(m.y), float(m.z)]

    def cb_imu(self, m):
        """Strapdown prediction. Both estimators share it; only the fused one
        is ever corrected."""
        if self.q is None or self.imu_p is None or self._closing:
            return
        t = m.timestamp * 1e-6
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        if dt <= 0.0 or dt > 0.2:
            self.last_t = t
            return
        self.last_t = t
        self.imu_updates += 1

        # body accel + a constant bias, rotated to world, gravity removed
        a_b = (float(m.accelerometer_m_s2[0]) + self.ab,
               float(m.accelerometer_m_s2[1]) + self.ab * 0.6,
               float(m.accelerometer_m_s2[2]))
        ax, ay, az = quat_rotate(self.q, a_b)
        az += G                      # NED: gravity is +z down

        for p, v in ((self.imu_p, self.imu_v), (self.fus_p, self.fus_v)):
            v[0] += ax * dt
            v[1] += ay * dt
            v[2] += az * dt
            p[0] += v[0] * dt
            p[1] += v[1] * dt
            p[2] += v[2] * dt

    def cb_gate(self, m):
        """Camera landmark fix.

        The camera measures where a gate is RELATIVE to the aircraft: rel_fwd
        along the course and rel_lat across it. Gates are fixed features at
        known positions, so

            implied aircraft position = gate_position - relative_measurement

        That is a genuine position fix. Using the mapper's world-frame estimate
        instead would be circular, because the mapper builds it from the
        vehicle's own pose.
        """
        if self._closing or self.fus_p is None or len(m.data) < 6:
            return
        bx, gy, gw, conf, rel_fwd, rel_lat = (float(v) for v in m.data)
        if conf < 0.3 or gw < 1.5 or rel_fwd < 1.0:
            return
        # associate the observation with the nearest known gate, using the
        # fused estimate's own prediction of where that gate should appear
        pred_x = self.fus_p[0] + rel_fwd
        gate = min(self.gates, key=lambda g: abs(g[0] - pred_x))
        if abs(gate[0] - pred_x) > 2.5:
            return                      # no confident association
        implied_x = gate[0] - rel_fwd
        implied_y = gate[1] - rel_lat
        self.fus_p[0] += self.k * (implied_x - self.fus_p[0])
        self.fus_p[1] += self.k * (implied_y - self.fus_p[1])
        # a position fix also says the velocity error was smaller than assumed
        self.fus_v[0] *= (1.0 - self.k * 0.7)
        self.fus_v[1] *= (1.0 - self.k * 0.7)
        self.fixes += 1

    # -- output ------------------------------------------------------------
    def record(self):
        if self._closing or self.ref is None or self.imu_p is None:
            return
        rx, ry, rz = self.ref
        ie = math.dist(self.imu_p[:2], [rx, ry])
        fe = math.dist(self.fus_p[:2], [rx, ry])
        self.samples.append({
            "ref_x": round(rx, 3), "ref_y": round(ry, 3), "ref_z": round(rz, 3),
            "imu_x": round(self.imu_p[0], 3), "imu_y": round(self.imu_p[1], 3),
            "fused_x": round(self.fus_p[0], 3), "fused_y": round(self.fus_p[1], 3),
            "imu_err_m": round(ie, 3), "fused_err_m": round(fe, 3),
        })

    def report(self):
        if not self.samples:
            return
        s = self.samples[-1]
        self.get_logger().info(
            f"imu_only err {s['imu_err_m']:6.2f} m | "
            f"camera+imu err {s['fused_err_m']:5.2f} m | "
            f"fixes {self.fixes} | imu updates {self.imu_updates}")

    def cb_done(self, m):
        if not m.data or self._closing:
            return
        self._closing = True
        try:
            self.save()
        except Exception as e:
            self.get_logger().error(f"save failed: {e}")
        self.create_timer(1.0, lambda: os._exit(0))

    def save(self):
        if not self.out_json or not self.samples:
            return
        ie = [s["imu_err_m"] for s in self.samples]
        fe = [s["fused_err_m"] for s in self.samples]

        def rms(v):
            return math.sqrt(sum(x * x for x in v) / len(v))

        doc = {
            "purpose": "quantify what the camera adds to an IMU-only estimate",
            "reference": "PX4 EKF2 vehicle_local_position (best available, "
                         "not ground truth)",
            "verification_level": "L1 (model-based)",
            "imu": {"modelled_accel_bias_ms2": self.ab,
                    "modelled_gyro_bias_rads": self.gb,
                    "updates": self.imu_updates},
            "camera": {"landmark_fixes": self.fixes,
                       "fix_gain": self.k,
                       "landmarks": len(self.gates)},
            "error_vs_reference_m": {
                "imu_only_final": round(ie[-1], 3),
                "imu_only_max": round(max(ie), 3),
                "imu_only_rms": round(rms(ie), 3),
                "camera_imu_final": round(fe[-1], 3),
                "camera_imu_max": round(max(fe), 3),
                "camera_imu_rms": round(rms(fe), 3),
                "rms_improvement_factor": round(rms(ie) / rms(fe), 2)
                if rms(fe) > 1e-6 else None,
            },
            "limits": [
                "camera fixes inherit the geometric sensor model's assumptions",
                "landmark association is nearest-gate; a wrong match injects error",
                "estimates position only; attitude comes from PX4",
                "the reference is PX4's EKF2, which itself uses GPS in this sim",
            ],
            "sample_count": len(self.samples),
            "samples": self.samples,
        }
        os.makedirs(os.path.dirname(os.path.abspath(self.out_json)), exist_ok=True)
        with open(self.out_json, "w") as f:
            json.dump(doc, f, indent=2, default=lambda o: float(o))
        e = doc["error_vs_reference_m"]
        self.get_logger().info(
            f"wrote {self.out_json}: IMU-only RMS {e['imu_only_rms']} m, "
            f"camera+IMU RMS {e['camera_imu_rms']} m "
            f"({e['rms_improvement_factor']}x better)")


def main():
    rclpy.init()
    node = StateEstimator()
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
            print(f"save on shutdown failed: {e}")
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
