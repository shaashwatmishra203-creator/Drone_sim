#!/usr/bin/env python3
"""
mission_runner — arm, take off, and fly a scenario profile in offboard mode.

Profiles (from a scenario YAML):
  hover        hold an altitude
  speed_steps  hold altitude, step through forward airspeeds
  climb        maximum-rate climb to a ceiling
  waypoints    fly a list of local-frame waypoints

Ends on: profile completion, the logger reporting the DoD gate reached, or
max_duration_s. Always attempts a land + disarm so the next run starts clean.
"""

import math
import os
import sys

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, Float32MultiArray

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)
# Commands INTO PX4 want reliable delivery — the bridge accepts RELIABLE here.
CMD_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

NAN = float("nan")


def resolve_topic(node, base, direction="out", timeout_s=20.0):
    """PX4 1.18 appends message-version suffixes (vehicle_status_v4,
    vehicle_local_position_v1, ...) to some topics and not others. Match the
    live graph rather than hardcoding a name that breaks on upgrade."""
    import time
    want = f"/fmu/{direction}/{base}"
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        names = [n for n, _ in node.get_topic_names_and_types()]
        if want in names:
            return want
        cands = [n for n in names
                 if n.startswith(want + "_v") and n[len(want) + 2:].isdigit()]
        if cands:
            return sorted(cands, key=lambda n: int(n.rsplit("_v", 1)[1]))[-1]
        time.sleep(0.5)
    node.get_logger().warn(f"topic {want}[_vN] not found — using {want}")
    return want


class MissionRunner(Node):

    def __init__(self):
        super().__init__("mission_runner")
        self.declare_parameter("scenario", "")
        path = self.get_parameter("scenario").value
        if not path or not os.path.exists(path):
            self.get_logger().fatal(f"scenario not found: {path!r}")
            raise SystemExit(2)
        with open(path) as f:
            self.sc = yaml.safe_load(f)

        self.profile = self.sc["profile"]
        self.kind = self.profile["type"]
        self.alt = float(self.profile.get("altitude_m", 5.0))
        self.max_duration = float(self.sc.get("max_duration_s", 1800.0))
        self.until_empty = bool(self.sc.get("until_empty", False))

        # Resolve outbound topics first: PX4 must already be up, and the
        # inbound side carries version suffixes too.
        t_ocm = resolve_topic(self, "offboard_control_mode", "in")
        t_sp = resolve_topic(self, "trajectory_setpoint", "in")
        t_cmd = resolve_topic(self, "vehicle_command", "in")
        self.get_logger().info(f"  publishing {t_ocm} / {t_sp} / {t_cmd}")

        self.pub_ocm = self.create_publisher(OffboardControlMode, t_ocm, CMD_QOS)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, t_sp, CMD_QOS)
        self.pub_cmd = self.create_publisher(VehicleCommand, t_cmd, CMD_QOS)

        t_status = resolve_topic(self, "vehicle_status")
        t_pos = resolve_topic(self, "vehicle_local_position")
        self.get_logger().info(f"  subscribing {t_status} / {t_pos}")
        self.create_subscription(VehicleStatus, t_status, self.cb_status, PX4_QOS)
        self.create_subscription(VehicleLocalPosition, t_pos, self.cb_pos, PX4_QOS)
        self.create_subscription(Bool, "/drone_eval/battery_empty",
                                 self.cb_empty, 10)

        self.pub_done = self.create_publisher(Bool, "/drone_eval/mission_done",
                                              CMD_QOS)

        # No /clock exists without a ros_gz bridge, so use_sim_time would
        # freeze this node. PX4 message timestamps are the simulation clock.
        self.px4_us = None
        self.nav_state = None
        self.arming = None
        self.pos = None
        self.empty = False
        self.t0 = None
        self.n = 0
        self.phase = "boot"
        self.phase_t = 0.0
        self.step_i = 0
        self.travel = 0.0

        self.speeds = [float(s) for s in self.profile.get("speeds", [])]
        self.dwell = float(self.profile.get("dwell_s", 30.0))
        self.waypoints = self.profile.get("waypoints", [])

        # --- factory_mission state ------------------------------------
        # outbound -> scan -> return -> land, each leg timed separately so the
        # report can say how much of the flight each phase actually cost.
        self.leg = "outbound"
        self.leg_t0 = {}
        self.scan_i = 0
        self.scan_yaw0 = None
        self.outbound = self.profile.get("outbound", [])
        self.scan = self.profile.get("scan", {})
        self.inbound = self.profile.get("inbound", [])
        self.base = self.profile.get("base", [0.0, 0.0])
        self.legs_report = []

        # --- camera-guided lateral steering ---------------------------
        # With this on, the planned waypoints set only the along-course
        # progression; the LATERAL position at each gate comes from the gap the
        # camera actually sees. Without it the aircraft flies the planned path
        # blind, which is what every earlier run did.
        self.camera_guided = bool(self.profile.get("camera_guided", False))
        self.gap = None            # (bearing_rad, range_m, confidence)
        self.gap_t = 0.0
        self.gap_min_conf = float(self.profile.get("gap_min_conf", 0.08))
        self.gap_lookahead = float(self.profile.get("gap_lookahead_m", 4.0))
        self.gap_max_offset = float(self.profile.get("gap_max_offset_m", 4.0))
        rng_x = self.profile.get("camera_guided_x_range", [3.0, 24.0])
        self.cam_x0, self.cam_x1 = float(rng_x[0]), float(rng_x[1])
        self.cam_corrections = 0
        self.cam_fallbacks = 0
        self.gate = None
        self.gate_t = 0.0
        self.create_subscription(Float32MultiArray, "/drone/nav/gap",
                                 self.cb_gap, 10)
        self.create_subscription(Float32MultiArray, "/drone/nav/gate_gap",
                                 self.cb_gate, 10)

        self.create_timer(0.1, self.tick)       # 10 Hz, PX4 offboard minimum
        self.get_logger().info(
            f"scenario '{self.sc.get('name','?')}' type={self.kind} "
            f"alt={self.alt} m until_empty={self.until_empty} "
            f"max={self.max_duration}s")

    def cb_status(self, m):
        self.nav_state = m.nav_state
        self.arming = m.arming_state

    def cb_pos(self, m):
        self.pos = m
        self.px4_us = m.timestamp

    def cb_gap(self, m):
        if len(m.data) >= 3:
            self.gap = (m.data[0], m.data[1], m.data[2])
            self.gap_t = self.now()

    def cb_gate(self, m):
        # [barrier_x, gap_y_centre, gap_width, confidence] from the built map
        if len(m.data) >= 4:
            self.gate = (m.data[0], m.data[1], m.data[2], m.data[3])
            self.gate_t = self.now()

    def camera_y(self, planned_y):
        """Lateral target from what the camera sees, or the planned value.

        Returns (y, used_camera). Falls back to the plan when the measurement
        is stale or low confidence — a navigator that steers on a bad reading
        is worse than one that ignores it.
        """
        if not self.camera_guided or self.gap is None or self.pos is None:
            return planned_y, False
        # Only steer on the camera inside the gated corridor. Beyond it the
        # "gap" is the 4 m room doorway, and projecting that bearing out to the
        # lookahead distance overshoots into the door frame — one run clipped
        # it. The slalom is the part that needs perception; the room survey is
        # a planned pattern.
        if not (self.cam_x0 <= self.pos.x <= self.cam_x1):
            return planned_y, False
        # Steer on the MAPPED gate gap, not a single frame's bearing. The map
        # averages hundreds of scans, so it does not jitter; chasing the
        # instantaneous bearing clipped a gate because small angular errors
        # become large lateral ones at close range.
        if self.gate is None or (self.now() - self.gate_t) > 2.0:
            self.cam_fallbacks += 1
            return planned_y, False
        bx, gy, gw, conf = self.gate
        if conf < self.gap_min_conf or gw < 1.5:
            self.cam_fallbacks += 1
            return planned_y, False
        if abs(gy - planned_y) > self.gap_max_offset:
            self.cam_fallbacks += 1
            return planned_y, False       # implausible; trust the plan
        self.cam_corrections += 1
        return gy, True

    def cb_empty(self, m):
        if m.data and not self.empty:
            self.empty = True
            self.get_logger().warn("logger reports DoD gate reached — landing")

    # ------------------------------------------------------------------
    def now(self):
        """Simulation time in seconds, taken from PX4's own timestamps.
        Falls back to wall clock only before the first message arrives."""
        if self.px4_us is not None:
            return self.px4_us * 1e-6
        return self.get_clock().now().nanoseconds * 1e-9

    def cmd(self, command, **kw):
        m = VehicleCommand()
        m.command = command
        m.param1 = kw.get("p1", 0.0)
        m.param2 = kw.get("p2", 0.0)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.now() * 1e6)
        self.pub_cmd.publish(m)

    def heartbeat(self, position=True, velocity=False):
        m = OffboardControlMode()
        m.position = position
        m.velocity = velocity
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        m.timestamp = int(self.now() * 1e6)
        self.pub_ocm.publish(m)

    def setpoint(self, pos=None, vel=None, yaw=0.0):
        m = TrajectorySetpoint()
        m.position = [float(x) for x in pos] if pos else [NAN, NAN, NAN]
        m.velocity = [float(x) for x in vel] if vel else [NAN, NAN, NAN]
        m.yaw = float(yaw)
        m.timestamp = int(self.now() * 1e6)
        self.pub_sp.publish(m)

    def finish(self, why):
        if getattr(self, "_finishing", False):
            return
        self._finishing = True
        self.get_logger().info(f"mission complete: {why}")
        b = Bool()
        b.data = True
        self.pub_done.publish(b)
        self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        # rclpy.shutdown() from inside a timer callback does NOT reliably end
        # the process — it leaves spin() wedged and the run script then waits
        # out the full max_duration for nothing. Give the land command a moment
        # to reach PX4, then exit hard.
        self.create_timer(2.0, self._hard_exit)

    def _hard_exit(self):
        self.get_logger().info("exiting")
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)

    # ------------------------------------------------------------------
    def tick(self):
        t = self.now()
        if self.t0 is None:
            self.t0 = t
        elapsed = t - self.t0
        self.n += 1

        # PX4 needs a stream of offboard heartbeats BEFORE it will accept the
        # mode switch, hence the warm-up before arming.
        use_vel = self.kind == "speed_steps" and self.phase == "run"
        self.heartbeat(position=not use_vel, velocity=use_vel)

        if self.phase == "boot":
            self.setpoint(pos=[0.0, 0.0, -self.alt])
            if self.n == 20:
                self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)
                self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
                self.get_logger().info("arm + offboard requested")
            if self.n > 40 and self.pos is not None and -self.pos.z > self.alt * 0.9:
                self.phase = "run"
                self.phase_t = t
                self.get_logger().info(f"reached {self.alt} m — profile start")
            return

        if self.empty:
            return self.finish("battery DoD gate reached")
        if elapsed > self.max_duration:
            return self.finish(f"max_duration_s {self.max_duration} reached")

        dt_phase = t - self.phase_t

        if self.kind == "hover":
            self.setpoint(pos=[0.0, 0.0, -self.alt])
            if not self.until_empty and dt_phase > self.profile.get("hold_s", 120):
                self.finish("hover hold complete")

        elif self.kind == "speed_steps":
            if self.step_i >= len(self.speeds):
                return self.finish("speed sweep complete")
            v = self.speeds[self.step_i]
            self.setpoint(vel=[v, 0.0, 0.0], yaw=0.0)
            if dt_phase > self.dwell:
                self.get_logger().info(f"step {self.step_i}: {v:.1f} m/s done")
                self.step_i += 1
                self.phase_t = t

        elif self.kind == "climb":
            ceiling = float(self.profile.get("ceiling_m", 100.0))
            rate = float(self.profile.get("climb_rate_ms", 5.0))
            self.setpoint(vel=[0.0, 0.0, -rate])
            if self.pos is not None and -self.pos.z >= ceiling:
                self.finish(f"reached ceiling {ceiling} m")

        elif self.kind == "waypoints":
            if self.step_i >= len(self.waypoints):
                return self.finish("waypoints complete")
            wp = self.waypoints[self.step_i]
            tgt = [float(wp[0]), float(wp[1]), -float(wp[2])]
            self.setpoint(pos=tgt)
            if self.pos is not None:
                d = math.dist([self.pos.x, self.pos.y, self.pos.z], tgt)
                if d < float(self.profile.get("accept_radius_m", 1.5)):
                    self.get_logger().info(f"waypoint {self.step_i} reached")
                    self.step_i += 1
        elif self.kind == "factory_mission":
            self.tick_factory(t)

        else:
            self.get_logger().error(f"unknown profile type {self.kind!r}")
            self.finish("bad profile")

    # ------------------------------------------------------------------
    def mark_leg(self, name, t):
        """Record how long the previous leg took and start timing the next."""
        if self.leg in self.leg_t0:
            dur = t - self.leg_t0[self.leg]
            self.legs_report.append((self.leg, dur))
            self.get_logger().info(f"LEG {self.leg}: {dur:.1f} s")
        self.leg = name
        self.leg_t0[name] = t

    def at(self, tgt, radius):
        if self.pos is None:
            return False
        return math.dist([self.pos.x, self.pos.y, self.pos.z], tgt) < radius

    def tick_factory(self, t):
        """Launch -> obstacle course -> scan the room -> return to base -> land.

        The scan leg is a lawnmower sweep at constant altitude with the airframe
        yawing to sweep the cameras across the room. This is the motion a real
        mapping pass would fly; the point here is what it COSTS in time and
        energy, not to actually build a map (that needs the ZED SDK, which does
        not run in SITL).
        """
        if self.leg not in self.leg_t0:
            self.leg_t0[self.leg] = t
        alt = self.alt
        acc = float(self.profile.get("accept_radius_m", 1.5))

        # ---- leg 1: outbound through the obstacle course ----------------
        if self.leg == "outbound":
            if self.step_i >= len(self.outbound):
                self.step_i = 0
                self.mark_leg("scan", t)
                return
            wp = self.outbound[self.step_i]
            y_plan = float(wp[1])
            y_cam, used = self.camera_y(y_plan)
            tgt = [float(wp[0]), y_cam, -float(wp[2] if len(wp) > 2 else alt)]
            # face the direction of travel so the camera looks where it is going
            yaw = 0.0
            if self.pos is not None:
                dx, dy = tgt[0] - self.pos.x, tgt[1] - self.pos.y
                if abs(dx) + abs(dy) > 0.5:
                    yaw = math.atan2(dy, dx)
            self.setpoint(pos=tgt, yaw=yaw)
            if self.at(tgt, acc):
                self.get_logger().info(
                    f"waypoint {self.step_i + 1}/{len(self.outbound)} reached")
                self.step_i += 1

        # ---- leg 2: scan the room ---------------------------------------
        elif self.leg == "scan":
            pts = self.scan.get("pattern", [])
            spin = float(self.scan.get("yaw_rate_rad_s", 0.35))
            if self.scan_yaw0 is None:
                self.scan_yaw0 = t
            if self.scan_i >= len(pts):
                self.step_i = 0
                self.mark_leg("inbound", t)
                return
            p = pts[self.scan_i]
            tgt = [float(p[0]), float(p[1]),
                   -float(p[2] if len(p) > 2 else self.scan.get("altitude_m", alt))]
            # continuous yaw sweep so the cameras cover the walls
            yaw = ((t - self.scan_yaw0) * spin) % (2 * math.pi)
            self.setpoint(pos=tgt, yaw=yaw - math.pi)
            if self.at(tgt, float(self.scan.get("accept_radius_m", 1.2))):
                self.get_logger().info(
                    f"scan point {self.scan_i + 1}/{len(pts)} covered")
                self.scan_i += 1

        # ---- leg 3: return to base --------------------------------------
        elif self.leg == "inbound":
            if self.step_i >= len(self.inbound):
                self.mark_leg("land", t)
                return
            wp = self.inbound[self.step_i]
            y_plan = float(wp[1])
            y_cam, used = self.camera_y(y_plan)
            tgt = [float(wp[0]), y_cam, -float(wp[2] if len(wp) > 2 else alt)]
            yaw = 0.0
            if self.pos is not None:
                dx, dy = tgt[0] - self.pos.x, tgt[1] - self.pos.y
                if abs(dx) + abs(dy) > 0.5:
                    yaw = math.atan2(dy, dx)
            self.setpoint(pos=tgt, yaw=yaw)
            if self.at(tgt, acc):
                self.get_logger().info(
                    f"return waypoint {self.step_i + 1}/{len(self.inbound)} reached")
                self.step_i += 1

        # ---- leg 4: descend onto the base pad and land -------------------
        elif self.leg == "land":
            self.setpoint(pos=[float(self.base[0]), float(self.base[1]), -1.2])
            if self.at([float(self.base[0]), float(self.base[1]), -1.2], 0.6):
                self.mark_leg("done", t)
                total = t - self.leg_t0.get("outbound", t)
                self.get_logger().info("=== MISSION LEG TIMES ===")
                for n, d in self.legs_report:
                    self.get_logger().info(f"  {n:10s} {d:7.1f} s")
                self.get_logger().info(f"  {'TOTAL':10s} {total:7.1f} s "
                                       f"({total/60:.2f} min)")
                if self.camera_guided:
                    n = self.cam_corrections + self.cam_fallbacks
                    pct = 100.0 * self.cam_corrections / n if n else 0.0
                    self.get_logger().info(
                        f"  camera-guided: {self.cam_corrections} setpoints "
                        f"steered by the camera, {self.cam_fallbacks} fell back "
                        f"to the plan ({pct:.0f}% camera)")
                self.finish("factory mission complete - returned to base")


def main():
    rclpy.init()
    node = MissionRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
