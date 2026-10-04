#!/usr/bin/env python3
"""
hall_mission - fly the factory hall course on what the camera sees, then
return to the pad and land.

    pad (0,0) --outbound--> scan point --360 deg scan--> --inbound--> pad, land

Nothing here knows where the obstacles are. The only obstacle information is
/drone/detections: bounding boxes from the cloud detector, which arrive over
the emulated link about 120 ms after the frame was captured. For each box the
drone looks up its own height, heading and pitch AT CAPTURE TIME (pitch from
PX4's EKF, i.e. the IMU), converts the box to a range and bearing
(avoid.box_to_polar), and marks the obstacle's front face in a world-frame grid.
The grid is what the drone avoids, so an obstacle that has slid out of the
66 degree field of view is still respected. The same grid is the room map
written out at the end.

Edge-side rules, which do NOT depend on the cloud:
  * detections older than detection_max_age_s (link lag or outage) -> hold
  * an obstacle closer than stop_range_m on the current heading     -> stop
  * the camera is not yet pointing within 30 deg of the chosen heading
    -> turn first, move after ("look before you move")
  * no free heading                                                  -> turn to rescan

Coordinates: everything in this node is Gazebo world ENU. PX4 positions are
converted on the way in, setpoints on the way out (drone_eval.frames). Earlier
missions skipped this and flew the course rotated 90 degrees.

Positioning comes from PX4's EKF using the simulator's GPS (an explicit choice
for this version). The camera does obstacle detection, not localisation.
"""

import heapq
import json
import math
import os
import sys
import time
from collections import deque

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleAttitude, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

from drone_eval.avoid import (Camera, blocked_intervals, box_to_polar, pixel_to_floor,
                              is_free, nearest_along)
from drone_eval.frames import enu_to_ned, ned_to_enu, wrap_pi, yaw_ned_to_enu, yaw_enu_to_ned
from drone_eval.mission_runner import CMD_QOS, PX4_QOS, resolve_topic

NAN = float("nan")
CELL = 0.25                     # map grid, metres (0.4 lost ~0.8 m of every gap to rounding)
CONFIRM = 2                     # sightings before a cell counts for local avoidance
PLAN_CONFIRM = 3                # ... and before it blocks the global planner
LOCAL_R = 2.5                   # local avoidance radius when following a global path
UNKNOWN_COST = 1.5              # extra A* cost per cell of floor never seen
DEAD_END_COST = 8.0             # extra A* cost per cell marked as a dead end
MAP_MAX = 7.0                   # map only floor fixes nearer than this (error grows with range)
CAM_FWD = 0.09                  # camera ahead of the body origin (model.sdf)
BODY_H = 0.15                   # body origin above the floor when landed


class HallMission(Node):

    def __init__(self):
        super().__init__("hall_mission")
        self.declare_parameter("config", "")
        self.declare_parameter("out_dir", "/tmp")
        self.declare_parameter("mode", "mission")       # or frame_check
        self.declare_parameter("max_duration_s", 600.0)
        cfg = yaml.safe_load(open(self.get_parameter("config").value))
        self.cfg = cfg
        self.out = self.get_parameter("out_dir").value
        self.mode = self.get_parameter("mode").value
        self.max_dur = float(self.get_parameter("max_duration_s").value)
        M = cfg["mission"]
        self.alt = float(M["cruise_alt_m"])
        self.speed = float(M["speed_ms"])
        self.scan_pt = tuple(M["scan_point"])
        self.pad = tuple(cfg["pad"]["xy"])
        self.goal_tol = float(M["goal_tolerance_m"])
        self.land_tol = float(M["land_tolerance_m"])
        self.clear = float(M["airframe_radius_m"]) + float(M["clearance_margin_m"])
        self.look = float(M["lookahead_m"])
        self.stop_r = float(M["stop_range_m"])
        self.max_age = float(M["detection_max_age_s"])
        self.wall_m = float(M["wall_margin_m"])
        self.scan_rate = math.radians(float(M["scan_yaw_rate_dps"]))
        c = cfg["camera"]
        self.cam = Camera(c["width"], c["height"], math.radians(c["hfov_deg"]),
                          math.radians(c["pitch_down_deg"]))
        self.heights = {k: float(v["height_m"]) for k, v in cfg["classes"].items()}
        (self.hx0, self.hx1), (self.hy0, self.hy1) = cfg["hall"]["x"], cfg["hall"]["y"]

        # PX4 interface (same handshake as mission_runner)
        self.pub_ocm = self.create_publisher(
            OffboardControlMode, resolve_topic(self, "offboard_control_mode", "in"), CMD_QOS)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, resolve_topic(self, "trajectory_setpoint", "in"), CMD_QOS)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, resolve_topic(self, "vehicle_command", "in"), CMD_QOS)
        self.create_subscription(VehicleStatus, resolve_topic(self, "vehicle_status"),
                                 self.cb_status, PX4_QOS)
        self.create_subscription(VehicleLocalPosition,
                                 resolve_topic(self, "vehicle_local_position"),
                                 self.cb_pos, PX4_QOS)
        self.create_subscription(VehicleAttitude, resolve_topic(self, "vehicle_attitude"),
                                 self.cb_att, PX4_QOS)
        self.create_subscription(Detection2DArray, "/drone/detections", self.cb_det, 10)
        self.pub_done = self.create_publisher(Bool, "/drone_eval/mission_done", CMD_QOS)

        # state
        self.pos = None                 # ENU (x, y, z)
        self.yaw = 0.0                  # ENU
        self.pitch_down = 0.0
        self.hist = deque(maxlen=400)   # (wall_t, x, y, z, yaw, pitch_down)
        self.cells = {}                 # (i, j) -> [count, class, last_t]
        self.transient = []             # (t_cap, world_lo, world_hi, range)
        self.path = None                # global plan, list of ENU (x, y)
        self._plan_t, self._plan_goal = 0.0, None
        self.n_plans = self.n_no_path = 0
        self.known_free = set()        # cells seen empty or flown over
        self.dead = set()              # cells around places the drone got stuck
        self.n_escapes = 0
        self.last_escape = 0.0
        self.plan_ms = []
        self.last_det_cap = 0.0
        self.n_det_msgs = 0
        self.n = 0
        self.leg = "boot"
        self.leg_t0 = {}
        self.legs = []
        self.hold_xy = None
        self.heading = None
        self.cmd_yaw = 0.0
        self.scan_turned = 0.0
        self.progress = (float("inf"), time.time())
        self.counts = {"stale_hold": 0, "stop": 0, "turn_first": 0,
                       "boxed_in": 0, "go": 0}
        self.status = "running"
        self.t_start = time.time()
        os.makedirs(self.out, exist_ok=True)
        self.nav_log = open(os.path.join(self.out, "nav.jsonl"), "w")
        self.det_log = open(os.path.join(self.out, "detections_edge.jsonl"), "w")
        self._add_walls()
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"hall mission ({self.mode}): pad {self.pad} -> scan {self.scan_pt} "
            f"-> pad, alt {self.alt} m, {self.speed} m/s, clearance {self.clear:.2f} m")

    # ------------------------------------------------------------ inputs
    def cb_status(self, m):
        self.arming = m.arming_state

    def cb_pos(self, m):
        x, y, z = ned_to_enu(m.x, m.y, m.z)
        self.pos = (x, y, z)
        self.yaw = yaw_ned_to_enu(m.heading)
        self.hist.append((time.time(), x, y, z, self.yaw, self.pitch_down))
        if z > 1.0:
            # floor the drone has flown over is known to be free
            ci, cj = int(math.floor(x / CELL)), int(math.floor(y / CELL))
            for di in range(-3, 4):
                for dj in range(-3, 4):
                    self.known_free.add((ci + di, cj + dj))

    def cb_att(self, m):
        w, x, y, z = m.q
        s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        self.pitch_down = -math.asin(s)        # PX4: +pitch is nose up

    def pose_at(self, t):
        if not self.hist:
            return None
        best = min(self.hist, key=lambda h: abs(h[0] - t))
        return best if abs(best[0] - t) < 0.5 else None

    def cb_det(self, msg):
        t_cap = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_det_cap = max(self.last_det_cap, t_cap)
        self.n_det_msgs += 1
        p = self.pose_at(t_cap)
        if p is None or self.leg == "boot":
            return
        _, px, py, pz, pyaw, ppitch = p
        cam_h = pz + BODY_H
        if cam_h < 0.8:                    # on/near the ground: floor geometry useless
            return
        cx = px + CAM_FWD * math.cos(pyaw)
        cy = py + CAM_FWD * math.sin(pyaw)
        rec = []
        for d in msg.detections:
            if not d.results:
                continue
            cls = d.results[0].hypothesis.class_id
            contacts = [(r.pose.pose.position.x, r.pose.pose.position.y)
                        for r in d.results[1:] if r.hypothesis.class_id == "floor_contact"]
            if contacts:
                # Ground-contact points: each is a floor measurement at its own
                # bearing, so a wall seen at an angle maps as the slanted line
                # it is, not as an arc at its nearest distance.
                for u, v in contacts:
                    pf = pixel_to_floor(self.cam, u, v, cam_h, ppitch)
                    if pf is None:
                        continue
                    rng, bb = pf
                    rec.append([cls, round(rng, 2), round(bb, 4), round(bb, 4), "contact"])
                    if rng > MAP_MAX:
                        continue
                    a = pyaw + bb
                    self.clear_ray(cx, cy, a, rng - 0.5)
                    for dr in (0.0, 0.4, 0.8):          # the face and its depth
                        self.mark_cell(cx + (rng + dr) * math.cos(a),
                                  cy + (rng + dr) * math.sin(a), cls, t_cap)
                continue
            b = d.bbox
            box = (b.center.position.x - b.size_x / 2, b.center.position.y - b.size_y / 2,
                   b.center.position.x + b.size_x / 2, b.center.position.y + b.size_y / 2)
            rng, bl, br, how = box_to_polar(self.cam, box, cam_h,
                                            self.heights.get(cls, 3.0), ppitch)
            rec.append([cls, round(rng, 2), round(bl, 4), round(br, 4), how])
            # No ground contact: the base is at or below the frame edge, so
            # this object is close and its range is NOT measured (see
            # avoid.box_to_polar). Don't map it; hold it as a short-lived
            # obstacle at half the returned range - nearer than the truth in
            # every case checked so far.
            self.transient.append((t_cap, wrap_pi(pyaw + br), wrap_pi(pyaw + bl),
                                   0.5 * rng))
        self.det_log.write(json.dumps({"t_cap": t_cap, "pose": [px, py, pz, pyaw, ppitch],
                                       "dets": rec}) + "\n")

    def mark_cell(self, wx, wy, cls, t):
        key = (int(math.floor(wx / CELL)), int(math.floor(wy / CELL)))
        c = self.cells.get(key)
        if c is None:
            self.cells[key] = [1, cls, t]
        elif c[1] != "wall":
            c[0] = min(c[0] + 1, 50)
            c[2] = t

    def clear_ray(self, x0, y0, a, length):
        """The camera saw floor all the way to the contact point, so nothing
        tall stands on this ray before it. Decrement those cells: a phantom
        cell from a bad range estimate disappears once it is looked through.
        Without this, run 2 boxed itself in with phantoms after the scan."""
        s = 0.6
        while s < length:
            key = (int(math.floor((x0 + s * math.cos(a)) / CELL)),
                   int(math.floor((y0 + s * math.sin(a)) / CELL)))
            self.known_free.add(key)
            c = self.cells.get(key)
            if c is not None and c[1] != "wall":
                c[0] -= 1
                if c[0] <= 0:
                    del self.cells[key]
            s += CELL * 0.5

    def _add_walls(self):
        # The hall boundary is known (it is the building). Pre-mark it so the
        # avoidance keeps wall_margin away from it even where the camera has not
        # looked yet. These are the only prior-known cells.
        for x in frange(self.hx0, self.hx1, CELL):
            for y in (self.hy0, self.hy1):
                self.cells[(int(math.floor(x / CELL)), int(math.floor(y / CELL)))] = [99, "wall", 0.0]
        for y in frange(self.hy0, self.hy1, CELL):
            for x in (self.hx0, self.hx1):
                self.cells[(int(math.floor(x / CELL)), int(math.floor(y / CELL)))] = [99, "wall", 0.0]

    # ------------------------------------------------------------ outputs
    def heartbeat(self):
        m = OffboardControlMode()
        m.position = True
        m.timestamp = int(time.time() * 1e6)
        self.pub_ocm.publish(m)

    def go_to(self, x, y, z, yaw_enu):
        n, e, d = enu_to_ned(x, y, z)
        m = TrajectorySetpoint()
        m.position = [n, e, d]
        m.velocity = [NAN, NAN, NAN]
        m.yaw = float(yaw_enu_to_ned(yaw_enu))
        m.timestamp = int(time.time() * 1e6)
        self.pub_sp.publish(m)

    def cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = command
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = int(time.time() * 1e6)
        self.pub_cmd.publish(m)

    # ------------------------------------------------------------ navigation
    def obstacles_rel(self, ref_bearing):
        """Confirmed cells within the look-ahead, as (range, a, b) bearing
        intervals relative to ref_bearing (world)."""
        x, y, _ = self.pos
        out = []
        r_cells = int(self.look / CELL) + 1
        ci, cj = int(math.floor(x / CELL)), int(math.floor(y / CELL))
        for i in range(ci - r_cells, ci + r_cells + 1):
            for j in range(cj - r_cells, cj + r_cells + 1):
                c = self.cells.get((i, j))
                if c is None or c[0] < CONFIRM:
                    continue
                ox, oy = (i + 0.5) * CELL, (j + 0.5) * CELL
                d = math.hypot(ox - x, oy - y)
                if d > self.look or d < 1e-3:
                    continue
                if c[1] == "wall":
                    d = max(d - (self.wall_m - self.clear), 0.05)
                half = math.atan2(CELL * 0.5, d)
                b = wrap_pi(math.atan2(oy - y, ox - x) - ref_bearing)
                out.append((d, b - half, b + half))
        now = time.time()
        self.transient = [tr for tr in self.transient if now - tr[0] < 0.6]
        mapped = [(d, (lo + hi) / 2.0) for d, lo, hi in out]
        self.n_transient_used = getattr(self, "n_transient_used", 0)
        self.n_transient_explained = getattr(self, "n_transient_explained", 0)
        for _, a_lo, a_hi, rng in self.transient:
            # centre +/- half-width, so an interval that straddles +/-180 deg
            # relative to the goal does not turn into "everything is blocked"
            half = abs(wrap_pi(a_hi - a_lo)) / 2.0
            c = wrap_pi(a_lo + wrap_pi(a_hi - a_lo) / 2.0 - ref_bearing)
            # Already explained by the map? A close flag in a direction where
            # confirmed cells already stand within reach is a known object seen
            # too close to range. Adding it again at half range blocked the
            # whole aisle entrance in run 7 (walls ~3 m away are always "cut").
            if any(abs(wrap_pi(b - c)) <= half and d <= 2.0 * rng + 1.0 for d, b in mapped):
                self.n_transient_explained += 1
                continue
            self.n_transient_used += 1
            out.append((rng, c - half, c + half))
        return out

    def plan(self, goal):
        """Global layer: A* over the camera-built map. Cells the camera has
        never seen count as free (optimistic), so the plan improves as the map
        grows; it is redone every second.

        A purely local planner was tried first (run 4): with rack_1 filling
        the view, "round the north end" and "round the south end" looked
        equally good, the choice flipped with every map update, and the drone
        swung between them at the aisle entrance until the watchdog stopped
        it. North was a dead end; only a planner that looks at the whole map
        can know that."""
        x, y, _ = self.pos
        # Inflate cell CENTRES by the clearance only. Adding half a cell on
        # top (run 6) double-counted the rounding and shrank a 4 m gap to
        # ~2.4 m on the map - narrower than the 2.3 m the drone needs.
        rad = self.clear
        r = int(math.ceil(rad / CELL))
        offs = [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1)
                if math.hypot(di, dj) * CELL <= rad]
        near = [(di, dj) for di in range(-r - 2, r + 3) for dj in range(-r - 2, r + 3)
                if math.hypot(di, dj) <= r + 2]
        blocked, cost = set(), {}
        for (i, j), c in self.cells.items():
            if c[0] < PLAN_CONFIRM or c[1] == "wall":
                continue                  # hall walls: a bounds check below
            for di, dj in offs:
                blocked.add((i + di, j + dj))
            for di, dj in near:                      # prefer the middle of a gap
                k = (i + di, j + dj)
                cost[k] = cost.get(k, 0.0) + 0.15

        def to_c(px, py):
            return (int(math.floor(px / CELL)), int(math.floor(py / CELL)))

        s, gcell = to_c(x, y), to_c(*goal)
        blocked.discard(s)
        blocked.discard(gcell)
        # stay wall_margin inside the hall (the walls are the building)
        i0 = int(math.floor((self.hx0 + self.wall_m) / CELL))
        i1 = int(math.floor((self.hx1 - self.wall_m) / CELL))
        j0 = int(math.floor((self.hy0 + self.wall_m) / CELL))
        j1 = int(math.floor((self.hy1 - self.wall_m) / CELL))

        def h(c):
            return math.hypot(c[0] - gcell[0], c[1] - gcell[1])

        openq = [(h(s), 0.0, s)]
        prev = {s: None}
        gsc = {s: 0.0}
        steps = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
        found, n_exp = False, 0
        while openq and n_exp < 40000:
            _, gc, c = heapq.heappop(openq)
            n_exp += 1
            if c == gcell:
                found = True
                break
            if gc > gsc.get(c, 1e18):
                continue
            for di, dj, w in steps:
                nb = (c[0] + di, c[1] + dj)
                if not (i0 <= nb[0] <= i1 and j0 <= nb[1] <= j1):
                    continue
                pen = 0.0
                if nb in blocked:
                    # Already inside an obstacle's clearance ring: the drone
                    # may still back OUT of it (at a cost); otherwise A* reports
                    # "no path" whenever the drone is near anything.
                    if math.hypot(nb[0] - s[0], nb[1] - s[1]) * CELL > self.clear + CELL:
                        continue
                    pen = 4.0
                # Floor never seen costs extra. Unseen space still counts as
                # passable (that is how the outbound leg explores), but on the
                # way home a known route beats a guessed one: run 10 took an
                # unseen "shortcut" round the outside of the aisle and was
                # trapped in the corner behind the storage block.
                unk = 0.0 if nb in self.known_free else UNKNOWN_COST
                dead = DEAD_END_COST if nb in self.dead else 0.0
                ng = gc + w * (1.0 + pen + unk + dead + cost.get(nb, 0.0))
                if ng < gsc.get(nb, 1e18):
                    gsc[nb] = ng
                    prev[nb] = c
                    heapq.heappush(openq, (ng + h(nb), ng, nb))
        if not found:
            return None
        path, c = [], gcell
        while c is not None:
            path.append(((c[0] + 0.5) * CELL, (c[1] + 0.5) * CELL))
            c = prev[c]
        return path[::-1]

    def navigate(self, goal, t):
        """One step toward goal (ENU xy). Returns distance to goal."""
        x, y, z = self.pos
        gx, gy = goal
        dist = math.hypot(gx - x, gy - y)
        if t - self._plan_t > 1.0 or self._plan_goal != goal:
            self._plan_t, self._plan_goal = t, goal
            t0 = time.time()
            self.path = self.plan(goal)
            self.plan_ms.append((time.time() - t0) * 1000)
            self.n_plans += 1
            if self.path is None:
                self.n_no_path += 1
        # Local layer steers at a point ~2 m along the planned path, or at the
        # goal itself only if the planner found no path.
        sx, sy = gx, gy
        if self.path:
            for px_, py_ in self.path:
                if math.hypot(px_ - x, py_ - y) >= 2.0:
                    sx, sy = px_, py_
                    break
            else:
                sx, sy = self.path[-1]
        g = math.atan2(sy - y, sx - x)
        # With a global path, the local layer only has to keep the short hop
        # to the steering point safe; the path already keeps clear of what is
        # mapped. Checking the full 7 m ray (run 8) left a 4 degree window
        # into a 4 m gap seen from 5 m, which one noisy cell closed.
        local_r = LOCAL_R if self.path else self.look
        iv = blocked_intervals(self.obstacles_rel(g), self.clear, local_r)
        prefer = wrap_pi(self.heading - g) if self.heading is not None else None
        lim = math.radians(110)
        best = None
        for k in range(int(2 * 110 / 2) + 1):
            h = -lim + math.radians(2) * k
            if is_free(h, iv) and (best is None or abs(h) < abs(best)):
                best = h
        if (best is not None and prefer is not None and abs(prefer) <= lim
                and is_free(prefer, iv) and abs(prefer) <= abs(best) + math.radians(10)):
            best = prefer

        age = time.time() - self.last_det_cap
        mode = "go"
        if age > self.max_age:
            mode = "stale_hold"
        elif best is None:
            mode = "boxed_in"
        else:
            hw = wrap_pi(g + best)
            ahead = nearest_along(wrap_pi(self.yaw - g), iv)
            if abs(wrap_pi(hw - self.yaw)) > math.radians(30):
                mode = "turn_first"
            elif ahead < self.stop_r:
                mode = "stop"
        self.counts[mode] += 1

        if mode == "go":
            hw = wrap_pi(g + best)
            self.heading = hw
            self.hold_xy = None
            step = min(1.5, dist)
            self.go_to(x + step * math.cos(hw), y + step * math.sin(hw), self.alt, hw)
            self.cmd_yaw = hw
        else:
            if self.hold_xy is None:
                self.hold_xy = (x, y)
            if mode in ("turn_first", "stop"):
                # turn (in place) to the free heading; move only once aligned
                self.cmd_yaw = wrap_pi(g + best)
                self.heading = self.cmd_yaw
            elif mode == "boxed_in":
                self.cmd_yaw = wrap_pi(self.cmd_yaw + math.radians(4))   # rescan
            self.go_to(self.hold_xy[0], self.hold_xy[1], self.alt, self.cmd_yaw)

        if self.n % 5 == 0:
            self.nav_log.write(json.dumps({
                "t": round(t, 2), "leg": self.leg, "pos": [round(v, 2) for v in self.pos],
                "yaw": round(self.yaw, 3), "goal": goal, "mode": mode,
                "heading": None if best is None else round(wrap_pi(g + best), 3),
                "det_age_ms": round(age * 1000), "cells": len(self.cells)}) + "\n")

        # Dead-end escape: no 0.5 m of progress for 20 s means the planned
        # route leads somewhere the drone cannot get through (run 14: an
        # unseen lane south of machine_2 that turned out blocked). Mark the
        # floor around the drone as a dead end - costly, not forbidden - and
        # replan at once, so the next plan takes another route. Up to three
        # escapes, then the 60 s watchdog below still gives up honestly.
        if dist < self.progress[0] - 0.5:
            self.progress = (dist, t)
        elif t - self.progress[1] > 20.0 and self.n_escapes < 3 \
                and t - self.last_escape > 20.0:
            ci, cj = int(math.floor(x / CELL)), int(math.floor(y / CELL))
            r = int(2.5 / CELL)
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if di * di + dj * dj <= r * r:
                        self.dead.add((ci + di, cj + dj))
            self.n_escapes += 1
            self.last_escape = t
            self.progress = (dist, t)          # give the new route its own time
            self._plan_t = 0.0                 # replan on the next tick
            self.get_logger().warn(f"no progress for 20 s at ({x:.1f},{y:.1f}): "
                                   f"marked a dead end, replanning (escape {self.n_escapes}/3)")
        elif t - self.progress[1] > 60.0:
            self.status = f"STUCK in {self.leg} at {self.pos[0]:.1f},{self.pos[1]:.1f}"
            self.get_logger().error(self.status)
            self.finish()
        return dist

    def mark(self, leg, t):
        if self.leg in self.leg_t0:
            self.legs.append((self.leg, t - self.leg_t0[self.leg]))
            self.get_logger().info(f"LEG {self.leg}: {t - self.leg_t0[self.leg]:.1f} s")
        self.leg = leg
        self.leg_t0[leg] = t
        self.nav_log.write(json.dumps({"t": round(t, 3), "leg_start": leg}) + "\n")
        self.progress = (float("inf"), t)
        self.heading = None
        self.hold_xy = None

    # ------------------------------------------------------------ main loop
    def tick(self):
        t = time.time()
        self.n += 1
        self.heartbeat()
        if self.pos is None:
            return
        if self.n % 10 == 0:
            # PX4's own pose once a second, on the ground and in the air, so the
            # report can compare heading against Gazebo truth (a 26 deg heading
            # error went unnoticed because only position was being checked)
            self.nav_log.write(json.dumps({"t": round(t, 3), "px4": [
                round(v, 3) for v in (*self.pos, self.yaw)]}) + "\n")
        if t - self.t_start > self.max_dur and self.leg not in ("done",):
            self.status = f"max duration {self.max_dur:.0f} s reached"
            return self.finish()
        x, y, z = self.pos

        if self.leg == "boot":
            self.go_to(self.pad[0], self.pad[1], self.alt, 0.0)
            if self.n == 20:
                self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.get_logger().info("arm + offboard requested")
            if self.n > 40 and z > self.alt * 0.9:
                self.get_logger().info(f"airborne at {z:.1f} m - starting {self.mode}")
                self.cmd_yaw = 0.0
                self.mark("frame_check" if self.mode == "frame_check" else "outbound", t)

        elif self.leg == "frame_check":
            # positive control: 3 m East, then 3 m North; the report compares
            # these against the Gazebo truth pose
            pts = [(3.0, 0.0), (3.0, 3.0), (0.0, 0.0)]
            k = int(self.leg_t0.get("_fc_i", 0))
            tx, ty = pts[k]
            self.go_to(tx, ty, self.alt, 0.0)
            if math.hypot(tx - x, ty - y) < 0.25:
                self.get_logger().info(f"frame_check point {k}: PX4 ENU ({x:.2f},{y:.2f})")
                self.nav_log.write(json.dumps({"t": t, "frame_check": k,
                                               "target": [tx, ty], "px4_enu": [x, y]}) + "\n")
                if k + 1 >= len(pts):
                    self.mark("land", t)
                else:
                    self.leg_t0["_fc_i"] = k + 1

        elif self.leg == "outbound":
            if self.navigate(self.scan_pt, t) < self.goal_tol:
                self.scan_turned = 0.0
                self.cmd_yaw = self.yaw
                self.hold_xy = (x, y)
                self.mark("scan", t)

        elif self.leg == "scan":
            dyaw = self.scan_rate * 0.1
            self.cmd_yaw = wrap_pi(self.cmd_yaw + dyaw)
            self.scan_turned += dyaw
            self.go_to(self.scan_pt[0], self.scan_pt[1], self.alt, self.cmd_yaw)
            if self.scan_turned >= 2 * math.pi:
                self.mark("inbound", t)

        elif self.leg == "inbound":
            if self.navigate(self.pad, t) < 1.0:
                self.mark("land", t)

        elif self.leg == "land":
            # Descend in offboard, centred over the pad, and hand over to
            # PX4's land mode only when low and centred. Run 1 commanded
            # NAV_LAND at 1 m and 0.5 m off and touched down 0.56 m out.
            err = math.hypot(x - self.pad[0], y - self.pad[1])
            if not getattr(self, "_land_sent", False):
                z_sp = max(0.35, z - 0.3) if err < 0.3 else max(z, 1.0)
                self.go_to(self.pad[0], self.pad[1], z_sp, self.yaw)
                if err < 0.2 and z < 0.45:
                    self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self._land_sent = True
                    self._land_t = t
            if getattr(self, "_land_sent", False) and (z < 0.25 or t - self._land_t > 15):
                self.status = "complete - returned to pad and landed"
                self.finish()

    def finish(self):
        if getattr(self, "_finishing", False):
            return
        self._finishing = True
        t = time.time()
        if self.leg in self.leg_t0:
            self.legs.append((self.leg, t - self.leg_t0[self.leg]))
        self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        b = Bool()
        b.data = True
        self.pub_done.publish(b)
        total = sum(d for _, d in self.legs)
        self.get_logger().info("=== HALL MISSION ===")
        for n, d in self.legs:
            self.get_logger().info(f"  {n:12s} {d:6.1f} s")
        self.get_logger().info(f"  {'TOTAL':12s} {total:6.1f} s  ({total / 60:.2f} min)")
        nav = sum(self.counts.values()) or 1
        self.get_logger().info(
            "  decisions: " + ", ".join(f"{k} {100 * v / nav:.0f}%" for k, v in self.counts.items()))
        self.get_logger().info(f"  detection messages {self.n_det_msgs}, map cells "
                               f"{sum(1 for c in self.cells.values() if c[1] != 'wall' and c[0] >= CONFIRM)}")
        if self.plan_ms:
            pm = sorted(self.plan_ms)
            self.get_logger().info(f"  global plans {self.n_plans} ({self.n_no_path} without a path), "
                                   f"{pm[len(pm) // 2]:.0f} ms median, {pm[-1]:.0f} ms max")
        self.get_logger().info(f"  status: {self.status}")
        cells = [{"x": round((i + 0.5) * CELL, 2), "y": round((j + 0.5) * CELL, 2),
                  "class": c[1], "hits": c[0]}
                 for (i, j), c in self.cells.items() if c[1] != "wall" and c[0] >= CONFIRM]
        with open(os.path.join(self.out, "camera_map.json"), "w") as f:
            json.dump({"frame": "Gazebo world ENU, metres; origin = launch pad",
                       "cell_m": CELL, "source": "cloud detections + drone pose",
                       "cells": cells}, f)
        with open(os.path.join(self.out, "mission_summary.json"), "w") as f:
            json.dump({"status": self.status, "legs_s": dict(self.legs),
                       "total_s": total, "decisions": self.counts,
                       "detection_msgs": self.n_det_msgs,
                       "map_cells": len(cells), "plans": self.n_plans,
                       "plans_without_path": self.n_no_path,
                       "dead_end_escapes": self.n_escapes,
                       "plan_ms_median": sorted(self.plan_ms)[len(self.plan_ms) // 2] if self.plan_ms else None,
                       "plan_ms_max": max(self.plan_ms) if self.plan_ms else None}, f, indent=2)
        self.nav_log.close()
        self.det_log.close()
        self.create_timer(2.0, self._exit)

    def _exit(self):
        sys.stdout.flush()
        os._exit(0)


def frange(a, b, s):
    v = a
    while v <= b:
        yield v
        v += s


def main():
    rclpy.init()
    n = HallMission()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
