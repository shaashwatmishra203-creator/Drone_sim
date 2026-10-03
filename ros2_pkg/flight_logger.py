#!/usr/bin/env python3
"""
flight_logger — record flight state and derive real power / endurance.

Why this node computes power itself instead of reading BatteryStatus:
PX4's SITL battery simulator (src/modules/simulation/battery_simulator) drains
the pack on WALL CLOCK via SIM_BAT_DRAIN, completely independently of what the
motors are actually doing. Its BatteryStatus is therefore useless for
endurance. Instead we take the actual motor commands PX4 produces, convert them
to rotor speed, run them through the propeller coefficients, and integrate the
same physical battery model the analytical tool uses. One source of truth.

QoS: PX4's uXRCE-DDS bridge publishes /fmu/out/* as BEST_EFFORT / KEEP_LAST 5.
A default rclpy subscriber is RELIABLE and will receive NOTHING, silently. If
the CSV comes out empty, check QoS before suspecting the model.
"""

import csv
import json
import math
import os
import sys

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from std_msgs.msg import Bool

from px4_msgs.msg import (ActuatorMotors, BatteryStatus, VehicleAttitude,
                          VehicleLocalPosition, VehicleStatus)

# The exact profile the PX4 bridge offers. Do not "simplify" this to the
# rclpy default — that is the single most common reason these nodes log nothing.
PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

IN2M = 0.0254


def resolve_topic(node, base, direction="out", timeout_s=20.0):
    """Find the real topic name for a PX4 message.

    PX4 1.18 appends a message-version suffix to DDS topic names — the same
    logical topic appears as vehicle_local_position_v1, vehicle_status_v4,
    battery_status_v1, while others carry no suffix at all. Hardcoding either
    form breaks on a PX4 upgrade, so match against the live graph instead.
    """
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
            # highest version wins
            return sorted(cands, key=lambda n: int(n.rsplit("_v", 1)[1]))[-1]
        time.sleep(0.5)
    node.get_logger().warn(
        f"topic {want}[_vN] not found after {timeout_s:.0f}s — using {want}")
    return want


class Battery:
    """Same OCV + internal-resistance model as tools/sizing.py."""

    def __init__(self, cfg):
        self.cells = int(cfg["cells_series"])
        self.cap_ah = float(cfg["capacity_ah"])
        self.r = float(cfg["internal_resistance_ohm"])
        self.dod = float(cfg["dod_limit"])
        self.curve = sorted([(float(s), float(v)) for s, v in cfg["ocv_curve"]])
        self.soc = 1.0

    def ocv(self, soc):
        soc = max(0.0, min(1.0, soc))
        for i in range(len(self.curve) - 1):
            s0, v0 = self.curve[i]
            s1, v1 = self.curve[i + 1]
            if s0 <= soc <= s1:
                f = 0.0 if s1 == s0 else (soc - s0) / (s1 - s0)
                return (v0 + f * (v1 - v0)) * self.cells
        return self.curve[-1][1] * self.cells

    def step(self, power_w, dt):
        vo = self.ocv(self.soc)
        disc = vo * vo - 4.0 * self.r * power_w
        if disc <= 0:
            return None, None, True
        i = (vo - math.sqrt(disc)) / (2.0 * self.r)
        self.soc -= (i * dt / 3600.0) / self.cap_ah
        empty = self.soc <= (1.0 - self.dod)
        return vo - i * self.r, i, empty


class FlightLogger(Node):

    def __init__(self):
        super().__init__("flight_logger")

        self.declare_parameter("config", "")
        self.declare_parameter("prop", "9x4.5")
        self.declare_parameter("payload", "pi5")
        self.declare_parameter("out_csv", "flight_log.csv")
        self.declare_parameter("out_json", "")
        self.declare_parameter("scenario", "unnamed")
        self.declare_parameter("ec_min", 150.0)

        cfg_path = self.get_parameter("config").value
        if not cfg_path or not os.path.exists(cfg_path):
            self.get_logger().fatal(f"config not found: {cfg_path!r}")
            raise SystemExit(2)
        with open(cfg_path) as f:
            self.cfg = yaml.safe_load(f)

        prop_id = self.get_parameter("prop").value
        self.prop = next((p for p in self.cfg["prop_options"]
                          if p["id"] == prop_id), None)
        if self.prop is None:
            self.get_logger().fatal(f"unknown prop {prop_id!r}")
            raise SystemExit(2)

        self.scenario = self.get_parameter("scenario").value
        self.payload = self.get_parameter("payload").value
        self.ec_min = float(self.get_parameter("ec_min").value)

        env = self.cfg["environment"]
        p = 101325.0 * (1.0 - 2.25577e-5 * env["altitude_m"]) ** 5.25588
        self.rho = p / (287.058 * (env["temperature_c"] + 273.15))

        self.d_m = self.prop["diameter_in"] * IN2M
        self.ct = self.prop["ct"]
        self.cp = self.prop["cp"]
        self.eta = self.cfg["assumptions"]["motor_esc_efficiency"]

        # Avionics load, per component, so the breakdown is visible rather than
        # a single lumped number. Every powered part on the aircraft appears here.
        self.power_breakdown = {}
        for c in self.cfg["components"]:
            w = float(c.get("power_w", 0.0)) * int(c.get("qty", 1))
            if w:
                self.power_breakdown[c["name"]] = w
        pv = self.cfg["payload_variants"][self.payload]
        self.power_breakdown[pv["name"]] = float(pv["power_w"])
        self.avionics_w = sum(self.power_breakdown.values())

        # All-up weight, with the prop mass taken from the selected option.
        self.auw_kg = float(pv["mass_kg"])
        for c in self.cfg["components"]:
            m = float(self.prop["mass_kg"]) if c["id"] == "prop" else float(c["mass_kg"])
            self.auw_kg += m * int(c.get("qty", 1))

        # Rotor velocity ceiling must match maxRotVelocity in the SDF and
        # SIM_GZ_EC_MAX in the airframe, or derived thrust is wrong.
        self.ec_max = self._max_omega()

        self.batt = Battery(self.cfg["battery"])

        # state
        # Time base. There is NO /clock here: without a ros_gz bridge nothing
        # publishes one, so use_sim_time would freeze the node clock at zero.
        # PX4's own message timestamps ARE the simulation clock (microseconds),
        # so we drive everything off those instead.
        self.px4_us = None

        self.pos = self.vel = None
        self.att = None
        self.motors = None
        self.nav_state = None
        self.arming = None
        self.batt_px4_v = float("nan")
        self.t0 = None
        self.last_t = None
        self.energy_wh = 0.0
        self.empty_reported = False

        # Thrust / power record, written as JSON at the end of the run.
        self.json_path = self.get_parameter("out_json").value
        self.samples = []
        self.t_peak_n = 0.0
        self.p_peak_w = 0.0

        out = self.get_parameter("out_csv").value
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        self.fh = open(out, "w", newline="")
        self.csv = csv.writer(self.fh)
        self.csv.writerow([
            "t_s", "scenario", "prop", "payload",
            "x", "y", "z", "vx", "vy", "vz", "speed_ms",
            "roll_deg", "pitch_deg", "yaw_deg",
            "m0", "m1", "m2", "m3",
            "rpm_mean", "thrust_total_n", "thrust_per_motor_g",
            "shaft_w", "prop_elec_w", "avionics_w", "power_total_w",
            "current_a", "v_terminal", "soc", "energy_wh",
            "endurance_remaining_min", "px4_batt_v", "nav_state", "arming",
        ])

        subs = [
            (VehicleLocalPosition, "vehicle_local_position", self.cb_pos),
            (VehicleAttitude, "vehicle_attitude", self.cb_att),
            (VehicleStatus, "vehicle_status", self.cb_status),
            (ActuatorMotors, "actuator_motors", self.cb_motors),
            (BatteryStatus, "battery_status", self.cb_batt),
        ]
        for msg_t, base, cb in subs:
            topic = resolve_topic(self, base)
            self.get_logger().info(f"  subscribing {topic}")
            self.create_subscription(msg_t, topic, cb, PX4_QOS)

        # Tells mission_runner to land; latched so a late subscriber still sees it.
        latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub_empty = self.create_publisher(
            Bool, "/drone_eval/battery_empty", latched)

        # Stop logging the moment the mission ends. Headless SITL runs well
        # above real time, so the seconds between the mission finishing and the
        # run script tearing things down can be MINUTES of simulated hovering -
        # which lands in the log as real energy and ruins the accounting. One
        # run recorded 1075 s airborne against 90 s of actual mission.
        self.create_subscription(Bool, "/drone_eval/mission_done",
                                 self.cb_done, latched)

        self.create_timer(0.05, self.tick)      # 20 Hz

        self.get_logger().info(
            f"logging '{self.scenario}' prop={prop_id} payload={self.payload} "
            f"rho={self.rho:.4f} ec_max={self.ec_max:.1f} rad/s "
            f"avionics={self.avionics_w:.1f} W -> {out}")

    # -- derivation of the rotor velocity ceiling --------------------------
    def _max_omega(self):
        """Recompute the same max operating point gen_model.py used, so the
        logger's scaling cannot drift from the SDF."""
        m = self.cfg["motor"]
        kv, rm, i0 = m["kv"], m["rm_ohm"], m["i0_a"]
        i_max = min(float(m["i_max_cont_a"]),
                    float(self.cfg["esc"]["i_max_cont_a"]))
        ke = 60.0 / (2.0 * math.pi * kv)
        b = self.cfg["battery"]
        v_bat = b["nominal_v_per_cell"] * b["cells_series"]
        # approximate: use the nominal-voltage OCV like sizing.py does
        v_bat = self.batt.ocv(0.5) if hasattr(self, "batt") else v_bat

        q = ke * max(i_max - i0, 0.0)
        n = math.sqrt(2.0 * math.pi * q / (self.cp * self.rho * self.d_m ** 5))
        if ke * 2.0 * math.pi * n + i_max * rm <= v_bat:
            return 2.0 * math.pi * n
        lo, hi = 0.0, kv * v_bat / 60.0
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            om = 2.0 * math.pi * mid
            qm = ke * max((v_bat - ke * om) / rm - i0, 0.0)
            qp = self.cp * self.rho * mid ** 2 * self.d_m ** 5 / (2.0 * math.pi)
            lo, hi = (mid, hi) if qm > qp else (lo, mid)
        return 2.0 * math.pi * 0.5 * (lo + hi)

    # -- callbacks ---------------------------------------------------------
    def cb_pos(self, m):
        self.pos = (m.x, m.y, m.z)
        self.vel = (m.vx, m.vy, m.vz)

    def cb_att(self, m):
        self.att = m.q

    def cb_status(self, m):
        self.nav_state = m.nav_state
        self.arming = m.arming_state

    def cb_motors(self, m):
        self.motors = list(m.control[:4])
        self.px4_us = m.timestamp          # simulation clock, microseconds

    def cb_batt(self, m):
        self.batt_px4_v = m.voltage_v

    def cb_done(self, m):
        if not m.data or getattr(self, "_closing", False):
            return
        self._closing = True
        self.get_logger().info(
            f"mission complete at t={self.px4_us*1e-6 - (self.t0 or 0):.1f}s — "
            f"closing log ({self.energy_wh:.2f} Wh)")
        try:
            self.fh.flush()
            self.fh.close()
        except Exception:
            pass
        try:
            self.write_json()
        except Exception as e:
            self.get_logger().error(f"json write failed: {e}")
        # Give the log a moment to settle, then stop hard: spin() does not
        # reliably return from inside a callback.
        self.create_timer(1.0, lambda: os._exit(0))

    # -- main loop ---------------------------------------------------------
    def tick(self):
        if getattr(self, "_closing", False):
            return
        if self.pos is None or self.motors is None or self.px4_us is None:
            return
        now = self.px4_us * 1e-6                          # PX4 simulation clock
        if self.t0 is None:
            self.t0 = now
            self.last_t = now
            return
        dt = now - self.last_t
        if dt <= 0.0:
            return
        self.last_t = now
        t = now - self.t0

        # motor command -> rotor speed -> thrust & shaft power
        omegas, thrusts, shaft = [], [], 0.0
        for u in self.motors:
            u = 0.0 if (u is None or math.isnan(u)) else max(0.0, min(1.0, u))
            om = self.ec_min + u * (self.ec_max - self.ec_min) if u > 0 else 0.0
            n = om / (2.0 * math.pi)
            thrusts.append(self.ct * self.rho * n ** 2 * self.d_m ** 4)
            shaft += self.cp * self.rho * n ** 3 * self.d_m ** 5
            omegas.append(om)

        prop_elec = shaft / self.eta
        total_w = prop_elec + self.avionics_w

        v_term, cur, empty = self.batt.step(total_w, dt)
        if v_term is None:
            self.get_logger().error("power demand exceeds pack capability")
            return
        self.energy_wh += total_w * dt / 3600.0

        # remaining endurance at the current draw
        usable_ah = self.batt.cap_ah * (self.batt.soc - (1.0 - self.batt.dod))
        rem_min = max(0.0, usable_ah / max(cur, 1e-6) * 60.0)

        if empty and not self.empty_reported:
            self.empty_reported = True
            b = Bool()
            b.data = True
            self.pub_empty.publish(b)
            self.get_logger().warn(
                f"DoD gate reached at t={t:.1f}s ({t/60.0:.2f} min), "
                f"{self.energy_wh:.1f} Wh used")

        roll = pitch = yaw = float("nan")
        if self.att is not None:
            w, x, y, z = self.att
            roll = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
            yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

        vx, vy, vz = self.vel
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        g = self.cfg["environment"]["gravity"]
        tt = sum(thrusts)

        self.csv.writerow([
            f"{t:.3f}", self.scenario, self.prop["id"], self.payload,
            f"{self.pos[0]:.3f}", f"{self.pos[1]:.3f}", f"{self.pos[2]:.3f}",
            f"{vx:.3f}", f"{vy:.3f}", f"{vz:.3f}", f"{speed:.3f}",
            f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}",
            *[f"{u:.4f}" for u in self.motors],
            f"{sum(omegas)/4*60/(2*math.pi):.0f}",
            f"{tt:.3f}", f"{tt/4/g*1000:.1f}",
            f"{shaft:.1f}", f"{prop_elec:.1f}", f"{self.avionics_w:.1f}",
            f"{total_w:.1f}", f"{cur:.2f}", f"{v_term:.2f}",
            f"{self.batt.soc:.5f}", f"{self.energy_wh:.3f}",
            f"{rem_min:.2f}", f"{self.batt_px4_v:.2f}",
            self.nav_state, self.arming,
        ])
        self.fh.flush()

        # --- thrust / power record --------------------------------------
        self.t_peak_n = max(self.t_peak_n, tt)
        self.p_peak_w = max(self.p_peak_w, total_w)
        self.samples.append({
            "t_s": round(float(t), 3),
            "altitude_m": round(float(-self.pos[2]), 3),
            "speed_ms": round(float(speed), 3),
            "rpm_mean": round(float(sum(omegas) / 4 * 60 / (2 * math.pi)), 1),
            # px4_msgs hands these over as numpy float32, which json cannot
            # serialise — cast explicitly rather than relying on round().
            "motor_cmds": [round(float(u), 4) for u in self.motors],
            "thrust_per_motor_n": [round(float(x), 4) for x in thrusts],
            "thrust_total_n": round(float(tt), 4),
            "thrust_per_motor_g": round(float(tt / 4 / g * 1000.0), 1),
            "thrust_to_weight": round(float(tt / (self.auw_kg * g)), 4),
            "shaft_power_w": round(float(shaft), 2),
            "propulsion_power_w": round(float(prop_elec), 2),
            "avionics_power_w": round(float(self.avionics_w), 2),
            "total_power_w": round(float(total_w), 2),
            "current_a": round(float(cur), 3),
            "v_terminal": round(float(v_term), 3),
            "soc": round(float(self.batt.soc), 5),
            "energy_wh": round(float(self.energy_wh), 4),
        })

    def write_json(self):
        """Thrust and power record for the run. Written once, at the end."""
        if not self.json_path or not self.samples:
            return
        g = self.cfg["environment"]["gravity"]
        air = [s for s in self.samples if s["altitude_m"] > 0.8]
        ref = air or self.samples

        def mean(k):
            v = [s[k] for s in ref]
            return sum(v) / len(v) if v else 0.0

        doc = {
            "scenario": self.scenario,
            "prop": self.prop["id"],
            "payload": self.payload,
            "aircraft": {
                "auw_kg": round(self.auw_kg, 4),
                "weight_n": round(self.auw_kg * g, 3),
                "air_density_kgm3": round(self.rho, 4),
                "prop_diameter_m": round(self.d_m, 4),
                "ct": self.ct, "cp": self.cp,
                "motor_esc_efficiency": self.eta,
                "ec_min_rad_s": self.ec_min,
                "ec_max_rad_s": round(self.ec_max, 2),
            },
            "power_breakdown_w": self.power_breakdown,
            "avionics_total_w": round(self.avionics_w, 2),
            "thrust": {
                "hover_required_total_n": round(self.auw_kg * g, 3),
                "hover_required_per_motor_g": round(self.auw_kg * 1000.0 / 4.0, 1),
                "mean_total_n": round(mean("thrust_total_n"), 3),
                "mean_per_motor_g": round(mean("thrust_per_motor_g"), 1),
                "peak_total_n": round(self.t_peak_n, 3),
                "peak_per_motor_g": round(self.t_peak_n / 4 / g * 1000.0, 1),
                "mean_thrust_to_weight": round(mean("thrust_to_weight"), 3),
            },
            "power": {
                "mean_total_w": round(mean("total_power_w"), 1),
                "mean_propulsion_w": round(mean("propulsion_power_w"), 1),
                "mean_avionics_w": round(self.avionics_w, 1),
                "peak_total_w": round(self.p_peak_w, 1),
                "mean_current_a": round(mean("current_a"), 2),
                "energy_used_wh": round(self.energy_wh, 3),
            },
            "sample_count": len(self.samples),
            "sample_rate_hz": 20,
            "samples": self.samples,
        }
        os.makedirs(os.path.dirname(os.path.abspath(self.json_path)), exist_ok=True)
        with open(self.json_path, "w") as f:
            # px4_msgs fields arrive as numpy float32 and propagate into every
            # derived quantity; json cannot serialise them. Catch the whole
            # class rather than chasing individual fields.
            json.dump(doc, f, indent=2, default=lambda o: float(o))
        self.get_logger().info(
            f"wrote {self.json_path}  ({len(self.samples)} thrust samples, "
            f"mean {doc['thrust']['mean_per_motor_g']:.0f} g/motor, "
            f"peak {doc['thrust']['peak_per_motor_g']:.0f} g/motor)")

    def destroy_node(self):
        try:
            self.fh.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = FlightLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        # Write the JSON even on an abrupt shutdown. The run script kills the
        # nodes when the mission ends, and an interrupted run is exactly when
        # the record is most worth keeping.
        try:
            if not getattr(node, "_closing", False):
                node.write_json()
        except Exception as e:
            print(f"json write on shutdown failed: {e}", file=sys.stderr)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
