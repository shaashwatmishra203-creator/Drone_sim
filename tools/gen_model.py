#!/usr/bin/env python3
"""
gen_model.py — generate the Gazebo model and PX4 airframe from airframe.yaml.

Everything the simulator uses is DERIVED from the same config the analytical
model reads, so the two can never silently disagree.

Method: parse PX4's own `x500_base/model.sdf` and patch it, rather than
hand-writing 600 lines of SDF. This reuses PX4's tuned sensor noise models and,
critically, preserves their rotor numbering and spin directions — which are easy
to get wrong and produce an aircraft that flips on takeoff.

Derivations (gz MulticopterMotorModel uses thrust = motorConstant * omega^2,
with omega in rad/s, and moment = momentConstant * thrust):

    T = Ct*rho*n^2*D^4,  n = omega/2pi   =>  motorConstant = Ct*rho*D^4/(4*pi^2)
    Q = Cp*rho*n^2*D^5/(2pi)             =>  momentConstant = Cp*D/(2*pi*Ct)

Sanity check against PX4's own x500 numbers is printed at generation time.

Usage:
  python3 tools/gen_model.py --config config/airframe.yaml --dry-run
  python3 tools/gen_model.py --config config/airframe.yaml --install
"""

from __future__ import annotations

import argparse
import copy
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sizing import analyse, air_density, IN2M          # noqa: E402

PX4 = os.path.expanduser("~/PX4-Autopilot")
GZ_MODELS = os.path.join(PX4, "Tools/simulation/gz/models")
AIRFRAMES = os.path.join(PX4, "ROMFS/px4fmu_common/init.d-posix/airframes")
BASE_MODEL = os.path.join(GZ_MODELS, "x500_base/model.sdf")

# PX4's x500 reference values, used only as a plausibility check on our maths.
X500_MOTOR_CONSTANT = 8.54858e-06
X500_MOMENT_CONSTANT = 0.016
X500_ARM = 0.174 * math.sqrt(2.0)


def motor_constant(ct, rho, dia_m):
    return ct * rho * dia_m ** 4 / (4.0 * math.pi ** 2)


def moment_constant(cp, ct, dia_m):
    return cp * dia_m / (2.0 * math.pi * ct)


# ---------------------------------------------------------------------------
def patch_sdf(res, cfg, prop, model_name):
    """Load x500_base and patch geometry, mass and motor constants."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(BASE_MODEL, parser=parser)
    root = tree.getroot()
    model = root.find("model")
    model.set("name", model_name)

    arm = cfg["geometry"]["arm_length_m"]
    scale = arm / X500_ARM
    d_m = prop["diameter_in"] * IN2M
    rho = res["rho"]

    # --- base link: our mass, our inertia, and wind coupling ---------------
    base = model.find("./link[@name='base_link']")
    inert = base.find("inertial")
    inert.find("mass").text = f"{res['auw_kg']:.6f}"
    I = inert.find("inertia")
    I.find("ixx").text = f"{res['inertia']['ixx']:.8f}"
    I.find("iyy").text = f"{res['inertia']['iyy']:.8f}"
    I.find("izz").text = f"{res['inertia']['izz']:.8f}"

    # Required for the world's wind to act on the airframe at all.
    if base.find("enable_wind") is None:
        ew = ET.SubElement(base, "enable_wind")
        ew.text = "true"

    # --- rotors: reposition to our arm length, re-mass to our prop ---------
    prop_m = prop["mass_kg"]
    # A propeller is a thin lamina: a rod of length D (span) with a small chord.
    # For a planar body the perpendicular-axis theorem gives Izz = Ixx + Iyy
    # exactly, and SDF rejects any tensor violating Ixx + Iyy >= Izz. Deriving
    # all three from the lamina relation satisfies it by construction.
    chord = 0.10 * d_m
    iyy = prop_m * d_m ** 2 / 12.0        # about the transverse in-plane axis
    ixx = prop_m * chord ** 2 / 12.0      # about the span axis
    izz = ixx + iyy                        # spin axis
    for i in range(4):
        link = model.find(f"./link[@name='rotor_{i}']")
        pose = link.find("pose")
        v = [float(x) for x in pose.text.split()]
        v[0] *= scale
        v[1] *= scale
        pose.text = " ".join(f"{x:.6f}" for x in v)

        ri = link.find("inertial")
        ri.find("mass").text = f"{prop_m:.6f}"
        rI = ri.find("inertia")
        rI.find("ixx").text = f"{ixx:.10e}"
        rI.find("iyy").text = f"{iyy:.10e}"
        rI.find("izz").text = f"{izz:.10e}"

    # --- motor plugins -----------------------------------------------------
    kf = motor_constant(prop["ct"], rho, d_m)
    km = moment_constant(prop["cp"], prop["ct"], d_m)
    max_omega = res["max"]["rpm"] / 60.0 * 2.0 * math.pi

    # rotor 0,1 = ccw ; 2,3 = cw  (PX4 x500 convention, preserved)
    for i in range(4):
        p = ET.SubElement(model, "plugin")
        p.set("filename", "gz-sim-multicopter-motor-model-system")
        p.set("name", "gz::sim::systems::MulticopterMotorModel")
        vals = [
            ("jointName", f"rotor_{i}_joint"),
            ("linkName", f"rotor_{i}"),
            ("turningDirection", "ccw" if i < 2 else "cw"),
            ("timeConstantUp", "0.0125"),
            ("timeConstantDown", "0.025"),
            ("maxRotVelocity", f"{max_omega:.2f}"),
            ("motorConstant", f"{kf:.6e}"),
            ("momentConstant", f"{km:.6f}"),
            ("commandSubTopic", "command/motor_speed"),
            ("motorNumber", str(i)),
            ("rotorDragCoefficient", "8.06428e-05"),
            ("rollingMomentCoefficient", "1e-06"),
            ("rotorVelocitySlowdownSim", "10"),
            ("motorType", "velocity"),
        ]
        for k, v in vals:
            ET.SubElement(p, k).text = v

    return tree, dict(kf=kf, km=km, max_omega=max_omega, scale=scale, arm=arm)


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>drone-team</name></author>
  <description>
    Generated by drone_sim/tools/gen_model.py from config/airframe.yaml.
    DO NOT EDIT BY HAND — regenerate instead.
    AUW {auw:.3f} kg, prop {prop}, compute {payload}.
  </description>
</model>
"""


AIRFRAME_TMPL = """#!/bin/sh
#
# @name {name}
#
# @type Quadrotor
#
# GENERATED by drone_sim/tools/gen_model.py from config/airframe.yaml.
# DO NOT EDIT BY HAND — regenerate instead.
#
#   AUW           {auw:.3f} kg
#   prop          {prop}
#   compute       {payload}
#   hover rpm     {hover_rpm:.0f}  ({rpm_frac:.1%} of max)
#   T/W           {twr:.2f}
#
# NOTE: PX4's SITL battery drains on wall-clock (SIM_BAT_DRAIN), NOT on actual
# power draw. Do not read endurance out of BatteryStatus. Endurance comes from
# drone_eval/flight_logger, which integrates a real battery model against the
# power implied by the actual motor commands.
#

. ${{R}}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${{PX4_SIMULATOR:=gz}}
PX4_GZ_WORLD=${{PX4_GZ_WORLD:={world}}}
PX4_SIM_MODEL=${{PX4_SIM_MODEL:={model}}}

param set-default SIM_GZ_EN 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

param set-default CA_ROTOR0_PX {px:.4f}
param set-default CA_ROTOR0_PY {py:.4f}
param set-default CA_ROTOR0_KM {km:.4f}

param set-default CA_ROTOR1_PX -{px:.4f}
param set-default CA_ROTOR1_PY -{py:.4f}
param set-default CA_ROTOR1_KM {km:.4f}

param set-default CA_ROTOR2_PX {px:.4f}
param set-default CA_ROTOR2_PY -{py:.4f}
param set-default CA_ROTOR2_KM -{km:.4f}

param set-default CA_ROTOR3_PX -{px:.4f}
param set-default CA_ROTOR3_PY {py:.4f}
param set-default CA_ROTOR3_KM -{km:.4f}

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

# Must equal maxRotVelocity in the SDF, or commanded thrust will be wrong.
param set-default SIM_GZ_EC_MAX1 {max_omega:.0f}
param set-default SIM_GZ_EC_MAX2 {max_omega:.0f}
param set-default SIM_GZ_EC_MAX3 {max_omega:.0f}
param set-default SIM_GZ_EC_MAX4 {max_omega:.0f}

# From the sizing model, not guessed. Accounts for the EC_MIN offset in
# omega = EC_MIN + u*(EC_MAX - EC_MIN).
param set-default MPC_THR_HOVER {thr_hover:.3f}

# Battery, for PX4's own failsafe logic and voltage display.
param set-default BAT1_N_CELLS {cells}
param set-default BAT1_CAPACITY {cap_mah:.0f}
param set-default BAT1_V_EMPTY {v_empty:.2f}
param set-default BAT1_V_CHARGED {v_full:.2f}
param set-default BAT1_R_INTERNAL {r_int:.4f}
param set-default SIM_BAT_MIN_PCT {min_pct:.0f}
param set-default SIM_BAT_DRAIN {drain_s:.0f}

# These runs are unattended and headless: no GCS, no RC. Without these, the
# preflight datalink/RC checks fail and the vehicle never reaches
# "Ready for takeoff". Do NOT copy these onto real hardware — losing the
# datalink failsafe on a real aircraft is exactly the wrong trade.
param set-default NAV_DLL_ACT 0
param set-default NAV_RCL_ACT 0
param set-default COM_RCL_EXCEPT 4
param set-default COM_RC_IN_MODE 4
"""


def generate(cfg, payload, prop, world, outdir, install, verbose=True):
    res = analyse(cfg, payload, prop)
    name = f"{cfg['meta']['name']}_{prop['id'].replace('.', '')}_{payload}"
    tree, d = patch_sdf(res, cfg, prop, name)

    mdir = os.path.join(outdir, "models", name)
    os.makedirs(mdir, exist_ok=True)
    tree.write(os.path.join(mdir, "model.sdf"), encoding="utf-8",
               xml_declaration=True)
    with open(os.path.join(mdir, "model.config"), "w") as f:
        f.write(MODEL_CONFIG.format(name=name, auw=res["auw_kg"],
                                    prop=prop["id"], payload=payload))

    b = cfg["battery"]
    batt_cfg = res
    # A crude wall-clock drain so PX4's own failsafes fire at a sensible time;
    # the real endurance number does not come from here.
    drain_s = res["hover"]["minutes"] * 60.0 / max(b["dod_limit"], 0.01)

    # PX4 reserves [22000, 22999] for custom models; stay inside it so a future
    # PX4 update cannot collide with our airframe IDs.
    # PX4 maps a normalized motor command u to rotor speed as
    #     omega = EC_MIN + u * (EC_MAX - EC_MIN)
    # so the hover command is NOT simply hover_rpm / max_rpm. Ignoring the
    # EC_MIN offset biases MPC_THR_HOVER high.
    ec_min = 150.0
    omega_hover = res["hover"]["rpm"] / 60.0 * 2.0 * math.pi
    thr_hover = (omega_hover - ec_min) / max(d["max_omega"] - ec_min, 1e-9)
    thr_hover = max(0.05, min(0.95, thr_hover))

    af_id = 22000 + generate.counter
    generate.counter += 1
    af_name = f"{af_id}_gz_{name}"
    afdir = os.path.join(outdir, "airframes")
    os.makedirs(afdir, exist_ok=True)
    body = AIRFRAME_TMPL.format(
        name=f"Gazebo {name}", auw=res["auw_kg"], prop=prop["id"],
        payload=payload, hover_rpm=res["hover"]["rpm"],
        rpm_frac=res["hover"]["rpm_fraction"], twr=res["twr"],
        world=world, model=name,
        px=cfg["geometry"]["arm_length_m"] / math.sqrt(2),
        py=cfg["geometry"]["arm_length_m"] / math.sqrt(2),
        km=d["km"], max_omega=d["max_omega"],
        thr_hover=thr_hover,
        cells=b["cells_series"], cap_mah=b["capacity_ah"] * 1000.0,
        v_empty=3.5, v_full=4.2, r_int=b["internal_resistance_ohm"],
        min_pct=(1.0 - b["dod_limit"]) * 100.0, drain_s=drain_s)
    afpath = os.path.join(afdir, af_name)
    with open(afpath, "w") as f:
        f.write(body)
    os.chmod(afpath, 0o755)

    if verbose:
        print(f"\n  {name}")
        print(f"    AUW            {res['auw_kg']:.3f} kg")
        print(f"    inertia        ixx={res['inertia']['ixx']:.4f} "
              f"iyy={res['inertia']['iyy']:.4f} izz={res['inertia']['izz']:.4f}")
        print(f"    motorConstant  {d['kf']:.6e}   (x500 ref "
              f"{X500_MOTOR_CONSTANT:.3e})")
        print(f"    momentConstant {d['km']:.6f}       (x500 ref "
              f"{X500_MOMENT_CONSTANT:.3f})")
        print(f"    maxRotVelocity {d['max_omega']:.1f} rad/s "
              f"({res['max']['rpm']:.0f} rpm)")
        print(f"    MPC_THR_HOVER  {res['hover']['rpm_fraction']:.3f}")
        print(f"    airframe       {af_name}")

    if install:
        dst = os.path.join(GZ_MODELS, name)
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(mdir, dst)
        shutil.copy2(afpath, os.path.join(AIRFRAMES, af_name))
        os.chmod(os.path.join(AIRFRAMES, af_name), 0o755)

    return af_name, name, res


generate.counter = 0


def register_cmake(af_names):
    """Add our airframes to PX4's airframe CMakeLists, idempotently."""
    path = os.path.join(AIRFRAMES, "CMakeLists.txt")
    with open(path) as f:
        text = f.read()
    added = []
    for n in af_names:
        if n in text:
            continue
        # insert before the closing paren of px4_add_romfs_files(...)
        idx = text.rfind(")")
        text = text[:idx] + f"\t{n}\n" + text[idx:]
        added.append(n)
    if added:
        with open(path, "w") as f:
            f.write(text)
    return added


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/airframe.yaml")
    ap.add_argument("--outdir", default="out/generated")
    ap.add_argument("--world", default="default")
    ap.add_argument("--install", action="store_true",
                    help="copy into the PX4 tree and register in CMakeLists")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    with open(a.config) as f:
        cfg = yaml.safe_load(f)

    install = a.install and not a.dry_run

    print("=" * 74)
    print("MODEL GENERATION" + ("  [DRY RUN]" if not install else ""))
    print("=" * 74)

    af_names = []
    for payload in cfg["payload_variants"]:
        for prop in cfg["prop_options"]:
            af, _, _ = generate(cfg, payload, prop, a.world, a.outdir, install)
            af_names.append(af)

    print(f"\nwrote {len(af_names)} model(s) + airframe(s) to {a.outdir}/")

    if install:
        added = register_cmake(af_names)
        print(f"installed into {GZ_MODELS}")
        print(f"registered in CMakeLists: {added if added else 'already present'}")
        print("\nNow rebuild:  cd ~/PX4-Autopilot && make px4_sitl_default")
    else:
        print("\nDry run — nothing copied into the PX4 tree. Re-run with --install.")


if __name__ == "__main__":
    main()
