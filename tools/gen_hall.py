#!/usr/bin/env python3
"""
gen_hall.py - build the v2 factory hall world from config/factory_hall.yaml.

One enclosed hall. The launch pad, the obstacle course and the scan zone are
all inside it:

    y
    ^   +-----------------------------------------------------------+
    |   |                                              scan zone    |
    |   |  PAD   rack_1  machine_1  rack_2  pallets  machine_2  [] |
    |   | (0,0)  ...obstacles staggered across the centreline...    |
    |   +-----------------------------------------------------------+
    +---------------------------------------------------------------> x

Reuses the world header (physics, the full plugin list, atmosphere) from
gen_world.py: declaring ANY world plugin replaces Gazebo's default systems, so
the header declares all ten. Leaving one out has failed silently before.

Usage:
  python3 tools/gen_hall.py --config config/factory_hall.yaml \
                            --out worlds/factory_hall.sdf
"""

import argparse
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_world  # noqa: E402  (HEAD, box(), TAIL)

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CFG = os.path.join(HERE, "..", "config", "factory_hall.yaml")

WALL_RGB = (0.80, 0.80, 0.82)


def load(cfg_path=DEFAULT_CFG):
    with open(cfg_path) as f:
        return yaml.safe_load(f)


def objects(cfg=None):
    """Every collidable box: (name, class, cx, cy, cz, sx, sy, sz).

    The ONE geometry list. The world is built from it, and the flight report
    and the computer-vision scorer measure against it. Walls are included,
    with class 'wall'.
    """
    cfg = cfg or load()
    h = cfg["hall"]
    (x0, x1), (y0, y1) = h["x"], h["y"]
    H, t = h["wall_height"], h["wall_thickness"]
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    lx, ly = x1 - x0, y1 - y0
    out = [
        ("wall_north", "wall", cx, y1 + t / 2, H / 2, lx + 2 * t, t, H),
        ("wall_south", "wall", cx, y0 - t / 2, H / 2, lx + 2 * t, t, H),
        ("wall_east", "wall", x1 + t / 2, cy, H / 2, t, ly, H),
        ("wall_west", "wall", x0 - t / 2, cy, H / 2, t, ly, H),
    ]
    for name, cls, x, y, sx, sy in cfg["objects"]:
        hz = float(cfg["classes"][cls]["height_m"])
        out.append((name, cls, float(x), float(y), hz / 2.0,
                    float(sx), float(sy), hz))
    return out


LIGHT = """
    <light name="{name}" type="point">
      <pose>{x} {y} {z} 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <diffuse>0.55 0.55 0.52 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>40</range><constant>0.6</constant>
        <linear>0.02</linear><quadratic>0.0</quadratic></attenuation>
    </light>
"""


def build(cfg):
    name = cfg["hall"]["name"]
    head = gen_world.HEAD.replace('<world name="factory">',
                                  f'<world name="{name}">')
    geo = cfg.get("geo")
    if geo:
        head = head.replace("<latitude_deg>37.4</latitude_deg>",
                            f"<latitude_deg>{geo['lat']}</latitude_deg>")
        head = head.replace("<longitude_deg>-122.1</longitude_deg>",
                            f"<longitude_deg>{geo['lon']}</longitude_deg>")
    parts = [head]

    # Overhead hall lighting so the camera never looks into a dark corner.
    h = cfg["hall"]
    (x0, x1), (y0, y1) = h["x"], h["y"]
    i = 0
    for lx in (x0 + 8, (x0 + x1) / 2, x1 - 8):
        for ly in (y0 + 7, y1 - 7):
            parts.append(LIGHT.format(name=f"hall_light_{i}", x=lx, y=ly,
                                      z=h["wall_height"] - 0.5))
            i += 1

    # Launch pad: a painted floor marking, visual only (a collision body made
    # an earlier vehicle fight the ground on takeoff).
    p = cfg["pad"]
    parts.append(gen_world.box("launch_pad", p["xy"][0], p["xy"][1], 0.01,
                               p["size"], p["size"], 0.02,
                               rgb=tuple(p["rgb"]), collide=False))

    for nm, cls, x, y, z, sx, sy, sz in objects(cfg):
        rgb = WALL_RGB if cls == "wall" else tuple(cfg["classes"][cls]["rgb"])
        parts.append(gen_world.box(nm, x, y, z, sx, sy, sz, rgb=rgb))

    parts.append(gen_world.TAIL)
    return "".join(parts)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=DEFAULT_CFG)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    cfg = load(a.config)
    with open(a.out, "w") as f:
        f.write(build(cfg))
    objs = objects(cfg)
    n_course = sum(1 for o in objs if o[1] != "wall")
    print(f"wrote {a.out}: hall {cfg['hall']['x']} x {cfg['hall']['y']} m, "
          f"{n_course} objects + 4 walls")
