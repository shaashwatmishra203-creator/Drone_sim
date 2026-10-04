"""
frames.py - the one place world <-> PX4 coordinate conversion happens.

Gazebo's world frame is ENU: x East, y North, z Up. PX4's local frame is NED:
x North, y East, z Down. PX4's gz bridge converts between them, so a
TrajectorySetpoint is always read as NED.

Every mission before v2 skipped this. Waypoints written in world ENU (course
along +x/East) went to PX4 unchanged and were flown as NED, i.e. along North:
the aircraft flew the course rotated 90 degrees in open ground beside the
walls. The simulated sensor, the clearance check and the estimator all made
the same mistake, so they agreed with each other inside a phantom frame and
nothing flagged it. Hence this module, and hence a truth logger that reads the
pose from Gazebo itself rather than from PX4.

Yaw: ENU yaw is measured from East, counter-clockwise; NED yaw from North,
clockwise. yaw_ned = pi/2 - yaw_enu (and the map is its own inverse).
"""

import math


def enu_to_ned(x, y, z):
    return (float(y), float(x), -float(z))


def ned_to_enu(n, e, d):
    return (float(e), float(n), -float(d))


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_enu_to_ned(yaw_enu):
    return wrap_pi(math.pi / 2.0 - yaw_enu)


def yaw_ned_to_enu(yaw_ned):
    return wrap_pi(math.pi / 2.0 - yaw_ned)
