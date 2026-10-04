"""
avoid.py - turn camera detections into a safe heading. Pure functions, no ROS,
so the geometry is unit-testable.

Detections arrive as bounding boxes in pixels. Converting a box into a range
and a bearing needs the camera's height and attitude at the moment the frame
was captured; the attitude comes from PX4's EKF, i.e. from the IMU. That is
where the IMU enters obstacle avoidance: a 5 degree pitch error moves a
ground-plane range estimate by metres at 6 m.

Range, three cases:
  1. "floor"  box bottom visible -> project the bottom edge onto the floor
              (camera height / tan(angle below horizon)). A MEASUREMENT.
  2. "height" bottom touches the frame edge, top visible -> pinhole on the
              object's known height, clamped to case 3.
  3. "cut"    both touch the frame edges -> the nearest floor distance the
              camera can see.
Cases 2 and 3 are NOT measurements. They were first documented as upper
bounds ("the object is closer than this"); flight data disproved that - in
the frame-check run the box reached the frame bottom while the true base was
still ~15 rows above it, and the true range exceeded the "bound" in 45 of 70
cases. They are flags meaning "close, range unknown". hall_mission maps only
case 1 and places 2/3 as short-lived obstacles at HALF the returned value; the
report checks that this placement is nearer than the truth.
"""

import math


class Camera:
    def __init__(self, width, height, hfov_rad, pitch_down_rad):
        self.w, self.h = width, height
        self.fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        self.fy = self.fx                    # square pixels
        self.cx, self.cy = width / 2.0, height / 2.0
        self.pitch0 = pitch_down_rad         # mounting pitch, +down
        self.vfov = 2.0 * math.atan(self.cy / self.fy)

    def ray_down_angle(self, v, vehicle_pitch_down=0.0):
        """Angle below the horizon of pixel row v (v grows downward)."""
        return self.pitch0 + vehicle_pitch_down + math.atan((v - self.cy) / self.fy)

    def nearest_floor_range(self, cam_height, vehicle_pitch_down=0.0):
        a = self.ray_down_angle(self.h - 1, vehicle_pitch_down)
        return cam_height / math.tan(a) if a > 1e-3 else float("inf")


def pixel_to_floor(cam, u, v, cam_height, vehicle_pitch_down=0.0):
    """A pixel that shows where an object meets the floor -> (horizontal
    range along the ray, bearing +left). None if the ray does not hit the
    floor ahead."""
    # ray in camera axes (forward, left, up), then pitched down by p
    p = cam.pitch0 + vehicle_pitch_down
    yl = (cam.cx - u) / cam.fx
    zu = (cam.cy - v) / cam.fy
    X = math.cos(p) + zu * math.sin(p)
    Z = -math.sin(p) + zu * math.cos(p)
    if Z >= -1e-4:
        return None                       # at or above the horizon
    t = cam_height / -Z
    return t * math.hypot(X, yl), math.atan2(yl, X)


def box_to_polar(cam, box, cam_height, obj_height, vehicle_pitch_down=0.0,
                 edge_px=3):
    """box = (u0, v0, u1, v1) pixels. Returns (range_m, bearing_left_rad,
    bearing_right_rad, method). Bearings are body-frame, +left (FLU)."""
    u0, v0, u1, v1 = box
    b_left = math.atan((cam.cx - u0) / cam.fx)
    b_right = math.atan((cam.cx - u1) / cam.fx)
    near = cam.nearest_floor_range(cam_height, vehicle_pitch_down)
    bottom_cut = v1 >= cam.h - 1 - edge_px
    top_cut = v0 <= edge_px
    if not bottom_cut:
        a = cam.ray_down_angle(v1, vehicle_pitch_down)
        if a > 1e-3:
            return cam_height / math.tan(a), b_left, b_right, "floor"
    if not top_cut and obj_height > 0:
        h_px = max(v1 - v0, 1.0)
        rng = cam.fy * obj_height / h_px
        return min(rng, near), b_left, b_right, "height"
    return near, b_left, b_right, "cut"


def blocked_intervals(obstacles, clearance_m, max_range=7.0):
    """obstacles: (range, bearing_a, bearing_b). Returns [(lo, hi, range)],
    each widened by asin(clearance / range): closer than the clearance, an
    obstacle blocks the whole half-plane facing it."""
    out = []
    for rng, a, b in obstacles:
        if rng > max_range:
            continue
        pad = math.asin(min(1.0, clearance_m / max(rng, 1e-3)))
        out.append((min(a, b) - pad, max(a, b) + pad, rng))
    return out


def is_free(h, intervals):
    return not any(lo <= h <= hi for lo, hi, _ in intervals)


def nearest_along(h, intervals):
    r = [rng for lo, hi, rng in intervals if lo <= h <= hi]
    return min(r) if r else float("inf")


def choose_heading(goal_bearing, obstacles, clearance_m, max_range=7.0,
                   step_deg=2.0, fov_limit=math.radians(80), prefer=None,
                   hysteresis=math.radians(10)):
    """VFH-lite. obstacles: list of (range, bearing_left, bearing_right) in the
    body frame. Returns (heading_rad or None if boxed in, nearest_range_ahead).

    prefer: the previous heading. Kept if it is still free and no more than
    `hysteresis` worse than the best, so the aircraft does not dither between
    two near-equal gaps."""
    iv = blocked_intervals(obstacles, clearance_m, max_range)
    nearest_ahead = nearest_along(0.0, iv)
    goal = max(-fov_limit, min(fov_limit, goal_bearing))
    best = None
    n = int(round(math.degrees(2 * fov_limit) / step_deg)) + 1
    for i in range(n):
        h = -fov_limit + math.radians(step_deg) * i
        if not is_free(h, iv):
            continue
        if best is None or abs(h - goal) < abs(best - goal):
            best = h
    if (best is not None and prefer is not None and abs(prefer) <= fov_limit
            and is_free(prefer, iv)
            and abs(prefer - goal) <= abs(best - goal) + hysteresis):
        best = prefer
    return best, nearest_ahead
