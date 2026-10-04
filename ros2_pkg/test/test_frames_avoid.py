import math

import pytest

from drone_eval.avoid import (Camera, box_to_polar, choose_heading,
                              pixel_to_floor)
from drone_eval.frames import (enu_to_ned, ned_to_enu, yaw_enu_to_ned,
                               yaw_ned_to_enu)


def test_east_in_world_is_east_in_px4():
    # 10 m East, 2 m up in Gazebo -> PX4 NED (north 0, east 10, down -2)
    assert enu_to_ned(10, 0, 2) == (0.0, 10.0, -2.0)
    assert enu_to_ned(0, 5, 0)[:2] == (5.0, 0.0)


@pytest.mark.parametrize("p", [(1, 2, 3), (-4.5, 7, 0.2), (0, 0, 0)])
def test_round_trip(p):
    assert ned_to_enu(*enu_to_ned(*p)) == pytest.approx(p)


def test_yaw():
    assert yaw_enu_to_ned(0.0) == pytest.approx(math.pi / 2)    # facing East
    assert yaw_enu_to_ned(math.pi / 2) == pytest.approx(0.0)    # facing North
    for a in (-3.0, -1.0, 0.3, 2.9):
        assert yaw_ned_to_enu(yaw_enu_to_ned(a)) == pytest.approx(a)


CAM = Camera(640, 360, math.radians(66), math.radians(15))


def test_floor_projection_recovers_range():
    # base of an object 6 m ahead, camera 2 m up: find the row it appears on,
    # then check the inverse recovers 6 m
    a = math.atan2(2.0, 6.0)
    v = CAM.cy + CAM.fy * math.tan(a - CAM.pitch0)
    rng, _, _, how = box_to_polar(CAM, (300, 50, 340, v), 2.0, 3.0)
    assert how == "floor"
    assert rng == pytest.approx(6.0, rel=1e-6)


def test_cut_boxes_return_the_nearest_floor_bound():
    # base below the frame: the object is closer than the nearest visible
    # floor point, so that distance is an upper bound and is what is returned
    near = CAM.nearest_floor_range(2.0)
    rng, _, _, how = box_to_polar(CAM, (100, 0, 500, 359), 2.0, 4.0)
    assert how == "cut" and rng == pytest.approx(near)
    rng2, _, _, how2 = box_to_polar(CAM, (100, 40, 500, 359), 2.0, 40.0)
    assert how2 == "height" and rng2 <= near + 1e-9


def test_floor_range_shrinks_when_nose_pitches_down():
    # same pixel row, nose pitched 5 deg down: the floor point is nearer.
    # This is why the IMU attitude has to go into the range estimate.
    box = (300, 50, 340, 250)
    r0, *_ = box_to_polar(CAM, box, 2.0, 3.0, 0.0)
    r5, *_ = box_to_polar(CAM, box, 2.0, 3.0, math.radians(5))
    assert r5 < r0 * 0.85


def test_pixel_to_floor_off_axis():
    # a floor point 6 m forward and 2 m to the left, camera 2 m up, level:
    # project it, then check the inverse recovers its true ground distance
    f = CAM.fx
    X, Y, Z = 6.0, 2.0, -2.0
    c, s = math.cos(CAM.pitch0), math.sin(CAM.pitch0)
    xc, zc = X * c - Z * s, X * s + Z * c       # rotate into the pitched camera
    u = CAM.cx - f * Y / xc
    v = CAM.cy - f * zc / xc
    rng, b = pixel_to_floor(CAM, u, v, 2.0)
    assert b == pytest.approx(math.atan2(2.0, 6.0), abs=1e-6)
    assert rng == pytest.approx(math.hypot(6.0, 2.0), rel=1e-6)


def test_bearing_sign_left_is_positive():
    _, bl, br, _ = box_to_polar(CAM, (0, 100, 50, 200), 2.0, 3.0)
    assert bl > 0 and br > 0


def test_heading_avoids_obstacle_dead_ahead():
    obs = [(4.0, math.radians(10), math.radians(-10))]
    h, near = choose_heading(0.0, obs, clearance_m=0.8)
    assert h is not None and abs(h) > math.radians(10)
    assert near == pytest.approx(4.0)


def test_heading_goes_straight_when_clear():
    h, near = choose_heading(math.radians(4), [], clearance_m=0.8)
    assert h == pytest.approx(math.radians(4), abs=math.radians(2))
    assert math.isinf(near)
