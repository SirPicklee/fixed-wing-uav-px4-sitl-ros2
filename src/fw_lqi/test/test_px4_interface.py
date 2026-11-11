# Copyright 2026 SirPicklee
# SPDX-License-Identifier: MIT
"""
Tests for fw_lqi.px4_interface with hand-computed expected values.

The servo mapping is checked against its inverse in
fw_aero/px4_actuators.hpp: aileron = 0.5 (u1 - u0) max_aileron,
elevator = -u2 max_elevator, rudder = -u3 max_rudder.
"""

import math

from fw_lqi.lqi_core import Command, Limits
from fw_lqi.px4_interface import (latlon_to_ne, motor_controls, ned_to_body,
                                  quat_to_roll_pitch, servo_controls)
import pytest


def axis_angle(axis, deg):
    """Return the quaternion (w, x, y, z) of a rotation about a unit axis."""
    h = math.radians(deg) / 2.0
    return (math.cos(h), *(a * math.sin(h) for a in axis))


def test_nose_east_sees_east_velocity_as_forward():
    q = axis_angle((0, 0, 1), 90.0)                    # yaw 90 deg: nose east
    u, v, w = ned_to_body(q, [0.0, 45.0, 0.0])
    assert (u, v, w) == pytest.approx((45.0, 0.0, 0.0), abs=1e-12)


def test_pitch_up_sees_level_velocity_with_positive_w():
    q = axis_angle((0, 1, 0), 10.0)                    # pitch 10 deg nose up
    u, v, w = ned_to_body(q, [45.0, 0.0, 0.0])
    assert u == pytest.approx(45.0 * math.cos(math.radians(10.0)))
    assert w == pytest.approx(45.0 * math.sin(math.radians(10.0)))
    assert v == pytest.approx(0.0, abs=1e-12)


def test_roll_pitch_from_quaternion():
    roll, pitch = quat_to_roll_pitch(*axis_angle((1, 0, 0), 30.0))
    assert math.degrees(roll) == pytest.approx(30.0)
    assert pitch == pytest.approx(0.0, abs=1e-12)
    roll, pitch = quat_to_roll_pitch(*axis_angle((0, 1, 0), -5.0))
    assert math.degrees(pitch) == pytest.approx(-5.0)


def test_servo_mapping_inverts_the_gazebo_plugin_mapping():
    lim = Limits()
    cmd = Command(elevator=math.radians(5.0), aileron=math.radians(10.0),
                  rudder=math.radians(-3.0), throttle=0.6)
    s = servo_controls(cmd, lim)
    assert s[0] == pytest.approx(-0.5) and s[1] == pytest.approx(0.5)   # 10 / 20 deg
    assert s[2] == pytest.approx(-0.25)                                 # -(5 / 20)
    assert s[3] == pytest.approx(0.2)                                   # -(-3 / 15)
    assert all(math.isnan(v) for v in s[4:])
    # Inverse mapping from px4_actuators.hpp gives the command back
    assert 0.5 * (s[1] - s[0]) * lim.aileron == pytest.approx(cmd.aileron)
    assert -s[2] * lim.elevator == pytest.approx(cmd.elevator)
    assert -s[3] * lim.rudder == pytest.approx(cmd.rudder)


def test_servo_values_are_clipped():
    s = servo_controls(Command(elevator=1.0, aileron=1.0, rudder=1.0, throttle=0.0), Limits())
    assert s[:4] == [-1.0, 1.0, -1.0, -1.0]


def test_motor_mapping():
    m = motor_controls(Command(elevator=0.0, aileron=0.0, rudder=0.0, throttle=1.3))
    assert m[0] == 1.0 and len(m) == 12
    assert all(math.isnan(v) for v in m[1:])


def test_latlon_to_ne_hand_values():
    # 0.01 deg of latitude = 6378137 * 0.01 * pi / 180 = 1113.19 m
    n, e = latlon_to_ne(47.01, 8.0, 47.0, 8.0)
    assert n == pytest.approx(1113.195, abs=1e-3) and e == pytest.approx(0.0)
    # 0.01 deg of longitude at 60 deg latitude: half of that (cos 60 = 0.5)
    n, e = latlon_to_ne(60.0, 8.01, 60.0, 8.0)
    assert e == pytest.approx(556.597, abs=1e-3) and n == pytest.approx(0.0)
