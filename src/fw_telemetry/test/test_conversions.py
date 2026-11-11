# Copyright 2026 SirPicklee
# SPDX-License-Identifier: MIT
"""Tests for fw_telemetry.conversions with independently known answers."""

import math

from fw_telemetry.conversions import quat_to_euler
import pytest


def axis_angle(axis, angle_deg):
    """Return the quaternion (w, x, y, z) of a rotation about a unit axis."""
    h = math.radians(angle_deg) / 2.0
    return (math.cos(h), *(a * math.sin(h) for a in axis))


def q_mul(a, b):
    """Return the Hamilton product a * b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def degrees(angles):
    """Convert a tuple of radians to degrees."""
    return tuple(math.degrees(a) for a in angles)


def test_identity_is_level_north():
    assert degrees(quat_to_euler(1.0, 0.0, 0.0, 0.0)) == pytest.approx((0, 0, 0))


def test_nose_east_is_yaw_90():
    q = axis_angle((0, 0, 1), 90.0)
    assert degrees(quat_to_euler(*q)) == pytest.approx((0, 0, 90))


def test_zyx_composition_round_trip():
    # yaw 120, then pitch 10, then roll -30 (aerospace ZYX sequence)
    q = q_mul(q_mul(axis_angle((0, 0, 1), 120.0), axis_angle((0, 1, 0), 10.0)),
              axis_angle((1, 0, 0), -30.0))
    assert degrees(quat_to_euler(*q)) == pytest.approx((-30, 10, 120))


def test_sample_received_from_px4_on_the_runway():
    # /fmu/out/vehicle_attitude echoed in SITL, aircraft parked nose east
    q = (0.6941810846328735, 0.008909565396606922,
         -0.0032312381081283092, 0.7197380065917969)
    roll, pitch, yaw = degrees(quat_to_euler(*q))
    assert yaw == pytest.approx(92.07, abs=0.01)
    assert abs(roll) < 1.0 and abs(pitch) < 1.5
