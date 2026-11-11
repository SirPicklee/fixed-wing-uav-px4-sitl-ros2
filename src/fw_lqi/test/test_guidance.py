# Copyright 2026 SirPicklee
# SPDX-License-Identifier: MIT
"""
Tests for fw_lqi.guidance with hand-computed expected values.

Geometry used below: a single leg due north from (0, 0) to (5000, 0),
L1 = 400 m, aircraft at 45 m/s.
"""

import math

from fw_lqi.guidance import G0, L1Guidance, wrap_pi
import pytest

V = 45.0


def north_leg(**kwargs):
    """Return guidance with one leg due north starting at the origin."""
    g = L1Guidance([(5000.0, 0.0, 120.0)], **kwargs)
    g.reset(0.0, 0.0)
    return g


def test_on_track_and_aligned_gives_wings_level():
    out = north_leg().update(1000.0, 0.0, V, 0.0)
    assert out.roll == pytest.approx(0.0, abs=1e-12)
    assert out.cross_track == pytest.approx(0.0, abs=1e-12)
    assert out.altitude == 120.0
    assert not out.done


def test_heading_30_deg_left_of_track():
    # Course 30 deg left of north, on track: eta = +30 deg, distance = L1
    vn, ve = V * math.cos(math.radians(-30.0)), V * math.sin(math.radians(-30.0))
    out = north_leg().update(1000.0, 0.0, vn, ve)
    a_lat = 2.0 * V**2 * 0.5 / 400.0                   # 5.0625 m/s^2
    assert out.roll == pytest.approx(math.atan(a_lat / G0))
    assert math.degrees(out.roll) == pytest.approx(27.30, abs=0.01)


def test_left_of_track_turns_right_and_reports_negative_cross_track():
    # 300 m west of the leg, flying north: line of sight to (1400, 0) is
    # atan2(300, 400) = 36.87 deg right; distance 500 m
    out = north_leg(max_bank=math.radians(60.0)).update(1000.0, -300.0, V, 0.0)
    a_lat = 2.0 * V**2 * 0.6 / 500.0
    assert out.cross_track == pytest.approx(-300.0)
    assert out.roll == pytest.approx(math.atan(a_lat / G0))
    assert out.roll > 0.0


def test_bank_is_limited():
    # Course 80 deg left of the leg: a = 2 V^2 sin(80 deg) / L1 = 9.97 m/s^2 (45 deg bank)
    vn, ve = V * math.cos(math.radians(-80.0)), V * math.sin(math.radians(-80.0))
    out = north_leg().update(1000.0, 0.0, vn, ve)
    assert out.roll == pytest.approx(math.radians(30.0))


def test_target_behind_is_treated_as_90_deg():
    out = north_leg(max_bank=math.radians(89.0)).update(1000.0, 0.0, -V, 1e-9)
    a_lat = 2.0 * V**2 * 1.0 / 400.0
    assert abs(out.roll) == pytest.approx(math.atan(a_lat / G0))


def test_leg_switch_and_done():
    g = L1Guidance([(2000.0, 0.0, 100.0), (2000.0, 2000.0, 80.0)],
                   switch_distance=400.0)
    g.reset(0.0, 0.0)
    assert g.update(1500.0, 0.0, V, 0.0).leg == 0      # 500 m to go
    out = g.update(1700.0, 0.0, V, 0.0)                # 300 m to go: switch
    assert out.leg == 1 and out.altitude == 80.0 and not out.done
    out = g.update(2000.0, 1700.0, 0.0, V)             # 300 m before the last
    assert out.done


def test_wrap_pi():
    assert wrap_pi(math.radians(190.0)) == pytest.approx(math.radians(-170.0))
    assert wrap_pi(math.radians(-190.0)) == pytest.approx(math.radians(170.0))
