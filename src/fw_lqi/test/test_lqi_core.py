# Copyright 2026 SirPicklee
# SPDX-License-Identifier: MIT
"""
Tests for fw_lqi.lqi_core with hand-computed expected values.

The gains here are simple hand-made matrices, not the designed ones, so every
expected command can be worked out on paper.
"""

import math

from fw_lqi.lqi_core import Limits, LqiController, Measurement, Reference
import numpy as np
import pytest

V0 = 45.0
ALPHA0 = math.radians(2.0)
ELEV0 = math.radians(0.5)
THR0 = 0.6


def make(k_lon=None, k_lat=None, limits=None):
    """Return a controller with zero gains unless given."""
    return LqiController(np.zeros((2, 6)) if k_lon is None else k_lon,
                         np.zeros((2, 5)) if k_lat is None else k_lat,
                         V0, ALPHA0, ELEV0, THR0, limits)


def trim_meas(**changes):
    """Return the trimmed level-flight measurement with optional changes."""
    m = {'u': V0 * math.cos(ALPHA0), 'v': 0.0, 'w': V0 * math.sin(ALPHA0),
         'p': 0.0, 'q': 0.0, 'r': 0.0, 'phi': 0.0, 'theta': ALPHA0,
         'altitude': 100.0, 'airspeed': V0}
    m.update(changes)
    return Measurement(**m)


def trim_ref(**changes):
    """Return the reference matching the trim point with optional changes."""
    r = {'altitude': 100.0, 'airspeed': V0, 'roll': 0.0}
    r.update(changes)
    return Reference(**r)


def test_trim_point_returns_trim_commands():
    rng = np.random.default_rng(0)
    c = make(rng.normal(size=(2, 6)), rng.normal(size=(2, 5)))
    cmd = c.update(trim_meas(), trim_ref(), 0.02)
    assert cmd.elevator == pytest.approx(ELEV0, abs=1e-12)
    assert cmd.throttle == pytest.approx(THR0, abs=1e-12)
    assert cmd.aileron == pytest.approx(0.0, abs=1e-12)
    assert cmd.rudder == pytest.approx(0.0, abs=1e-12)
    assert c.xi_v == 0.0 and c.xi_phi == 0.0


def test_pitch_feedback_is_minus_k_times_deviation():
    k_lon = np.zeros((2, 6))
    k_lon[0, 3] = 2.0                          # elevator per rad of pitch error
    c = make(k_lon=k_lon)
    cmd = c.update(trim_meas(theta=ALPHA0 + 0.05), trim_ref(), 0.02)
    assert cmd.elevator == pytest.approx(ELEV0 - 2.0 * 0.05)


def test_altitude_error_is_clipped():
    k_lon = np.zeros((2, 6))
    k_lon[1, 4] = -0.01                        # throttle per metre of e_h
    c = make(k_lon=k_lon)
    low = c.update(trim_meas(altitude=95.0), trim_ref(), 0.02).throttle
    far = c.update(trim_meas(altitude=0.0), trim_ref(), 0.02).throttle
    assert low == pytest.approx(THR0 + 0.05)
    assert far == pytest.approx(THR0 + 0.01 * Limits().altitude_error)


def test_airspeed_integrator_and_anti_windup():
    c = make()
    c.update(trim_meas(airspeed=V0 - 2.0), trim_ref(), 0.5)
    assert c.xi_v == pytest.approx(1.0)        # 2 m/s for 0.5 s
    for _ in range(100):
        c.update(trim_meas(airspeed=V0 - 2.0), trim_ref(), 0.5)
    assert c.xi_v == pytest.approx(Limits().xi_v)
    c.reset()
    assert c.xi_v == 0.0


def test_roll_integrator_sign():
    c = make()
    c.update(trim_meas(phi=0.0), trim_ref(roll=0.2), 0.1)
    assert c.xi_phi == pytest.approx(0.02)     # integral of (phi_ref - phi)


def test_roll_error_commands_aileron_toward_reference():
    k_lat = np.zeros((2, 5))
    k_lat[0, 3] = 0.9
    c = make(k_lat=k_lat)
    cmd = c.update(trim_meas(phi=0.0), trim_ref(roll=math.radians(10.0)), 0.02)
    assert cmd.aileron == pytest.approx(0.9 * math.radians(10.0))
    assert cmd.aileron > 0.0                   # positive aileron rolls right


def test_commands_saturate_at_limits():
    k_lon = np.zeros((2, 6))
    k_lon[0, 3], k_lon[1, 0] = 100.0, 100.0
    k_lat = np.zeros((2, 5))
    k_lat[0, 3], k_lat[1, 2] = 100.0, 100.0
    lim = Limits()
    c = make(k_lon, k_lat)
    # Large positive deviations: u = u_trim - K x drives every command to its lower limit
    pos = c.update(trim_meas(theta=1.0, u=100.0, phi=1.0, r=1.0), trim_ref(), 0.02)
    assert pos.elevator == pytest.approx(-lim.elevator)
    assert pos.throttle == pytest.approx(0.0)
    assert pos.aileron == pytest.approx(-lim.aileron)
    assert pos.rudder == pytest.approx(-lim.rudder)
    neg = c.update(trim_meas(theta=-1.0, u=0.0, phi=-1.0, r=-1.0), trim_ref(), 0.02)
    assert neg.elevator == pytest.approx(lim.elevator)
    assert neg.throttle == pytest.approx(1.0)
    assert neg.aileron == pytest.approx(lim.aileron)
    assert neg.rudder == pytest.approx(lim.rudder)
