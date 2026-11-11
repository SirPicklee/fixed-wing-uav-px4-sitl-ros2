"""
LQI flight controller core (numpy only, no ROS dependency).

Longitudinal: x = [u, w, q, theta, e_h, xi_v]       -> [elevator, throttle]
Lateral:      x = [v, p, r, phi - phi_ref, xi_phi]  -> [aileron, rudder]

All quantities are deviations from the level-flight trim point the gains were
designed at (tools/lqi_design.py). Units: SI, angles in rad, body axes FRD.
Control law: u = u_trim - K x, then clipped to the physical limits.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class Measurement:
    """Aircraft state needed by the controller."""

    u: float          # body-axis velocity components, m/s
    v: float
    w: float
    p: float          # body rates, rad/s
    q: float
    r: float
    phi: float        # roll, rad
    theta: float      # pitch, rad
    altitude: float   # m, positive up
    airspeed: float   # m/s


@dataclass
class Reference:
    """Commands from the guidance layer."""

    altitude: float   # m
    airspeed: float   # m/s
    roll: float       # rad


@dataclass
class Command:
    """Physical actuator commands."""

    elevator: float   # rad, positive trailing edge down (nose down)
    aileron: float    # rad, positive rolls right
    rudder: float     # rad, positive yaws left
    throttle: float   # 0..1


@dataclass
class Limits:
    """Saturation and anti-windup limits."""

    elevator: float = math.radians(20.0)
    aileron: float = math.radians(20.0)
    rudder: float = math.radians(15.0)
    altitude_error: float = 10.0   # m, error fed to the controller is clipped
    xi_v: float = 30.0             # m, airspeed integrator clamp
    xi_phi: float = 0.5            # rad s, roll integrator clamp


class LqiController:
    """Discrete-time implementation of the continuous LQI design."""

    def __init__(self, k_lon, k_lat, trim_airspeed, trim_alpha,
                 trim_elevator, trim_throttle, limits=None):
        self.k_lon = np.asarray(k_lon, dtype=float).reshape(2, 6)
        self.k_lat = np.asarray(k_lat, dtype=float).reshape(2, 5)
        self.u0 = trim_airspeed * math.cos(trim_alpha)
        self.w0 = trim_airspeed * math.sin(trim_alpha)
        self.theta0 = trim_alpha          # level flight: pitch = alpha
        self.elevator0 = trim_elevator
        self.throttle0 = trim_throttle
        self.limits = limits or Limits()
        self.reset()

    def reset(self):
        """Clear the integrator states."""
        self.xi_v = 0.0
        self.xi_phi = 0.0

    def update(self, m: Measurement, ref: Reference, dt: float) -> Command:
        """Advance the integrators by dt and return the actuator command."""
        lim = self.limits
        e_h = float(np.clip(ref.altitude - m.altitude, -lim.altitude_error, lim.altitude_error))
        self.xi_v = float(np.clip(self.xi_v + (ref.airspeed - m.airspeed) * dt,
                                  -lim.xi_v, lim.xi_v))
        roll_err = m.phi - ref.roll
        self.xi_phi = float(np.clip(self.xi_phi - roll_err * dt, -lim.xi_phi, lim.xi_phi))

        x_lon = np.array([m.u - self.u0, m.w - self.w0, m.q, m.theta - self.theta0,
                          e_h, self.xi_v])
        x_lat = np.array([m.v, m.p, m.r, roll_err, self.xi_phi])
        d_lon = -self.k_lon @ x_lon
        d_lat = -self.k_lat @ x_lat

        return Command(
            elevator=float(np.clip(self.elevator0 + d_lon[0], -lim.elevator, lim.elevator)),
            aileron=float(np.clip(d_lat[0], -lim.aileron, lim.aileron)),
            rudder=float(np.clip(d_lat[1], -lim.rudder, lim.rudder)),
            throttle=float(np.clip(self.throttle0 + d_lon[1], 0.0, 1.0)),
        )
