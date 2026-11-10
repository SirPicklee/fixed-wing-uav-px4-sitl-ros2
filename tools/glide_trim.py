#!/usr/bin/env python3
"""
Steady power-off glide of the reference Python model (uav-6dof-autopilot).

Solves the 3-DoF longitudinal equilibrium with zero thrust and zero
elevator: pitch moment = 0, lift = W cos(gamma), drag = W sin(gamma).
Used as the independent expected value for the Gazebo glide test.

Usage:
  python tools/glide_trim.py ~/uav-6dof-autopilot [altitude_m]
"""

import math
import sys
from pathlib import Path

import numpy as np


def main():
    ref = Path(sys.argv[1]).expanduser()
    altitude = float(sys.argv[2]) if len(sys.argv) > 2 else 1500.0
    sys.path.insert(0, str(ref / "src"))
    from uavsim.aerodynamics import Controls, aero_forces_moments
    from uavsim.aircraft import DEFAULT_AIRCRAFT as P
    from uavsim.atmosphere import G0, atmosphere

    weight = P.mass * G0
    rho = atmosphere(altitude).density

    # 1) Pitch trim: Cm(alpha) = 0 with q = 0, elevator = 0 (linear in alpha)
    alpha = -P.cm0 / P.cm_alpha
    cl = P.cl0 + P.cl_alpha * alpha
    cd = P.cd0 + P.induced_drag_k * cl**2
    gamma = math.atan(cd / cl)                      # glide angle (descending)
    v = math.sqrt(2 * weight * math.cos(gamma) / (rho * P.wing_area * cl))

    # 2) Check with the full model: forces in wind axes must balance weight
    v_body = np.array([v * math.cos(alpha), 0.0, v * math.sin(alpha)])
    res = aero_forces_moments(P, v_body, np.zeros(3), Controls(), altitude)

    print(f"Reference glide at {altitude:.0f} m (rho = {rho:.4f} kg/m^3)")
    print(f"  alpha        = {math.degrees(alpha):.3f} deg")
    print(f"  CL, CD       = {cl:.4f}, {cd:.5f}")
    print(f"  L/D          = {cl / cd:.3f}")
    print(f"  gamma        = {math.degrees(gamma):.3f} deg")
    print(f"  airspeed     = {v:.3f} m/s")
    print(f"  sink rate    = {v * math.sin(gamma):.3f} m/s")
    print(f"  check: pitch moment = {res.moment_body[1]:.2e} N m")
    print(f"  check: lift - W cos(gamma) = {res.lift - weight * math.cos(gamma):.2e} N")
    print(f"  check: drag - W sin(gamma) = {res.drag - weight * math.sin(gamma):.2e} N")


if __name__ == "__main__":
    main()