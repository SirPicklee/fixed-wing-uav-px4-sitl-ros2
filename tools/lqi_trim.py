#!/usr/bin/env python3
"""
Step 1 of the LQI design: trim the TB2 model in level flight and linearize it.

Uses the reference Python model (uav-6dof-autopilot), whose aircraft
parameters are identical to fw_aero::AircraftParams used in Gazebo.
Prints the trim point, the longitudinal/lateral A, B matrices and the
open-loop modes, with textbook approximations as an independent check.

Usage:
  python tools/lqi_trim.py ~/uav-6dof-autopilot [airspeed_m_s] [altitude_m]
"""

import math
import sys
from pathlib import Path

import numpy as np


def main():
    ref = Path(sys.argv[1]).expanduser()
    airspeed = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
    altitude = float(sys.argv[3]) if len(sys.argv) > 3 else 100.0
    sys.path.insert(0, str(ref / "src"))
    from uavsim.aircraft import DEFAULT_AIRCRAFT as P
    from uavsim.atmosphere import G0, atmosphere
    from uavsim.dynamics import trim_level_flight
    from uavsim.linearize import describe_eigenvalues, lateral, linearize, longitudinal

    trim = trim_level_flight(airspeed, altitude)
    print(f"Trim at {airspeed:.1f} m/s, {altitude:.0f} m (residual {trim.residual:.1e})")
    print(f"  alpha    = {math.degrees(trim.alpha):7.3f} deg")
    print(f"  elevator = {math.degrees(trim.elevator):7.3f} deg")
    print(f"  throttle = {trim.throttle:7.4f}")

    # Independent check: lift coefficient needed to carry the weight
    rho = atmosphere(altitude).density
    qbar = 0.5 * rho * airspeed**2
    cl_req = P.mass * G0 / (qbar * P.wing_area)
    print(f"  CL required by W = qS*CL : {cl_req:.4f}")

    A, B, _, _ = linearize(trim)
    A_lon, B_lon = longitudinal(A, B)
    A_lat, B_lat = lateral(A, B)
    np.set_printoptions(precision=4, suppress=True, linewidth=110)
    print("\nLongitudinal  x = [u, w, q, theta], u = [elevator, throttle]")
    print("A_lon =\n", A_lon)
    print("B_lon =\n", B_lon)
    print("\nLateral  x = [v, p, r, phi], u = [aileron, rudder]")
    print("A_lat =\n", A_lat)
    print("B_lat =\n", B_lat)

    for name, Am in (("longitudinal", A_lon), ("lateral", A_lat)):
        print(f"\nOpen-loop {name} modes:")
        for m in describe_eigenvalues(Am):
            if m["type"] == "oscillatory":
                print(f"  oscillatory: wn={m['wn']:.3f} rad/s, zeta={m['zeta']:.3f}, "
                      f"period={m['period']:.2f} s")
            else:
                print(f"  real:        lambda={m['lambda'].real:+.4f} 1/s, tau={m['tau']:.2f} s")

    # Lanchester phugoid approximation: period = pi * sqrt(2) * V / g
    print(f"\nLanchester phugoid period pi*sqrt(2)*V/g = {math.pi * math.sqrt(2) * airspeed / G0:.2f} s")


if __name__ == "__main__":
    main()