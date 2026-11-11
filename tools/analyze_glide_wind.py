#!/usr/bin/env python3
"""
Check the Gazebo glide in a steady wind against the Python 6-DoF model.

Both start level at 2000 m, nose east, 46 m/s ground speed along the nose,
zero controls, in the same steady wind. The Python model (uav-6dof-autopilot)
applies the wind to the aerodynamics only, exactly as the plugin should, so
the two trajectories must agree. The Gazebo log is in ENU (x east, y north).

Usage:
  python tools/analyze_glide_wind.py /tmp/glide_wind_test.csv ~/uav-6dof-autopilot [wn we wd]
"""

import math
import sys
from pathlib import Path

import numpy as np

TIMES = (20, 60, 100, 200, 300)


def python_reference(ref_repo, wind_ned):
    """Return {t: (alt, vN, vE, vD, airspeed)} from the reference model."""
    sys.path.insert(0, str(Path(ref_repo).expanduser() / "src"))
    from uavsim import quaternion as quat
    from uavsim.aerodynamics import Controls
    from uavsim.dynamics import constant_controls, make_dynamics, make_state, post_step
    from uavsim.integrators import rk4_step

    f = make_dynamics(constant_controls(Controls()), wind_model=lambda t, y: wind_ned)
    y = make_state(46.0, 0.0, 0.0, 2000.0, yaw=math.pi / 2)
    dt, out = 0.004, {}
    for k in range(1, int(max(TIMES) / dt) + 1):
        y = post_step(rk4_step(f, (k - 1) * dt, y, dt))
        t = round(k * dt, 6)
        if t in TIMES:
            v = quat.rotate(y[6:10], y[3:6])
            out[t] = (-y[2], v[0], v[1], v[2], float(np.linalg.norm(v - wind_ned)))
    return out


def row(values):
    """Format alt, vN, vE, vD, Vair."""
    return f"{values[0]:7.1f} " + " ".join(f"{v:6.2f}" for v in values[1:])


def main():
    d = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
    wind = np.array([float(a) for a in sys.argv[3:6]]) if len(sys.argv) > 5 else np.array([10.0, 0, 0])
    ref = python_reference(sys.argv[2], wind)
    print(f"Wind NED {wind} m/s; max |gust| in log {np.abs(d['gust_u']).max():.1e} m/s")
    print(f"{'t':>4s} | {'alt':>7s} {'vN':>6s} {'vE':>6s} {'vD':>5s} {'Vair':>6s}  Gazebo"
          f"\n{'':4s} | {'alt':>7s} {'vN':>6s} {'vE':>6s} {'vD':>5s} {'Vair':>6s}  Python")
    worst = np.zeros(5)
    for tt in TIMES:
        i = int(np.argmin(np.abs(d["t"] - tt)))
        g = np.array([d["z"][i], d["vy"][i], d["vx"][i], -d["vz"][i], d["airspeed"][i]])
        p = np.array(ref[tt])
        worst = np.maximum(worst, np.abs(g - p))
        print(f"{d['t'][i]:4.0f} | {row(g)}")
        print(f"{'':4s} | {row(p)}")
    print("Max |Gazebo - Python|: alt {:.2f} m, vN {:.3f}, vE {:.3f}, vD {:.3f}, "
          "Vair {:.3f} m/s".format(*worst))


if __name__ == "__main__":
    main()