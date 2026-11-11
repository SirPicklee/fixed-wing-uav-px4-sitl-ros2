#!/usr/bin/env python3
"""
Fly the TB2 mission legs with L1 guidance + LQI in the nonlinear 6-DoF model.

Same waypoints as scripts/fly_mission.py (tb2), from the end of the PX4
takeoff to the last waypoint before the landing sequence. Both modules are
the ones the ROS 2 node uses (src/fw_lqi/fw_lqi/guidance.py, lqi_core.py).

Usage:
  python tools/lqi_mission_sim.py ~/uav-6dof-autopilot
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DT = 0.02
T_MAX = 900.0
SETTLE = 20.0                    # s, capture window excluded from the tracking metrics
START = (0.0, 600.0, 60.0)       # north, east, altitude after the takeoff climb-out
START_YAW = math.radians(90.0)   # runway heading: east
WAYPOINTS = [(1500.0, 3000.0, 120.0), (3000.0, 0.0, 120.0),
             (1500.0, -3500.0, 100.0), (0.0, -3200.0, 80.0)]


def main():
    ref_repo = Path(sys.argv[1]).expanduser()
    sys.path.insert(0, str(ref_repo / "src"))
    sys.path.insert(0, str(ROOT / "src" / "fw_lqi"))
    sys.path.insert(0, str(ROOT / "tools"))
    from fw_lqi.guidance import L1Guidance
    from fw_lqi.lqi_core import LqiController, Measurement, Reference
    from lqi_design import AIRSPEED, ALTITUDE, Q_LAT, Q_LON, R_LAT, R_LON, augment_lat, augment_lon
    from uavsim import quaternion as quat
    from uavsim.aerodynamics import Controls
    from uavsim.dynamics import constant_controls, make_dynamics, make_state, post_step, trim_level_flight
    from uavsim.integrators import rk4_step
    from uavsim.linearize import lateral, linearize, longitudinal
    from uavsim.lqr import lqr

    trim = trim_level_flight(AIRSPEED, ALTITUDE)
    A, B, _, _ = linearize(trim)
    k_lon, _, _ = lqr(*augment_lon(*longitudinal(A, B), AIRSPEED), Q_LON, R_LON)
    k_lat, _, _ = lqr(*augment_lat(*lateral(A, B)), Q_LAT, R_LAT)
    ctrl = LqiController(k_lon, k_lat, AIRSPEED, trim.alpha, trim.elevator, trim.throttle)
    guide = L1Guidance(WAYPOINTS)

    y = make_state(AIRSPEED, trim.alpha, trim.pitch, START[2], yaw=START_YAW)
    y[0], y[1] = START[0], START[1]
    guide.reset(START[0], START[1])
    log = []
    t = 0.0
    while t < T_MAX:
        phi, theta, _ = quat.to_euler(y[6:10])
        v_ned = quat.rotate(y[6:10], y[3:6])   # body -> NED ground velocity (no wind)
        g = guide.update(y[0], y[1], v_ned[0], v_ned[1])
        if g.done:
            break
        meas = Measurement(u=y[3], v=y[4], w=y[5], p=y[10], q=y[11], r=y[12], phi=phi, theta=theta,
                           altitude=-y[2], airspeed=float(np.linalg.norm(y[3:6])))
        c = ctrl.update(meas, Reference(altitude=g.altitude, airspeed=AIRSPEED, roll=g.roll), DT)
        log.append((t, g.leg, g.cross_track, g.altitude + y[2], meas.airspeed, phi, c.elevator, c.throttle))
        u = Controls(elevator=c.elevator, aileron=c.aileron, rudder=c.rudder, throttle=c.throttle)
        y = post_step(rk4_step(make_dynamics(constant_controls(u)), t, y, DT))
        t += DT

    t, leg, xt, eh, spd, phi, elev, thr = np.array(log).T
    print(f"Mission legs finished: {g.done} after {t[-1]:.1f} s")
    print(f"Tracking after a {SETTLE:.0f} s capture window on each leg:")
    print(f"{'leg':>4s} {'start s':>8s} {'xt rms m':>9s} {'xt max m':>9s} {'alt rms m':>10s} {'alt max m':>10s}")
    for k in range(len(WAYPOINTS)):
        t0 = t[leg == k][0]
        sel = (leg == k) & (t >= t0 + SETTLE)
        print(f"{k + 1:4d} {t0:8.1f} {np.sqrt(np.mean(xt[sel]**2)):9.2f} {np.abs(xt[sel]).max():9.2f} "
              f"{np.sqrt(np.mean(eh[sel]**2)):10.2f} {np.abs(eh[sel]).max():10.2f}")
    print(f"airspeed {spd.min():.1f} .. {spd.max():.1f} m/s, max bank {math.degrees(np.abs(phi).max()):.1f} deg, "
          f"elevator {math.degrees(elev.min()):.1f} .. {math.degrees(elev.max()):.1f} deg, "
          f"throttle {thr.min():.2f} .. {thr.max():.2f}")
    sat = np.abs(elev) >= math.radians(20.0) - 1e-9
    print(f"elevator saturated {100 * sat.mean():.1f} % of the time")


if __name__ == "__main__":
    main()
