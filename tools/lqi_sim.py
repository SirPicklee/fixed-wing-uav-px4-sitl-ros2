#!/usr/bin/env python3
"""
Step 3 of the LQI design: fly the LQI controller in the nonlinear 6-DoF model.

The controller code is the same module the ROS 2 node uses
(src/fw_lqi/fw_lqi/lqi_core.py); the gains come from tools/lqi_design.py.
Scenarios, all starting in trimmed level flight at 45 m/s, 60 m, heading north:
  climb : altitude step 60 -> 120 m, wings level
  turn  : heading step 0 -> 90 deg at constant altitude (bank limit 30 deg)

Usage:
  python tools/lqi_sim.py ~/uav-6dof-autopilot
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DT = 0.02
T_END = 90.0
K_HEADING = 1.0                  # rad bank per rad heading error (as in the reference)
MAX_BANK = math.radians(30.0)


def main():
    ref_repo = Path(sys.argv[1]).expanduser()
    sys.path.insert(0, str(ref_repo / "src"))
    sys.path.insert(0, str(ROOT / "src" / "fw_lqi"))
    sys.path.insert(0, str(ROOT / "tools"))
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

    scenarios = {"climb": (120.0, 0.0), "turn": (60.0, math.radians(90.0))}
    print(f"{'scenario':8s} {'alt err':>9s} {'alt dev':>9s} {'hdg err':>9s} {'spd min':>8s} "
          f"{'spd max':>8s} {'bank max':>9s} {'alpha max':>9s} {'elev sat':>8s} {'thr sat':>8s} {'settle 2m':>9s}")
    print(f"{'':8s} {'final m':>9s} {'max m':>9s} {'final deg':>9s} {'m/s':>8s} {'m/s':>8s} "
          f"{'deg':>9s} {'deg':>9s} {'% time':>8s} {'% time':>8s} {'s':>9s}")
    for name, (h_ref, psi_ref) in scenarios.items():
        ctrl = LqiController(k_lon, k_lat, AIRSPEED, trim.alpha, trim.elevator, trim.throttle)
        y = make_state(AIRSPEED, trim.alpha, trim.pitch, 60.0)
        log = []
        for i in range(int(T_END / DT)):
            t = i * DT
            phi, theta, psi = quat.to_euler(y[6:10])
            hdg_err = (psi_ref - psi + math.pi) % (2 * math.pi) - math.pi
            ref = Reference(altitude=h_ref, airspeed=AIRSPEED,
                            roll=float(np.clip(K_HEADING * hdg_err, -MAX_BANK, MAX_BANK)))
            meas = Measurement(u=y[3], v=y[4], w=y[5], p=y[10], q=y[11], r=y[12],
                               phi=phi, theta=theta, altitude=-y[2],
                               airspeed=float(np.linalg.norm(y[3:6])))
            c = ctrl.update(meas, ref, DT)
            log.append((t, -y[2], meas.airspeed, phi, math.atan2(y[5], y[3]), hdg_err,
                        c.elevator, c.throttle))
            ctrl_in = Controls(elevator=c.elevator, aileron=c.aileron, rudder=c.rudder, throttle=c.throttle)
            y = post_step(rk4_step(make_dynamics(constant_controls(ctrl_in)), t, y, DT))

        t, h, spd, phi, alpha, hdg, elev, thr = np.array(log).T
        err = h_ref - h
        outside = np.nonzero(np.abs(err) > 2.0)[0]
        settle = t[outside[-1] + 1] if len(outside) and outside[-1] + 1 < len(t) else 0.0
        elev_sat = 100 * np.mean(np.abs(elev) >= ctrl.limits.elevator - 1e-9)
        thr_sat = 100 * np.mean((thr <= 0.0) | (thr >= 1.0))
        print(f"{name:8s} {err[-1]:9.3f} {np.abs(err).max():9.2f} {math.degrees(hdg[-1]):9.3f} "
              f"{spd.min():8.2f} {spd.max():8.2f} {math.degrees(np.abs(phi).max()):9.2f} "
              f"{math.degrees(alpha.max()):9.2f} {elev_sat:8.1f} {thr_sat:8.1f} {settle:9.1f}")


if __name__ == "__main__":
    main()