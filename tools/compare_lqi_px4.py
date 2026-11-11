#!/usr/bin/env python3
"""
Compare LQI (offboard) and PX4 TECS/NPFG on the same TB2 mission legs.

Both flights fly the tb2 mission; in the LQI flight the fw_lqi node takes
over above 55 m and flies the four legs, in the PX4 flight PX4 flies them.
Each ULog is scored the same way: from the first sample above 55 m, the
position is replayed through the L1 guidance leg logic (same waypoints and
leg-switch rule) to assign each sample to a leg; the window ends when that
logic reports the last leg done. Lateral error is the distance to the
planned path (the polyline start -> WP1 -> ... -> WP4), which does not
favour either way of turning a corner. Altitude and airspeed errors
exclude the first SETTLE seconds of each leg, where the reference steps.

Usage:
  python tools/compare_lqi_px4.py PX4_FLIGHT.ulg LQI_FLIGHT.ulg
"""

import os
import sys

import numpy as np
from pyulog import ULog

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "src", "fw_lqi"))
from fw_lqi.guidance import L1Guidance  # noqa: E402
from ulog_path import home_ne  # noqa: E402
from ulog_tracking import interp_to, quat_to_roll_pitch, topic  # noqa: E402

WAYPOINTS = [(1500.0, 3000.0, 120.0), (3000.0, 0.0, 120.0),
             (1500.0, -3500.0, 100.0), (0.0, -3200.0, 80.0)]
ENGAGE_ALT = 55.0
V_REF = 45.0
SETTLE = 20.0
MAX_ELEV = 20.0


def path_distance(n, e, points):
    """Distance from (n, e) to the nearest point of a polyline."""
    best = np.full(n.shape, np.inf)
    for (an, ae), (bn, be) in zip(points[:-1], points[1:]):
        dn, de = bn - an, be - ae
        s = np.clip(((n - an) * dn + (e - ae) * de) / (dn * dn + de * de), 0.0, 1.0)
        best = np.minimum(best, np.hypot(n - an - s * dn, e - ae - s * de))
    return best


def score(path):
    """Return per-sample arrays for one flight over the scored window."""
    ulog = ULog(path, ["vehicle_local_position", "vehicle_land_detected", "vehicle_attitude",
                       "airspeed_validated", "actuator_motors", "actuator_servos",
                       "vehicle_global_position", "home_position"])
    lp = topic(ulog, "vehicle_local_position")
    land = topic(ulog, "vehicle_land_detected")
    t_air = land["timestamp"][np.flatnonzero(land["landed"] == 0)[0]]
    i0 = np.flatnonzero((lp["timestamp"] > t_air) & (-lp["z"] > ENGAGE_ALT))[0]
    t = lp["timestamp"][i0:] * 1e-6
    n, e = home_ne(ulog, t)          # mission frame: immune to EKF local-origin shifts
    alt = -lp["z"][i0:]
    vn, ve = lp["vx"][i0:], lp["vy"][i0:]

    guide = L1Guidance(WAYPOINTS)
    guide.reset(n[0], e[0])
    leg, alt_ref = [], []
    for k in range(len(t)):
        g = guide.update(n[k], e[k], vn[k], ve[k])
        if g.done:
            break
        leg.append(g.leg)
        alt_ref.append(g.altitude)
    m = len(leg)
    t = t[:m]
    xt = path_distance(n[:m], e[:m], [(n[0], e[0])] + [wp[:2] for wp in WAYPOINTS])

    att = topic(ulog, "vehicle_attitude")
    ta = att["timestamp"] * 1e-6
    roll, _ = quat_to_roll_pitch(*(interp_to(t, ta, att[f"q[{i}]"]) for i in range(4)))
    asp = topic(ulog, "airspeed_validated")
    tas = interp_to(t, asp["timestamp"] * 1e-6, asp["true_airspeed_m_s"])
    mo = topic(ulog, "actuator_motors")
    thr = interp_to(t, mo["timestamp"] * 1e-6, mo["control[0]"])
    sv = topic(ulog, "actuator_servos")
    elev = -interp_to(t, sv["timestamp"] * 1e-6, sv["control[2]"]) * MAX_ELEV
    return {"name": os.path.basename(path), "t": t - t[0], "leg": np.array(leg),
            "xt": xt, "eh": np.array(alt_ref) - alt[:m], "ev": tas - V_REF,
            "roll": np.degrees(roll), "thr": thr, "elev": elev}
def rms(x):
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x**2)))


def settled(d):
    """Mask of samples more than SETTLE s after their leg started."""
    mask = np.zeros(len(d["t"]), dtype=bool)
    for k in np.unique(d["leg"]):
        sel = d["leg"] == k
        mask |= sel & (d["t"] >= d["t"][sel][0] + SETTLE)
    return mask


def main():
    flights = [("PX4", score(sys.argv[1])), ("LQI", score(sys.argv[2]))]
    for label, d in flights:
        print(f"{label}: {d['name']}, legs flown in {d['t'][-1]:.1f} s")
    print()
    rows = [
        ("legs time [s]", lambda d, s: d["t"][-1]),
        ("path deviation RMS [m]", lambda d, s: rms(d["xt"])),
        ("path deviation max [m]", lambda d, s: np.nanmax(np.abs(d["xt"]))),
        ("altitude err RMS, settled [m]", lambda d, s: rms(d["eh"][s])),
        ("altitude err max, settled [m]", lambda d, s: np.nanmax(np.abs(d["eh"][s]))),
        ("airspeed err RMS, settled [m/s]", lambda d, s: rms(d["ev"][s])),
        ("airspeed min [m/s]", lambda d, s: np.nanmin(d["ev"]) + V_REF),
        ("max |bank| [deg]", lambda d, s: np.nanmax(np.abs(d["roll"]))),
        ("mean throttle [-]", lambda d, s: np.nanmean(d["thr"])),
        ("elevator RMS [deg]", lambda d, s: rms(d["elev"])),
    ]
    print(f"  {'metric':32s} {'PX4':>9s} {'LQI':>9s}")
    for name, f in rows:
        vals = [f(d, settled(d)) for _, d in flights]
        print(f"  {name:32s} {vals[0]:9.2f} {vals[1]:9.2f}")
    print()
    print(f"  {'leg':4s} " + " ".join(f"{lab + ' path rms':>13s} {lab + ' h rms':>10s}"
                                      for lab, _ in flights))
    for k in range(len(WAYPOINTS)):
        cells = []
        for _, d in flights:
            s = settled(d) & (d["leg"] == k)
            cells.append(f"{rms(d['xt'][d['leg'] == k]):13.1f} {rms(d['eh'][s]):10.2f}")
        print(f"  {k + 1:<4d} " + " ".join(cells))


if __name__ == "__main__":
    main()        