#!/usr/bin/env python3
"""
Compare the LQI shadow commands with what PX4 actually commanded.

In shadow mode the LQI node computes surface and throttle commands on the
live state while PX4 (TECS + NPFG) flies the aircraft. Before giving the
LQI control authority, this checks on real PX4 data that its commands have
the right signs and magnitudes: in the same turns and climbs both
controllers should deflect the surfaces the same way.

The node logs time from its takeover (first sample above engage_altitude
in mission mode); the ULog is aligned to the first local-position sample
above the same altitude after takeoff.

Usage:
  python tools/analyze_shadow.py /tmp/lqi_controller.csv [log.ulg] [engage_alt]
"""

import os
import sys

import numpy as np
from pyulog import ULog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ulog_tracking import interp_to, newest_log, topic  # noqa: E402

MAX_AIL, MAX_ELEV, MAX_RUD = 20.0, 20.0, 15.0     # deg at normalized +-1


def main():
    node = np.genfromtxt(sys.argv[1], delimiter=",", names=True, dtype=None, encoding=None)
    path = sys.argv[2] if len(sys.argv) > 2 else newest_log()
    engage_alt = float(sys.argv[3]) if len(sys.argv) > 3 else 55.0

    ulog = ULog(path, ["vehicle_local_position", "actuator_servos", "actuator_motors",
                       "vehicle_land_detected", "airspeed_validated"])
    lpos = topic(ulog, "vehicle_local_position")
    land = topic(ulog, "vehicle_land_detected")
    t_air = land["timestamp"][np.flatnonzero(land["landed"] == 0)[0]]
    above = np.flatnonzero((lpos["timestamp"] > t_air) & (-lpos["z"] > engage_alt))
    t0 = lpos["timestamp"][above[0]] * 1e-6
    t = node["t"] + t0

    sv, mo = topic(ulog, "actuator_servos"), topic(ulog, "actuator_motors")
    ts, tm = sv["timestamp"] * 1e-6, mo["timestamp"] * 1e-6
    c = [interp_to(t, ts, sv[f"control[{i}]"]) for i in range(4)]
    px4 = {
        "aileron": 0.5 * (c[1] - c[0]) * MAX_AIL,
        "elevator": -c[2] * MAX_ELEV,
        "rudder": -c[3] * MAX_RUD,
        "throttle": interp_to(t, tm, mo["control[0]"]),
    }
    dt = np.diff(node["t"])
    print(f"Node log : {len(node)} samples over {node['t'][-1]:.1f} s, "
          f"median {1e3 * np.median(dt):.1f} ms, max gap {1e3 * dt.max():.0f} ms")
    print(f"ULog     : {os.path.basename(path)}, takeover aligned at t = {t0:.1f} s")
    print("Legs     : " + ", ".join(
        f"{k + 1} from {node['t'][node['leg'] == k][0]:.0f} s"
        for k in range(int(node['leg'].max()) + 1) if np.any(node['leg'] == k)))
    print(f"Guidance : roll_ref {node['roll_ref'].min():+.1f} .. {node['roll_ref'].max():+.1f} deg, "
          f"|cross-track| max {np.abs(node['cross_track']).max():.0f} m")
    print()
    print(f"  {'channel':9s} {'LQI min':>8s} {'LQI max':>8s} {'PX4 min':>8s} {'PX4 max':>8s} "
          f"{'corr':>6s} {'at min %':>8s} {'at max %':>8s}")
    limits = {"aileron": MAX_AIL, "elevator": MAX_ELEV, "rudder": MAX_RUD}
    for name, ref in px4.items():
        lqi = node[name]
        ok = np.isfinite(ref)
        corr = np.corrcoef(lqi[ok], ref[ok])[0, 1]
        lo, hi = (-limits[name], limits[name]) if name in limits else (0.0, 1.0)
        print(f"  {name:9s} {lqi.min():8.2f} {lqi.max():8.2f} {np.nanmin(ref):8.2f} "
              f"{np.nanmax(ref):8.2f} {corr:6.2f} {100 * np.mean(lqi <= lo + 1e-6):8.1f} "
              f"{100 * np.mean(lqi >= hi - 1e-6):8.1f}")

    # The LQI integrators get no feedback in shadow mode: a steady airspeed
    # offset winds xi_v up to its clamp (30 m) in 30 / |V_ref - V| seconds.
    asp = topic(ulog, "airspeed_validated")
    tas = interp_to(t, asp["timestamp"] * 1e-6, asp["true_airspeed_m_s"])
    print()
    print(f"True airspeed while shadowing: mean {np.nanmean(tas):.2f} m/s, "
          f"median {np.nanmedian(tas):.2f} m/s (LQI reference 45.00 m/s)")
    err = 45.0 - np.nanmean(tas)
    print(f"Mean airspeed error {err:+.2f} m/s -> xi_v reaches its clamp after "
          f"{30.0 / max(abs(err), 1e-3):.0f} s")


if __name__ == "__main__":
    main()