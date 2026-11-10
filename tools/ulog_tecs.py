#!/usr/bin/env python3
"""
Print the TECS (total energy control) picture from a PX4 ULog as a time
table: navigation mode, altitude and airspeed setpoints vs. measured, and
the pitch / throttle TECS commands.

Time is seconds since boot (same clock as the PX4 console).

Usage:
  python tools/ulog_tecs.py [log.ulg] t_start t_end [step]
"""

import os
import sys

import numpy as np
from pyulog import ULog

from ulog_tracking import interp_to, newest_log, topic

TOPICS = ["tecs_status", "vehicle_global_position", "vehicle_status"]

NAV_STATES = {3: "MISSION", 4: "LOITER", 5: "RTL", 17: "TAKEOFF", 18: "LAND"}


def main():
    args = sys.argv[1:]
    path = args.pop(0) if args and args[0].endswith(".ulg") else newest_log()
    t_start, t_end = float(args[0]), float(args[1])
    step = float(args[2]) if len(args) > 2 else 0.5

    ulog = ULog(path, TOPICS)
    t = np.arange(t_start, t_end + 1e-9, step)

    def resample(name, field, scale=1.0):
        d = topic(ulog, name)
        return scale * interp_to(t, d["timestamp"] * 1e-6, d[field])

    nav = resample("vehicle_status", "nav_state")
    cols = {
        "alt_sp": resample("tecs_status", "altitude_sp"),
        "alt_ref": resample("tecs_status", "altitude_reference"),
        "alt": resample("vehicle_global_position", "alt"),
        "hdot_sp": resample("tecs_status", "height_rate_setpoint"),
        "hdot": resample("tecs_status", "height_rate"),
        "eas_sp": resample("tecs_status", "equivalent_airspeed_sp"),
        "tas": resample("tecs_status", "true_airspeed_filtered"),
        "th_sp": resample("tecs_status", "pitch_sp_rad", np.degrees(1.0)),
        "thr_sp": resample("tecs_status", "throttle_sp"),
        "undersp": resample("tecs_status", "underspeed_ratio"),
    }

    print(f"Log: {os.path.basename(path)}   (m, m/s, deg)")
    print("   t [s]  mode    " + "".join(f"{k:>8s}" for k in cols))
    for i, ti in enumerate(t):
        mode = NAV_STATES.get(int(nav[i]), str(int(nav[i]))) if np.isfinite(nav[i]) else "-"
        print(f"{ti:8.1f}  {mode:7s} " + "".join(f"{cols[k][i]:8.2f}" for k in cols))


if __name__ == "__main__":
    main()