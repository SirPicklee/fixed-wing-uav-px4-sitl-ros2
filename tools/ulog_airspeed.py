#!/usr/bin/env python3
"""
Trace the airspeed chain in a PX4 ULog and compare it with the true airspeed
logged by the fw_aero plugin (independent ground truth from Gazebo).

  differential_pressure  ->  airspeed_validated  ->  TECS
  (sensor_airspeed_sim)      (airspeed_selector)

Also prints the EKF ground speed, which sensor_airspeed_sim uses to
synthesize the differential pressure.

Usage:
  python tools/ulog_airspeed.py [log.ulg] t_start t_end [step] [plugin.csv]
  Times are PX4 boot time; with lockstep the plugin's sim time uses the
  same clock to within a few tens of milliseconds.
"""

import os
import sys

import numpy as np
from pyulog import ULog

from ulog_tracking import interp_to, newest_log, topic

TOPICS = ["differential_pressure", "airspeed_validated", "tecs_status",
          "vehicle_local_position"]
SOURCES = {-1: "off", 0: "gnd-wind", 1: "sensor1", 2: "sensor2", 3: "sensor3",
           4: "synth"}


def main():
    args = sys.argv[1:]
    path = args.pop(0) if args and args[0].endswith(".ulg") else newest_log()
    csv = args.pop() if args and args[-1].endswith(".csv") else None
    t_start, t_end = float(args[0]), float(args[1])
    step = float(args[2]) if len(args) > 2 else 0.5

    ulog = ULog(path, TOPICS)
    t = np.arange(t_start, t_end + 1e-9, step)

    def resample(name, field):
        d = topic(ulog, name)
        return interp_to(t, d["timestamp"] * 1e-6, d[field])

    dp = resample("differential_pressure", "differential_pressure_pa")
    src = resample("airspeed_validated", "airspeed_source")
    cols = {
        "dp[Pa]": dp,
        "ias_dp": np.sign(dp) * np.sqrt(2.0 * np.abs(dp) / 1.225),
        "tas_val": resample("airspeed_validated", "true_airspeed_m_s"),
        "cas_gmw": resample("airspeed_validated", "calibrated_ground_minus_wind_m_s"),
        "tecs_tas": resample("tecs_status", "true_airspeed_filtered"),
        "ekf_gs": np.hypot(resample("vehicle_local_position", "vx"),
                           resample("vehicle_local_position", "vy")),
    }
    if csv:
        d = np.genfromtxt(csv, delimiter=",", names=True)
        cols["plugin_V"] = interp_to(t, d["t"], d["airspeed"])

    print(f"Log: {os.path.basename(path)}   (m/s unless noted)")
    print("   t [s]  source  " + "".join(f"{k:>9s}" for k in cols))
    for i, ti in enumerate(t):
        s = SOURCES.get(int(src[i]), "?") if np.isfinite(src[i]) else "-"
        print(f"{ti:8.1f}  {s:8s}" + "".join(f"{cols[k][i]:9.2f}" for k in cols))


if __name__ == "__main__":
    main()