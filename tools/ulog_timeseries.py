#!/usr/bin/env python3
"""
Print setpoint vs. measured attitude, rates and torque from a PX4 ULog as a
time table, to see which control loop drives an event.

Time is seconds since boot (same clock as the PX4 console).

Usage:
  python tools/ulog_timeseries.py [log.ulg] t_start t_end [step]
"""

import os
import sys

import numpy as np
from pyulog import ULog

from ulog_tracking import interp_to, newest_log, quat_to_roll_pitch, topic

TOPICS = ["vehicle_rates_setpoint", "vehicle_angular_velocity",
          "vehicle_attitude_setpoint", "vehicle_attitude",
          "vehicle_torque_setpoint", "vehicle_local_position"]


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

    att = topic(ulog, "vehicle_attitude")
    roll, pitch = quat_to_roll_pitch(*(att[f"q[{i}]"] for i in range(4)))
    t_att = att["timestamp"] * 1e-6
    asp = topic(ulog, "vehicle_attitude_setpoint")
    roll_sp, pitch_sp = quat_to_roll_pitch(*(asp[f"q_d[{i}]"] for i in range(4)))
    t_asp = asp["timestamp"] * 1e-6

    cols = {
        "phi_sp": np.degrees(interp_to(t, t_asp, roll_sp)),
        "phi": np.degrees(interp_to(t, t_att, roll)),
        "p_sp": resample("vehicle_rates_setpoint", "roll", np.degrees(1.0)),
        "p": resample("vehicle_angular_velocity", "xyz[0]", np.degrees(1.0)),
        "tau_x": resample("vehicle_torque_setpoint", "xyz[0]"),
        "th_sp": np.degrees(interp_to(t, t_asp, pitch_sp)),
        "theta": np.degrees(interp_to(t, t_att, pitch)),
        "q": resample("vehicle_angular_velocity", "xyz[1]", np.degrees(1.0)),
        "tau_y": resample("vehicle_torque_setpoint", "xyz[1]"),
        "alt": resample("vehicle_local_position", "z", -1.0),
    }

    print(f"Log: {os.path.basename(path)}   (angles deg, rates deg/s, torque normalized)")
    print("   t [s] " + "".join(f"{k:>8s}" for k in cols))
    for i, ti in enumerate(t):
        print(f"{ti:8.1f} " + "".join(f"{cols[k][i]:8.2f}" for k in cols))


if __name__ == "__main__":
    main()