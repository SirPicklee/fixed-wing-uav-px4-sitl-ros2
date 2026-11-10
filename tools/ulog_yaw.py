#!/usr/bin/env python3
"""
Compare the EKF yaw estimate with Gazebo ground truth and show the EKF
magnetometer status flags, to test the hypothesis of a heading jump.

  vehicle_attitude              EKF estimate
  vehicle_attitude_groundtruth  published by PX4 gz_bridge from Gazebo poses
  estimator_status_flags        magnetometer fusion / fault flags

Usage:
  python tools/ulog_yaw.py [log.ulg] t_start t_end [step]
"""

import os
import sys

import numpy as np
from pyulog import ULog

from ulog_tracking import interp_to, newest_log, topic

TOPICS = ["vehicle_attitude", "vehicle_attitude_groundtruth",
          "estimator_status_flags"]
FLAGS = ["cs_mag_3d", "cs_mag_hdg", "cs_mag_fault", "cs_mag_field_disturbed",
         "cs_mag_heading_consistent"]


def yaw_of(d):
    qw, qx, qy, qz = (d[f"q[{i}]"] for i in range(4))
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))


def main():
    args = sys.argv[1:]
    path = args.pop(0) if args and args[0].endswith(".ulg") else newest_log()
    t_start, t_end = float(args[0]), float(args[1])
    step = float(args[2]) if len(args) > 2 else 0.5

    ulog = ULog(path, TOPICS)
    t = np.arange(t_start, t_end + 1e-9, step)

    att = topic(ulog, "vehicle_attitude")
    gt = topic(ulog, "vehicle_attitude_groundtruth")
    flg = topic(ulog, "estimator_status_flags")
    if gt is None:
        sys.exit("vehicle_attitude_groundtruth not in log")

    yaw_ekf = np.degrees(interp_to(t, att["timestamp"] * 1e-6, yaw_of(att)))
    yaw_gt = np.degrees(interp_to(t, gt["timestamp"] * 1e-6, yaw_of(gt)))
    err = (yaw_ekf - yaw_gt + 180.0) % 360.0 - 180.0
    resets = interp_to(t, att["timestamp"] * 1e-6, att["quat_reset_counter"])
    flags = {f: interp_to(t, flg["timestamp"] * 1e-6, flg[f]) for f in FLAGS}

    print(f"Log: {os.path.basename(path)}   (deg; flags 1 = true)")
    print("   t [s]  yaw_ekf   yaw_gt  yaw_err  resets  " +
          "  ".join(f[3:] for f in FLAGS))
    for i, ti in enumerate(t):
        fl = "  ".join(f"{int(flags[f][i]) if np.isfinite(flags[f][i]) else '-':>{len(f) - 3}}"
                       for f in FLAGS)
        print(f"{ti:8.1f} {yaw_ekf[i]:8.2f} {yaw_gt[i]:8.2f} {err[i]:8.2f} "
              f"{resets[i]:7.0f}  {fl}")


if __name__ == "__main__":
    main()