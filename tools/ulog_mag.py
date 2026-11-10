#!/usr/bin/env python3
"""
Magnetometer consistency check against Gazebo ground truth.

The Earth field is constant in the NED frame. Rotating the body-frame
magnetometer reading (sensor_mag, FRD) into NED with the ground-truth
attitude (vehicle_attitude_groundtruth) must therefore give the same vector
at every heading. If it changes with heading, the sensor data is in a
different frame than PX4 assumes.

Usage:
  python tools/ulog_mag.py [log.ulg] [step]
"""

import os
import sys

import numpy as np
from pyulog import ULog

from ulog_tracking import interp_to, newest_log, topic


def quat_to_dcm(qw, qx, qy, qz):
    """Body(FRD) -> NED rotation matrices, shape (N, 3, 3)."""
    return np.stack([
        np.stack([1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)], -1),
        np.stack([2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)], -1),
        np.stack([2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)], -1),
    ], -2)


def main():
    args = sys.argv[1:]
    path = args.pop(0) if args and args[0].endswith(".ulg") else newest_log()
    step = float(args[0]) if args else 5.0

    ulog = ULog(path, ["sensor_mag", "vehicle_attitude_groundtruth"])
    mag = topic(ulog, "sensor_mag")
    gt = topic(ulog, "vehicle_attitude_groundtruth")

    t_mag = mag["timestamp"] * 1e-6
    t = np.arange(max(t_mag[0], gt["timestamp"][0] * 1e-6), t_mag[-1], step)
    b_body = np.stack([interp_to(t, t_mag, mag[c]) for c in ("x", "y", "z")], -1)
    q = [interp_to(t, gt["timestamp"] * 1e-6, gt[f"q[{i}]"]) for i in range(4)]
    dcm = quat_to_dcm(*q)
    b_ned = np.einsum("nij,nj->ni", dcm, b_body)
    yaw = np.degrees(np.arctan2(dcm[:, 1, 0], dcm[:, 0, 0]))
    roll = np.degrees(np.arctan2(dcm[:, 2, 1], dcm[:, 2, 2]))

    print(f"Log: {os.path.basename(path)}   (gauss; heading/roll from ground truth)")
    print("   t [s]  heading    roll    B_N      B_E      B_D     |B|   decl[deg]")
    for i, ti in enumerate(t):
        n, e, d = b_ned[i]
        print(f"{ti:8.1f} {yaw[i]:8.1f} {roll[i]:7.1f} {n:8.4f} {e:8.4f} {d:8.4f} "
              f"{np.linalg.norm(b_ned[i]):7.4f} {np.degrees(np.arctan2(e, n)):8.1f}")
    print("\nIf the frame is right, B_N/B_E/B_D and the declination stay constant.")


if __name__ == "__main__":
    main()