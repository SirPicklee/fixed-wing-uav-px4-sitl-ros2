#!/usr/bin/env python3
"""
Check the PX4 -> ROS 2 bridge: compare the CSV written by fw_telemetry's
telemetry_logger with PX4's own ULog of the same flight.

The two logs use different clocks (ROS: uXRCE-DDS synchronised Unix time,
relative to the first sample; ULog: time since PX4 boot). In lockstep SITL
the PX4 clock follows simulation time while the uXRCE-DDS time sync follows
the wall clock, so the mapping can have a scale factor as well as an offset.
It is estimated as an affine map from two events seen in both logs: the
first climb and the last descent through 5 m. Position and attitude are
then compared sample by sample.

Usage:
  python tools/compare_ros_ulog.py /tmp/px4_telemetry.csv [log.ulg]
"""

import os
import sys

import numpy as np
from pyulog import ULog

from ulog_tracking import interp_to, newest_log, quat_to_roll_pitch, topic


def crossing_times(t, z_ned, height=5.0):
    """Return (first climb, last descent) times through `height` m."""
    above = -z_ned > height
    idx = np.flatnonzero(above)
    return t[idx[0]], t[idx[-1]]


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    csv = sys.argv[1]
    path = sys.argv[2] if len(sys.argv) > 2 else newest_log()
    r = np.genfromtxt(csv, delimiter=",", names=True)
    ulog = ULog(path, ["vehicle_local_position", "vehicle_attitude"])
    lp = topic(ulog, "vehicle_local_position")
    att = topic(ulog, "vehicle_attitude")
    t_lp = lp["timestamp"] * 1e-6
    t_att = att["timestamp"] * 1e-6

    r_up, r_down = crossing_times(r["t"], r["z"])
    u_up, u_down = crossing_times(t_lp, lp["z"])
    scale = (u_down - u_up) / (r_down - r_up)
    offset = u_up - scale * r_up
    t = scale * r["t"] + offset
    n_total, dur_total = len(r), r["t"][-1]

    # PX4 logs from arming to disarming (SDLOG_MODE default), while the ROS 2
    # node keeps logging: compare only the time span covered by both logs.
    t_end = min(t_lp[-1], t_att[-1])
    keep = (t >= max(t_lp[0], t_att[0])) & (t <= t_end)
    r, t = r[keep], t[keep]
    print(f"ROS log : {os.path.basename(csv)}, {n_total} samples over {dur_total:.1f} s")
    print(f"ULog    : {os.path.basename(path)}")
    print(f"Airborne above 5 m: ROS {r_down - r_up:.2f} s, ULog {u_down - u_up:.2f} s")
    print(f"Clock map: ulog = {scale:.5f} * ros + {offset:.3f} s")
    print(f"Compared : t_ulog = {t[0]:.1f} .. {t[-1]:.1f} s (end of ULog), "
          f"{len(t)} samples")

    dt = np.diff(r["t"])
    print(f"ROS sample interval: median {1e3 * np.median(dt):.1f} ms, "
          f"max gap {1e3 * dt.max():.0f} ms "
          f"(effective rate {1.0 / np.median(dt):.1f} Hz)")

    roll, pitch = quat_to_roll_pitch(*(att[f"q[{i}]"] for i in range(4)))
    qw, qx, qy, qz = (att[f"q[{i}]"] for i in range(4))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))

    # Short time history of the position and yaw differences, to tell a
    # constant offset, a step (estimator reset) and a slow drift apart.
    x_u = interp_to(t, t_lp, lp["x"])
    y_u = interp_to(t, t_lp, lp["y"])
    yaw_u = np.degrees(interp_to(t, t_att, yaw))
    print("\n  t_ulog [s]   x_ulog    dx      y_ulog    dy     dyaw [deg]")
    for i in range(0, len(t), max(1, len(t) // 20)):
        print(f"  {t[i]:9.1f} {x_u[i]:9.1f} {r['x'][i] - x_u[i]:6.2f} "
              f"{y_u[i]:9.1f} {r['y'][i] - y_u[i]:6.2f} "
              f"{wrap_deg(r['yaw'][i] - yaw_u[i]):8.2f}")

    print("\n  signal    RMS diff    max |diff|   (ROS 2 minus ULog)")
    for name, ulg_t, ulg_v, ros_v, unit, wrap in (
            ("x", t_lp, lp["x"], r["x"], "m", False),
            ("y", t_lp, lp["y"], r["y"], "m", False),
            ("z", t_lp, lp["z"], r["z"], "m", False),
            ("roll", t_att, np.degrees(roll), r["roll"], "deg", False),
            ("pitch", t_att, np.degrees(pitch), r["pitch"], "deg", False),
            ("yaw", t_att, np.degrees(yaw), r["yaw"], "deg", True)):
        ref = interp_to(t, ulg_t, ulg_v)
        d = ros_v - ref
        if wrap:
            d = wrap_deg(d)
        d = d[np.isfinite(d)]
        print(f"  {name:6s} {np.sqrt(np.mean(d**2)):9.4f} {unit:3s} "
              f"{np.max(np.abs(d)):9.4f} {unit}")


if __name__ == "__main__":
    main()