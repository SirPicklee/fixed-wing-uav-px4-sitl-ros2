#!/usr/bin/env python3
"""
Controller tracking analysis from a PX4 ULog (setpoint vs. measured).

Inner loop : body rates   vehicle_rates_setpoint  vs vehicle_angular_velocity
Outer loop : attitude     vehicle_attitude_setpoint (q_d) vs vehicle_attitude (q)
Actuation  : normalized torque setpoint (vehicle_torque_setpoint), saturation

Time is measured from takeoff (first vehicle_land_detected.landed == False).

Usage:
  python tools/ulog_tracking.py [log.ulg] [window_end_s ...]
  (default log: newest file in ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log)
"""

import glob
import os
import sys

import numpy as np
from pyulog import ULog

TOPICS = ["vehicle_rates_setpoint", "vehicle_angular_velocity",
          "vehicle_attitude_setpoint", "vehicle_attitude",
          "vehicle_torque_setpoint", "vehicle_land_detected"]


def newest_log():
    pattern = os.path.expanduser(
        "~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg")
    files = glob.glob(pattern)
    if not files:
        sys.exit(f"No ULog files found in {pattern}")
    return max(files, key=os.path.getmtime)


def topic(ulog, name):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == 0:
            return d.data
    return None


def quat_to_roll_pitch(qw, qx, qy, qz):
    """Roll and pitch [rad] of a body(FRD)->NED Hamilton quaternion."""
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx**2 + qy**2))
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    return roll, pitch


def interp_to(t_target, t_src, x_src):
    """Zero-order-hold resample (setpoints are piecewise constant)."""
    idx = np.searchsorted(t_src, t_target, side="right") - 1
    valid = idx >= 0
    out = np.full(t_target.shape, np.nan)
    out[valid] = x_src[idx[valid]]
    return out


def stats(err):
    err = err[np.isfinite(err)]
    return np.sqrt(np.mean(err**2)), np.max(np.abs(err))


def main():
    args = sys.argv[1:]
    path = args.pop(0) if args and args[0].endswith(".ulg") else newest_log()
    edges = [float(a) for a in args] or [10.0, 60.0]
    edges = [0.0] + edges

    ulog = ULog(path, TOPICS)
    land = topic(ulog, "vehicle_land_detected")
    airborne = np.flatnonzero(land["landed"] == 0)
    if airborne.size == 0:
        sys.exit("Vehicle never left the ground in this log.")
    t0 = land["timestamp"][airborne[0]] * 1e-6
    print(f"Log: {os.path.basename(path)}   takeoff at t = {t0:.1f} s (boot time)")

    rsp = topic(ulog, "vehicle_rates_setpoint")
    angv = topic(ulog, "vehicle_angular_velocity")
    att = topic(ulog, "vehicle_attitude")
    asp = topic(ulog, "vehicle_attitude_setpoint")
    trq = topic(ulog, "vehicle_torque_setpoint")

    t_w = angv["timestamp"] * 1e-6 - t0
    t_rsp = rsp["timestamp"] * 1e-6 - t0
    rates = {ax: (np.degrees(angv[f"xyz[{i}]"]),
                  np.degrees(interp_to(t_w, t_rsp, rsp[ax])))
             for i, ax in enumerate(("roll", "pitch", "yaw"))}

    t_a = att["timestamp"] * 1e-6 - t0
    roll, pitch = quat_to_roll_pitch(*(att[f"q[{i}]"] for i in range(4)))
    t_asp = asp["timestamp"] * 1e-6 - t0
    roll_sp, pitch_sp = quat_to_roll_pitch(*(asp[f"q_d[{i}]"] for i in range(4)))
    angles = {"roll": (np.degrees(roll), np.degrees(interp_to(t_a, t_asp, roll_sp))),
              "pitch": (np.degrees(pitch), np.degrees(interp_to(t_a, t_asp, pitch_sp)))}

    t_q = trq["timestamp"] * 1e-6 - t0

    for lo, hi in zip(edges[:-1], edges[1:]):
        print(f"\n=== {lo:.0f} .. {hi:.0f} s after takeoff ===")
        m = (t_w >= lo) & (t_w < hi)
        print("  rates [deg/s]   RMS err   max err   max |meas|")
        for ax, (meas, sp) in rates.items():
            rms, mx = stats(meas[m] - sp[m])
            print(f"    {ax:5s}        {rms:8.2f}  {mx:8.2f}   {np.max(np.abs(meas[m])):8.2f}")
        m = (t_a >= lo) & (t_a < hi)
        print("  attitude [deg]  RMS err   max err   max |meas|")
        for ax, (meas, sp) in angles.items():
            rms, mx = stats(meas[m] - sp[m])
            print(f"    {ax:5s}        {rms:8.2f}  {mx:8.2f}   {np.max(np.abs(meas[m])):8.2f}")
        m = (t_q >= lo) & (t_q < hi)
        sat = [100.0 * np.mean(np.abs(trq[f"xyz[{i}]"][m]) > 0.99) for i in range(3)]
        print(f"  torque setpoint saturated [% of time]: roll {sat[0]:.1f}, "
              f"pitch {sat[1]:.1f}, yaw {sat[2]:.1f}")


if __name__ == "__main__":
    main()