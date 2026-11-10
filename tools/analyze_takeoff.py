#!/usr/bin/env python3
"""
Analyze a runway takeoff from the fw_aero plugin log.

Ground roll start = first sample with throttle > 0.05.
Liftoff           = first sample after that with the CG more than 0.10 m
                    above its resting height on the gear.

Prints ground roll distance/time, liftoff airspeed, lateral deviation from
the runway centreline, and a short time history of the climb-out.

Usage:
  python tools/analyze_takeoff.py /tmp/tb2_flight.csv
"""

import sys

import numpy as np

GEAR_HEIGHT = 1.15   # m, CG height when resting on the wheels
LIFTOFF_MARGIN = 0.10


def main():
    d = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
    t, x, y, z = d["t"], d["x"], d["y"], d["z"]

    roll = np.flatnonzero(d["throttle"] > 0.05)
    if roll.size == 0:
        print("No throttle command found: no takeoff in this log.")
        return
    i0 = roll[0]
    air = np.flatnonzero((z > GEAR_HEIGHT + LIFTOFF_MARGIN) & (np.arange(len(t)) >= i0))
    if air.size == 0:
        print(f"Ground roll started at t = {t[i0]:.1f} s but no liftoff in the log.")
        print(f"  max airspeed on ground = {d['airspeed'][i0:].max():.1f} m/s, "
              f"distance = {x[-1] - x[i0]:.0f} m")
        return
    i1 = air[0]

    print(f"Ground roll start : t = {t[i0]:6.1f} s, x = {x[i0]:6.1f} m")
    print(f"Liftoff           : t = {t[i1]:6.1f} s, x = {x[i1]:6.1f} m")
    print(f"  ground roll     = {x[i1] - x[i0]:6.1f} m in {t[i1] - t[i0]:5.1f} s")
    print(f"  liftoff airspeed= {d['airspeed'][i1]:6.1f} m/s")
    print(f"  max |y| on roll = {np.abs(y[i0:i1 + 1]).max():6.2f} m (runway half-width 15 m)")
    print(f"  max elevator on roll = {np.degrees(d['elevator'][i0:i1 + 1]).min():+.1f} deg "
          f"(negative = nose up)")

    print("\nClimb-out:")
    print("   t-t_lo [s]  alt [m]  V [m/s]  alpha [deg]  q [deg/s]  p [deg/s]")
    for dt in (0, 2, 5, 10, 15, 20, 30, 45, 60):
        i = int(np.argmin(np.abs(t - (t[i1] + dt))))
        if t[i] < t[i1] + dt - 1.0:
            break
        print(f"  {t[i] - t[i1]:9.1f}  {z[i]:7.1f}  {d['airspeed'][i]:7.1f}  "
              f"{np.degrees(d['alpha'][i]):10.1f}  {np.degrees(d['q'][i]):9.1f}  "
              f"{np.degrees(d['p'][i]):9.1f}")


if __name__ == "__main__":
    main()