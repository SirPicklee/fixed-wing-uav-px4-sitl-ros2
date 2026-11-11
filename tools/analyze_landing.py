#!/usr/bin/env python3
"""
Analyze a landing from the fw_aero plugin log.

Touchdown = last transition of the CG from above to below 0.10 m over its
resting height on the gear. Rollout ends when ground speed drops below
1 m/s (or at the end of the log).

Prints touchdown point relative to the land waypoint (home), touchdown
sink rate and airspeed, and the rollout distance and time.

Usage:
  python tools/analyze_landing.py /tmp/tb2_flight.csv
"""

import sys

import numpy as np

GEAR_HEIGHT = 1.15   # m, CG height when resting on the wheels
MARGIN = 0.10


def main():
    d = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
    t, x, y, z = d["t"], d["x"], d["y"], d["z"]
    gs = np.hypot(d["vx"], d["vy"])

    on_ground = z < GEAR_HEIGHT + MARGIN
    downs = np.flatnonzero(on_ground[1:] & ~on_ground[:-1]) + 1
    if downs.size == 0:
        print("No touchdown found in this log.")
        return
    i_td = downs[-1]
    stop = np.flatnonzero((np.arange(len(t)) > i_td) & (gs < 1.0))
    i_end = stop[0] if stop.size else len(t) - 1

    # sink rate: mean vertical speed over the last 0.5 s before touchdown
    m = (t > t[i_td] - 0.5) & (t <= t[i_td])
    print(f"Touchdown        : t = {t[i_td]:6.1f} s")
    print(f"  position       = x {x[i_td]:+7.1f} m (along runway), "
          f"y {y[i_td]:+6.2f} m (from centreline), relative to home")
    print(f"  airspeed       = {d['airspeed'][i_td]:6.1f} m/s")
    print(f"  sink rate      = {-d['vz'][m].mean():6.2f} m/s")
    print(f"  pitch rate     = {np.degrees(d['q'][i_td]):6.1f} deg/s, "
          f"alpha = {np.degrees(d['alpha'][i_td]):5.1f} deg")
    status = "stopped" if stop.size else "still rolling at end of log"
    print(f"Rollout          : {x[i_end] - x[i_td]:6.0f} m in {t[i_end] - t[i_td]:5.0f} s ({status})")
    print(f"  end position   = x {x[i_end]:+7.1f} m, max |y| = "
          f"{np.abs(y[i_td:i_end + 1]).max():.2f} m")


if __name__ == "__main__":
    main()