#!/usr/bin/env python3
"""
Print the flown path from a PX4 ULog every few seconds (diagnostic).

Columns: time since the first sample above 55 m, north/east of the EKF
local frame, north/east relative to home from the global position (the
frame the mission waypoints are defined in), altitude, true airspeed and
the navigation state (3 mission, 14 offboard).

Usage:
  python tools/ulog_path.py LOG.ulg [step_s]
"""

import math
import os
import sys

import numpy as np
from pyulog import ULog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ulog_tracking import interp_to, topic  # noqa: E402

EARTH_RADIUS_M = 6378137.0     # same flat-earth conversion as scripts/fly_mission.py


def home_ne(ulog, t):
    """North/east [m] relative to home from vehicle_global_position, at times t."""
    gp = topic(ulog, "vehicle_global_position")
    home = topic(ulog, "home_position")
    # Home at arming (the mission was built from it); PX4 moves home again
    # after landing, so the last logged value is the rollout end, not home.
    first = np.flatnonzero(home["valid_hpos"] == 1)[0]
    lat0, lon0 = home["lat"][first], home["lon"][first]
    tg = gp["timestamp"] * 1e-6
    lat, lon = interp_to(t, tg, gp["lat"]), interp_to(t, tg, gp["lon"])
    north = np.radians(lat - lat0) * EARTH_RADIUS_M
    east = np.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return north, east


def main():
    step = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    ulog = ULog(sys.argv[1], ["vehicle_local_position", "vehicle_land_detected",
                              "airspeed_validated", "vehicle_status",
                              "vehicle_global_position", "home_position"])
    lp = topic(ulog, "vehicle_local_position")
    land = topic(ulog, "vehicle_land_detected")
    t_air = land["timestamp"][np.flatnonzero(land["landed"] == 0)[0]]
    i0 = np.flatnonzero((lp["timestamp"] > t_air) & (-lp["z"] > 55.0))[0]
    t = lp["timestamp"] * 1e-6
    t0 = t[i0]
    grid = np.arange(t0, t[-1], step)
    asp = topic(ulog, "airspeed_validated")
    st = topic(ulog, "vehicle_status")
    cols = {
        "n": interp_to(grid, t, lp["x"]), "e": interp_to(grid, t, lp["y"]),
        "alt": -interp_to(grid, t, lp["z"]),
        "tas": interp_to(grid, asp["timestamp"] * 1e-6, asp["true_airspeed_m_s"]),
        "nav": interp_to(grid, st["timestamp"] * 1e-6, st["nav_state"]),
    }
    cols["hn"], cols["he"] = home_ne(ulog, grid)
    print(f"{os.path.basename(sys.argv[1])}: first above 55 m at boot t = {t0:.1f} s, "
          f"xy resets {int(lp['xy_reset_counter'][-1])}")
    print(f"{'t':>6s} {'local n':>8s} {'local e':>8s} {'home n':>8s} {'home e':>8s} "
          f"{'alt':>6s} {'tas':>5s} {'nav':>4s}")
    for k, tk in enumerate(grid):
        print(f"{tk - t0:6.0f} {cols['n'][k]:8.0f} {cols['e'][k]:8.0f} {cols['hn'][k]:8.0f} "
              f"{cols['he'][k]:8.0f} {cols['alt'][k]:6.1f} {cols['tas'][k]:5.1f} "
              f"{cols['nav'][k]:4.0f}")


if __name__ == "__main__":
    main()