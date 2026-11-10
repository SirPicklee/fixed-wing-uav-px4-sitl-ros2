#!/usr/bin/env python3
"""
Analyze the Gazebo glide test log written by FixedWingAeroSystem.

Glide angle, airspeed and L/D are computed from the world-frame velocity
logged by Gazebo, not from the plugin's own air-data outputs, so the check
covers the full chain (frames, wrench application, rigid-body integration).

Usage:
  python tools/analyze_glide.py /tmp/glide_test.csv [settle_start_s]
"""

import sys

import numpy as np


def main():
    path = sys.argv[1]
    settle_start = float(sys.argv[2]) if len(sys.argv) > 2 else 240.0
    d = np.genfromtxt(path, delimiter=",", names=True)

    t = d["t"]
    v_h = np.hypot(d["vx"], d["vy"])        # horizontal speed (ENU)
    sink = -d["vz"]                          # positive down
    speed = np.sqrt(v_h**2 + d["vz"]**2)
    gamma = np.degrees(np.arctan2(sink, v_h))

    print(f"Log: {len(t)} samples, t = {t[0]:.1f} .. {t[-1]:.1f} s")
    print("   t [s]   alt [m]   V [m/s]   gamma [deg]")
    for tt in (0, 20, 40, 60, 100, 150, 200, 240, 300):
        i = int(np.argmin(np.abs(t - tt)))
        print(f"  {t[i]:6.1f}  {d['z'][i]:8.1f}  {speed[i]:8.2f}  {gamma[i]:10.2f}")

    m = t >= settle_start
    print(f"\nMean over t >= {settle_start:.0f} s "
          f"(altitude ~{d['z'][m].mean():.0f} m):")
    print(f"  airspeed  = {speed[m].mean():.3f} m/s")
    print(f"  gamma     = {gamma[m].mean():.3f} deg")
    print(f"  sink rate = {sink[m].mean():.3f} m/s")
    print(f"  L/D       = {v_h[m].mean() / sink[m].mean():.3f}")
    print(f"  alpha     = {np.degrees(d['alpha'][m]).mean():.3f} deg (plugin)")
    print(f"  max |beta| over whole run = {np.degrees(np.abs(d['beta'])).max():.2e} deg")
    print(f"  max lateral drift |y|     = {np.abs(d['y']).max():.2e} m")


if __name__ == "__main__":
    main()