#!/usr/bin/env python3
"""
Summarize control-surface and throttle values seen by the fw_aero plugin
during a PX4 actuator_test sequence (ground check, no flight).

Each channel is split into segments of (nearly) constant value: a new
segment starts when the value moves by more than the channel's tolerance.
Segments shorter than MIN_DURATION are dropped, so sensor-noise flicker on
the ground does not show up as separate segments.

Usage:
  python tools/check_actuators.py /tmp/tb2_flight.csv [t_start]
"""

import sys

import numpy as np

MIN_DURATION = 0.5  # s


def segments(t, x, tol):
    """Return (t_start, t_end, median) for runs of x that stay within tol."""
    out = []
    start = 0
    for i in range(1, len(x) + 1):
        if i == len(x) or abs(x[i] - np.median(x[start:i])) > tol:
            if t[i - 1] - t[start] >= MIN_DURATION:
                out.append((t[start], t[i - 1], float(np.median(x[start:i]))))
            start = i
    return out


def main():
    d = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
    t_start = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    m = d["t"] >= t_start
    t = d["t"][m]
    channels = (("aileron", np.degrees(1.0), "deg", 0.2),
                ("elevator", np.degrees(1.0), "deg", 0.2),
                ("rudder", np.degrees(1.0), "deg", 0.2),
                ("throttle", 1.0, "", 0.01))
    for name, scale, unit, tol in channels:
        print(f"{name}:")
        for t0, t1, v in segments(t, d[name][m] * scale, tol):
            print(f"  {v:+7.2f} {unit:3s}  t = {t0:6.1f} .. {t1:6.1f} s")


if __name__ == "__main__":
    main()