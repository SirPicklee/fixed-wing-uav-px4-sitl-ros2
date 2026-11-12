#!/usr/bin/env python3
"""
Summarize a Monte Carlo campaign written by scripts/monte_carlo.py.

Every successful flight is scored with the same metrics as
tools/compare_lqi_px4.py (same legs, same window, same references). Altitude
is also reported against each controller's own reference ("own": step for
the LQI, ramp for PX4). Prints one line per case, the distribution of each
metric per controller and a paired comparison over the cases where both
controllers completed the mission; writes all per-flight metrics to
metrics.csv in the campaign directory.

Usage:
  python tools/analyze_monte_carlo.py ~/fw_mc/campaign1
"""

import csv
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compare_lqi_px4 import rms, score, settled, V_REF  # noqa: E402

# name, function of (flight data, settled mask), aggregate of the worst case
METRICS = [
    ("legs_time", lambda d, s: d["t"][-1], max),
    ("path_rms", lambda d, s: rms(d["xt"]), max),
    ("path_max", lambda d, s: np.nanmax(d["xt"]), max),
    ("alt_step_rms", lambda d, s: rms(d["eh"][s]), max),
    ("alt_ramp_rms", lambda d, s: rms(d["er"]), max),
    ("airspeed_err_rms", lambda d, s: rms(d["ev"][s]), max),
    ("airspeed_min", lambda d, s: np.nanmin(d["ev"]) + V_REF, min),
    ("max_bank", lambda d, s: np.nanmax(np.abs(d["roll"])), max),
    ("mean_throttle", lambda d, s: np.nanmean(d["thr"]), max),
    ("elevator_rms", lambda d, s: rms(d["elev"]), max),
]


def flight_metrics(ulog, controller):
    d = score(ulog)
    s = settled(d)
    m = {name: float(f(d, s)) for name, f, _ in METRICS}
    m["alt_own_rms"] = m["alt_step_rms"] if controller == "lqi" else m["alt_ramp_rms"]
    return m
def main():
    camp = Path(sys.argv[1]).expanduser()
    cases = json.loads((camp / "cases.json").read_text())
    rows = []
    for c in cases:
        for ctrl in ("px4", "lqi"):
            res_file = camp / f"case{c['case']:02d}_{ctrl}" / "result.json"
            if not res_file.exists():
                continue
            r = json.loads(res_file.read_text())
            row = {"case": c["case"], "controller": ctrl, "wind_speed": c["wind_speed"],
                   "wind_from_deg": c["wind_from_deg"], "w20": c["w20"],
                   "model": c.get("model", ""), "success": r["success"],
                   "reason": r["reason"]}
            if r["success"] and r.get("ulog"):
                row.update(flight_metrics(r["ulog"], ctrl))
            rows.append(row)

    names = [n for n, _, _ in METRICS] + ["alt_own_rms"]
    with open(camp / "metrics.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())[:8] + names, restval="")
        w.writeheader()
        w.writerows(rows)

    by = {(r["case"], r["controller"]): r for r in rows}
    print(f"{'case':>4s} {'wind':>5s} {'from':>5s} {'W20':>5s} | "
          f"{'path rms PX4':>12s} {'LQI':>6s} | {'alt own PX4':>11s} {'LQI':>6s} | "
          f"{'Vmin PX4':>8s} {'LQI':>6s}")
    for c in cases:
        p, q = by.get((c["case"], "px4")), by.get((c["case"], "lqi"))
        if p is None or q is None:
            continue

        def cell(r, key, fmt):
            return format(r[key], fmt) if r["success"] else "FAIL".rjust(int(fmt.split(".")[0]))

        print(f"{c['case']:4d} {c['wind_speed']:5.1f} {c['wind_from_deg']:5.0f} {c['w20']:5.1f} | "
              f"{cell(p, 'path_rms', '12.1f')} {cell(q, 'path_rms', '6.1f')} | "
              f"{cell(p, 'alt_own_rms', '11.2f')} {cell(q, 'alt_own_rms', '6.2f')} | "
              f"{cell(p, 'airspeed_min', '8.1f')} {cell(q, 'airspeed_min', '6.1f')}")

    print()
    for ctrl in ("px4", "lqi"):
        flown = [r for r in rows if r["controller"] == ctrl]
        ok = [r for r in flown if r["success"]]
        print(f"{ctrl.upper()}: {len(ok)}/{len(flown)} missions completed")
        if not ok:
            continue
        print(f"  {'metric':18s} {'median':>9s} {'p90':>9s} {'worst':>9s}")
        for name, _, worst in METRICS + [("alt_own_rms", None, max)]:
            v = np.array([r[name] for r in ok])
            print(f"  {name:18s} {np.median(v):9.2f} {np.percentile(v, 90):9.2f} "
                  f"{worst(v):9.2f}")

    pairs = []
    for c in cases:
        p, q = by.get((c["case"], "px4")), by.get((c["case"], "lqi"))
        if p is not None and q is not None and p["success"] and q["success"]:
            pairs.append((p, q))
    if pairs:
        print(f"\nPaired over {len(pairs)} cases flown by both (LQI better = lower):")
        for name in ("path_rms", "path_max", "alt_own_rms", "airspeed_err_rms",
                     "mean_throttle"):
            ratio = np.array([q[name] / p[name] for p, q in pairs])
            better = sum(q[name] < p[name] for p, q in pairs)
            print(f"  {name:18s} LQI/PX4 median {np.median(ratio):5.2f}, "
                  f"range {ratio.min():5.2f} .. {ratio.max():5.2f}, LQI better in "
                  f"{better}/{len(pairs)}")
    print(f"\nPer-flight metrics written to {camp / 'metrics.csv'}")


if __name__ == "__main__":
    main()