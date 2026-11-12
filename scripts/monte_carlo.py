#!/usr/bin/env python3
"""
Monte Carlo campaign: PX4 (TECS/NPFG) vs LQI under random wind and turbulence.

Each case draws a steady wind (direction it blows from uniform in 0..360 deg,
speed uniform in 0..--max-wind), a Dryden intensity W20 (uniform in
0..--max-w20) and a turbulence seed. Every case is flown twice, once per
controller, with the same wind and the same turbulence seed, so both face
the same random gust sequence (the gust time history still differs a little,
because the gust correlation depends on the flown airspeed).

With --uncertainty each case also draws model errors for the simulated
aircraft (mass, lift, drag, Cm_alpha, control effectiveness, thrust), from a
separate random stream: the wind and turbulence of a case are then the same
as in a campaign without --uncertainty and the same --seed. The controllers
keep the nominal model.

The case list is drawn from --seed and saved as cases.json, so a campaign is
reproducible. Flights that already have a result.json are skipped, so an
interrupted campaign (Ctrl+C) continues where it stopped.

Usage:
  python scripts/monte_carlo.py --cases 15 --out ~/fw_mc/campaign1
"""

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
CONTROLLERS = ("px4", "lqi")
# Model-uncertainty ranges: uniform in 1 +- value.
MODEL_RANGES = {"mass": 0.10, "lift": 0.10, "drag": 0.20, "cm_alpha": 0.20,
                "control": 0.20, "thrust": 0.10}


def draw_cases(n, seed, max_wind, max_w20, uncertainty):
    """Return n reproducible random cases."""
    rng = np.random.default_rng(seed)
    rng_model = np.random.default_rng(seed + 1)
    cases = []
    for k in range(n):
        wind_from = float(rng.uniform(0.0, 360.0))
        speed = float(rng.uniform(0.0, max_wind))
        # Wind blowing from wind_from travels towards wind_from + 180 deg.
        north = -speed * math.cos(math.radians(wind_from))
        east = -speed * math.sin(math.radians(wind_from))
        cases.append({"case": k, "wind_from_deg": round(wind_from, 1),
                      "wind_speed": round(speed, 2),
                      "wind_ned": f"{north:.2f} {east:.2f} 0",
                      "w20": round(float(rng.uniform(0.0, max_w20)), 2),
                      "turb_seed": int(rng.integers(1, 2**31 - 1))})
        if uncertainty:
            cases[-1]["model"] = " ".join(
                f"{k}={1.0 + rng_model.uniform(-r, r):.3f}" for k, r in MODEL_RANGES.items())
    return cases


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--cases", type=int, default=15)
    ap.add_argument("--seed", type=int, default=2026, help="seed of the case draw")
    ap.add_argument("--max-wind", type=float, default=10.0, help="steady wind limit [m/s]")
    ap.add_argument("--max-w20", type=float, default=15.4, help="W20 limit [m/s] (30 kt)")
    ap.add_argument("--uncertainty", action="store_true", help="also draw model errors")
    ap.add_argument("--speed", type=float, default=4.0, help="simulation speed factor")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    cases_file = out / "cases.json"
    if cases_file.exists():
        cases = json.loads(cases_file.read_text())
        print(f"Continuing campaign in {out} ({len(cases)} cases)")
    else:
        cases = draw_cases(args.cases, args.seed, args.max_wind, args.max_w20,
                           args.uncertainty)
        cases_file.write_text(json.dumps(cases, indent=2) + "\n")
        print(f"New campaign in {out}: {len(cases)} cases, draw seed {args.seed}")

    for c in cases:
        for ctrl in CONTROLLERS:
            flight = out / f"case{c['case']:02d}_{ctrl}"
            if (flight / "result.json").exists():
                continue
            print(f"case {c['case']:2d} {ctrl}: wind {c['wind_speed']:4.1f} m/s from "
                  f"{c['wind_from_deg']:5.1f} deg, W20 {c['w20']:4.1f} m/s ... ",
                  end="", flush=True)
            try:
                subprocess.run([sys.executable, str(REPO / "scripts/run_flight.py"),
                                "--controller", ctrl, "--wind", c["wind_ned"],
                                "--w20", str(c["w20"]), "--seed", str(c["turb_seed"]),
                                "--model", c.get("model", ""),
                                "--speed", str(args.speed), "--out", str(flight)],
                               stdout=subprocess.DEVNULL, check=False)
            except KeyboardInterrupt:
                # An interrupted flight is not a failed one: fly it again on resume.
                (flight / "result.json").unlink(missing_ok=True)
                print("interrupted; run the same command again to continue")
                return
            r = json.loads((flight / "result.json").read_text())
            print("ok" if r["success"] else f"FAILED ({r['reason']})",
                  f"in {r['wall_time_s']:.0f} s", flush=True)


if __name__ == "__main__":
    main()