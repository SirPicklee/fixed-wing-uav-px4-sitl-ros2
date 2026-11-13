#!/usr/bin/env python3
"""
Make the README figures (PNG, written to docs/img/).

  flight_paths.png  top view of a calm PX4 flight and a calm LQI flight
                    against the planned legs, in the home (mission) frame
  monte_carlo.png   per-case LQI vs PX4 path RMS and own-reference altitude
                    RMS, for the wind/turbulence and the model-uncertainty
                    campaigns (results/*/metrics.csv)

Usage:
  python tools/plot_results.py PX4_CALM.ulg LQI_CALM.ulg
"""

import csv
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from pyulog import ULog  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from compare_lqi_px4 import WAYPOINTS  # noqa: E402
from ulog_path import home_ne  # noqa: E402
from ulog_tracking import topic  # noqa: E402

PX4_COLOR, LQI_COLOR = "#2a78d6", "#eb6834"   # validated categorical pair
CAMPAIGN_COLORS = ("#1baf7a", "#6250d6")       # validated pair, not the controller hues
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
OUT = ROOT / "docs" / "img"


def style(ax):
    """Recessive grid and axes."""
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=9)


def airborne_track(path):
    """Home-frame north/east of the whole airborne part of a flight."""
    ulog = ULog(str(path), ["vehicle_local_position", "vehicle_land_detected",
                            "vehicle_global_position", "home_position"])
    lp = topic(ulog, "vehicle_local_position")
    land = topic(ulog, "vehicle_land_detected")
    flying = land["timestamp"][land["landed"] == 0]
    t = lp["timestamp"]
    t = t[(t >= flying[0]) & (t <= flying[-1])] * 1e-6
    return home_ne(ulog, t)


def plot_paths(px4_ulog, lqi_ulog):
    fig, ax = plt.subplots(figsize=(7.5, 6.0), dpi=150)
    style(ax)
    plan_n = [0.0] + [w[0] for w in WAYPOINTS]
    plan_e = [800.0] + [w[1] for w in WAYPOINTS]
    ax.plot(plan_e, plan_n, color=MUTED, linewidth=1.2, linestyle=(0, (4, 3)),
            label="planned legs (from the LQI takeover point)", zorder=1)
    for k, (n, e, alt) in enumerate(WAYPOINTS):
        ax.plot(e, n, marker="s", markersize=7, color=MUTED, zorder=3)
        ax.annotate(f"WP{k + 1}  {alt:.0f} m", (e, n), xytext=(8, 6),
                    textcoords="offset points", fontsize=9, color=INK)
    for path, label, color, ls in ((px4_ulog, "PX4 TECS + NPFG", PX4_COLOR, "-"),
                                   (lqi_ulog, "LQI + L1 (offboard)", LQI_COLOR, "-")):
        n, e = airborne_track(path)
        ax.plot(e, n, color=color, linewidth=2.0, linestyle=ls, label=label, zorder=2)
    ax.plot(0, 0, marker="^", markersize=9, color=INK, zorder=4)
    ax.annotate("home / runway", (0, 0), xytext=(8, -14), textcoords="offset points",
                fontsize=9, color=INK)
    ax.set_aspect("equal")
    ax.set_xlabel("east of home [m]", color=INK)
    ax.set_ylabel("north of home [m]", color=INK)
    ax.set_title("TB2 mission in Gazebo, no wind: flown paths", color=INK, fontsize=11,
                 loc="left")
    ax.legend(frameon=False, fontsize=9, loc="lower left")
    fig.tight_layout()
    fig.savefig(OUT / "flight_paths.png", bbox_inches="tight")
    plt.close(fig)


def campaign(name):
    rows = list(csv.DictReader(open(ROOT / "results" / name / "metrics.csv")))
    by = {(r["case"], r["controller"]): r for r in rows if r["success"] == "True"}
    pairs = [(by[(c, "px4")], by[(c, "lqi")]) for c in sorted({r["case"] for r in rows})
             if (c, "px4") in by and (c, "lqi") in by]
    return pairs


def plot_monte_carlo():
    sets = (("mc_wind_turbulence", "wind + turbulence", "o", CAMPAIGN_COLORS[0]),
            ("mc_model_uncertainty", "wind + turbulence + model error", "^",
             CAMPAIGN_COLORS[1]))
    metrics = (("path_rms", "path deviation RMS [m]"),
               ("alt_own_rms", "altitude error RMS, own reference [m]"))
    fig, axes = plt.subplots(1, 2, figsize=(10.0, 4.6), dpi=150)
    for ax, (key, title) in zip(axes, metrics):
        style(ax)
        hi = 0.0
        for name, label, marker, color in sets:
            pairs = campaign(name)
            x = np.array([float(p[key]) for p, _ in pairs])
            y = np.array([float(q[key]) for _, q in pairs])
            hi = max(hi, x.max(), y.max())
            ax.scatter(x, y, s=46, marker=marker, color=color,
                       edgecolor="white", linewidth=1.5, label=f"{label} ({len(pairs)} cases)",
                       zorder=3)
        lim = hi * 1.08
        ax.plot([0, lim], [0, lim], color=MUTED, linewidth=1.0, zorder=1)
        ax.plot([0, lim], [0, lim / 2], color=MUTED, linewidth=1.0, linestyle=(0, (4, 3)),
                zorder=1)
        ax.annotate("LQI = PX4", (lim * 0.97, lim * 0.97), ha="right", va="bottom",
                    fontsize=8, color=MUTED, rotation=0)
        ax.annotate("LQI = PX4 / 2", (lim * 0.97, lim * 0.485), ha="right", va="bottom",
                    fontsize=8, color=MUTED)
        ax.set_xlim(0, lim)
        ax.set_ylim(0, lim)
        ax.set_aspect("equal")
        ax.set_xlabel(f"PX4 {title}", color=INK, fontsize=9)
        ax.set_ylabel(f"LQI {title}", color=INK, fontsize=9)
    axes[0].set_title("Same case flown by both controllers (below the diagonal: LQI better)",
                      color=INK, fontsize=11, loc="left")
    axes[0].legend(frameon=False, fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(OUT / "monte_carlo.png", bbox_inches="tight")
    plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    plot_paths(Path(sys.argv[1]).expanduser(), Path(sys.argv[2]).expanduser())
    plot_monte_carlo()
    for f in ("flight_paths.png", "monte_carlo.png"):
        print(f"wrote {os.path.relpath(OUT / f, ROOT)}")


if __name__ == "__main__":
    main()
