#!/usr/bin/env python3
"""
Fly one complete TB2 mission in SITL without manual steps.

Starts, in their own process groups:
  1. Gazebo + PX4  (scripts/run_sitl.sh, headless, given speed / wind / turbulence)
  2. the uXRCE-DDS agent
  3. the fw_lqi node: engaged for --controller lqi, shadow for --controller px4
  4. the tb2 mission (scripts/fly_mission.py)
then waits for the landing, stops everything and writes result.json to the
output directory together with the logs of each process and the node CSV.

Run from the repository root with the project venv active, in a shell where
ROS 2 and the px4_ros_ws underlay are sourced (as in ~/.bashrc).

Usage:
  python scripts/run_flight.py --controller lqi --wind "0 -8 0" --w20 7.7 \
      --seed 3 --speed 4 --out /tmp/flights/lqi_s3
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PX4_DIR = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot"))
LOG_ROOT = PX4_DIR / "build/px4_sitl_default/rootfs/log"


def start(cmd, log_path, env=None, stdin=None):
    """Start a shell command in its own process group, output to log_path.

    Output is line-buffered (stdbuf for C/C++ programs, PYTHONUNBUFFERED and
    the rcutils setting for Python/ROS), so the logs can be watched live.
    """
    env = dict(env or os.environ, PYTHONUNBUFFERED="1", RCUTILS_LOGGING_BUFFERED_STREAM="0")
    log = open(log_path, "w")
    return subprocess.Popen(["bash", "-c", f"exec stdbuf -oL -eL bash -c '{cmd}'"], cwd=REPO,
                            env=env, stdout=log, stderr=subprocess.STDOUT, stdin=stdin,
                            start_new_session=True)


def wait_for_text(path, text, timeout_s, procs=()):
    """Wait until text appears in a log file; False on timeout or early exit."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if text in Path(path).read_text(errors="replace"):
            return True
        if any(p.poll() is not None for p in procs):
            return False
        time.sleep(0.5)
    return False


def stop(proc, grace_s=10.0):
    """Interrupt a process group, then kill it if it does not exit."""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=grace_s)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def kill_leftovers():
    """Make sure no simulator from an earlier run is still alive."""
    subprocess.run(["pkill", "-f", "gz sim"], check=False)
    subprocess.run(["pkill", "-f", "lib/fw_lqi/lqi_offboard"], check=False)
    # A mission script left over from an interrupted run would reconnect to the
    # new PX4 and send its own arm and mission-start commands mid-flight.
    subprocess.run(["pkill", "-f", "scripts/fly_mission.py"], check=False)
    for name in ("MicroXRCEAgent", "px4", "mavsdk_server"):
        subprocess.run(["pkill", "-x", name], check=False)
    time.sleep(2.0)

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--controller", choices=["px4", "lqi"], required=True)
    ap.add_argument("--wind", default="0 0 0", help='steady wind "north east down" [m/s]')
    ap.add_argument("--w20", type=float, default=0.0, help="Dryden W20 [m/s]")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--speed", type=float, default=4.0, help="simulation speed factor")
    ap.add_argument("--timeout", type=float, default=900.0, help="wall-clock limit [s]")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)

    kill_leftovers()
    t_start = time.time()
    env = dict(os.environ, HEADLESS="1", SIM_SPEED=str(args.speed), WIND_NED=args.wind,
               TURB_W20=str(args.w20), TURB_SEED=str(args.seed))
    result = {"controller": args.controller, "wind_ned": args.wind, "w20": args.w20,
              "seed": args.seed, "speed": args.speed, "success": False, "reason": ""}
    sitl = agent = node = mission = None
    try:
        # stdin stays open: PX4's shell would exit on end of input.
        sitl = start("exec scripts/run_sitl.sh", out / "px4.log", env, subprocess.PIPE)
        if not wait_for_text(out / "px4.log", "Ready for takeoff!", 180, [sitl]):
            result["reason"] = "PX4 not ready"
            return result
        agent = start("exec MicroXRCEAgent udp4 -p 8888", out / "agent.log")
        engage = "true" if args.controller == "lqi" else "false"
        node = start("source install/setup.bash && exec ros2 run fw_lqi lqi_offboard "
                     "--ros-args --params-file src/fw_lqi/config/lqi_gains.yaml "
                     f"-p engage:={engage} -p log_file:={out / 'lqi_node.csv'}",
                     out / "node.log")
        if not wait_for_text(out / "node.log", "waypoints, engage above", 60, [node]):
            result["reason"] = "LQI node did not start"
            return result
        mission = start(f"exec {sys.executable} scripts/fly_mission.py tb2",
                        out / "mission.log")
        try:
            mission.wait(timeout=max(args.timeout - (time.time() - t_start), 1.0))
        except subprocess.TimeoutExpired:
            result["reason"] = "timeout"
            return result
        result["landed"] = "Landed." in (out / "mission.log").read_text(errors="replace")
        time.sleep(5.0 / args.speed + 2.0)           # let PX4 disarm and close the log
        return result
    finally:
        for p in (mission, node, agent, sitl):
            stop(p)
        kill_leftovers()
        node_log = out / "node.log"
        node_text = node_log.read_text(errors="replace") if node_log.exists() else ""
        result["node_took_over"] = "Taking over" in node_text
        result["node_finished_legs"] = "Last waypoint reached" in node_text
        result["node_aborted"] = "stopping" in node_text
        took = re.search(r"Taking over at ([-\d.]+) m", node_text)
        result["takeover_alt"] = float(took.group(1)) if took else None
        # A late takeover means PX4 flew part of the legs: not a valid LQI flight.
        on_time = result["takeover_alt"] is not None and result["takeover_alt"] < 65.0
        if result.get("landed"):
            result["success"] = not result["node_aborted"] and (
                args.controller == "px4" or (result["node_finished_legs"] and on_time))
            if not result["success"]:
                result["reason"] = ("late takeover" if result["node_finished_legs"]
                                    else "LQI did not complete the legs")
        elif not result["reason"]:
            result["reason"] = "no landing"
        logs = sorted(LOG_ROOT.glob("*/*.ulg"), key=os.path.getmtime)
        result["ulog"] = str(logs[-1]) if logs and os.path.getmtime(logs[-1]) > t_start else None
        result["wall_time_s"] = round(time.time() - t_start, 1)
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    sys.exit(0 if main()["success"] else 1)