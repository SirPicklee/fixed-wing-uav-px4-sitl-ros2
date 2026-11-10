#!/usr/bin/env bash
# Launch Gazebo with the TB2 runway world and attach PX4 SITL to the model.
#
# Usage:  scripts/run_sitl.sh            (with Gazebo GUI)
#         HEADLESS=1 scripts/run_sitl.sh (server only)
#
# Requires: colcon build (fw_aero_gazebo installed), PX4 built with
# "make px4_sitl" in $PX4_DIR (default ~/PX4-Autopilot, v1.17.0).

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
SHARE="$REPO/install/fw_aero_gazebo/share/fw_aero_gazebo"
WORLD_NAME="tb2_runway"
MODEL_NAME="tb2_0"

if [[ ! -f "$SHARE/worlds/$WORLD_NAME.sdf" ]]; then
  echo "World not installed. Run 'colcon build' in $REPO first." >&2
  exit 1
fi
if [[ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
  echo "PX4 SITL binary not found. Run 'make px4_sitl' in $PX4_DIR first." >&2
  exit 1
fi
if pgrep -f "gz sim" >/dev/null; then
  echo "Another Gazebo instance is running. Stop it first (pkill -f 'gz sim')." >&2
  exit 1
fi

# --- Gazebo -----------------------------------------------------------------
export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO/install/fw_aero_gazebo/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$SHARE/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

gz sim -r -s -v 2 "$SHARE/worlds/$WORLD_NAME.sdf" &
GZ_SERVER_PID=$!
GZ_GUI_PID=""
if [[ -z "${HEADLESS:-}" ]]; then
  gz sim -g >/dev/null 2>&1 &
  GZ_GUI_PID=$!
fi
cleanup() {
  kill "$GZ_SERVER_PID" ${GZ_GUI_PID:+"$GZ_GUI_PID"} 2>/dev/null || true
}
trap cleanup EXIT

# --- PX4 --------------------------------------------------------------------
export PX4_GZ_STANDALONE=1          # Gazebo is started above, not by PX4
export PX4_GZ_WORLD="$WORLD_NAME"
export PX4_GZ_MODEL_NAME="$MODEL_NAME"
export PX4_SYS_AUTOSTART=4003       # base: generic gz fixed-wing airframe
# shellcheck source=../px4/tb2_params.env
source "$REPO/px4/tb2_params.env"

"$PX4_DIR/build/px4_sitl_default/bin/px4"