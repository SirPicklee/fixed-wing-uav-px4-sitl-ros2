#!/usr/bin/env bash
# Launch Gazebo with the TB2 runway world and attach PX4 SITL to the model.
#
# Usage:  scripts/run_sitl.sh            (with Gazebo GUI)
#         HEADLESS=1 scripts/run_sitl.sh (server only)
#         WIND_NED="0 -8 0" TURB_W20=7.7 TURB_SEED=3 scripts/run_sitl.sh
#           steady wind north/east/down [m/s], Dryden W20 [m/s] and seed
#           (defaults: no wind, no turbulence, seed 1)
#         MODEL_SCALES="mass=1.05 lift=0.95 drag=1.1" scripts/run_sitl.sh
#           model-uncertainty factors for the simulated aircraft (mass,
#           lift, drag, cm_alpha, control, thrust); the controllers keep
#           the nominal model
#         SIM_SPEED=4 HEADLESS=1 scripts/run_sitl.sh
#           run faster than real time (PX4 follows the Gazebo clock)
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

# --- Wind and turbulence ----------------------------------------------------
# A temporary copy of the model with the wind parameters added to the
# fw_aero plugin is put first on the resource path, so model://tb2 in the
# world resolves to it; the installed model is left untouched.
WIND_NED="${WIND_NED:-0 0 0}"
TURB_W20="${TURB_W20:-0}"
TURB_SEED="${TURB_SEED:-1}"
MODEL_TMP="$(mktemp -d /tmp/fw_sitl_models.XXXXXX)"
cp -r "$SHARE/models/tb2" "$MODEL_TMP/"
MODEL_SCALES="${MODEL_SCALES:-}"
SCALE_PARAMS=""
MASS_SCALE=""
for kv in $MODEL_SCALES; do
  key="${kv%%=*}"
  val="${kv#*=}"
  case "$key" in
    mass) MASS_SCALE="$val" ;;
    lift|drag|cm_alpha|control|thrust)
      SCALE_PARAMS="$SCALE_PARAMS\n      <${key}_scale>$val</${key}_scale>" ;;
    *) echo "run_sitl: unknown model scale '$key' in MODEL_SCALES" >&2; exit 1 ;;
  esac
done
sed -i "s#      <pitot_x>3.3</pitot_x>#      <pitot_x>3.3</pitot_x>\n      <wind_ned>$WIND_NED</wind_ned>\n      <turbulence_w20>$TURB_W20</turbulence_w20>\n      <turbulence_seed>$TURB_SEED</turbulence_seed>$SCALE_PARAMS#" \
  "$MODEL_TMP/tb2/model.sdf"
if ! grep -q "<wind_ned>" "$MODEL_TMP/tb2/model.sdf"; then
  echo "run_sitl: could not add the wind parameters to the model" >&2
  exit 1
fi
export GZ_SIM_RESOURCE_PATH="$MODEL_TMP:$GZ_SIM_RESOURCE_PATH"
echo "run_sitl: wind NED [$WIND_NED] m/s, turbulence W20 $TURB_W20 m/s, seed $TURB_SEED"
if [[ -n "$MASS_SCALE" ]]; then
  # Same geometry, scaled mass: the inertia scales with it.
  python3 -c '
import re, sys
path, k = sys.argv[1], float(sys.argv[2])
text = open(path).read()
text, n = re.subn(r"<(mass|ixx|iyy|izz)>([-\d.eE+]+)</\1>",
                  lambda m: f"<{m.group(1)}>{float(m.group(2)) * k:.6g}</{m.group(1)}>", text)
if n != 4:
    sys.exit(f"run_sitl: expected 4 mass/inertia entries, found {n}")
open(path, "w").write(text)
' "$MODEL_TMP/tb2/model.sdf" "$MASS_SCALE"
fi
echo "run_sitl: model scales [${MODEL_SCALES:-none}]"
grep -E "<mass>|<i(xx|yy|zz)>|_scale>" "$MODEL_TMP/tb2/model.sdf" | sed 's/^ */run_sitl:   /'

gz sim -r -s -v 2 "$SHARE/worlds/$WORLD_NAME.sdf" &
GZ_SERVER_PID=$!

# --- Simulation speed -------------------------------------------------------
# PX4 runs in lockstep with the Gazebo clock, so raising the real-time factor
# speeds up the whole simulation without changing it. PX4_SIM_SPEED_FACTOR
# makes PX4's rcS scale its link-loss timeouts to match.
if [[ -n "${SIM_SPEED:-}" ]]; then
  export PX4_SIM_SPEED_FACTOR="$SIM_SPEED"
  for _ in $(seq 50); do
    if gz service -s "/world/$WORLD_NAME/set_physics" --reqtype gz.msgs.Physics \
         --reptype gz.msgs.Boolean --timeout 1000 \
         --req "real_time_factor: $SIM_SPEED" 2>/dev/null | grep -q "data: true"; then
      echo "run_sitl: simulation speed x$SIM_SPEED"
      break
    fi
    sleep 0.2
  done
fi
GZ_GUI_PID=""
if [[ -z "${HEADLESS:-}" ]]; then
  gz sim -g >/dev/null 2>&1 &
  GZ_GUI_PID=$!
fi
cleanup() {
  kill "$GZ_SERVER_PID" ${GZ_GUI_PID:+"$GZ_GUI_PID"} 2>/dev/null || true
  rm -rf "$MODEL_TMP"
}
trap cleanup EXIT

# --- PX4 --------------------------------------------------------------------
export PX4_GZ_STANDALONE=1          # Gazebo is started above, not by PX4
export PX4_GZ_WORLD="$WORLD_NAME"
export PX4_GZ_MODEL_NAME="$MODEL_NAME"
export PX4_SYS_AUTOSTART=4003       # base: generic gz fixed-wing airframe
# shellcheck source=../px4/tb2_params.env
source "$REPO/px4/tb2_params.env"

# PX4_PARAM_ overrides that equal the firmware default are treated as
# unchanged, so the airframe's set-default replaces them: rc_cessna raises
# SENS_EN_ARSPDSIM to 1 (the EKF-derived airspeed simulator then runs as a
# second airspeed source) and NAV_DLL_ACT to 2 (arming then waits for a GCS).
# Once the startup script has applied the airframe (commander is running),
# set both again and stop the simulator if it was started.
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin"
(
  for _ in $(seq 240); do
    sleep 0.5
    "$PX4_BIN/px4-commander" status >/dev/null 2>&1 && break
  done
  "$PX4_BIN/px4-param" set NAV_DLL_ACT 0 >/dev/null 2>&1
  "$PX4_BIN/px4-param" set SENS_EN_ARSPDSIM 0 >/dev/null 2>&1
  echo "run_sitl: NAV_DLL_ACT and SENS_EN_ARSPDSIM set to 0"
  for _ in $(seq 20); do
    if "$PX4_BIN/px4-sensor_airspeed_sim" status >/dev/null 2>&1; then
      "$PX4_BIN/px4-sensor_airspeed_sim" stop >/dev/null 2>&1
      echo "run_sitl: stopped sensor_airspeed_sim (EKF-derived airspeed)"
      break
    fi
    sleep 0.5
  done
) &

"$PX4_BIN/px4"
