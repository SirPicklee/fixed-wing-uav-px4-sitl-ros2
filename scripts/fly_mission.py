#!/usr/bin/env python3
"""
Fixed-wing SITL mission: takeoff -> waypoints -> landing.

Usage:
  1) Start PX4 SITL in one terminal:  make px4_sitl gz_rc_cessna
  2) Run this script in another terminal.

Waypoints are defined as north/east offsets [m] from home and converted
to lat/lon with a flat-earth approximation (error < 1 m over a few km).
"""

import asyncio
import math

from mavsdk import System
from mavsdk.mission_raw import MissionItem

# --- MAVLink constants -------------------------------------------------------
MAV_FRAME_MISSION = 2                   # items without position (DO commands)
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6   # altitude relative to home
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_LAND_START = 189             # marks the start of the landing sequence

EARTH_RADIUS_M = 6378137.0              # WGS-84 equatorial radius


# --- SITL parameter overrides ------------------------------------------------
# The rc_cessna model has no distance sensor, so terrain-based landing logic
# must be disabled. The simulated ground is flat, so flaring relative to the
# landing waypoint altitude is physically correct.
PX4_PARAMS_INT = {
    "NAV_DLL_ACT": 0,      # no data-link-loss action (no GCS in SITL)
    "FW_LND_USETER": 0,    # do not use terrain estimate during landing
    "FW_LND_ABORT": 0,     # disable terrain-based landing abort criteria
}

# --- Mission definition: (command, north [m], east [m], altitude [m]) --------
MISSION_NE = [
    (MAV_CMD_NAV_TAKEOFF,     0.0,  400.0, 40.0),
    (MAV_CMD_NAV_WAYPOINT,  400.0,  800.0, 50.0),
    (MAV_CMD_NAV_WAYPOINT,  800.0,  200.0, 50.0),
    (MAV_CMD_NAV_WAYPOINT,  400.0, -900.0, 50.0),
    (MAV_CMD_DO_LAND_START,   0.0,    0.0,  0.0),  # position-less marker
    (MAV_CMD_NAV_WAYPOINT,    0.0, -500.0, 30.0),  # approach point
    (MAV_CMD_NAV_LAND,        0.0,    0.0,  0.0),  # touchdown point = home
]


def ne_to_latlon(lat0_deg, lon0_deg, north_m, east_m):
    """Convert a north/east offset from home to latitude/longitude."""
    dlat = math.degrees(north_m / EARTH_RADIUS_M)
    dlon = math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat0_deg))))
    return lat0_deg + dlat, lon0_deg + dlon


def build_mission(home_lat, home_lon):
    """Build MAVLink mission items from MISSION_NE."""
    items = []
    for seq, (cmd, north, east, alt) in enumerate(MISSION_NE):
        if cmd == MAV_CMD_DO_LAND_START:
            # PX4 accepts this item only in the position-less frame
            frame, x, y, z = MAV_FRAME_MISSION, 0, 0, 0.0
        else:
            lat, lon = ne_to_latlon(home_lat, home_lon, north, east)
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            x = int(round(lat * 1e7))   # latitude * 1e7
            y = int(round(lon * 1e7))   # longitude * 1e7
            z = float(alt)              # altitude [m]
        items.append(MissionItem(
            seq,
            frame,
            cmd,
            1 if seq == 0 else 0,       # current: first item is active
            1,                          # autocontinue
            0.0, 0.0, 0.0,              # param1-3: PX4 defaults
            float("nan"),               # param4 (yaw): let PX4 decide
            x, y, z,
            0,                          # mission_type: MISSION
        ))
    return items


async def print_progress(drone):
    async for p in drone.mission_raw.mission_progress():
        print(f"  Mission item: {p.current}/{p.total}")


async def wait_until_landed(drone):
    """Wait until the vehicle has taken off and landed again."""
    was_in_air = False
    async for in_air in drone.telemetry.in_air():
        if in_air and not was_in_air:
            print("Airborne.")
            was_in_air = True
        elif not in_air and was_in_air:
            return


async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Connecting to PX4...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected.")
            break

    print("Waiting for GPS fix and home position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    async for home in drone.telemetry.home():
        home_lat, home_lon = home.latitude_deg, home.longitude_deg
        break
    print(f"Home: {home_lat:.7f}, {home_lon:.7f}")

    
    for name, value in PX4_PARAMS_INT.items():
        await drone.param.set_param_int(name, value)
        readback = await drone.param.get_param_int(name)
        print(f"Param {name} = {readback}")

    items = build_mission(home_lat, home_lon)
    await drone.mission_raw.upload_mission(items)
    print(f"Mission uploaded ({len(items)} items).")

    await drone.action.arm()
    await drone.mission_raw.start_mission()
    print("Mission started.")

    progress_task = asyncio.ensure_future(print_progress(drone))
    await wait_until_landed(drone)
    progress_task.cancel()
    print("Landed.")


if __name__ == "__main__":
    asyncio.run(run())