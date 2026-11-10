#!/usr/bin/env python3
"""
Diagnostic: upload the mission incrementally to find the first item PX4 rejects.
Uses the exact mission definition from fly_mission.py.
"""

import asyncio

from mavsdk import System
from mavsdk.mission_raw import MissionRawError

from fly_mission import MISSION_NE, build_mission

CMD_NAMES = {16: "NAV_WAYPOINT", 21: "NAV_LAND", 22: "NAV_TAKEOFF", 189: "DO_LAND_START"}


async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    async for home in drone.telemetry.home():
        home_lat, home_lon = home.latitude_deg, home.longitude_deg
        break

    items = build_mission(home_lat, home_lon)

    for k in range(1, len(items) + 1):
        cmd = MISSION_NE[k - 1][0]
        try:
            await drone.mission_raw.upload_mission(items[:k])
            print(f"First {k} items: OK      (last added: seq={k - 1}, {CMD_NAMES[cmd]})")
        except MissionRawError as err:
            print(f"First {k} items: REJECTED (last added: seq={k - 1}, {CMD_NAMES[cmd]})"
                  f" -> {err._result.result}")
            break


if __name__ == "__main__":
    asyncio.run(run())