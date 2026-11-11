# Copyright 2026 SirPicklee
# SPDX-License-Identifier: MIT
"""Pure helper functions shared by the fw_telemetry nodes (no ROS dependency)."""

import math


def quat_to_euler(w, x, y, z):
    """
    Convert a Hamilton quaternion (body FRD -> NED) to Euler angles.

    Returns (roll, pitch, yaw) in radians, ZYX convention, matching PX4's
    vehicle_attitude / vehicle_odometry quaternion.
    """
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw
