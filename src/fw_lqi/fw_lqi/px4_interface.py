"""
Conversions between PX4 uORB/ROS 2 messages and the LQI controller.

PX4 conventions (v1.17): attitude quaternion (w, x, y, z) rotates body FRD
into the local NED frame; actuator_servos.control[i] is normalized to
[-1, 1] and drives gz servo_i; actuator_motors.control[0] is the throttle.
Servo order and signs follow the rc_cessna airframe used by the TB2 model
(see fw_aero/px4_actuators.hpp, which applies the inverse of this mapping):
servo_0 left aileron, servo_1 right aileron, servo_2 elevator, servo_3
rudder; +1 means positive torque about the FRD axis (roll right, pitch
up, yaw right).
"""

import math

NAN = float('nan')


def quat_to_dcm(w, x, y, z):
    """Return the rotation matrix (body -> NED) as nested lists."""
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def quat_to_roll_pitch(w, x, y, z):
    """Return (roll, pitch) in rad for a 3-2-1 Euler sequence."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def ned_to_body(q, v_ned):
    """Rotate a NED vector into body FRD axes (transpose of the DCM)."""
    r = quat_to_dcm(*q)
    return [sum(r[i][j] * v_ned[i] for i in range(3)) for j in range(3)]


def servo_controls(cmd, limits):
    """Return the 8 normalized actuator_servos values for a Command."""
    roll = max(-1.0, min(1.0, cmd.aileron / limits.aileron))
    pitch = max(-1.0, min(1.0, -cmd.elevator / limits.elevator))
    yaw = max(-1.0, min(1.0, -cmd.rudder / limits.rudder))
    return [-roll, roll, pitch, yaw, NAN, NAN, NAN, NAN]


def motor_controls(cmd):
    """Return the 12 normalized actuator_motors values for a Command."""
    return [max(0.0, min(1.0, cmd.throttle))] + [NAN] * 11
