// Mapping from PX4 SITL actuator outputs (gz_bridge) to fw_aero Controls.
//
// PX4 v1.17 gz_bridge publishes, for a model named <model>:
//   /model/<model>/servo_<i>       gz.msgs.Double, servo angle [rad], linear
//                                  in the normalized command u in [-1, 1]
//                                  between SIM_GZ_SV_MINA and SIM_GZ_SV_MAXA
//                                  (default -45 .. +45 deg)
//   /<model>/command/motor_speed   gz.msgs.Actuators, velocity[i] in
//                                  [SIM_GZ_EC_MIN, SIM_GZ_EC_MAX]
//
// Sign conventions (the reason this mapping exists):
//   PX4 control allocation (CA_SV_CS*_TRQ_*): a positive normalized command
//   produces positive torque about the FRD body axis: roll right, pitch up,
//   yaw right. Ailerons are split: CS0 left (TRQ_R = -0.5), CS1 right
//   (TRQ_R = +0.5).
//   fw_aero (from the aero derivatives): positive aileron rolls right
//   (cl_da > 0), positive elevator pitches nose down (cm_de < 0), positive
//   rudder yaws nose left (cn_dr < 0).
//   Hence: aileron = +roll command, elevator = -pitch command,
//          rudder = -yaw command.
#pragma once

#include <algorithm>
#include <cmath>

#include "fw_aero/aerodynamics.hpp"

namespace fw_aero {

inline constexpr double kDegToRad = M_PI / 180.0;

struct Px4ActuatorMapping {
  // Servo angle published by PX4 at u = +1 (SIM_GZ_SV_MAXA, default 45 deg).
  double servo_full_scale_rad = 45.0 * kDegToRad;
  // Physical surface deflection at u = +1. Limits from uav-6dof-autopilot
  // (autopilot.py MAX_ELEVATOR, lateral.py MAX_AILERON / MAX_RUDDER).
  double max_aileron_rad = 20.0 * kDegToRad;
  double max_elevator_rad = 20.0 * kDegToRad;
  double max_rudder_rad = 15.0 * kDegToRad;
  // Motor command at full throttle (SIM_GZ_EC_MAX).
  double motor_full_scale = 1000.0;
};

// Servo angles as published by PX4 [rad], in CA_SV_CS order.
struct Px4ServoAngles {
  double left_aileron = 0.0;   // servo_0
  double right_aileron = 0.0;  // servo_1
  double elevator = 0.0;       // servo_2
  double rudder = 0.0;         // servo_3
};

inline Controls MapPx4Commands(const Px4ServoAngles& servo,
                               double motor_command,
                               const Px4ActuatorMapping& m = {}) {
  const auto normalized = [&m](double angle_rad) {
    return std::clamp(angle_rad / m.servo_full_scale_rad, -1.0, 1.0);
  };
  const double u_left = normalized(servo.left_aileron);
  const double u_right = normalized(servo.right_aileron);
  const double u_pitch = normalized(servo.elevator);
  const double u_yaw = normalized(servo.rudder);

  Controls c;
  // Roll command = differential part of the two ailerons.
  c.aileron = 0.5 * (u_right - u_left) * m.max_aileron_rad;
  c.elevator = -u_pitch * m.max_elevator_rad;
  c.rudder = -u_yaw * m.max_rudder_rad;
  c.throttle = std::clamp(motor_command / m.motor_full_scale, 0.0, 1.0);
  return c;
}

}  // namespace fw_aero