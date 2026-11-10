// Aircraft parameter set for a medium-altitude fixed-wing UAV.
//
// Port of uavsim/aircraft.py (uav-6dof-autopilot v1.0, commit 1dab032).
//
// NOTE: Values are representative of a TB2-class UAV (open-source figures for
// span, wing area and mass; aerodynamic coefficients are estimated). They are
// NOT official manufacturer data.
#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <string>

namespace fw_aero {

// Geometry, mass properties and aerodynamic coefficients (SI units).
// Angular derivatives are per rad; rate derivatives use nondimensional rates
// p_hat = p b / (2V), q_hat = q c / (2V), r_hat = r b / (2V).
struct AircraftParams {
  std::string name = "TB2-class UAV (representative)";

  // Geometry
  double wing_area = 11.0;   // S, m^2
  double wing_span = 12.0;   // b, m
  double mean_chord = 0.92;  // c_bar, m

  // Mass properties
  double mass = 650.0;  // kg
  Eigen::Matrix3d inertia =
      Eigen::Vector3d(1200.0, 1500.0, 2400.0).asDiagonal();  // kg m^2, body

  // Lift
  double cl0 = 0.30;
  double cl_alpha = 5.2;
  double cl_max = 1.4;
  double cl_q = 6.0;
  double cl_de = 0.4;

  // Drag (induced drag factor k = 1 / (pi e AR) is derived below)
  double cd0 = 0.030;
  double oswald = 0.80;

  // Pitch moment
  double cm0 = 0.05;
  double cm_alpha = -1.2;  // static stability
  double cm_q = -18.0;     // pitch damping
  double cm_de = -1.5;     // elevator

  // Side force and lateral-directional
  double cy_beta = -0.40;
  double cy_dr = 0.15;
  double cl_beta = -0.10;  // dihedral effect (rolling moment)
  double cl_p = -0.45;     // roll damping
  double cl_r = 0.10;
  double cl_da = 0.20;     // aileron
  double cn_beta = 0.10;   // weathercock stability
  double cn_p = -0.05;
  double cn_r = -0.15;     // yaw damping
  double cn_da = -0.01;
  double cn_dr = -0.08;

  // Propulsion (simple propeller model)
  double max_thrust_sl = 1800.0;  // N at sea level, zero airspeed
  double thrust_decay = 0.012;    // per m/s of airspeed

  double aspect_ratio() const { return wing_span * wing_span / wing_area; }

  double induced_drag_k() const {
    return 1.0 / (M_PI * oswald * aspect_ratio());
  }
};

}  // namespace fw_aero