// Aerodynamic forces and moments for a fixed-wing UAV (body axes).
//
// Port of uavsim/aerodynamics.py (uav-6dof-autopilot v1.0, commit 1dab032).
//
// Body axes: x forward, y right, z down. Wind axes are related to body axes
// by angle of attack (alpha) and sideslip (beta).
#pragma once

#include <Eigen/Dense>

#include "fw_aero/aircraft_params.hpp"

namespace fw_aero {

// Control inputs. Surface deflections in rad, throttle in [0, 1].
struct Controls {
  double elevator = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;
  double throttle = 0.0;
};

// Output of the aerodynamic model.
struct AeroResult {
  Eigen::Vector3d force_body;   // N, [Fx, Fy, Fz] incl. thrust, excl. gravity
  Eigen::Vector3d moment_body;  // N m, [L, M, N]
  double alpha;                 // rad
  double beta;                  // rad
  double airspeed;              // m/s
  double dynamic_pressure;      // Pa
  double lift;                  // N
  double drag;                  // N
};

struct AirData {
  double airspeed;  // m/s
  double alpha;     // rad
  double beta;      // rad
};

// Airspeed, alpha and beta from the body-axis air-relative velocity.
AirData air_data(const Eigen::Vector3d& v_body_rel_air);

// Simple propeller thrust model: decays with airspeed, scales with density.
double thrust(const AircraftParams& params, double throttle, double airspeed,
              double density);

// Total aerodynamic + thrust force and moment in body axes.
AeroResult aero_forces_moments(const AircraftParams& params,
                               const Eigen::Vector3d& v_body_rel_air,
                               const Eigen::Vector3d& omega_body,
                               const Controls& controls, double altitude_m);

}  // namespace fw_aero