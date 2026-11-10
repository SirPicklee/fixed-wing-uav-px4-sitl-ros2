// Port of uavsim/aerodynamics.py (uav-6dof-autopilot v1.0, commit 1dab032).
#include "fw_aero/aerodynamics.hpp"

#include <algorithm>
#include <cmath>

#include "fw_aero/atmosphere.hpp"

namespace fw_aero {

AirData air_data(const Eigen::Vector3d& v_body_rel_air) {
  const double u = v_body_rel_air.x();
  const double v = v_body_rel_air.y();
  const double w = v_body_rel_air.z();
  const double vt = std::sqrt(u * u + v * v + w * w);
  if (vt < 1e-6) {
    return AirData{0.0, 0.0, 0.0};
  }
  const double alpha = std::atan2(w, u);
  const double beta = std::asin(std::clamp(v / vt, -1.0, 1.0));
  return AirData{vt, alpha, beta};
}

double thrust(const AircraftParams& params, double throttle, double airspeed,
              double density) {
  throttle = std::clamp(throttle, 0.0, 1.0);
  const double decay = std::max(0.0, 1.0 - params.thrust_decay * airspeed);
  return params.max_thrust_sl * throttle * decay * (density / kRho0);
}

AeroResult aero_forces_moments(const AircraftParams& params,
                               const Eigen::Vector3d& v_body_rel_air,
                               const Eigen::Vector3d& omega_body,
                               const Controls& controls, double altitude_m) {
  const AtmosphereState atm = atmosphere(std::clamp(altitude_m, 0.0, kHMax));
  const AirData ad = air_data(v_body_rel_air);
  const double vt = ad.airspeed;
  const double alpha = ad.alpha;
  const double beta = ad.beta;
  const double qbar = dynamic_pressure(atm.density, vt);
  const double p = omega_body.x();
  const double q = omega_body.y();
  const double r = omega_body.z();

  const double S = params.wing_area;
  const double b = params.wing_span;
  const double c = params.mean_chord;

  double p_hat = 0.0;
  double q_hat = 0.0;
  double r_hat = 0.0;
  if (vt > 1e-3) {
    p_hat = p * b / (2.0 * vt);
    q_hat = q * c / (2.0 * vt);
    r_hat = r * b / (2.0 * vt);
  }

  // Lift with soft stall: linear region clipped at +-cl_max
  const double cl_lin = params.cl0 + params.cl_alpha * alpha +
                        params.cl_q * q_hat + params.cl_de * controls.elevator;
  const double cl = std::clamp(cl_lin, -params.cl_max, params.cl_max);

  // Drag polar plus a simple post-stall penalty
  double cd = params.cd0 + params.induced_drag_k() * cl * cl;
  if (std::fabs(cl_lin) > params.cl_max) {
    const double excess = std::fabs(cl_lin) - params.cl_max;
    cd += 0.5 * excess * excess;
  }

  const double cy = params.cy_beta * beta + params.cy_dr * controls.rudder;

  const double lift = qbar * S * cl;
  const double drag = qbar * S * cd;
  const double side = qbar * S * cy;

  // Wind axes -> body axes (lift/drag in the x-z plane, side force along y)
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);
  const double fx_aero = -drag * ca + lift * sa;
  const double fz_aero = -drag * sa - lift * ca;
  const double fy_aero = side;

  const double t = thrust(params, controls.throttle, vt, atm.density);

  const double cl_roll = params.cl_beta * beta + params.cl_p * p_hat +
                         params.cl_r * r_hat + params.cl_da * controls.aileron;
  const double cm = params.cm0 + params.cm_alpha * alpha +
                    params.cm_q * q_hat + params.cm_de * controls.elevator;
  const double cn = params.cn_beta * beta + params.cn_p * p_hat +
                    params.cn_r * r_hat + params.cn_da * controls.aileron +
                    params.cn_dr * controls.rudder;

  AeroResult res;
  res.force_body = Eigen::Vector3d(fx_aero + t, fy_aero, fz_aero);
  res.moment_body = Eigen::Vector3d(qbar * S * b * cl_roll,
                                    qbar * S * c * cm,
                                    qbar * S * b * cn);
  res.alpha = alpha;
  res.beta = beta;
  res.airspeed = vt;
  res.dynamic_pressure = qbar;
  res.lift = lift;
  res.drag = drag;
  return res;
}

}  // namespace fw_aero