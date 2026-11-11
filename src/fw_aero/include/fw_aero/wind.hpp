// Atmospheric disturbances: Dryden-type turbulence for the fw_aero model.
//
// Intensities and length scales follow the MIL-F-8785C low-altitude model
// (also MIL-HDBK-1797; Beard & McLain, "Small Unmanned Aircraft", ch. 4),
// with h the altitude in feet, clamped to the model range 10 .. 1000 ft:
//   L_w = h,          L_u = L_v = h / (0.177 + 0.000823 h)^1.2     [ft]
//   sigma_w = 0.1 W20, sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4
// W20 is the mean wind speed at 20 ft: light 15 kt, moderate 30 kt,
// severe 45 kt.
//
// Each body-axis component is a first-order Gauss-Markov process with
// correlation distance L, discretized exactly for a step dt at airspeed V:
//   x[k+1] = a x[k] + sigma sqrt(1 - a^2) n[k],   a = exp(-V dt / L)
// This reproduces the Dryden longitudinal (u) spectrum exactly; for the
// lateral and vertical components it keeps the Dryden variance and length
// scale but uses a first-order instead of the second-order spectral shape.
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>

namespace fw_aero {

struct DrydenScales {
  double sigma_u, sigma_v, sigma_w;  // gust intensities [m/s]
  double length_u, length_v, length_w;  // correlation lengths [m]
};

inline DrydenScales DrydenLowAltitude(double altitude_m, double w20_mps) {
  constexpr double kFt = 0.3048;
  const double h_ft = std::clamp(altitude_m / kFt, 10.0, 1000.0);
  const double d = 0.177 + 0.000823 * h_ft;
  DrydenScales s{};
  s.sigma_w = 0.1 * w20_mps;
  s.sigma_u = s.sigma_v = s.sigma_w / std::pow(d, 0.4);
  s.length_w = h_ft * kFt;
  s.length_u = s.length_v = h_ft / std::pow(d, 1.2) * kFt;
  return s;
}

// Body-axis (FRD) gust velocity generator. Deterministic for a given seed,
// so Monte Carlo runs can be reproduced.
class DrydenTurbulence {
 public:
  DrydenTurbulence(double w20_mps, std::uint32_t seed)
      : w20_mps_(w20_mps), rng_(seed) {}

  // Advance by dt [s] at the given altitude [m] and airspeed [m/s] and
  // return the gust velocity in body FRD axes [m/s]. The first call draws
  // the state from the stationary distribution (no start-up transient).
  Eigen::Vector3d Step(double altitude_m, double airspeed_mps, double dt) {
    if (w20_mps_ <= 0.0) {
      return Eigen::Vector3d::Zero();
    }
    const DrydenScales s = DrydenLowAltitude(altitude_m, w20_mps_);
    const double sigma[3] = {s.sigma_u, s.sigma_v, s.sigma_w};
    const double length[3] = {s.length_u, s.length_v, s.length_w};
    const double travel = std::max(airspeed_mps, 0.0) * dt;
    for (int i = 0; i < 3; ++i) {
      if (!started_) {
        gust_[i] = sigma[i] * normal_(rng_);
        continue;
      }
      const double a = std::exp(-travel / length[i]);
      gust_[i] = a * gust_[i] + sigma[i] * std::sqrt(1.0 - a * a) * normal_(rng_);
    }
    started_ = true;
    return gust_;
  }

 private:
  double w20_mps_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  Eigen::Vector3d gust_ = Eigen::Vector3d::Zero();
  bool started_ = false;
};

}  // namespace fw_aero
