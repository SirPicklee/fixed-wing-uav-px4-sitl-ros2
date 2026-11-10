// Port of uavsim/atmosphere.py (uav-6dof-autopilot v1.0, commit 1dab032).
#include "fw_aero/atmosphere.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace fw_aero {

namespace {

constexpr double kTTropopause = kT0 + kLapse * kHTropopause;  // 216.65 K
const double kPTropopause =
    kP0 * std::pow(kTTropopause / kT0, -kG0 / (kLapse * kRAir));

}  // namespace

AtmosphereState atmosphere(double altitude_m, double delta_isa) {
  const double h = altitude_m;
  if (h < 0.0 || h > kHMax) {
    throw std::out_of_range("Altitude " + std::to_string(h) +
                            " m is outside the supported range 0-20000 m.");
  }

  double t_std = 0.0;
  double p = 0.0;
  if (h <= kHTropopause) {
    t_std = kT0 + kLapse * h;
    p = kP0 * std::pow(t_std / kT0, -kG0 / (kLapse * kRAir));
  } else {
    t_std = kTTropopause;
    p = kPTropopause *
        std::exp(-kG0 * (h - kHTropopause) / (kRAir * kTTropopause));
  }

  const double t = t_std + delta_isa;
  const double rho = p / (kRAir * t);
  const double a = std::sqrt(kGamma * kRAir * t);
  return AtmosphereState{h, t, p, rho, a};
}

double dynamic_pressure(double density, double airspeed) {
  return 0.5 * density * airspeed * airspeed;
}

double mach_number(double airspeed, double speed_of_sound) {
  return airspeed / speed_of_sound;
}

}  // namespace fw_aero