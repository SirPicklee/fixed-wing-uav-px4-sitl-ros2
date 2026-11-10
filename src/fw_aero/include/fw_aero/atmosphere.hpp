// International Standard Atmosphere (ISA), 0-20 km geopotential altitude.
//
// Port of uavsim/atmosphere.py (uav-6dof-autopilot v1.0, commit 1dab032).
// Layers:
//   troposphere   0 - 11 km : linear temperature lapse rate (-6.5 K/km)
//   tropopause   11 - 20 km : isothermal at 216.65 K
#pragma once

namespace fw_aero {

// Sea-level standard constants
inline constexpr double kT0 = 288.15;            // K
inline constexpr double kP0 = 101325.0;          // Pa
inline constexpr double kRho0 = 1.225;           // kg/m^3
inline constexpr double kG0 = 9.80665;           // m/s^2
inline constexpr double kRAir = 287.05287;       // J/(kg K)
inline constexpr double kGamma = 1.4;
inline constexpr double kLapse = -0.0065;        // K/m, troposphere
inline constexpr double kHTropopause = 11000.0;  // m
inline constexpr double kHMax = 20000.0;         // m

struct AtmosphereState {
  double altitude;        // m
  double temperature;     // K
  double pressure;        // Pa
  double density;         // kg/m^3
  double speed_of_sound;  // m/s
};

// ISA properties at altitude_m. delta_isa is a temperature offset from the
// standard day [K]; pressure follows the standard profile and density is
// corrected with the offset temperature.
// Throws std::out_of_range if altitude_m is outside [0, kHMax].
AtmosphereState atmosphere(double altitude_m, double delta_isa = 0.0);

// Dynamic pressure q = 0.5 * rho * V^2 [Pa].
double dynamic_pressure(double density, double airspeed);

// Mach number from true airspeed and local speed of sound.
double mach_number(double airspeed, double speed_of_sound);

}  // namespace fw_aero