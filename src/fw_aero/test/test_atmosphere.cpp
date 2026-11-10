// Port of tests/test_atmosphere.py (uav-6dof-autopilot v1.0, commit 1dab032).
#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "fw_aero/atmosphere.hpp"
#include "is_close.hpp"

using fw_aero::atmosphere;
using fw_aero::dynamic_pressure;
using fw_aero::mach_number;
using fw_aero::test::IsClose;

TEST(Atmosphere, SeaLevelValues) {
  const auto s = atmosphere(0.0);
  EXPECT_TRUE(IsClose(s.temperature, 288.15));
  EXPECT_TRUE(IsClose(s.pressure, 101325.0));
  EXPECT_TRUE(IsClose(s.density, fw_aero::kRho0, 1e-3));
  EXPECT_TRUE(IsClose(s.speed_of_sound, 340.294, 1e-4));
}

TEST(Atmosphere, Matches5000mIsaTable) {
  const auto s = atmosphere(5000.0);
  EXPECT_TRUE(IsClose(s.temperature, 255.65, 1e-5, 1e-2));
  EXPECT_TRUE(IsClose(s.pressure, 54019.9, 1e-3));
  EXPECT_TRUE(IsClose(s.density, 0.7364, 1e-3));
}

TEST(Atmosphere, MatchesTropopause11kmIsaTable) {
  const auto s = atmosphere(11000.0);
  EXPECT_TRUE(IsClose(s.temperature, 216.65, 1e-5, 1e-2));
  EXPECT_TRUE(IsClose(s.pressure, 22632.1, 1e-3));
  EXPECT_TRUE(IsClose(s.density, 0.3639, 1e-3));
  EXPECT_TRUE(IsClose(s.speed_of_sound, 295.07, 1e-3));
}

TEST(Atmosphere, Matches20kmIsaTable) {
  const auto s = atmosphere(20000.0);
  EXPECT_TRUE(IsClose(s.temperature, 216.65, 1e-5, 1e-2));
  EXPECT_TRUE(IsClose(s.pressure, 5474.89, 1e-3));
  EXPECT_TRUE(IsClose(s.density, 0.08803, 1e-3));
}

TEST(Atmosphere, DensityDecreasesWithAltitude) {
  // Same grid as np.linspace(0, 20000, 50)
  const int n = 50;
  double prev = atmosphere(0.0).density;
  for (int i = 1; i < n; ++i) {
    const double h = 20000.0 * i / (n - 1);
    const double rho = atmosphere(h).density;
    EXPECT_LT(rho, prev) << "at h = " << h << " m";
    prev = rho;
  }
}

TEST(Atmosphere, PressureIsContinuousAtTropopause) {
  const double below = atmosphere(10999.99).pressure;
  const double above = atmosphere(11000.01).pressure;
  EXPECT_TRUE(IsClose(below, above, 1e-5));
}

TEST(Atmosphere, HotDayLowersDensity) {
  const auto std_day = atmosphere(3000.0);
  const auto hot_day = atmosphere(3000.0, 15.0);
  EXPECT_LT(hot_day.density, std_day.density);
  EXPECT_TRUE(IsClose(hot_day.pressure, std_day.pressure));
}

TEST(Atmosphere, OutOfRangeThrows) {
  EXPECT_THROW(atmosphere(-1.0), std::out_of_range);
  EXPECT_THROW(atmosphere(25000.0), std::out_of_range);
}

TEST(Atmosphere, DynamicPressureAndMach) {
  EXPECT_TRUE(IsClose(dynamic_pressure(1.225, 100.0), 6125.0));
  EXPECT_TRUE(IsClose(mach_number(340.294, 340.294), 1.0));
}