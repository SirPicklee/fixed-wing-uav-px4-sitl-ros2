// Port of tests/test_aerodynamics.py (uav-6dof-autopilot v1.0, commit 1dab032).
#include <gtest/gtest.h>

#include <cmath>

#include "fw_aero/aerodynamics.hpp"
#include "is_close.hpp"

using fw_aero::AeroResult;
using fw_aero::AircraftParams;
using fw_aero::Controls;
using fw_aero::aero_forces_moments;
using fw_aero::air_data;
using fw_aero::thrust;
using fw_aero::test::IsClose;

namespace {

const AircraftParams P{};

Eigen::Vector3d Vel(double alpha, double vt = 50.0) {
  return {vt * std::cos(alpha), 0.0, vt * std::sin(alpha)};
}

// Named Compute (not Run) to avoid clashing with testing::Test::Run().
AeroResult Compute(const Eigen::Vector3d& v,
                   const Eigen::Vector3d& omega = Eigen::Vector3d::Zero(),
                   const Controls& controls = Controls{}, double alt = 1000.0) {
  return aero_forces_moments(P, v, omega, controls, alt);
}

}  // namespace

TEST(Aerodynamics, AirDataStraightFlight) {
  const auto ad = air_data(Eigen::Vector3d(50.0, 0.0, 0.0));
  EXPECT_TRUE(IsClose(ad.airspeed, 50.0));
  EXPECT_TRUE(IsClose(ad.alpha, 0.0));
  EXPECT_TRUE(IsClose(ad.beta, 0.0));
}

TEST(Aerodynamics, AirDataAlphaAndBeta) {
  auto ad = air_data(Eigen::Vector3d(50.0, 0.0, 5.0));
  EXPECT_TRUE(IsClose(ad.alpha, std::atan2(5.0, 50.0)));
  EXPECT_TRUE(IsClose(ad.beta, 0.0));
  ad = air_data(Eigen::Vector3d(50.0, 5.0, 0.0));
  EXPECT_TRUE(IsClose(ad.beta, std::asin(5.0 / ad.airspeed)));
  EXPECT_TRUE(IsClose(ad.alpha, 0.0));
}

TEST(Aerodynamics, ZeroAirspeedGivesZeroForceAndMoment) {
  const auto res = Compute(Eigen::Vector3d::Zero());
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(IsClose(res.force_body[i], 0.0)) << "force " << i;
    EXPECT_TRUE(IsClose(res.moment_body[i], 0.0)) << "moment " << i;
  }
}

TEST(Aerodynamics, LiftIncreasesWithAlpha) {
  const auto low = Compute(Vel(0.0));
  const auto high = Compute(Vel(0.05));
  EXPECT_GT(high.lift, low.lift);
  // Lift acts upward: negative body z force (z is down)
  EXPECT_LT(high.force_body.z(), 0.0);
}

TEST(Aerodynamics, LiftSaturatesAtClMax) {
  const auto a = Compute(Vel(0.5));
  const auto b = Compute(Vel(0.6));
  const double expected = a.dynamic_pressure * P.wing_area * P.cl_max;
  EXPECT_TRUE(IsClose(a.lift, expected));
  EXPECT_TRUE(IsClose(b.lift, expected));
  // Post-stall drag penalty grows with depth of stall
  EXPECT_GT(b.drag, a.drag);
}

TEST(Aerodynamics, DragMatchesPolar) {
  const auto res = Compute(Vel(0.0));
  const double cl = P.cl0;
  const double cd = P.cd0 + P.induced_drag_k() * cl * cl;
  EXPECT_TRUE(IsClose(res.drag, res.dynamic_pressure * P.wing_area * cd));
}

TEST(Aerodynamics, StaticPitchStability) {
  const double m0 = Compute(Vel(0.0)).moment_body.y();
  const double m1 = Compute(Vel(0.1)).moment_body.y();
  EXPECT_LT(m1, m0);
}

TEST(Aerodynamics, PitchDampingOpposesPitchRate) {
  const double m0 =
      Compute(Vel(0.0), Eigen::Vector3d(0.0, 0.0, 0.0)).moment_body.y();
  const double m1 =
      Compute(Vel(0.0), Eigen::Vector3d(0.0, 0.5, 0.0)).moment_body.y();
  EXPECT_LT(m1, m0);
}

TEST(Aerodynamics, RollDampingOpposesRollRate) {
  const auto res = Compute(Eigen::Vector3d(50.0, 0.0, 0.0),
                           Eigen::Vector3d(0.5, 0.0, 0.0));
  EXPECT_LT(res.moment_body.x(), 0.0);
}

TEST(Aerodynamics, SideslipRestoringSigns) {
  const auto res = Compute(Eigen::Vector3d(50.0, 5.0, 0.0));
  EXPECT_GT(res.beta, 0.0);
  EXPECT_LT(res.force_body.y(), 0.0);   // side force opposes sideslip
  EXPECT_LT(res.moment_body.x(), 0.0);  // dihedral effect
  EXPECT_GT(res.moment_body.z(), 0.0);  // weathercock stability
}

TEST(Aerodynamics, ElevatorGivesNoseDownMoment) {
  const double m0 = Compute(Vel(0.0)).moment_body.y();
  Controls c;
  c.elevator = 0.1;
  const double m1 =
      Compute(Vel(0.0), Eigen::Vector3d::Zero(), c).moment_body.y();
  EXPECT_LT(m1, m0);
}

TEST(Aerodynamics, ThrustModel) {
  EXPECT_TRUE(IsClose(thrust(P, 1.0, 0.0, 1.225), P.max_thrust_sl));
  EXPECT_TRUE(IsClose(thrust(P, 0.0, 0.0, 1.225), 0.0));
  EXPECT_TRUE(IsClose(thrust(P, 1.0, 50.0, 1.225), P.max_thrust_sl * 0.4));
  EXPECT_TRUE(IsClose(thrust(P, 1.0, 0.0, 0.6125), P.max_thrust_sl * 0.5));
  EXPECT_TRUE(IsClose(thrust(P, 2.0, 0.0, 1.225), P.max_thrust_sl));
  EXPECT_TRUE(IsClose(thrust(P, 1.0, 100.0, 1.225), 0.0));
}