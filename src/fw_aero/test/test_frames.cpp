// Frame conversion tests. Each case is a physical scenario whose correct
// answer is known independently of the code (sign conventions from flight
// mechanics), so a sign error in any conversion fails at least one test.
#include <gtest/gtest.h>

#include <cmath>

#include "fw_aero/aerodynamics.hpp"
#include "fw_aero/frames.hpp"
#include "is_close.hpp"

using fw_aero::BodyFrdToWorldEnu;
using fw_aero::EnuToNed;
using fw_aero::FluToFrd;
using fw_aero::FrdToFlu;
using fw_aero::NedToEnu;
using fw_aero::WorldEnuToBodyFrd;
using fw_aero::test::IsClose;

namespace {

// Gazebo yaw is measured counter-clockwise from east about world up (z).
Eigen::Quaterniond YawEnu(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

void ExpectVecClose(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(IsClose(a[i], b[i], 1e-12, 1e-12)) << "component " << i;
  }
}

}  // namespace

TEST(Frames, ConversionsAreSelfInverse) {
  const Eigen::Vector3d v(1.0, 2.0, 3.0);
  ExpectVecClose(FrdToFlu(FluToFrd(v)), v);
  ExpectVecClose(NedToEnu(EnuToNed(v)), v);
}

TEST(Frames, EnuToNedAxisMapping) {
  ExpectVecClose(EnuToNed({1.0, 0.0, 0.0}), {0.0, 1.0, 0.0});  // east
  ExpectVecClose(EnuToNed({0.0, 1.0, 0.0}), {1.0, 0.0, 0.0});  // north
  ExpectVecClose(EnuToNed({0.0, 0.0, 1.0}), {0.0, 0.0, -1.0}); // up
}

TEST(Frames, FlyingNorthGivesPureForwardVelocity) {
  // Nose pointing north = Gazebo yaw +90 deg (counter-clockwise from east)
  const auto q = YawEnu(M_PI / 2.0);
  ExpectVecClose(WorldEnuToBodyFrd(q, {0.0, 50.0, 0.0}), {50.0, 0.0, 0.0});
}

TEST(Frames, DescendingGivesPositiveAlpha) {
  // Level attitude, nose east, sinking at 5 m/s: relative wind from below.
  const auto q = YawEnu(0.0);
  const auto v_frd = WorldEnuToBodyFrd(q, {50.0, 0.0, -5.0});
  EXPECT_GT(v_frd.z(), 0.0);  // w > 0 (z down)
  EXPECT_GT(fw_aero::air_data(v_frd).alpha, 0.0);
}

TEST(Frames, DriftingRightGivesPositiveBeta) {
  // Nose east, drifting to the right (south = -y ENU).
  const auto q = YawEnu(0.0);
  const auto v_frd = WorldEnuToBodyFrd(q, {50.0, -5.0, 0.0});
  EXPECT_GT(v_frd.y(), 0.0);  // v > 0 (y right)
  EXPECT_GT(fw_aero::air_data(v_frd).beta, 0.0);
}

TEST(Frames, RollRightRateIsPositiveP) {
  // Nose east; right wing down rotation: angular velocity along +x (forward).
  const auto q = YawEnu(0.0);
  ExpectVecClose(WorldEnuToBodyFrd(q, {0.5, 0.0, 0.0}), {0.5, 0.0, 0.0});
}

TEST(Frames, NoseUpRateIsPositiveQ) {
  // Nose east; nose-up rotation is about the right wing (south = -y ENU).
  const auto q = YawEnu(0.0);
  ExpectVecClose(WorldEnuToBodyFrd(q, {0.0, -0.5, 0.0}), {0.0, 0.5, 0.0});
}

TEST(Frames, NoseRightRateIsPositiveR) {
  // Nose east; nose-right (clockwise seen from above) is about world down.
  const auto q = YawEnu(0.0);
  ExpectVecClose(WorldEnuToBodyFrd(q, {0.0, 0.0, -0.5}), {0.0, 0.0, 0.5});
}

TEST(Frames, LiftPointsUpInWorld) {
  // Level flight, any heading: body -z (FRD up) must map to world +z (ENU up).
  for (double yaw : {0.0, 1.0, -2.5}) {
    ExpectVecClose(BodyFrdToWorldEnu(YawEnu(yaw), {0.0, 0.0, -1000.0}),
                   {0.0, 0.0, 1000.0});
  }
}

TEST(Frames, BankedTurnLiftTiltsTowardTurn) {
  // Nose north, banked 30 deg right (right wing down): lift tilts east.
  // In FLU, right wing down is a positive rotation about the forward axis.
  const auto q = YawEnu(M_PI / 2.0) *
                 Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 6.0,
                                                      Eigen::Vector3d::UnitX()));
  const auto lift_enu = BodyFrdToWorldEnu(q, {0.0, 0.0, -1000.0});
  EXPECT_TRUE(IsClose(lift_enu.x(), 1000.0 * std::sin(M_PI / 6.0), 1e-12));
  EXPECT_TRUE(IsClose(lift_enu.z(), 1000.0 * std::cos(M_PI / 6.0), 1e-12));
}

TEST(Frames, RoundTripForArbitraryAttitude) {
  const Eigen::Quaterniond q =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d v(12.0, -3.5, 7.25);
  ExpectVecClose(BodyFrdToWorldEnu(q, WorldEnuToBodyFrd(q, v)), v);
}