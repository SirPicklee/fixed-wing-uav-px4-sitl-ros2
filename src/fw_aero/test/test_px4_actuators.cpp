// PX4 -> fw_aero actuator mapping tests. Signs are checked end-to-end: a
// PX4 command for positive torque about an FRD axis must produce positive
// moment about that axis in the fw_aero model.
#include <gtest/gtest.h>

#include <cmath>

#include "fw_aero/aerodynamics.hpp"
#include "fw_aero/px4_actuators.hpp"
#include "is_close.hpp"

using fw_aero::Controls;
using fw_aero::kDegToRad;
using fw_aero::MapPx4Commands;
using fw_aero::Px4ActuatorMapping;
using fw_aero::Px4ServoAngles;
using fw_aero::test::IsClose;

namespace {

const Px4ActuatorMapping kMap{};
const double kFullScale = kMap.servo_full_scale_rad;

// Moments at a representative cruise point for the given controls.
Eigen::Vector3d CruiseMoment(const Controls& c) {
  const Eigen::Vector3d v(50.0 * std::cos(0.05), 0.0, 50.0 * std::sin(0.05));
  return fw_aero::aero_forces_moments(fw_aero::AircraftParams{}, v,
                                      Eigen::Vector3d::Zero(), c, 1000.0)
      .moment_body;
}

}  // namespace

TEST(Px4Actuators, NeutralCommandsGiveNeutralControls) {
  const auto c = MapPx4Commands(Px4ServoAngles{}, 0.0);
  EXPECT_EQ(c.aileron, 0.0);
  EXPECT_EQ(c.elevator, 0.0);
  EXPECT_EQ(c.rudder, 0.0);
  EXPECT_EQ(c.throttle, 0.0);
}

TEST(Px4Actuators, Px4PitchUpGivesNoseUpMoment) {
  Px4ServoAngles s;
  s.elevator = 0.5 * kFullScale;
  const auto m0 = CruiseMoment(MapPx4Commands({}, 0.0));
  const auto m1 = CruiseMoment(MapPx4Commands(s, 0.0));
  EXPECT_GT(m1.y(), m0.y());
}

TEST(Px4Actuators, Px4RollRightGivesRightRollMoment) {
  // PX4 allocation for positive roll: left aileron -u, right aileron +u.
  Px4ServoAngles s;
  s.left_aileron = -0.5 * kFullScale;
  s.right_aileron = 0.5 * kFullScale;
  const auto m0 = CruiseMoment(MapPx4Commands({}, 0.0));
  const auto m1 = CruiseMoment(MapPx4Commands(s, 0.0));
  EXPECT_GT(m1.x(), m0.x());
}

TEST(Px4Actuators, Px4YawRightGivesNoseRightMoment) {
  Px4ServoAngles s;
  s.rudder = 0.5 * kFullScale;
  const auto m0 = CruiseMoment(MapPx4Commands({}, 0.0));
  const auto m1 = CruiseMoment(MapPx4Commands(s, 0.0));
  EXPECT_GT(m1.z(), m0.z());
}

TEST(Px4Actuators, SymmetricAileronsGiveNoRollCommand) {
  // Both ailerons deflected the same way (flaperon-like): no roll command.
  Px4ServoAngles s;
  s.left_aileron = 0.3 * kFullScale;
  s.right_aileron = 0.3 * kFullScale;
  EXPECT_TRUE(IsClose(MapPx4Commands(s, 0.0).aileron, 0.0));
}

TEST(Px4Actuators, FullScaleGivesMaxDeflectionAndClamps) {
  Px4ServoAngles s;
  s.left_aileron = -kFullScale;
  s.right_aileron = kFullScale;
  s.elevator = 2.0 * kFullScale;  // beyond full scale: must clamp
  s.rudder = -kFullScale;
  const auto c = MapPx4Commands(s, 0.0);
  EXPECT_TRUE(IsClose(c.aileron, 20.0 * kDegToRad));
  EXPECT_TRUE(IsClose(c.elevator, -20.0 * kDegToRad));
  EXPECT_TRUE(IsClose(c.rudder, 15.0 * kDegToRad));
}

TEST(Px4Actuators, MotorCommandMapsToThrottle) {
  EXPECT_TRUE(IsClose(MapPx4Commands({}, 500.0).throttle, 0.5));
  EXPECT_TRUE(IsClose(MapPx4Commands({}, 1000.0).throttle, 1.0));
  EXPECT_TRUE(IsClose(MapPx4Commands({}, 1500.0).throttle, 1.0));
  EXPECT_TRUE(IsClose(MapPx4Commands({}, -100.0).throttle, 0.0));
}