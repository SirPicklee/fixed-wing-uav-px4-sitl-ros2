// Frame conversions between the aerospace conventions used by fw_aero and
// the robotics conventions used by Gazebo / ROS (REP 103).
//
//   fw_aero (aerospace)                 Gazebo (robotics)
//   body  FRD: x fwd, y right, z down   body  FLU: x fwd, y left, z up
//   world NED: north, east, down        world ENU: east, north, up
//
// Both FRD<->FLU and NED<->ENU are proper rotations by 180 deg and 90 deg
// combinations that are their own inverse when applied to vector components.
// The Gazebo link orientation q_enu_flu rotates FLU body vectors into the
// ENU world frame.
#pragma once

#include <Eigen/Dense>

namespace fw_aero {

// Body axes: FLU <-> FRD (rotation of 180 deg about x).
inline Eigen::Vector3d FluToFrd(const Eigen::Vector3d& v) {
  return {v.x(), -v.y(), -v.z()};
}
inline Eigen::Vector3d FrdToFlu(const Eigen::Vector3d& v) {
  return {v.x(), -v.y(), -v.z()};
}

// World axes: ENU <-> NED (swap x/y, negate z).
inline Eigen::Vector3d EnuToNed(const Eigen::Vector3d& v) {
  return {v.y(), v.x(), -v.z()};
}
inline Eigen::Vector3d NedToEnu(const Eigen::Vector3d& v) {
  return {v.y(), v.x(), -v.z()};
}

// World-ENU vector (velocity, angular velocity, wind) -> body-FRD components.
inline Eigen::Vector3d WorldEnuToBodyFrd(const Eigen::Quaterniond& q_enu_flu,
                                         const Eigen::Vector3d& v_enu) {
  return FluToFrd(q_enu_flu.conjugate() * v_enu);
}

// Body-FRD vector (force, moment) -> world-ENU components.
inline Eigen::Vector3d BodyFrdToWorldEnu(const Eigen::Quaterniond& q_enu_flu,
                                         const Eigen::Vector3d& v_frd) {
  return q_enu_flu * FrdToFlu(v_frd);
}

}  // namespace fw_aero