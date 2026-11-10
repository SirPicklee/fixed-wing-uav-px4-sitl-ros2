// Gazebo Harmonic system plugin: fw_aero aerodynamics + PX4 SITL actuators.
//
// Each PreUpdate step:
//   1. read link pose, linear and angular velocity from Gazebo (ENU / FLU)
//   2. convert to body FRD                       (fw_aero/frames.hpp)
//   3. map latest PX4 servo / motor commands     (fw_aero/px4_actuators.hpp)
//   4. compute aero + thrust force and moment    (fw_aero/aerodynamics.hpp)
//   5. convert back to world ENU and apply at the link origin
//
// The link origin must coincide with the centre of gravity: AddWorldWrench
// applies the force at the link origin, and the aero moments are about the CG.
//
// SDF parameters (all optional):
//   <link_name>           link to act on                    (default base_link)
//   <reference_altitude>  altitude of the world origin [m]  (default 0)
//   <initial_airspeed>    forward speed set once at start [m/s] (default 0,
//                         disabled; for standalone tests without PX4)
//   <log_file>            CSV log path (default empty, disabled)
//   <log_rate>            CSV log rate [Hz] (default 20)

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>

#include <array>
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "fw_aero/aerodynamics.hpp"
#include "fw_aero/frames.hpp"
#include "fw_aero/px4_actuators.hpp"

namespace fw_aero_gazebo {

namespace {

Eigen::Vector3d ToEigen(const gz::math::Vector3d& v) {
  return {v.X(), v.Y(), v.Z()};
}

Eigen::Quaterniond ToEigen(const gz::math::Quaterniond& q) {
  return {q.W(), q.X(), q.Y(), q.Z()};
}

gz::math::Vector3d ToGz(const Eigen::Vector3d& v) {
  return {v.x(), v.y(), v.z()};
}

}  // namespace

class FixedWingAeroSystem : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& /*event_mgr*/) override {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      gzerr << "[FixedWingAeroSystem] Must be attached to a model.\n";
      return;
    }

    const std::string link_name =
        sdf->Get<std::string>("link_name", "base_link").first;
    link_ = gz::sim::Link(model_.LinkByName(ecm, link_name));
    if (!link_.Valid(ecm)) {
      gzerr << "[FixedWingAeroSystem] Link [" << link_name
            << "] not found.\n";
      return;
    }
    // Make Gazebo populate world velocity components for this link.
    link_.EnableVelocityChecks(ecm, true);

    reference_altitude_m_ =
        sdf->Get<double>("reference_altitude", 0.0).first;
    initial_airspeed_ = sdf->Get<double>("initial_airspeed", 0.0).first;

    const std::string log_file =
        sdf->Get<std::string>("log_file", "").first;
    log_period_s_ = 1.0 / sdf->Get<double>("log_rate", 20.0).first;
    if (!log_file.empty()) {
      log_.open(log_file);
      if (log_) {
        log_ << "t,x,y,z,vx,vy,vz,airspeed,alpha,beta,p,q,r,"
                "elevator,aileron,rudder,throttle,fx,fy,fz,l,m,n\n";
      } else {
        gzerr << "[FixedWingAeroSystem] Cannot open log file " << log_file
              << "\n";
      }
    }

    // PX4 gz_bridge topics (see fw_aero/px4_actuators.hpp).
    const std::string model_name = model_.Name(ecm);
    for (std::size_t i = 0; i < servo_rad_.size(); ++i) {
      const std::string topic =
          "/model/" + model_name + "/servo_" + std::to_string(i);
      std::function<void(const gz::msgs::Double&)> cb =
          [this, i](const gz::msgs::Double& msg) {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            servo_rad_[i] = msg.data();
          };
      if (!node_.Subscribe(topic, cb)) {
        gzerr << "[FixedWingAeroSystem] Failed to subscribe to " << topic
              << "\n";
      }
    }

    const std::string motor_topic = "/" + model_name + "/command/motor_speed";
    std::function<void(const gz::msgs::Actuators&)> motor_cb =
        [this](const gz::msgs::Actuators& msg) {
          if (msg.velocity_size() > 0) {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            motor_command_ = msg.velocity(0);
          }
        };
    if (!node_.Subscribe(motor_topic, motor_cb)) {
      gzerr << "[FixedWingAeroSystem] Failed to subscribe to " << motor_topic
            << "\n";
    }

    gzmsg << "[FixedWingAeroSystem] Active on model [" << model_name
          << "], link [" << link_name << "], aircraft [" << params_.name
          << "], reference altitude " << reference_altitude_m_
          << " m, initial airspeed " << initial_airspeed_ << " m/s\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
    if (info.paused || !link_.Valid(ecm)) {
      return;
    }

    // Optional initial airspeed (standalone tests). Gazebo keeps a zeroed
    // LinearVelocityCmd component after applying it and would then force
    // zero velocity on every step, so the component is removed one step
    // after it has been applied.
    if (initial_airspeed_ > 0.0 && init_stage_ < 2) {
      if (init_stage_ == 0) {
        link_.SetLinearVelocity(ecm, {initial_airspeed_, 0.0, 0.0});
      } else {
        ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(
            link_.Entity());
      }
      ++init_stage_;
      return;
    }

    const auto pose = link_.WorldPose(ecm);
    const auto vel = link_.WorldLinearVelocity(ecm);
    const auto omega = link_.WorldAngularVelocity(ecm);
    if (!pose || !vel || !omega) {
      return;  // velocity components are created after the first step
    }

    const Eigen::Quaterniond q_enu_flu = ToEigen(pose->Rot());
    const Eigen::Vector3d v_frd =
        fw_aero::WorldEnuToBodyFrd(q_enu_flu, ToEigen(*vel));
    const Eigen::Vector3d omega_frd =
        fw_aero::WorldEnuToBodyFrd(q_enu_flu, ToEigen(*omega));

    fw_aero::Px4ServoAngles servo;
    double motor = 0.0;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      servo.left_aileron = servo_rad_[0];
      servo.right_aileron = servo_rad_[1];
      servo.elevator = servo_rad_[2];
      servo.rudder = servo_rad_[3];
      motor = motor_command_;
    }
    const fw_aero::Controls controls =
        fw_aero::MapPx4Commands(servo, motor, mapping_);

    const double altitude_m = reference_altitude_m_ + pose->Pos().Z();
    const fw_aero::AeroResult res = fw_aero::aero_forces_moments(
        params_, v_frd, omega_frd, controls, altitude_m);

    link_.AddWorldWrench(
        ecm, ToGz(fw_aero::BodyFrdToWorldEnu(q_enu_flu, res.force_body)),
        ToGz(fw_aero::BodyFrdToWorldEnu(q_enu_flu, res.moment_body)));

    const double t = std::chrono::duration<double>(info.simTime).count();
    if (log_.is_open() && t >= next_log_time_s_) {
      next_log_time_s_ = t + log_period_s_;
      const auto& p = pose->Pos();
      log_ << t << ',' << p.X() << ',' << p.Y() << ',' << p.Z() << ','
           << vel->X() << ',' << vel->Y() << ',' << vel->Z() << ','
           << res.airspeed << ',' << res.alpha << ',' << res.beta << ','
           << omega_frd.x() << ',' << omega_frd.y() << ',' << omega_frd.z()
           << ',' << controls.elevator << ',' << controls.aileron << ','
           << controls.rudder << ',' << controls.throttle << ','
           << res.force_body.x() << ',' << res.force_body.y() << ','
           << res.force_body.z() << ',' << res.moment_body.x() << ','
           << res.moment_body.y() << ',' << res.moment_body.z() << '\n';
    }
  }

 private:
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};
  fw_aero::AircraftParams params_;
  fw_aero::Px4ActuatorMapping mapping_;
  double reference_altitude_m_ = 0.0;
  double initial_airspeed_ = 0.0;
  int init_stage_ = 0;  // 0: set velocity, 1: remove command, 2: done

  std::ofstream log_;
  double log_period_s_ = 0.05;
  double next_log_time_s_ = 0.0;

  gz::transport::Node node_;
  std::mutex cmd_mutex_;
  // Latest PX4 servo angles [rad]: left aileron, right aileron, elevator,
  // rudder (servo_0 .. servo_3).
  std::array<double, 4> servo_rad_{};
  double motor_command_ = 0.0;
};

}  // namespace fw_aero_gazebo

GZ_ADD_PLUGIN(fw_aero_gazebo::FixedWingAeroSystem, gz::sim::System,
              fw_aero_gazebo::FixedWingAeroSystem::ISystemConfigure,
              fw_aero_gazebo::FixedWingAeroSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(fw_aero_gazebo::FixedWingAeroSystem,
                    "fw_aero_gazebo::FixedWingAeroSystem")
