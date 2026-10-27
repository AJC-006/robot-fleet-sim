// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
//
// Helpers shared by the actuator hardware plugins: parameter parsing, gz-sim joint component
// access, and the check that the URDF joint dynamics carry the motor's linear terms.

#ifndef GZ_JOINT_IO_HPP_
#define GZ_JOINT_IO_HPP_

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>

#include "fleet_gz_plugins/actuator_models.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fleet_gz_plugins
{
namespace io
{

using Params = std::unordered_map<std::string, std::string>;

inline double required(const Params & p, const std::string & key, const std::string & where)
{
  auto it = p.find(key);
  if (it == p.end()) {
    throw std::invalid_argument(where + ": missing parameter '" + key + "'");
  }
  return std::stod(it->second);
}

inline double optional(const Params & p, const std::string & key, double fallback)
{
  auto it = p.find(key);
  return it == p.end() ? fallback : std::stod(it->second);
}

// Reads the gearmotor constants of one joint (catalog `sim` block, see urdf/actuators.xacro).
inline DcMotorParams motor_params(const Params & p, const std::string & where)
{
  DcMotorParams m;
  m.resistance = required(p, "resistance", where);
  m.back_emf_constant = required(p, "back_emf_constant", where);
  m.torque_constant = required(p, "torque_constant", where);
  m.coulomb_current = optional(p, "coulomb_current", 0.0);
  m.viscous_current = optional(p, "viscous_current", 0.0);
  return m;
}

// Makes gz-sim publish position and velocity for the joint (as the stock GazeboSimSystem does).
inline void ensure_state_components(gz::sim::EntityComponentManager & ecm, gz::sim::Entity e)
{
  if (!ecm.Component<gz::sim::components::JointPosition>(e)) {
    ecm.CreateComponent(e, gz::sim::components::JointPosition());
  }
  if (!ecm.Component<gz::sim::components::JointVelocity>(e)) {
    ecm.CreateComponent(e, gz::sim::components::JointVelocity());
  }
}

inline double joint_position(const gz::sim::EntityComponentManager & ecm, gz::sim::Entity e)
{
  auto c = ecm.Component<gz::sim::components::JointPosition>(e);
  return (c && !c->Data().empty()) ? c->Data()[0] : 0.0;
}

inline double joint_velocity(const gz::sim::EntityComponentManager & ecm, gz::sim::Entity e)
{
  auto c = ecm.Component<gz::sim::components::JointVelocity>(e);
  return (c && !c->Data().empty()) ? c->Data()[0] : 0.0;
}

inline void set_force(gz::sim::EntityComponentManager & ecm, gz::sim::Entity e, double tau)
{
  auto c = ecm.Component<gz::sim::components::JointForceCmd>(e);
  if (c) {
    c->Data()[0] = tau;
  } else {
    ecm.CreateComponent(e, gz::sim::components::JointForceCmd({tau}));
  }
}

inline void reset_position(gz::sim::EntityComponentManager & ecm, gz::sim::Entity e, double q)
{
  ecm.CreateComponent(e, gz::sim::components::JointPositionReset({q}));
}

// The joint's SDF <dynamics> must equal the motor's implicit terms (actuator_models.hpp), else
// the simulated motor would be wrong without any visible error. Relative tolerance 1e-3 covers
// the printed precision of the URDF numbers. Also reports effort/velocity limits that would clip
// the motor (DART enforces both: gz-physics dartsim SDFFeatures.cc, dart GenericJoint setCommand).
inline bool check_joint_dynamics(
  const gz::sim::EntityComponentManager & ecm, gz::sim::Entity e, const std::string & name,
  const DcGearmotor & motor, double max_torque, const rclcpp::Logger & log)
{
  auto axis = ecm.Component<gz::sim::components::JointAxis>(e);
  if (!axis) {
    RCLCPP_ERROR(log, "joint %s has no axis", name.c_str());
    return false;
  }
  const auto & a = axis->Data();
  auto close = [](double x, double ref) {
      return std::abs(x - ref) <= 1e-3 * std::max(std::abs(ref), 1e-6);
    };
  bool ok = true;
  if (!close(a.Damping(), motor.joint_damping())) {
    RCLCPP_ERROR(
      log,
      "joint %s: URDF dynamics damping %.6g != motor back-EMF + viscous damping %.6g N m s/rad",
      name.c_str(), a.Damping(), motor.joint_damping());
    ok = false;
  }
  if (!close(a.Friction(), motor.joint_friction())) {
    RCLCPP_ERROR(
      log, "joint %s: URDF dynamics friction %.6g != motor Coulomb friction %.6g N m",
      name.c_str(), a.Friction(), motor.joint_friction());
    ok = false;
  }
  if (a.Effort() >= 0.0 && a.Effort() < max_torque * (1.0 - 1e-3)) {
    RCLCPP_WARN(
      log, "joint %s: URDF effort limit %.4g N m is below the drive's stall torque %.4g N m; "
      "DART clips the motor torque there", name.c_str(), a.Effort(), max_torque);
  }
  return ok;
}

}  // namespace io
}  // namespace fleet_gz_plugins

#endif  // GZ_JOINT_IO_HPP_
