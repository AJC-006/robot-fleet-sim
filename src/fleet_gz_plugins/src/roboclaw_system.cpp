// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
//
// RoboClawSystem: gz_ros2_control hardware plugin that stands in for a Basicmicro RoboClaw motor
// controller driving brushed DC gearmotors with quadrature encoders, behind the ROS 2 interface
// of the dumbotics roboclaw_hardware_interface driver
// (github.com/dumbotics/roboclaw_hardware_interface):
//   command interface  <joint>/velocity  [rad/s]  -> int32 counts/s = (int)(v * qppr / 2 pi)
//   state interface    <joint>/position  [rad]    =  encoder count * 2 pi / qppr
// (that driver has no velocity state; diff_drive_controller runs with position_feedback: true).
// Real-driver parameters are accepted unchanged (serial_port; joint: address, qppr, motor_type).
//
// Per physics step (gz_ros2_control calls write() in PreUpdate every step):
//   encoder count -> velocity PID (RoboClawVelocityLoop) -> duty -> terminal voltage duty * Vbat
//   -> DcGearmotor::commanded_torque -> JointForceCmd.
// The joint's URDF <dynamics damping friction> must carry the motor's back-EMF/viscous damping
// and Coulomb friction (checked in initSim); see include/fleet_gz_plugins/actuator_models.hpp.
//
// Parameters (urdf/actuators.xacro writes them from the component catalog):
//   hardware: supply_voltage [V], current_limit [A] (driver), on_resistance [ohm] (per switch;
//             two switches conduct in an H-bridge), pid_loop_rate [Hz]
//   joint:    qppr, qpps, velocity_p, velocity_i, velocity_d,
//             resistance, back_emf_constant, torque_constant, coulomb_current, viscous_current

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include "fleet_gz_plugins/actuator_models.hpp"
#include "gz_joint_io.hpp"
#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace fleet_gz_plugins
{

class RoboClawSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    for (const auto & j : info.joints) {
      if (j.command_interfaces.size() != 1 ||
        j.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY ||
        j.state_interfaces.size() != 1 ||
        j.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("RoboClawSystem"),
          "joint %s: roboclaw_hardware_interface offers exactly one velocity command and one "
          "position state", j.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    return CallbackReturn::SUCCESS;
  }

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh, std::map<std::string, gz::sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hw, gz::sim::EntityComponentManager & ecm,
    unsigned int /*update_rate*/) override
  {
    nh_ = model_nh;
    ecm_ = &ecm;
    const auto log = nh_->get_logger();
    try {
      const auto & hp = hw.hardware_parameters;
      supply_voltage_ = io::required(hp, "supply_voltage", hw.name);
      const double on_r = io::optional(hp, "on_resistance", 0.0);
      const double i_lim = io::optional(hp, "current_limit", 0.0);
      const double loop_rate = io::required(hp, "pid_loop_rate", hw.name);
      for (const auto & ji : hw.joints) {
        auto it = joints.find(ji.name);
        if (it == joints.end()) {
          RCLCPP_ERROR(log, "joint %s not found in the model", ji.name.c_str());
          return false;
        }
        Joint j;
        j.name = ji.name;
        j.entity = it->second;
        const std::string where = hw.name + "/" + ji.name;
        auto mp = io::motor_params(ji.parameters, where);
        mp.resistance += 2.0 * on_r;
        mp.current_limit = i_lim;
        j.motor = DcGearmotor(mp);
        j.qppr = io::required(ji.parameters, "qppr", where);
        j.encoder = QuadratureEncoder(j.qppr);
        RoboClawParams rp;
        rp.qpps = io::required(ji.parameters, "qpps", where);
        rp.p = io::required(ji.parameters, "velocity_p", where);
        rp.i = io::required(ji.parameters, "velocity_i", where);
        rp.d = io::required(ji.parameters, "velocity_d", where);
        rp.loop_rate = loop_rate;
        j.loop = RoboClawVelocityLoop(rp);
        io::ensure_state_components(ecm, j.entity);
        if (!io::check_joint_dynamics(
            ecm, j.entity, j.name, j.motor, j.motor.stall_torque(supply_voltage_) +
            j.motor.joint_friction(), log))
        {
          return false;
        }
        const double q0 = io::joint_position(ecm, j.entity);
        j.encoder.reset(q0);
        j.loop.reset(0);
        RCLCPP_INFO(
          log,
          "%s: RoboClaw channel, %.0f counts/rev, QPPS %.0f, %.2f V; motor no-load %.3f rad/s, "
          "stall %.3f N m", j.name.c_str(), j.qppr, rp.qpps, supply_voltage_,
          j.motor.no_load_speed(supply_voltage_), j.motor.stall_torque(supply_voltage_));
        joints_.push_back(std::move(j));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(log, "RoboClawSystem: %s", e.what());
      return false;
    }
    return !joints_.empty();
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> out;
    for (auto & j : joints_) {
      out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.position_state);
    }
    return out;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> out;
    for (auto & j : joints_) {
      out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.velocity_command);
    }
    return out;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (auto & j : joints_) {
      const auto count = j.encoder.count(io::joint_position(*ecm_, j.entity));
      j.position_state = static_cast<double>(count) * 2.0 * M_PI / j.qppr;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration &)
  override
  {
    // write() runs every physics step; its period argument is the time since the last
    // controller update, so the step is taken from the time stamps
    const double t = time.seconds();
    const double dt = has_last_ ? t - last_time_ : 0.0;
    last_time_ = t;
    has_last_ = true;
    for (auto & j : joints_) {
      const double q = io::joint_position(*ecm_, j.entity);
      const double w = io::joint_velocity(*ecm_, j.entity);
      double cmd = std::isfinite(j.velocity_command) ? j.velocity_command : 0.0;
      // roboclaw_hardware_interface: static_cast<int32_t>(velocity * qppr / (2 pi))
      const auto cmd_counts = static_cast<int32_t>(cmd * j.qppr / (2.0 * M_PI));
      double duty = 0.0;
      if (dt > 0.0) {
        duty = j.loop.step(dt, cmd_counts, j.encoder.count(q));
      }
      io::set_force(*ecm_, j.entity, j.motor.commanded_torque(duty * supply_voltage_, w));
    }
    return hardware_interface::return_type::OK;
  }

private:
  struct Joint
  {
    std::string name;
    gz::sim::Entity entity{gz::sim::kNullEntity};
    DcGearmotor motor;
    QuadratureEncoder encoder;
    RoboClawVelocityLoop loop;
    double qppr{1.0};
    double position_state{0.0};
    double velocity_command{0.0};
  };
  std::vector<Joint> joints_;
  gz::sim::EntityComponentManager * ecm_{nullptr};
  double supply_voltage_{0.0};
  double last_time_{0.0};
  bool has_last_{false};
};

}  // namespace fleet_gz_plugins

PLUGINLIB_EXPORT_CLASS(
  fleet_gz_plugins::RoboClawSystem, gz_ros2_control::GazeboSimSystemInterface)
