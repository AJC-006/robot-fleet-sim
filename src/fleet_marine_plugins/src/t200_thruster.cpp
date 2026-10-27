// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
// T200Thruster: gz-sim system plugin modelling a Blue Robotics T200 + Basic ESC.
//
// Input gz.msgs.Double <pwm_topic> ESC pulse width in microseconds (what an ESC really receives)
// Output  gz.msgs.Double  <thrust_topic>   thrust along the axis, N (ground truth, for tests)
//         gz.msgs.Double  <current_topic>  motor current, A (summed by the battery plugin)
// gz.msgs.Double <rpm_topic> propeller speed, rev/min (table value, drives the visual spin)
// Input   gz.msgs.Double  <voltage_topic>  bus voltage, V (from the battery plugin)
//
// Steady state: bilinear interpolation of the vendor table in (pwm, bus voltage). Dynamics:
// identified
// third-order transfer function (von Benzon et al. 2022, J. Mar. Sci. Eng. 10:1898) applied to
// force and current.
// Force is applied at the thruster position (no reaction torque: none published). All numbers come
// from the SDF
// parameters, written by fleet_auv from the component catalog.
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>  // NOLINT(build/include_order)
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include "fleet_marine_plugins/t200_core.hh"

namespace fleet_marine
{

class T200Thruster
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(_entity);
    auto get = [&](const char * k, const std::string & d) {
        return _sdf->HasElement(k) ? _sdf->Get<std::string>(k) : d;
      };
    auto getd = [&](const char * k, double d) {
        return _sdf->HasElement(k) ? _sdf->Get<double>(k) : d;
      };
    const std::string name = get("thruster_name", "thruster");
    const std::string table = get("table", "");
    if (!table_.Load(table)) {
      gzerr << "[T200Thruster " << name << "] cannot load performance table '" << table << "'\n";
      return;
    }
    bodyLink_ = model_.LinkByName(_ecm, get("body_link", "base_link"));
    propLink_ = model_.LinkByName(_ecm, get("prop_link", ""));
    joint_ = model_.JointByName(_ecm, get("joint_name", ""));
    if (bodyLink_ == gz::sim::kNullEntity || propLink_ == gz::sim::kNullEntity) {
      gzerr << "[T200Thruster " << name << "] body_link or prop_link not found\n";
      return;
    }
    axis_ = _sdf->Get<gz::math::Vector3d>("axis", gz::math::Vector3d::UnitX).first;
    axis_.Normalize();
    pwmLo_ = getd("pwm_min_us", 1100);
    pwmStop_ = getd("pwm_stop_us", 1500);
    pwmHi_ = getd("pwm_max_us", 1900);
    deadband_ = getd("deadband_us", 25);
    voltage_ = getd("initial_voltage_v", 16.0);
    pwm_ = pwmStop_;
    publishPeriod_ = 1.0 / getd("publish_rate_hz", 100.0);
    // dynamics: K(s) = (b1 s + b0) / (s^3 + a2 s^2 + a1 s + a0)
    const double b1 = getd("tf_b1", 6136), b0 = getd("tf_b0", 108700), a2 = getd("tf_a2", 89),
      a1 = getd("tf_a1", 9258), a0 = getd("tf_a0", 108700);
    forceLag_ = ThirdOrderLag(b1, b0, a2, a1, a0);
    currentLag_ = ThirdOrderLag(b1, b0, a2, a1, a0);
    auto r0 = table_.Eval(pwmStop_, voltage_);
    forceLag_.Reset(r0.force);
    currentLag_.Reset(r0.current);

    const std::string base =
      get("topic_prefix", "/model/" + model_.Name(_ecm) + "/thrusters/" + name);
    node_.Subscribe(base + "/pwm_us", &T200Thruster::OnPwm, this);
    node_.Subscribe(get("voltage_topic", "/model/" + model_.Name(_ecm) + "/battery/voltage"),
      &T200Thruster::OnVoltage, this);
    thrustPub_ = node_.Advertise<gz::msgs::Double>(base + "/thrust");
    currentPub_ = node_.Advertise<gz::msgs::Double>(base + "/current");
    rpmPub_ = node_.Advertise<gz::msgs::Double>(base + "/rpm");
    ok_ = true;
    gzmsg << "[T200Thruster " << name << "] table " << table << ", listening on " << base
          << "/pwm_us\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm) override
  {
    if (!ok_ || _info.paused) {return;}
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0) {return;}
    double pwm, volt;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      pwm = pwm_;
      volt = voltage_;
    }
    const double eff = EffectivePwm(pwm, pwmLo_, pwmStop_, pwmHi_, deadband_);
    const auto ss = table_.Eval(eff, volt);
    const double force = forceLag_.Step(ss.force, dt);
    const double current = std::max(0.0, currentLag_.Step(ss.current, dt));

    // thrust: force at the thruster, expressed about the CoM of the body link
    gz::sim::Link body(bodyLink_);
    const auto propPose = gz::sim::worldPose(propLink_, _ecm);
    const gz::math::Vector3d dirW = propPose.Rot().RotateVector(axis_);
    const gz::math::Vector3d F = dirW * force;
    auto comPose = body.WorldInertialPose(_ecm);
    if (comPose) {
      const gz::math::Vector3d r = propPose.Pos() - comPose->Pos();
      // Link::AddWorldWrench(force, torque) applies the torque about the CoM and the force at the
      // CoM
      // (checked in gz-sim8 Link.cc and Physics.cc), so the moment of the offset force is added
      // here.
      body.AddWorldWrench(_ecm, F, r.Cross(F));
    }
    if (joint_ != gz::sim::kNullEntity) {
      const double rad_s = ss.rpm * (eff >= pwmStop_ ? 1.0 : -1.0) * 0.10471975512;
      gz::sim::Joint(joint_).SetVelocity(_ecm, {rad_s});
    }
    sinceLast_ += dt;
    if (sinceLast_ >= publishPeriod_) {
      sinceLast_ = 0;
      gz::msgs::Double m;
      m.set_data(force);
      thrustPub_.Publish(m);
      m.set_data(current);
      currentPub_.Publish(m);
      m.set_data(ss.rpm);
      rpmPub_.Publish(m);
    }
  }

private:
  void OnPwm(const gz::msgs::Double & m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pwm_ = m.data();
  }
  void OnVoltage(const gz::msgs::Double & m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    voltage_ = m.data();
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity bodyLink_{gz::sim::kNullEntity}, propLink_{gz::sim::kNullEntity},
  joint_{gz::sim::kNullEntity};
  gz::math::Vector3d axis_{1, 0, 0};
  ThrusterTable table_;
  ThirdOrderLag forceLag_, currentLag_;
  double pwmLo_{1100}, pwmStop_{1500}, pwmHi_{1900}, deadband_{25}, pwm_{1500}, voltage_{16};
  double publishPeriod_{0.01}, sinceLast_{0};
  bool ok_{false};
  std::mutex mtx_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher thrustPub_, currentPub_, rpmPub_;
};

}  // namespace fleet_marine

GZ_ADD_PLUGIN(
  fleet_marine::T200Thruster, gz::sim::System,
  fleet_marine::T200Thruster::ISystemConfigure,
  fleet_marine::T200Thruster::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(fleet_marine::T200Thruster, "fleet_marine::T200Thruster")
