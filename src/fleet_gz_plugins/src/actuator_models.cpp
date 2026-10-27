// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
// Model equations and sources: include/fleet_gz_plugins/actuator_models.hpp

#include "fleet_gz_plugins/actuator_models.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fleet_gz_plugins
{

namespace
{
constexpr double kTwoPi = 6.283185307179586;
double sgn(double x) {return (x > 0.0) - (x < 0.0);}
}  // namespace

DcGearmotor::DcGearmotor(const DcMotorParams & p)
: p_(p) {}

double DcGearmotor::joint_damping() const
{
  return p_.torque_constant * p_.back_emf_constant / p_.resistance +
         p_.torque_constant * p_.viscous_current;
}

double DcGearmotor::joint_friction() const
{
  return p_.torque_constant * p_.coulomb_current;
}

double DcGearmotor::current(double v, double w) const
{
  const double i = (v - p_.back_emf_constant * w) / p_.resistance;
  if (p_.current_limit > 0.0) {
    return std::clamp(i, -p_.current_limit, p_.current_limit);
  }
  return i;
}

double DcGearmotor::commanded_torque(double v, double w) const
{
  // kt/R * V_eff with V_eff = ke w + R i: equals kt V / R unless the current limit clamps i
  return p_.torque_constant / p_.resistance *
         (p_.back_emf_constant * w + p_.resistance * current(v, w));
}

double DcGearmotor::output_torque(double v, double w) const
{
  return p_.torque_constant *
         (current(v, w) - p_.coulomb_current * sgn(w) - p_.viscous_current * w);
}

double DcGearmotor::no_load_speed(double v) const
{
  // 0 = (v - ke w)/R - i_c - i_v w  (current limit inactive at no load)
  return (v - p_.resistance * p_.coulomb_current) /
         (p_.back_emf_constant + p_.resistance * p_.viscous_current);
}

double DcGearmotor::stall_torque(double v) const
{
  return p_.torque_constant * (current(v, 0.0) - p_.coulomb_current);
}

QuadratureEncoder::QuadratureEncoder(double counts_per_rev)
: cpr_(counts_per_rev) {}

void QuadratureEncoder::reset(double angle) {zero_ = angle;}

int64_t QuadratureEncoder::count(double angle) const
{
  return static_cast<int64_t>(std::floor((angle - zero_) * cpr_ / kTwoPi));
}

RoboClawVelocityLoop::RoboClawVelocityLoop(const RoboClawParams & p)
: p_(p) {}

void RoboClawVelocityLoop::reset(int64_t count)
{
  last_count_ = count;
  t_since_ = 0.0;
  meas_ = 0.0;
  integ_ = 0.0;
  duty_ = 0.0;
}

double RoboClawVelocityLoop::step(double dt, int32_t cmd, int64_t count)
{
  t_since_ += dt;
  const double period = 1.0 / p_.loop_rate;
  if (t_since_ + 1e-12 < period) {
    return duty_;
  }
  const double meas = static_cast<double>(count - last_count_) / t_since_;
  const double dmeas = (meas - meas_) / t_since_;
  const double e = (static_cast<double>(cmd) - meas) / p_.qpps;
  const double ff = static_cast<double>(cmd) / p_.qpps;
  const double integ_new = integ_ + e * t_since_;
  const double unclamped = ff + p_.p * e + p_.i * integ_new - p_.d * dmeas / p_.qpps;
  duty_ = std::clamp(unclamped, -1.0, 1.0);
  if (unclamped == duty_ || (unclamped > 1.0 && e < 0.0) || (unclamped < -1.0 && e > 0.0)) {
    integ_ = integ_new;  // integrate only when not pushing further into saturation
  }
  meas_ = meas;
  last_count_ = count;
  t_since_ = 0.0;
  return duty_;
}

DynamixelPositionLoop::DynamixelPositionLoop(const DynamixelParams & p)
: p_(p) {}

void DynamixelPositionLoop::reset(double present_position)
{
  desired_ = present_position;
  desired_vel_ = 0.0;
  integ_ = 0.0;
  last_err_ = 0.0;
  first_ = true;
}

int32_t DynamixelPositionLoop::pulses(double angle) const
{
  return static_cast<int32_t>(std::floor(angle / p_.position_unit));
}

double DynamixelPositionLoop::step(double dt, double goal, double present)
{
  // velocity-based trapezoidal profile (Drive Mode bit 2 = 0); none when Profile Velocity = 0
  if (p_.profile_velocity <= 0.0) {
    desired_ = goal;
    desired_vel_ = 0.0;
  } else {
    const double a = p_.profile_acceleration > 0.0 ? p_.profile_acceleration :
      std::numeric_limits<double>::infinity();
    const double dist = goal - desired_;
    double v_target = sgn(dist) *
      std::min(p_.profile_velocity, std::sqrt(2.0 * a * std::abs(dist)));
    const double dv = std::clamp(v_target - desired_vel_, -a * dt, a * dt);
    desired_vel_ += dv;
    const double move = desired_vel_ * dt;
    if (std::abs(move) >= std::abs(dist)) {
      desired_ = goal;
      desired_vel_ = 0.0;
    } else {
      desired_ += move;
    }
  }
  // PID on the error in pulses (both sides quantised as the firmware sees them)
  const double err = static_cast<double>(pulses(desired_) - static_cast<int32_t>(
      std::floor(present / p_.position_unit + 1e-9)));
  const double derr = first_ ? 0.0 : (err - last_err_) / dt;
  first_ = false;
  last_err_ = err;
  const double integ_new = integ_ + err * dt;
  const double unclamped = p_.kpp * err + p_.kpi * integ_new + p_.kpd * derr;
  const double out = std::clamp(unclamped, -p_.pwm_limit, p_.pwm_limit);
  if (unclamped == out) {
    integ_ = integ_new;  // anti-windup (the firmware's Ka gain is not published)
  }
  return out;
}

}  // namespace fleet_gz_plugins
