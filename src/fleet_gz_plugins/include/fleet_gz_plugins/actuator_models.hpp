// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
//
// Actuator models used by the gz_ros2_control hardware plugins of this package. No Gazebo or ROS
// types: the models are unit-tested on their own (test/test_actuator_models.cpp).
//
// DcGearmotor: brushed DC motor + gearbox, quasi-static electrical model (the inductance is not
// published for the motors in the catalog, so the electrical pole L/R is dropped), all constants
// referred to the gearbox output shaft:
//   i      = clamp((V - ke w) / R, -i_lim, i_lim)            armature current
//   tau    = kt_eta (i - i_c sgn(w) - i_v w)                  output torque
// kt_eta is the output torque per amp including the gearbox efficiency, i_c / i_v are the
// Coulomb / viscous parts of the no-load current (motor + gearbox friction).
// Integration split (why: the back-EMF damping kt_eta ke / R of a geared motor is far stiffer
// than a light link can take with an explicit force, b dt / J > 2): the hardware plugin commands
// only the voltage part
//   tau_cmd = kt_eta / R (ke w + R i)        (= kt_eta V / R while the current limit is inactive)
// and the joint carries the linear terms as DART joint dynamics, which DART integrates
// implicitly:
//   damping  b   = kt_eta ke / R + kt_eta i_v   [N m s/rad]
//   friction tau_f = kt_eta i_c                 [N m]       (DART Coulomb joint friction)
// so the net torque on the joint is exactly the model torque above. While the current limit is
// active tau_cmd - b_emf w = kt_eta i_lim.
//
// QuadratureEncoder: incremental encoder, counts = floor(angle * counts_per_rev / 2 pi), zero at
// power-up (an incremental encoder has no absolute reference).
//
// RoboClawVelocityLoop: velocity PID of a Basicmicro RoboClaw (user manual rev 5.7, "Velocity
// PID": QPPS = encoder speed at 100 % duty, gains P, I, D, QPPS). The firmware equation and loop
// rate are not published; implemented form, in duty units with the speed error normalised by
// QPPS:  duty = cmd/QPPS + P e + I integral(e dt) - D d(meas/QPPS)/dt,  e = (cmd - meas)/QPPS,
// with meas = encoder count difference over one loop period. The integrator stops while the duty
// is saturated (anti-windup). Gains and rate are configuration, as on the real controller (tuned
// with Basicmicro Motion Studio).
//
// DynamixelPositionLoop: ROBOTIS X-series Position Control Mode (e-manual, "Position Controller
// (Position Control Mode)" block diagram): Goal Position -> Profile -> PID on the position error
// in encoder pulses -> limiter (Goal PWM / PWM Limit, 885 = 100 % duty) -> inverter.
// KPP = P/128, KPI = I/65536, KPD = D/16 (e-manual). Interpretation (the e-manual gives no unit
// for the PID output): the PID output is in PWM units. Feedforward gains are not implemented and
// must be 0 (their default). Current Limit is not used in this mode (e-manual).

#ifndef FLEET_GZ_PLUGINS__ACTUATOR_MODELS_HPP_
#define FLEET_GZ_PLUGINS__ACTUATOR_MODELS_HPP_

#include <cstdint>

namespace fleet_gz_plugins
{

struct DcMotorParams
{
  double resistance{0.0};         // ohm, winding plus the driver's switch resistance in the path
  double back_emf_constant{0.0};  // V s/rad at the output shaft
  double torque_constant{0.0};    // N m/A at the output shaft, gearbox efficiency included
  double coulomb_current{0.0};    // A, Coulomb part of the no-load current
  double viscous_current{0.0};    // A s/rad, viscous part of the no-load current
  double current_limit{0.0};      // A, driver current limit; <= 0: none
};

class DcGearmotor
{
public:
  DcGearmotor() = default;
  explicit DcGearmotor(const DcMotorParams & p);

  const DcMotorParams & params() const {return p_;}
  // Joint dynamics the URDF must carry (see the integration split above).
  double joint_damping() const;
  double joint_friction() const;
  // Armature current [A] at terminal voltage v [V] and output speed w [rad/s].
  double current(double v, double w) const;
  // Torque the plugin applies through the joint force command [N m].
  double commanded_torque(double v, double w) const;
  // Full model output torque [N m] (what commanded_torque + joint dynamics add up to).
  double output_torque(double v, double w) const;
  // Steady no-load speed [rad/s] at voltage v (> 0), friction included.
  double no_load_speed(double v) const;
  // Stall torque [N m] at voltage v: output torque as w -> 0+ (Coulomb friction subtracted).
  double stall_torque(double v) const;

private:
  DcMotorParams p_;
};

class QuadratureEncoder
{
public:
  QuadratureEncoder() = default;
  explicit QuadratureEncoder(double counts_per_rev);
  void reset(double angle);              // power-up: count 0 at this shaft angle
  int64_t count(double angle) const;     // counts since power-up
  double counts_per_rev() const {return cpr_;}

private:
  double cpr_{1.0};
  double zero_{0.0};
};

struct RoboClawParams
{
  double qpps{1.0};        // counts/s at 100 % duty (RoboClaw setting)
  double p{0.0};           // duty per normalised speed error
  double i{0.0};           // duty per (normalised speed error * s)
  double d{0.0};           // duty * s per normalised speed change
  double loop_rate{1.0};   // Hz
};

class RoboClawVelocityLoop
{
public:
  RoboClawVelocityLoop() = default;
  explicit RoboClawVelocityLoop(const RoboClawParams & p);
  // Advance by dt seconds; cmd in counts/s (as sent by the driver), count = encoder reading.
  // Returns the duty in [-1, 1]. The loop recomputes the duty once per loop period.
  double step(double dt, int32_t cmd, int64_t count);
  double measured_speed() const {return meas_;}  // counts/s, last loop sample
  void reset(int64_t count);

private:
  RoboClawParams p_;
  double t_since_{0.0};
  int64_t last_count_{0};
  double meas_{0.0};
  double integ_{0.0};
  double duty_{0.0};
};

struct DynamixelParams
{
  double position_unit{0.0015339807878856412};  // rad per pulse (2 pi / 4096)
  double kpp{800.0 / 128.0};                    // Position P Gain / 128
  double kpi{0.0};                              // Position I Gain / 65536
  double kpd{0.0};                              // Position D Gain / 16
  double pwm_limit{885.0};                      // PWM Limit (and Goal PWM), units of 0.113 %
  double pwm_full_scale{885.0};                 // 885 units = 100 % duty
  double profile_velocity{0.0};                 // rad/s, 0 = no profile (e-manual default)
  double profile_acceleration{0.0};             // rad/s^2, 0 = infinite
};

class DynamixelPositionLoop
{
public:
  DynamixelPositionLoop() = default;
  explicit DynamixelPositionLoop(const DynamixelParams & p);
  void reset(double present_position);
  // Advance by dt; goal and present position in rad (present = quantised encoder reading).
  // Returns the PWM output in units (-pwm_limit..pwm_limit).
  double step(double dt, double goal, double present);
  double duty(double pwm) const {return pwm / p_.pwm_full_scale;}
  int32_t pulses(double angle) const;  // encoder reading of an angle, floor quantisation

private:
  DynamixelParams p_;
  double desired_{0.0};      // profile output [rad]
  double desired_vel_{0.0};  // profile velocity [rad/s]
  double integ_{0.0};        // pulse * s
  double last_err_{0.0};
  bool first_{true};
};

}  // namespace fleet_gz_plugins

#endif  // FLEET_GZ_PLUGINS__ACTUATOR_MODELS_HPP_
