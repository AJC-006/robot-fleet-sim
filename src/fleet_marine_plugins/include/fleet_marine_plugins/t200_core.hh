// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
// Pure C++ core of the thruster model (no gz dependency, unit tested by
// test/test_thruster_core.cpp).
#ifndef FLEET_MARINE_PLUGINS__T200_CORE_HH_
#define FLEET_MARINE_PLUGINS__T200_CORE_HH_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace fleet_marine
{

/// Vendor bollard table: rows (pwm_us, bus_voltage_v) -> force_n, current_a, rpm.
/// Duplicated rows (the vendor table repeats some PWM values) are averaged.
class ThrusterTable
{
public:
  struct Row { double force, current, rpm; };

  bool Load(const std::string & path)
  {
    std::ifstream f(path);
    if (!f) {return false;}
    std::string line;
    std::map<std::pair<int, int>, std::pair<Row, int>> acc;   // (V*10, pwm) -> (sum, n)
    int ic[5] = {-1, -1, -1, -1, -1};
    bool header = false;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') {continue;}
      std::vector<std::string> c;
      std::stringstream ss(line);
      std::string t;
      while (std::getline(ss, t, ',')) {c.push_back(t);}
      if (!header) {
        for (size_t i = 0; i < c.size(); ++i) {
          if (c[i] == "bus_voltage_v") {ic[0] = i;}
          if (c[i] == "pwm_us") {ic[1] = i;}
          if (c[i] == "rpm") {ic[2] = i;}
          if (c[i] == "current_a") {ic[3] = i;}
          if (c[i] == "force_n") {ic[4] = i;}
        }
        for (int k = 0; k < 5; ++k) {if (ic[k] < 0) {return false;}}
        header = true;
        continue;
      }
      if (c.size() < 6) {continue;}
      int v = static_cast<int>(std::lround(std::stod(c[ic[0]]) * 10));
      int p = static_cast<int>(std::lround(std::stod(c[ic[1]])));
      auto & a = acc[{v, p}];
      a.first.force += std::stod(c[ic[4]]);
      a.first.current += std::stod(c[ic[3]]);
      a.first.rpm += std::stod(c[ic[2]]);
      a.second++;
    }
    for (auto & kv : acc) {
      double v = kv.first.first / 10.0;
      if (volts_.empty() || volts_.back() != v) {volts_.push_back(v); cols_.emplace_back();}
      auto & r = kv.second.first;
      int n = kv.second.second;
      cols_.back().push_back({static_cast<double>(kv.first.second),
                              {r.force / n, r.current / n, r.rpm / n}});
    }
    return !volts_.empty();
  }

  /// Bilinear interpolation in (pwm, voltage); voltage clamped to the table range.
  Row Eval(double pwm_us, double volt) const
  {
    volt = std::min(std::max(volt, volts_.front()), volts_.back());
    size_t hi = std::upper_bound(volts_.begin(), volts_.end(), volt) - volts_.begin();
    if (hi == 0) {hi = 1;}
    if (hi >= volts_.size()) {hi = volts_.size() - 1;}
    size_t lo = hi - 1;
    double w = (volt - volts_[lo]) / (volts_[hi] - volts_[lo]);
    Row a = Col(lo, pwm_us), b = Col(hi, pwm_us);
    return {a.force + w * (b.force - a.force), a.current + w * (b.current - a.current),
      a.rpm + w * (b.rpm - a.rpm)};
  }

  double VMin() const {return volts_.front();}
  double VMax() const {return volts_.back();}

private:
  Row Col(size_t i, double pwm) const
  {
    const auto & c = cols_[i];
    if (pwm <= c.front().first) {return c.front().second;}
    if (pwm >= c.back().first) {return c.back().second;}
    size_t hi = std::lower_bound(c.begin(), c.end(), pwm,
        [](const std::pair<double, Row> & e, double x) {return e.first < x;}) - c.begin();
    const auto & p0 = c[hi - 1];
    const auto & p1 = c[hi];
    double w = (pwm - p0.first) / (p1.first - p0.first);
    return {p0.second.force + w * (p1.second.force - p0.second.force),
      p0.second.current + w * (p1.second.current - p0.second.current),
      p0.second.rpm + w * (p1.second.rpm - p0.second.rpm)};
  }
  std::vector<double> volts_;
  std::vector<std::vector<std::pair<double, Row>>> cols_;
};

/// Third-order transfer function K(s) = (b1 s + b0) / (s^3 + a2 s^2 + a1 s + a0), unity DC gain
/// when
/// b0 == a0. Controllable canonical form, integrated with fixed RK4 sub-steps (max 0.5 ms).
class ThirdOrderLag
{
public:
  ThirdOrderLag() = default;
  ThirdOrderLag(double b1, double b0, double a2, double a1, double a0)
  : b1_(b1), b0_(b0), a2_(a2), a1_(a1), a0_(a0) {}

  /// Start in equilibrium at output value y0 (x2 = x3 = 0, x1 = y0 / DC gain).
  void Reset(double y0) {x_[0] = y0 * a0_ / b0_; x_[1] = 0; x_[2] = 0;}

  double Step(double u, double dt)
  {
    int n = std::max(1, static_cast<int>(std::ceil(dt / 5e-4)));
    double h = dt / n;
    for (int i = 0; i < n; ++i) {
      double k1[3], k2[3], k3[3], k4[3], t[3];
      F(x_, u, k1);
      for (int j = 0; j < 3; ++j) {t[j] = x_[j] + 0.5 * h * k1[j];}
      F(t, u, k2);
      for (int j = 0; j < 3; ++j) {t[j] = x_[j] + 0.5 * h * k2[j];}
      F(t, u, k3);
      for (int j = 0; j < 3; ++j) {t[j] = x_[j] + h * k3[j];}
      F(t, u, k4);
      for (int j = 0; j < 3; ++j) {x_[j] += h / 6.0 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);}
    }
    return b0_ * x_[0] + b1_ * x_[1];
  }

private:
  void F(const double * x, double u, double * dx) const
  {
    dx[0] = x[1];
    dx[1] = x[2];
    dx[2] = -a0_ * x[0] - a1_ * x[1] - a2_ * x[2] + u;
  }
  double b1_{0}, b0_{1}, a2_{0}, a1_{0}, a0_{1};
  double x_[3]{0, 0, 0};
};

/// ESC pulse handling: clamp to the valid range, then apply the dead band around the stop pulse.
inline double EffectivePwm(double pwm, double lo, double stop, double hi, double deadband)
{
  pwm = std::min(std::max(pwm, lo), hi);
  return std::fabs(pwm - stop) <= deadband ? stop : pwm;
}

/// Piecewise-linear interpolation y(x) over sorted knots, clamped at the ends.
inline double Interp1(const std::vector<std::pair<double, double>> & k, double x)
{
  if (x <= k.front().first) {return k.front().second;}
  if (x >= k.back().first) {return k.back().second;}
  size_t hi = std::lower_bound(k.begin(), k.end(), x,
      [](const std::pair<double, double> & e, double v) {return e.first < v;}) - k.begin();
  double w = (x - k[hi - 1].first) / (k[hi].first - k[hi - 1].first);
  return k[hi - 1].second + w * (k[hi].second - k[hi - 1].second);
}

}  // namespace fleet_marine

#endif  // FLEET_MARINE_PLUGINS__T200_CORE_HH_
