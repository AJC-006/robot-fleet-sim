// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
// GNSS receiver error model (no gz dependencies, unit tested by test_gnss_core).
//   position error per axis (north, east, up): first-order Gauss-Markov process,
//     e[k+1] = exp(-dt/tau) e[k] + N(0, sigma sqrt(1 - exp(-2 dt/tau)))   (stationary stddev sigma)
//   velocity error per axis: white noise, stddev sigma_v (receiver velocity is Doppler based).
// sigma comes from the receiver datasheet (CEP -> per-axis stddev
// sigma = CEP / 1.1774 for a circular normal error), tau is not published by receiver vendors
// (see the component catalog entry).
#ifndef FLEET_AIRCRAFT__GNSS_CORE_HH_
#define FLEET_AIRCRAFT__GNSS_CORE_HH_

#include <cmath>
#include <random>

namespace fleet_aircraft
{

class GaussMarkov
{
public:
  GaussMarkov(double sigma, double tau, unsigned seed)
  : sigma_(sigma), tau_(tau), rng_(seed) {}
  // Start from a draw of the stationary distribution (a receiver is never switched on at e = 0).
  void Reset() {x_ = sigma_ > 0 ? std::normal_distribution<double>(0, sigma_)(rng_) : 0.0;}
  double Step(double dt)
  {
    if (sigma_ <= 0) {return 0.0;}
    const double phi = tau_ > 0 ? std::exp(-dt / tau_) : 0.0;
    const double s = sigma_ * std::sqrt(1.0 - phi * phi);
    x_ = phi * x_ + std::normal_distribution<double>(0, s)(rng_);
    return x_;
  }
  double Value() const {return x_;}

private:
  double sigma_, tau_, x_{0};
  std::mt19937 rng_;
};

// Metres per radian of latitude / longitude on the WGS84 ellipsoid at latitude lat [rad].
inline void MetresPerRadian(double lat, double & north, double & east)
{
  const double a = 6378137.0, e2 = 6.69437999014e-3;
  const double s = std::sin(lat), w = 1.0 - e2 * s * s;
  north = a * (1.0 - e2) / std::pow(w, 1.5);          // meridian radius M
  east = a / std::sqrt(w) * std::cos(lat);            // N cos(lat)
}

}  // namespace fleet_aircraft

#endif  // FLEET_AIRCRAFT__GNSS_CORE_HH_
