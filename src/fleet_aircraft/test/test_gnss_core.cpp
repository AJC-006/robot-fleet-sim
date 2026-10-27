// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
// Pass/fail check of the GNSS error model: stationary stddev, lag-1 autocorrelation, WGS84 radii.
#include <cmath>
#include <cstdio>

#include "fleet_aircraft/gnss_core.hh"

using fleet_aircraft::GaussMarkov;
using fleet_aircraft::MetresPerRadian;

static int fails = 0;
#define CHECK(c, ...) \
  do { \
    if (!(c)) {++fails; std::printf("FAIL: " __VA_ARGS__); std::printf("\n");} \
  } while (0)

int main()
{
  const double sigma = 1.27, tau = 60.0, dt = 0.1;
  GaussMarkov g(sigma, tau, 7);
  const int n = 2000000;
  double s1 = 0, s2 = 0, sxy = 0, prev = 0;
  for (int i = 0; i < n; ++i) {
    const double x = g.Step(dt);
    s1 += x;
    s2 += x * x;
    sxy += x * prev;
    prev = x;
  }
  const double mean = s1 / n, var = s2 / n - mean * mean;
  CHECK(std::fabs(std::sqrt(var) - sigma) < 0.08 * sigma,
    "stationary stddev %.3f (want %.3f)", std::sqrt(var), sigma);
  const double rho = (sxy / n - mean * mean) / var, want = std::exp(-dt / tau);
  CHECK(std::fabs(rho - want) < 2e-3, "lag-1 autocorrelation %.5f (want %.5f)", rho, want);
  double mn, me;
  MetresPerRadian(47.397742 * M_PI / 180.0, mn, me);
  CHECK(std::fabs(mn - 6368.4e3) < 3e3, "meridian %.0f m/rad", mn);
  CHECK(std::fabs(me / mn - 0.6757 / 0.99) < 0.05, "east/north ratio %.4f", me / mn);
  std::printf("%s\n", fails ? "FAILED" : "OK");
  return fails ? 1 : 0;
}
