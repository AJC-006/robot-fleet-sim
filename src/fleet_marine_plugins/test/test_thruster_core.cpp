// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
// Pass/fail check of the thruster core against the vendor table and the identified transfer
// function.
// usage: test_thruster_core [table.csv]   (exit code != 0 on failure)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "fleet_marine_plugins/t200_core.hh"

static int fails = 0;
#define CHECK(c, ...) \
  do { \
    if (!(c)) {++fails; std::printf("FAIL: " __VA_ARGS__); std::printf("\n");} \
  } while (0)

int main(int argc, char ** argv)
{
  using fleet_marine::ThirdOrderLag;
  using fleet_marine::ThrusterTable;
  using fleet_marine::EffectivePwm;
  // step response of K(s): t63 = 20.0 ms, t10 = 6.3 ms, t90 = 29.3 ms, no overshoot, settled by
  // 0.2 s (notes/sea.md)
  ThirdOrderLag k(6136, 108700, 89, 9258, 108700);
  double t = 0, y = 0, t10 = -1, t63 = -1, t90 = -1, ymax = 0;
  const double dt = 1e-3;
  while (t < 0.5) {
    y = k.Step(1.0, dt);
    t += dt;
    ymax = std::fmax(ymax, y);
    if (t10 < 0 && y >= 0.1) {t10 = t;}
    if (t63 < 0 && y >= 0.632) {t63 = t;}
    if (t90 < 0 && y >= 0.9) {t90 = t;}
  }
  CHECK(std::fabs(t63 - 0.020) < 0.002, "t63 %.4f s (want 0.020)", t63);
  CHECK(std::fabs(t10 - 0.0063) < 0.0015, "t10 %.4f s (want 0.0063)", t10);
  CHECK(std::fabs(t90 - 0.0293) < 0.003, "t90 %.4f s (want 0.0293)", t90);
  CHECK(ymax < 1.02, "overshoot %.4f", ymax);
  CHECK(std::fabs(y - 1.0) < 1e-3, "DC gain %.5f", y);
  CHECK(EffectivePwm(1520, 1100, 1500, 1900, 25) == 1500, "deadband");
  CHECK(EffectivePwm(2500, 1100, 1500, 1900, 25) == 1900, "clamp");
  if (argc > 1) {
    ThrusterTable tb;
    CHECK(tb.Load(argv[1]), "load %s", argv[1]);
    auto a = tb.Eval(1900, 16.0);
    CHECK(std::fabs(a.force - 51.44) < 0.3, "F(1900,16V) %.2f", a.force);
    CHECK(std::fabs(a.current - 23.83) < 0.1, "I(1900,16V) %.2f", a.current);
    auto b = tb.Eval(1100, 16.0);
    CHECK(std::fabs(b.force + 39.9) < 0.3, "F(1100,16V) %.2f", b.force);
    CHECK(std::fabs(b.current - 24.3) < 0.1, "I(1100,16V) %.2f", b.current);
    auto c = tb.Eval(1900, 15.0);
    auto d = tb.Eval(1900, 14.0);
    auto e = tb.Eval(1900, 16.0);
    CHECK(c.force > d.force && c.force < e.force, "voltage interpolation monotone");
    CHECK(std::fabs(tb.Eval(1500, 16.0).force) < 1e-6, "zero at 1500");
  }
  std::printf(fails ? "thruster core: %d failures\n" : "thruster core: ok\n", fails);
  return fails ? 1 : 0;
}
