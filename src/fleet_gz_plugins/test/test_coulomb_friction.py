# Copyright 2026 Alan
# SPDX-License-Identifier: Apache-2.0
"""
Pass/fail checks of the CoulombFriction system against closed-form Coulomb friction.

Each case builds a small world, steps gz-sim 8 in-process (TestFixture, no rendering) with DART and
the plugin, and compares the motion with the analytic answer:

- a block pushed at 1.5x its friction limit at 45 deg to the world axes slides with
  a = (F - mu m g) / m along the push (DART's own box friction gives the wrong size and direction);
- the same block pushed at 0.9x the limit stays at rest;
- a four-footed rigid body under a slowly ramped force at 45 deg breaks away at F = mu W
  (DART's box gives sqrt(2) mu W on the diagonal).

GZ_SIM_SYSTEM_PLUGIN_PATH must contain the built plugin (CMakeLists.txt sets it for colcon test).
"""

import math
import os
import tempfile

from gz.common5 import set_verbosity
import gz.math7 as gm
from gz.sim8 import Link, Model, TestFixture, World, world_entity
import pytest

STEP = 0.002      # s, the worlds' step (docs/notes/ground.md G2)
G = 9.80665       # m/s^2, standard gravity
MU = 0.8          # body friction; the ground carries 1.0, DART combines with min()


def _surf(mu):
    return f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>'


def _world(models):
    return f"""<?xml version="1.0"?>
<sdf version="1.9"><world name="t">
  <physics name="p" type="dart"><max_step_size>{STEP}</max_step_size>
    <real_time_factor>0</real_time_factor>
    <dart><collision_detector>ode</collision_detector>
      <solver><solver_type>dantzig</solver_type></solver></dart></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
    <engine><filename>gz-physics-dartsim-plugin</filename></engine></plugin>
  <plugin filename="fleet_coulomb_friction" name="fleet_gz_plugins::CoulombFriction"/>
  <gravity>0 0 -{G}</gravity>
  <model name="ground"><static>true</static><link name="l"><collision name="c">
    <pose>0 0 -0.1 0 0 0</pose><geometry><box><size>20 20 0.2</size></box></geometry>
    {_surf(1.0)}</collision></link></model>
  {models}
</world></sdf>"""


def _body(mass, size, z, collisions):
    sx, sy, sz = size
    return f"""<model name="b"><pose>0 0 {z} 0 0 0</pose><link name="l">
  <inertial><mass>{mass}</mass><inertia><ixx>{mass * (sy * sy + sz * sz) / 12}</ixx>
  <iyy>{mass * (sx * sx + sz * sz) / 12}</iyy><izz>{mass * (sx * sx + sy * sy) / 12}</izz>
  </inertia></inertial>{collisions}</link></model>"""


def _run(sdf, force_of_t, t_end):
    """Step the world with world-frame force (fx, fy) = force_of_t(t); log (t, x, y, vx, vy)."""
    fd, path = tempfile.mkstemp(suffix='.sdf')
    with os.fdopen(fd, 'w') as f:
        f.write(sdf)
    log = []

    def link(ecm):
        return Link(Model(World(world_entity(ecm)).model_by_name(ecm, 'b')).link_by_name(ecm, 'l'))

    def pre(info, ecm):
        fx, fy = force_of_t(info.sim_time.total_seconds())
        if fx or fy:
            link(ecm).add_world_force(ecm, gm.Vector3d(fx, fy, 0.0))

    def post(info, ecm):
        lk = link(ecm)
        lk.enable_velocity_checks(ecm, True)
        p, v = lk.world_pose(ecm), lk.world_linear_velocity(ecm)
        if p is not None and v is not None:
            log.append((info.sim_time.total_seconds(), p.pos().x(), p.pos().y(), v.x(), v.y()))

    set_verbosity(1)
    fx = TestFixture(path)
    fx.on_pre_update(pre)
    fx.on_post_update(post)
    fx.finalize()
    fx.server().run(True, int(round(t_end / STEP)), False)
    os.remove(path)
    return log


def _slope(ts, ys):
    mt, my = sum(ts) / len(ts), sum(ys) / len(ys)
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / sum((t - mt) ** 2 for t in ts)


def _slide(theta_deg, ratio):
    m, th = 10.0, math.radians(theta_deg)
    f = ratio * MU * m * G
    col = (f'<collision name="c"><geometry><box><size>0.2 0.2 0.1</size></box></geometry>'
           f'{_surf(MU)}</collision>')
    log = _run(_world(_body(m, (0.2, 0.2, 0.1), 0.05, col)),
               lambda t: (f * math.cos(th), f * math.sin(th)) if t >= 0.5 else (0.0, 0.0), 2.5)
    seg = [r for r in log if r[0] >= 1.0]
    ts = [r[0] for r in seg]
    return _slope(ts, [r[3] for r in seg]), _slope(ts, [r[4] for r in seg]), seg[-1]


def test_slide_45deg_matches_coulomb():
    ax, ay, _ = _slide(45.0, 1.5)
    a_exp = 0.5 * MU * G
    assert math.hypot(ax, ay) == pytest.approx(a_exp, rel=0.01)
    assert abs(math.degrees(math.atan2(ay, ax)) - 45.0) < 1.0


def test_stick_below_limit():
    _, _, last = _slide(45.0, 0.9)
    assert math.hypot(last[3], last[4]) < 1e-3


def test_feet_breakaway_45deg():
    m, r, hx, hy = 8.0, 0.02, 0.15, 0.10
    th, ramp, t0 = math.radians(45.0), 0.2, 2.0   # force rises at 0.2 W per second after t0
    cols = (f'<collision name="body"><pose>0 0 0.1 0 0 0</pose><geometry><box><size>0.3 0.2 0.05'
            f'</size></box></geometry>{_surf(MU)}</collision>')
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        cols += (f'<collision name="f{i}"><pose>{sx * hx} {sy * hy} 0 0 0 0</pose>'
                 f'<geometry><sphere>'
                 f'<radius>{r}</radius></sphere></geometry>{_surf(MU)}</collision>')
    k = ramp * m * G

    def force(t):
        f = k * (t - t0) if t >= t0 else 0.0
        return f * math.cos(th), f * math.sin(th)
    log = _run(_world(_body(m, (0.3, 0.2, 0.2), r, cols)), force, t0 + 1.5 * MU / ramp)
    slip = next(row for row in log if row[0] > t0 and math.hypot(row[3], row[4]) > 1e-3)
    mu_slip = k * (slip[0] - t0) / (m * G)
    # breakaway resolution: one step of the ramp plus the plugin's one-step direction lag
    assert mu_slip == pytest.approx(MU, rel=0.02)
