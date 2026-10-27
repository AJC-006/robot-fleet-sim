#!/usr/bin/env python3
"""G2 physics bench: small worlds stepped in-process (gz.sim8 TestFixture), one JSON line per run.

cases:
  fall   free-fall sphere, g from v(t) (semi-implicit Euler gives v = -g t exactly)
  slide  box pushed by a horizontal force along heading --theta; F = --ratio * mu m g
  feet   rigid body on 4 sphere feet: settle depth, jitter, slip onset under a force ramp
  drive  robot from xacro, wheels on ideal velocity sources: spin / straight / accel
  stand  legged robot from xacro held at the crouch pose (RTF of a many-joint model)
"""
import argparse
import json
import math
import os
import re
import statistics
import subprocess
import tempfile
import time

from gz.common5 import set_verbosity
import gz.math7 as gm
from gz.sim8 import TestFixture, World, Model, Link, Joint, world_entity

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))  # repo root
A = None


def surf(mu, mu2=None, fdir=None, torsional=None):
    mu2 = mu if mu2 is None else mu2
    fd = f'<fdir1>{fdir}</fdir1>' if fdir else ''
    tor = f'<torsional><coefficient>{torsional}</coefficient></torsional>' if torsional is not None else ''
    return (f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu2}</mu2>{fd}</ode>'
            f'<bullet><friction>{mu}</friction><friction2>{mu2}</friction2></bullet>{tor}</friction></surface>')


def world_sdf(models):
    dart = ''
    if A.engine == 'dartsim' and (A.cd or A.solver):
        dart = '<dart>' + (f'<collision_detector>{A.cd}</collision_detector>' if A.cd else '') + \
               (f'<solver><solver_type>{A.solver}</solver_type></solver>' if A.solver else '') + '</dart>'
    cone = (f'<plugin filename="fleet_coulomb_friction" name="fleet_gz_plugins::CoulombFriction">'
            f'<slip_threshold>{A.slip_thr}</slip_threshold><stick_match_radius>{A.match_r}</stick_match_radius></plugin>') if A.cone else ''
    ground = f'''<model name="ground"><static>true</static><link name="l"><collision name="c">
    <pose>0 0 -0.1 0 0 0</pose><geometry><box><size>60 60 0.2</size></box></geometry>
    {surf(A.ground_mu, torsional=A.torsional)}</collision></link></model>'''
    return f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="t">
  <physics name="p" type="ignored"><max_step_size>{A.step}</max_step_size><real_time_factor>0</real_time_factor>{dart}</physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"><engine><filename>gz-physics-{A.engine}-plugin</filename></engine></plugin>
  {cone}
  <gravity>0 0 -{A.g}</gravity>
  {ground if A.case != 'fall' else ''}
  {models}
</world></sdf>'''


def robot_sdf(robot):
    urdf = subprocess.check_output(['xacro', f'{WS}/src/fleet_description/urdf/robot.urdf.xacro',
                                    f'robot:={robot}'], text=True)
    init = {}
    for m in re.finditer(r'<joint name="([a-z0-9_]+)">\s*<command_interface[^>]*/>\s*<state_interface name="position">'
                         r'\s*<param name="initial_value">([-0-9.e]+)</param>', urdf):
        init[m.group(1)] = float(m.group(2))
    urdf = re.sub(r'<ros2_control.*?</ros2_control>', '', urdf, flags=re.S)
    urdf = re.sub(r'<gazebo>\s*<plugin filename="gz_ros2_control-system".*?</gazebo>', '', urdf, flags=re.S)
    urdf = re.sub(r'<sensor .*?</sensor>', '', urdf, flags=re.S)
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(urdf)
    sdf = subprocess.check_output(['gz', 'sdf', '-p', f.name], text=True)

    def wheel(m):
        blk = m.group(0)
        if A.wheel == 'sphere':
            blk = re.sub(r'<cylinder>\s*<length>[^<]*</length>\s*<radius>([^<]*)</radius>\s*</cylinder>',
                         r'<sphere><radius>\1</radius></sphere>', blk, flags=re.S)
            assert '<sphere>' in blk, blk
        blk = re.sub(r'<fdir1>[^<]*</fdir1>', '', blk)
        fd = {'none': '', 'axle': '<fdir1>0 0 1</fdir1>', 'spin': '<fdir1>1 0 0</fdir1>'}[A.fdir]
        mu = A.wheel_mu if A.wheel_mu is not None else None
        if mu is not None:
            blk = re.sub(r'<mu>[^<]*</mu>', f'<mu>{mu}</mu>', blk)
            blk = re.sub(r'<mu2>[^<]*</mu2>', f'<mu2>{mu}</mu2>', blk)
        blk = blk.replace('</mu2>', '</mu2>' + fd, 1)
        mm = re.search(r'<mu>([^<]*)</mu>\s*<mu2>([^<]*)</mu2>', blk)
        bul = f'<bullet><friction>{mm.group(1)}</friction><friction2>{mm.group(2)}</friction2></bullet>'
        if A.torsional is not None:
            bul += f'<torsional><coefficient>{A.torsional}</coefficient></torsional>'
        blk = blk.replace('</ode>', '</ode>' + bul, 1)
        return blk
    sdf = re.sub(r"<collision name='[a-z_]*wheel_link_collision'>.*?</collision>", wheel, sdf, flags=re.S)
    if A.base_mu is not None:   # surface on the FIRST collision of the root link (bfs reads friction only from it)
        tor = f'<torsional><coefficient>{A.torsional}</coefficient></torsional>' if A.torsional is not None else ''
        sdf = re.sub(r"(<collision name='base_footprint_fixed_joint_lump__base_link_collision'>.*?</geometry>)",
                     r'\1' + f'<surface><friction><ode><mu>{A.base_mu}</mu><mu2>{A.base_mu}</mu2></ode><bullet><friction>{A.base_mu}</friction><friction2>{A.base_mu}</friction2></bullet>{tor}</friction></surface>',
                     sdf, count=1, flags=re.S)
    if A.bullet_all:   # give every non-wheel surface (casters, feet) its <bullet> friction too
        def addb(m):
            blk = m.group(0)
            if '<bullet>' in blk:
                return blk
            mm = re.search(r'<mu>([^<]*)</mu>\s*<mu2>([^<]*)</mu2>', blk)
            tor = f'<torsional><coefficient>{A.torsional}</coefficient></torsional>' if A.torsional is not None else ''
            return blk.replace('</ode>', f'</ode><bullet><friction>{mm.group(1)}</friction><friction2>{mm.group(2)}</friction2></bullet>{tor}', 1)
        sdf = re.sub(r'<friction>\s*<ode>.*?</ode>', addb, sdf, flags=re.S)
    sdf = re.sub(r'<\?xml[^>]*\?>', '', sdf)
    sdf = re.sub(r'</?sdf[^>]*>', '', sdf)
    sdf = re.sub(r'<model name=[^>]*>', '<model name="bot"><pose>0 0 %f 0 0 %f</pose>' % (A.z0, math.radians(A.yaw0)),
                 sdf, count=1)
    return sdf, init


def box_model(name, size, mass, pose, mu, collisions=None):
    sx, sy, sz = size
    ixx = mass * (sy * sy + sz * sz) / 12
    iyy = mass * (sx * sx + sz * sz) / 12
    izz = mass * (sx * sx + sy * sy) / 12
    cols = collisions or [f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>{surf(mu)}</collision>']
    return f'''<model name="{name}"><pose>{pose}</pose><link name="l">
  <inertial><mass>{mass}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz></inertia></inertial>
  {''.join(cols)}</link></model>'''


def run(sdf, pre, post, t_end):
    path = tempfile.mktemp(suffix='.sdf')
    open(path, 'w').write(sdf)
    set_verbosity(1)
    fx = TestFixture(path)
    fx.on_pre_update(pre)
    fx.on_post_update(post)
    fx.finalize()
    n = int(round(t_end / A.step))
    w = min(50, n // 4)              # warm-up steps (plugin load, first contact) excluded from RTF
    fx.server().run(True, w, False)
    t0 = time.time()
    fx.server().run(True, n - w, False)
    return (n - w) * A.step / (time.time() - t0)


def ent(ecm, model, link=None):
    w = World(world_entity(ecm))
    m = Model(w.model_by_name(ecm, model))
    if link is None:
        return m
    return Link(m.link_by_name(ecm, link))


def linfit(ts, ys):
    n = len(ts)
    mt, my = sum(ts) / n, sum(ys) / n
    sxx = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / sxx


def case_fall():
    st = {}

    def pre(info, ecm):
        pass

    def post(info, ecm):
        l = ent(ecm, 'b', 'l')
        l.enable_velocity_checks(ecm, True)
        v = l.world_linear_velocity(ecm)
        if v is not None:
            st['v'] = v.z()
            st['t'] = info.sim_time.total_seconds()
    sdf = world_sdf(box_model('b', (0.1, 0.1, 0.1), 1.0, '0 0 100 0 0 0', 1.0))
    rtf = run(sdf, pre, post, 1.0)
    return dict(g_meas=-st['v'] / st['t'], g_cfg=A.g, err_pct=100 * (-st['v'] / st['t'] - A.g) / A.g, rtf=rtf)


def case_slide():
    m, mu = A.mass, A.body_mu
    th = math.radians(A.theta)
    mu_eff = min(mu, A.ground_mu)
    F = A.ratio * mu_eff * m * A.g
    st = {'log': []}

    def pre(info, ecm):
        t = info.sim_time.total_seconds()
        if t >= 0.5:
            ent(ecm, 'b', 'l').add_world_force(ecm, gm.Vector3d(F * math.cos(th), F * math.sin(th), 0))

    def post(info, ecm):
        l = ent(ecm, 'b', 'l')
        l.enable_velocity_checks(ecm, True)
        v = l.world_linear_velocity(ecm)
        if v is not None:
            st['log'].append((info.sim_time.total_seconds(), v.x(), v.y()))
    sz = 0.1
    sdf = world_sdf(box_model('b', (0.2, 0.2, sz), m, f'0 0 {sz / 2} 0 0 {th}', mu))
    rtf = run(sdf, pre, post, 2.5)
    seg = [r for r in st['log'] if r[0] >= 1.0]
    ts = [r[0] for r in seg]
    ax, ay = linfit(ts, [r[1] for r in seg]), linfit(ts, [r[2] for r in seg])
    a = math.hypot(ax, ay)
    a_exp = max(0.0, (F - mu_eff * m * A.g) / m)
    vend = math.hypot(seg[-1][1], seg[-1][2])
    return dict(theta=A.theta, ratio=A.ratio, mu=mu_eff, a_meas=round(a, 4), a_exp=round(a_exp, 4),
                dir_deg=round(math.degrees(math.atan2(ay, ax)), 1) if a > 1e-3 else None,
                v_end=round(vend, 4), rtf=round(rtf, 2))


def case_feet():
    M, mu, r = A.mass, A.body_mu, A.foot_r
    th = math.radians(A.theta)
    hx, hy, hz = 0.15, 0.10, 0.10
    cols = [f'<collision name="body"><pose>0 0 {hz} 0 0 0</pose><geometry><box><size>0.3 0.2 0.05</size></box></geometry>{surf(mu)}</collision>']
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        cols.append(f'<collision name="f{i}"><pose>{sx * hx} {sy * hy} 0 0 0 0</pose><geometry><sphere><radius>{r}</radius></sphere></geometry>{surf(mu)}</collision>')
    sdf = world_sdf(box_model('b', (0.3, 0.2, 0.2), M, f'0 0 {r} 0 0 0', mu, cols))
    t_settle, k = 2.0, A.ramp * M * A.g   # force ramp rate k [N/s] = ramp * W per second
    st = {'log': []}

    def pre(info, ecm):
        t = info.sim_time.total_seconds()
        if t >= t_settle:
            F = k * (t - t_settle)
            ent(ecm, 'b', 'l').add_world_force(ecm, gm.Vector3d(F * math.cos(th), F * math.sin(th), 0))

    def post(info, ecm):
        l = ent(ecm, 'b', 'l')
        l.enable_velocity_checks(ecm, True)
        p, v = l.world_pose(ecm), l.world_linear_velocity(ecm)
        if p is not None and v is not None:
            st['log'].append((info.sim_time.total_seconds(), p.pos().z(), math.hypot(v.x(), v.y()), p.pos().x(), p.pos().y()))
    rtf = run(sdf, pre, post, t_settle + 1.5 * mu / A.ramp)
    zs = [r[1] for r in st['log'] if 1.0 <= r[0] < t_settle]
    pen = r - statistics.mean(zs)
    slip = next((r for r in st['log'] if r[0] > t_settle and r[2] > 1e-3), None)
    mu_slip = k * (slip[0] - t_settle) / (M * A.g) if slip else None
    pre_drift = math.hypot(*[a - b for a, b in zip(
        [r for r in st['log'] if r[0] >= t_settle][0][3:5], [r for r in st['log'] if r[0] >= 1.0][0][3:5])])
    return dict(theta=A.theta, mass=M, mu=min(mu, A.ground_mu), foot_r=r, pen_mm=round(1000 * pen, 4),
                jitter_um=round(1e6 * statistics.pstdev(zs), 3), drift_settle_mm=round(1000 * pre_drift, 4),
                mu_slip=round(mu_slip, 4) if mu_slip else None, rtf=round(rtf, 2))


def case_drive():
    import yaml
    spec = yaml.safe_load(open(f'{WS}/src/fleet_description/config/robots.yaml'))[A.robot]
    r, B = spec['wheel_radius'], spec['wheel_separation']
    sdf, _ = robot_sdf(A.robot)
    sdf = world_sdf(sdf)
    if A.mode == 'spin':
        vl, vr = -A.wz * B / 2, A.wz * B / 2
    elif A.mode == 'arc':
        vl, vr = A.vx - A.wz * B / 2, A.vx + A.wz * B / 2
    else:
        vl = vr = A.vx
    st = {'log': [], 'mass': None}
    names = spec['left_wheels'] + spec['right_wheels']

    def pre(info, ecm):
        m = ent(ecm, 'bot')
        on = info.sim_time.total_seconds() > A.settle
        for n in names:
            j = Joint(m.joint_by_name(ecm, f'{n}_joint'))
            j.set_velocity(ecm, [((vl if n in spec['left_wheels'] else vr) / r) if on else 0.0])

    def post(info, ecm):
        m = ent(ecm, 'bot')
        if st['mass'] is None:
            tot, cx = 0.0, 0.0
            for le in m.links(ecm):
                li = Link(le)
                wi = li.world_inertial(ecm)
                if wi is not None:
                    tot += wi.mass_matrix().mass()
                    cx += wi.mass_matrix().mass() * wi.pose().pos().x()
            st['mass'], st['com_x'] = tot, cx / tot if tot else None
        wv = []
        for n in names:
            j = Joint(m.joint_by_name(ecm, f'{n}_joint'))
            j.enable_velocity_check(ecm, True)
            v = j.velocity(ecm)
            if v:
                wv.append(v[0] * r)
        if wv and info.sim_time.total_seconds() > A.settle + 0.3 * A.time:
            st.setdefault('wv', []).append(sum(abs(x) for x in wv) / len(wv))
        b = Link(m.link_by_name(ecm, 'base_footprint'))
        b.enable_velocity_checks(ecm, True)
        p, v = b.world_pose(ecm), b.world_linear_velocity(ecm)
        if p is None or v is None:
            return
        st['log'].append((info.sim_time.total_seconds(), p.pos().x(), p.pos().y(), p.pos().z(), p.rot().yaw(),
                          p.rot().roll(), p.rot().pitch(), v.x(), v.y()))
    rtf = run(sdf, pre, post, A.settle + A.time)
    log = st['log']
    if st.get('wv'):
        A.wheel_surface_speed = round(statistics.mean(st['wv']), 4)
    res = dict(robot=A.robot, mode=A.mode, wheel=A.wheel, fdir=A.fdir, yaw0=A.yaw0, mu=A.wheel_mu,
               mass=round(st['mass'], 3), rtf=round(rtf, 2))
    zs = [l[3] for l in log if l[0] > 0.5]
    res.update(z_mm=round(1000 * statistics.mean(zs), 3), z_jit_um=round(1e6 * statistics.pstdev(zs), 2),
               tilt_max_deg=round(math.degrees(max(max(abs(l[5]), abs(l[6])) for l in log if l[0] > 0.5)), 3))
    if A.mode == 'accel':
        seg = [l for l in log if A.settle + 0.02 <= l[0] <= A.settle + 0.15]
        ts = [l[0] for l in seg]
        c, s = math.cos(math.radians(A.yaw0)), math.sin(math.radians(A.yaw0))
        res['a_meas'] = round(linfit(ts, [l[7] * c + l[8] * s for l in seg]), 4)
        res['mu_g'] = round(A.wheel_mu * A.g, 4) if A.wheel_mu else None
        res['com_x'] = round(st['com_x'], 4)
        return res
    t0 = A.settle + 0.3 * A.time
    seg = [l for l in log if l[0] >= t0]
    yaw = [seg[0][4]]
    for l in seg[1:]:
        yaw.append(yaw[-1] + (l[4] - yaw[-1] + math.pi) % (2 * math.pi) - math.pi)
    dt = seg[-1][0] - seg[0][0]
    wz = (yaw[-1] - yaw[0]) / dt
    dx, dy = seg[-1][1] - seg[0][1], seg[-1][2] - seg[0][2]
    if A.mode in ('spin', 'arc'):
        res.update(wz_cmd=A.wz, wz_meas=round(wz, 4), yaw_ratio=round(wz / A.wz, 4),
                   B_eff=round(B * A.wz / wz, 4) if abs(wz) > 1e-6 else None, drift_m=round(math.hypot(dx, dy), 4))
        # half-window ratios (heading dependence)
        h = len(yaw) // 2
        res['yaw_ratio_halves'] = [round((yaw[h] - yaw[0]) / (seg[h][0] - seg[0][0]) / A.wz, 4),
                                   round((yaw[-1] - yaw[h]) / (seg[-1][0] - seg[h][0]) / A.wz, 4)]
    if A.mode in ('straight', 'arc'):
        res['wheel_rim_speed'] = getattr(A, 'wheel_surface_speed', None)
        res.update(v_cmd=A.vx, v_meas=round(math.hypot(dx, dy) / dt, 4), heading_drift_deg=round(math.degrees(yaw[-1] - yaw[0]), 3))
    return res


def case_stand():
    sdf, init = robot_sdf(A.robot)
    sdf = world_sdf(sdf)
    st = {'log': [], 'init': False}

    def pre(info, ecm):
        m = ent(ecm, 'bot')
        for n, q in init.items():
            j = Joint(m.joint_by_name(ecm, n))
            if not st['init']:
                j.reset_position(ecm, [q])
            j.set_velocity(ecm, [0.0])
        st['init'] = True

    def post(info, ecm):
        b = ent(ecm, 'bot', 'base_footprint')
        p = b.world_pose(ecm)
        if p is not None:
            st['log'].append((info.sim_time.total_seconds(), p.pos().z()))
    rtf = run(sdf, pre, post, A.time)
    zs = [l[1] for l in st['log'] if l[0] > A.time / 2]
    return dict(robot=A.robot, joints=len(init), rtf=round(rtf, 2), z_mm=round(1000 * statistics.mean(zs), 2),
                z_jit_um=round(1e6 * statistics.pstdev(zs), 2))


def main():
    global A
    ap = argparse.ArgumentParser()
    ap.add_argument('case', choices=['fall', 'slide', 'feet', 'drive', 'stand'])
    ap.add_argument('--engine', default='dartsim')
    ap.add_argument('--step', type=float, default=0.002)
    ap.add_argument('--solver', default=None)
    ap.add_argument('--cd', default=None)
    ap.add_argument('--g', type=float, default=9.80665)
    ap.add_argument('--ground-mu', type=float, default=1.0)
    ap.add_argument('--torsional', type=float, default=None)
    ap.add_argument('--mass', type=float, default=2.0)
    ap.add_argument('--body-mu', type=float, default=0.8)
    ap.add_argument('--theta', type=float, default=0.0)
    ap.add_argument('--ratio', type=float, default=1.5)
    ap.add_argument('--foot-r', type=float, default=0.022)
    ap.add_argument('--ramp', type=float, default=0.5, help='feet: force ramp in body weights per second')
    ap.add_argument('--robot', default='round')
    ap.add_argument('--wheel', default='sphere', choices=['cylinder', 'sphere'])
    ap.add_argument('--fdir', default='none', choices=['none', 'axle', 'spin'])
    ap.add_argument('--wheel-mu', type=float, default=None)
    ap.add_argument('--mode', default='spin', choices=['spin', 'straight', 'arc', 'accel'])
    ap.add_argument('--wz', type=float, default=1.0)
    ap.add_argument('--vx', type=float, default=0.3)
    ap.add_argument('--yaw0', type=float, default=0.0)
    ap.add_argument('--z0', type=float, default=0.0)
    ap.add_argument('--settle', type=float, default=1.0)
    ap.add_argument('--time', type=float, default=6.0)
    ap.add_argument('--tag', default='')
    ap.add_argument('--bullet-all', action='store_true')
    ap.add_argument('--cone', action='store_true')
    ap.add_argument('--slip-thr', type=float, default=1e-4)
    ap.add_argument('--match-r', type=float, default=0.02)
    ap.add_argument('--base-mu', type=float, default=None)
    A = ap.parse_args()
    res = globals()['case_' + A.case]()
    out = dict(case=A.case, engine=A.engine.replace('bullet-featherstone', 'bfs') + ('+cone' if A.cone else ''), step=A.step)
    if A.solver:
        out['solver'] = A.solver
    if A.cd:
        out['cd'] = A.cd
    if A.tag:
        out['tag'] = A.tag
    out.update({k: (round(v, 4) if isinstance(v, float) else v) for k, v in res.items()})
    print('RESULT ' + json.dumps(out))


if __name__ == '__main__':
    main()
