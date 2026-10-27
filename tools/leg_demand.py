#!/usr/bin/env python3
"""Offline leg actuator demand: joint speed from the existing gait at max command, stance torque
J^T F with a half-sine vertical GRF of peak pi*m*g/(2*duty*n_legs) (Alexander 2003, Principles of
Animal Locomotion ch. 5). Usage: leg_demand.py <robot> [mass_kg] [speed_scale] [period_s]

mass_kg defaults to the total_mass of config/mass_properties.yaml (tools/mass_properties.py)."""
import math
import sys
from pathlib import Path
import yaml
WS = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, WS + '/src/fleet_legged')
from fleet_legged import kinematics as K
from fleet_legged.gait import Gait

def jac(leg, q, h=1e-6):
    J = [[0.0] * 3 for _ in range(3)]
    p0 = leg.fk(q)
    for j in range(3):
        qq = list(q); qq[j] += h
        p1 = leg.fk(qq)
        for i in range(3):
            J[i][j] = (p1[i] - p0[i]) / h
    return J

def main():
    name = sys.argv[1]
    r = K.load(name, WS + '/src/fleet_description/config/robots.yaml')
    spec = r.spec
    m = float(sys.argv[2]) if len(sys.argv) > 2 else float(yaml.safe_load(open(WS + '/src/fleet_description/config/mass_properties.yaml'))[name]['total_mass'])
    g = spec['gait']
    if len(sys.argv) > 4: g['period'] = r.gait['period'] = float(sys.argv[4])  # gait period override (s)
    duty = float(g['duty']); n = len(r.legs); T = float(g['period'])
    fpk = math.pi * m * 9.80665 / (2 * duty * n)
    print(f'{name}: m={m} kg legs={n} duty={duty} period={T} peak GRF/leg={fpk:.2f} N ({fpk/(m*9.80665):.3f} mg)')
    # static, all feet down, nominal stance
    q0 = r.stance_q()
    for leg, q in zip(r.legs[:1], q0):
        J = jac(leg, q)
        tau = [-(J[2][j]) * (m * 9.80665 / n) for j in range(3)]
        print('  4/all-feet static tau (coxa femur tibia) N m:', ' '.join(f'{t:+.2f}' for t in tau), ' q=', ' '.join(f'{v:+.2f}' for v in q))
    ks = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0  # command speed scale
    vx, vy, wz = (ks * float(spec['max_vel_x']), ks * float(spec.get('max_vel_y', 0.0)), ks * float(spec['max_vel_theta']))
    for cmd in [(vx, 0, 0), (0, vy, 0), (0, 0, wz), (vx, 0, wz * 0.5)]:
        G = Gait(r); dt = 0.002
        qprev = None; wmax = [0.0] * 3; tmax = [0.0] * 3; pts = []
        steps = int(6 * T / dt)
        for k in range(steps):
            feet = G.update(dt, cmd)
            qs = [leg.ik(f)[0] for leg, f in zip(r.legs, feet)]
            if qprev is not None and k > steps // 3:
                for li, (leg, q, qp) in enumerate(zip(r.legs, qs, qprev)):
                    w = [(a - b) / dt for a, b in zip(q, qp)]
                    psi = (G.phi + G.offsets[li]) % 1.0
                    if psi < duty:
                        F = fpk * math.sin(math.pi * psi / duty)
                        J = jac(leg, q)
                        tau = [-(J[2][j]) * F for j in range(3)]
                    else:
                        tau = [0.0, 0.0, 0.0]
                    for j in range(3):
                        wmax[j] = max(wmax[j], abs(w[j])); tmax[j] = max(tmax[j], abs(tau[j]))
                        pts.append((j, abs(w[j]), abs(tau[j])))
            qprev = qs
        print(f'  cmd {cmd}: |w|max rad/s', ' '.join(f'{v:.2f}' for v in wmax), '| |tau|max N m', ' '.join(f'{v:.2f}' for v in tmax))
        # worst-case operating point per joint against a linear T-N line (needs w/w0 + tau/ts <= 1)
        globals().setdefault('PTS', []).extend(pts)
    for j, jn in enumerate(('coxa', 'femur', 'tibia')):
        P = [(w, t) for jj, w, t in PTS if jj == j]
        print(f'  {jn}: points={len(P)} max w={max(p[0] for p in P):.2f} max tau={max(p[1] for p in P):.2f} '
              f'tau at max w={max(P)[1]:.2f}')
    # linear T-N check at 11.1 V datasheet points (ROBOTIS e-manual); <=1 at 10 V pack cut-off
    # (3.33 V/cell) <=> <=0.90 at 11.1 V since stall torque and no-load speed both scale with V.
    ACT = {'XL430-W250': (1.4, 57), 'XM430-W210': (2.7, 70), 'XM430-W350': (3.8, 43), 'XH540-W150': (6.6, 66), 'XH540-W270': (9.2, 36)}
    for j, jn in enumerate(('coxa', 'femur', 'tibia')):
        P = [(w, t) for jj, w, t in PTS if jj == j]
        rms = math.sqrt(sum(t * t for _, t in P) / len(P))
        out = []
        for a, (ts, rpm) in ACT.items():
            w0 = rpm * 2 * math.pi / 60
            u = max(w / w0 + t / ts for w, t in P)
            out.append(f'{a}={u:.2f}')
        print(f'  TN {jn}: rms tau={rms:.2f} ' + ' '.join(out))
main()
