#!/usr/bin/env python3
"""Check that the lidar plane is free of the robot's own geometry beyond the lidar's min range.

  ros2 run fleet_legged check_lidar_plane.py wyrm hound spider [--min-range 0.12] [--tol 0.005]

The gpu_lidar sees visual meshes, the robot's own included. Every link mesh
(meshes/<robot>/{base,coxa,femur,tibia}[_l|_r].obj) is posed by the forward kinematics through the
stand-up and the gait at the robot's top speeds (the same sweep as the footprint, see envelope.py),
and every triangle that reaches within `tol` of the lidar plane is reported when any point of it is
further than `min_range` from the lidar axis. The legs are checked in base_link, which is where the
lidar is fixed, so body roll and pitch while walking do not change the result.
"""
import argparse
import math
import os
import sys

import numpy as np

from fleet_legged.gait import Gait
from fleet_legged.kinematics import load


def read_obj(path):
    v, f = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith('v '):
                v.append([float(x) for x in line.split()[1:4]])
            elif line.startswith('f '):
                idx = [int(t.split('/')[0]) for t in line.split()[1:]]
                idx = [i - 1 if i > 0 else len(v) + i for i in idx]
                for k in range(1, len(idx) - 1):
                    f.append((idx[0], idx[k], idx[k + 1]))
    return np.array(v, dtype=float), np.array(f, dtype=int)


def poses(robot, dt=0.02):
    """Joint configurations to test: crouch, stand, and the gait sweep at top speeds."""
    spec = robot.spec
    vx, vy, wz = float(spec['max_vel_x']), float(spec.get('max_vel_y', 0.0)), float(spec['max_vel_theta'])
    out = [robot.stance_q(robot.crouch), robot.stance_q(robot.height)]
    for c in [(vx, 0, 0), (-vx, 0, 0), (0, vy, 0), (0, -vy, 0), (0, 0, wz), (0, 0, -wz), (vx, 0, wz), (vx, vy, 0)]:
        g = Gait(robot)
        for _ in range(int(2 * g.period / dt)):
            feet = g.update(dt, c)
            out.append([leg.ik(f)[0] for leg, f in zip(robot.legs, feet)])
    return out


def offenders(v, f, zl, cx, cy, min_range, tol):
    """Triangles with a vertex within tol of z = zl (or straddling it) and a point beyond min_range."""
    z = v[f][:, :, 2]
    lo, hi = z.min(axis=1), z.max(axis=1)
    near = (lo <= zl + tol) & (hi >= zl - tol)
    if not near.any():
        return 0, None
    tri = v[f[near]]
    r = np.hypot(tri[:, :, 0] - cx, tri[:, :, 1] - cy).max(axis=1)
    bad = r > min_range
    return int(bad.sum()), (float(r[bad].max()) if bad.any() else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('robot', nargs='+')
    ap.add_argument('--yaml', default='')
    ap.add_argument('--meshes', default='')
    ap.add_argument('--min-range', type=float, default=0.12)
    ap.add_argument('--tol', type=float, default=0.005)
    a = ap.parse_args()
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('fleet_description')
    path = a.yaml or os.path.join(share, 'config', 'robots.yaml')
    failed = False
    for name in a.robot:
        rb = load(name, path)
        mdir = os.path.join(a.meshes or os.path.join(share, 'meshes'), name)
        mirrored = rb.spec['body'].get('mirrored_meshes', [])
        lid = rb.spec['lidar']
        zl = float(lid['z']) - rb.height           # lidar plane in base_link
        cx, cy = float(lid['x']), float(lid['y'])
        bv, bf = read_obj(os.path.join(mdir, 'base.obj'))
        n_bad, r_bad = offenders(bv, bf, zl, cx, cy, a.min_range, a.tol)
        # highest point of the base beyond min_range, below the plane, and lowest above it
        rr = np.hypot(bv[:, 0] - cx, bv[:, 1] - cy)
        far = bv[rr > a.min_range]
        below = far[far[:, 2] < zl, 2]
        above = far[far[:, 2] > zl, 2]
        gap_lo = zl - below.max() if below.size else math.inf
        gap_hi = above.min() - zl if above.size else math.inf
        print(f'{name}: lidar plane z = {lid["z"]:.3f} m above the floor ({zl:+.3f} in base_link)')
        print(f'  base: {n_bad} triangles in the plane beyond {a.min_range} m; clearance below {gap_lo * 1000:.0f} mm, '
              f'above {gap_hi * 1000:.0f} mm')
        failed |= n_bad > 0
        meshes = {}
        for part in ('coxa', 'femur', 'tibia'):
            for side in ('l', 'r'):
                fn = f'{part}_{side}.obj' if part in mirrored else f'{part}.obj'
                meshes[(part, side)] = read_obj(os.path.join(mdir, fn))
        worst_gap = math.inf
        worst = None
        leg_bad = 0
        for qs in poses(rb):
            for leg, q in zip(rb.legs, qs):
                frames = leg.frames(q)
                side = 'l' if leg.side > 0 else 'r'
                for k, part in enumerate(('coxa', 'femur', 'tibia')):
                    v, f = meshes[(part, side)]
                    m = np.array(frames[k], dtype=float)
                    w = v @ m[:3, :3].T + m[:3, 3]
                    nb, _ = offenders(w, f, zl, cx, cy, a.min_range, a.tol)
                    leg_bad += nb
                    rr = np.hypot(w[:, 0] - cx, w[:, 1] - cy)
                    zz = w[rr > a.min_range, 2]
                    if zz.size:
                        gap = zl - zz.max()
                        if gap < worst_gap:
                            worst_gap, worst = gap, f'{leg.name}_{part}'
        print(f'  legs: {leg_bad} triangle hits over the sweep; closest leg geometry {worst_gap * 1000:.0f} mm below the '
              f'plane ({worst})')
        failed |= leg_bad > 0
        print(f'  -> {"FAIL" if (n_bad or leg_bad) else "PASS"}')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
