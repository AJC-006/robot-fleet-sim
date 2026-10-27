"""Swept stance envelope of a legged robot -> Nav2 footprint polygon.

The feet are swept through the gait at the robot's top speeds (forward, back, sideways, turning and
mixes) plus the stand-up; every sample adds the foot circle and the knee circle (femur radius). `body.outline` (base_link xy polygon of
the visual: body, head, tail, wings) is added too, everything grown by `margin`. The footprint is the
convex hull of that, cut down to `n` vertices by extending edges outwards (never inwards).
"""
import math

from fleet_legged.gait import Gait


def ramp(cur, target, dt, acc=(0.5, 0.5, 1.5)):
    """The gait node's acceleration limit on the command (m/s^2, m/s^2, rad/s^2)."""
    return tuple(c + max(-a * dt, min(a * dt, t - c)) for c, t, a in zip(cur, target, acc))


def foot_samples(robot, dt=0.01, cycles=3):
    spec = robot.spec
    vx, vy, wz = float(spec['max_vel_x']), float(spec.get('max_vel_y', 0.0)), float(spec['max_vel_theta'])
    cmds = [(vx, 0, 0), (-vx, 0, 0), (0, vy, 0), (0, -vy, 0), (0, 0, wz), (0, 0, -wz),
            (vx, 0, wz), (vx, 0, -wz), (vx, vy, 0), (vx, -vy, 0), (-vx, vy, 0), (-vx, -vy, 0),
            (0, vy, wz), (0, -vy, -wz), (0, vy, -wz), (0, -vy, wz)]
    rk = float(spec['link'].get('femur_radius', 0.0))
    pts = []
    for c in cmds:
        g = Gait(robot)
        v = (0.0, 0.0, 0.0)
        for k in range(int((cycles + 2) * g.period / dt)):
            v = ramp(v, c if k * dt < (cycles + 0.5) * g.period else (0.0, 0.0, 0.0), dt)
            for leg, f in zip(robot.legs, g.update(dt, v)):
                pts.append((f[0], f[1], leg.rf))
                if k % 3 == 0:          # knees (femur/tibia joint) stick out behind mammal hind legs
                    kn = leg.frames(leg.ik(f)[0])[2]
                    pts.append((kn[0][3], kn[1][3], rk))
    for leg in robot.legs:
        for h in (robot.crouch, robot.height):
            n = leg.nominal(h)
            pts.append((n[0], n[1], leg.rf))
    return pts


def _hull(pts):
    pts = sorted(set(pts))
    if len(pts) < 3:
        return pts
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]       # counter-clockwise


def _simplify(poly, n):
    """Drop edges of a convex CCW polygon by extending both neighbours (never cuts into it) until n
    vertices are left, always taking the edge that adds the least area."""
    poly = list(poly)
    while len(poly) > n:
        best = None
        m = len(poly)
        for i in range(m):
            a0, a1 = poly[i - 1], poly[i]
            b0, b1 = poly[(i + 1) % m], poly[(i + 2) % m]
            d1 = (a1[0] - a0[0], a1[1] - a0[1])
            d2 = (b1[0] - b0[0], b1[1] - b0[1])
            den = d1[0] * d2[1] - d1[1] * d2[0]
            if den <= 1e-12:        # the neighbours do not meet outside this edge
                continue
            t = ((b0[0] - a1[0]) * d2[1] - (b0[1] - a1[1]) * d2[0]) / den
            if t < 0:
                continue
            p = (a1[0] + d1[0] * t, a1[1] + d1[1] * t)
            area = abs((b0[0] - a1[0]) * (p[1] - a1[1]) - (b0[1] - a1[1]) * (p[0] - a1[0])) / 2
            if best is None or area < best[0]:
                best = (area, i, p)
        if best is None:
            break
        _, i, p = best
        j = (i + 1) % m
        poly[i] = p
        del poly[j]
    return poly


def footprint(robot, margin=0.01, n=12, k=24):
    """Convex footprint polygon (n vertices, CCW) around the swept feet and body outline, + margin."""
    pts = []
    for x, y, r in foot_samples(robot)[::3] + [(float(x), float(y), 0.0)
                                                for x, y in robot.spec.get('body', {}).get('outline', [])]:
        rr = (r + margin) / math.cos(math.pi / k)     # circumscribed k-gon of the grown circle
        pts += [(round(x + rr * math.cos(2 * math.pi * i / k), 5), round(y + rr * math.sin(2 * math.pi * i / k), 5))
                for i in range(k)]
    poly = _simplify(_hull(pts), n)
    poly = [(round(x, 3), round(y, 3)) for x, y in poly]
    radius = max(math.hypot(x, y) for x, y in poly)
    return poly, radius


def extents(robot):
    pts = foot_samples(robot)
    return (min(p[0] - p[2] for p in pts), max(p[0] + p[2] for p in pts),
            min(p[1] - p[2] for p in pts), max(p[1] + p[2] for p in pts))
