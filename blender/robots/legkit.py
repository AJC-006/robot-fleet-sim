"""Shared helpers for the legged robots (wyrm, hound, spider).

Link meshes are modelled directly in their URDF link frames (see
src/fleet_legged/fleet_legged/kinematics.py, imported here so the meshes, the URDF and the gait
agree on one convention):

  base   base_link: body centre, x forward, z up, standing_height above the floor
  coxa   at the hip joint; yaw leg: x along the coxa (outward), z up; roll leg: x forward, femur
         joint at (0, +-coxa, 0)
  femur  at the femur joint, x along the femur (to the knee at x = femur), y = joint axis
  tibia  at the knee, x along the tibia, foot sphere centre at x = tibia

The preview poses linked copies of the link meshes with the forward kinematics.
"""
import math
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector

from . import common
from .common import _obj_from_bm

sys.path.insert(0, str(common.ROOT / "src" / "fleet_legged"))
from fleet_legged import kinematics as K  # noqa: E402
from fleet_legged.gait import Gait  # noqa: E402

PARTS = ("coxa", "femur", "tibia")


def robot(name, cfg):
    return K.load(name, cfg=cfg)


def stance_theta(rb, leg=0):
    """Femur pitch q1 and tibia pitch q1 + q2 (rad, > 0 = pointing down) of a leg in the standing pose."""
    q = rb.stance_q()[leg]
    return q[1], q[1] + q[2]


# ----------------------------------------------------------------------------- organic lofts
def _catmull(pts, u):
    """Catmull-Rom through pts (list of Vector or float) at u in [0, 1]."""
    n = len(pts) - 1
    s = min(max(u, 0.0), 1.0) * n
    i = min(int(s), n - 1)
    t = s - i
    p0 = pts[max(i - 1, 0)]
    p1, p2 = pts[i], pts[i + 1]
    p3 = pts[min(i + 2, n)]
    t2, t3 = t * t, t * t * t
    return 0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)


class Loft:
    """Tube along a Catmull-Rom spine with elliptic (or superelliptic) cross-sections.

    spine: [Vector], ry/rz: per spine point half-widths (side / up), up: reference up vector.
    S(u, a): point on the surface; a = 0 at +side, pi/2 at the top."""

    def __init__(self, spine, ry, rz, up=(0, 0, 1), sq=None, roll=None):
        self.spine = [Vector(p) for p in spine]
        self.ry, self.rz = list(ry), list(rz)
        self.up = Vector(up)
        self.sq = sq or [2.0] * len(self.spine)        # superellipse exponent (2 = ellipse)
        self.roll = roll or [0.0] * len(self.spine)

    def frame(self, u):
        c = _catmull(self.spine, u)
        e = 1e-3
        t = (_catmull(self.spine, min(u + e, 1)) - _catmull(self.spine, max(u - e, 0))).normalized()
        up = self.up if abs(t.dot(self.up)) < 0.95 else Vector((1, 0, 0))
        ey = up.cross(t).normalized()
        ez = t.cross(ey).normalized()
        r = _catmull(self.roll, u)
        if r:
            ey, ez = ey * math.cos(r) + ez * math.sin(r), ez * math.cos(r) - ey * math.sin(r)
        return c, t, ey, ez

    def S(self, u, a):
        c, t, ey, ez = self.frame(u)
        ry, rz = max(_catmull(self.ry, u), 1e-5), max(_catmull(self.rz, u), 1e-5)
        p = _catmull(self.sq, u)
        ca, sa = math.cos(a), math.sin(a)
        cx = math.copysign(abs(ca) ** (2.0 / p), ca)
        sx = math.copysign(abs(sa) ** (2.0 / p), sa)
        return c + ey * (ry * cx) + ez * (rz * sx)

    def N(self, u, a):
        e = 1e-3
        du = self.S(min(u + e, 1), a) - self.S(max(u - e, 0), a)
        da = self.S(u, a + e) - self.S(u, a - e)
        n = da.cross(du)
        if n.length < 1e-12:
            c, t, ey, ez = self.frame(u)
            return (self.S(u, a) - c).normalized()
        n.normalize()
        c = self.frame(u)[0]
        return n if n.dot(self.S(u, a) - c) >= 0 else -n

    def mesh(self, ctx, name, material, nu=24, na=24, caps=(True, True), cap_len=(None, None), u0=0.0, u1=1.0):
        bm = bmesh.new()
        rings = []
        for i in range(nu + 1):
            u = u0 + (u1 - u0) * i / nu
            rings.append([bm.verts.new(self.S(u, 2 * math.pi * j / na)) for j in range(na)])
        for a, b in zip(rings[:-1], rings[1:]):
            for j in range(na):
                k = (j + 1) % na
                bm.faces.new((a[j], a[k], b[k], b[j]))
        for end, (cap, cl) in enumerate(zip(caps, cap_len)):
            if not cap:
                continue
            u = u0 if end == 0 else u1
            c, t, ey, ez = self.frame(u)
            ring = rings[0] if end == 0 else rings[-1]
            r = 0.5 * (_catmull(self.ry, u) + _catmull(self.rz, u))
            L = r * 0.6 if cl is None else cl
            sgn = -1 if end == 0 else 1
            # two shrinking rings + a pole: a rounded end
            prev = ring
            for f, d in ((0.72, 0.55), (0.38, 0.88)):
                cur = [bm.verts.new(c + (v.co - c) * f + t * (sgn * L * d)) for v in ring]
                for j in range(na):
                    k = (j + 1) % na
                    if end == 0:
                        bm.faces.new((cur[j], cur[k], prev[k], prev[j]))
                    else:
                        bm.faces.new((prev[j], prev[k], cur[k], cur[j]))
                prev = cur
            pole = bm.verts.new(c + t * (sgn * L))
            for j in range(na):
                k = (j + 1) % na
                if end == 0:
                    bm.faces.new((pole, prev[k], prev[j]))
                else:
                    bm.faces.new((prev[j], prev[k], pole))
        return _obj_from_bm(name, bm, material, (0, 0, 0), (0, 0, 0), ctx)

    def patch(self, ctx, name, material, u0, u1, a0, a1, off=0.002, th=0.003, nu=12, na=12, chamfer=0.6):
        """Panel lying on the surface: grid over [u0,u1] x [a0,a1], raised `off` along the normals,
        `th` thick, with chamfered rims (the outer face is shrunk by chamfer * th at the border)."""
        bm = bmesh.new()

        def P(i, j, o, shrink):
            u = u0 + (u1 - u0) * i / nu
            a = a0 + (a1 - a0) * j / na
            if shrink:
                du = shrink / max(self._len(), 1e-6)
                da = shrink / max(0.5 * (_catmull(self.ry, u) + _catmull(self.rz, u)), 1e-6)
                if i == 0:
                    u = min(u + du, u1)
                if i == nu:
                    u = max(u - du, u0)
                if j == 0:
                    a = min(a + da, a1)
                if j == na:
                    a = max(a - da, a0)
            return self.S(u, a) + self.N(u, a) * o

        top = [[bm.verts.new(P(i, j, off, chamfer * th)) for j in range(na + 1)] for i in range(nu + 1)]
        bot = [[bm.verts.new(P(i, j, off - th, 0.0)) for j in range(na + 1)] for i in range(nu + 1)]
        for i in range(nu):
            for j in range(na):
                bm.faces.new((top[i][j], top[i + 1][j], top[i + 1][j + 1], top[i][j + 1]))
                bm.faces.new((bot[i][j], bot[i][j + 1], bot[i + 1][j + 1], bot[i + 1][j]))
        rim = [(i, 0) for i in range(nu)] + [(nu, j) for j in range(na)] + \
              [(i, na) for i in range(nu, 0, -1)] + [(0, j) for j in range(na, 0, -1)]
        for (i, j), (k, l) in zip(rim, rim[1:] + rim[:1]):
            bm.faces.new((top[i][j], bot[i][j], bot[k][l], top[k][l]))
        return _obj_from_bm(name, bm, material, (0, 0, 0), (0, 0, 0), ctx)

    def _len(self):
        return sum((b - a).length for a, b in zip(self.spine[:-1], self.spine[1:]))


# ----------------------------------------------------------------------------- scales
def shingle_bm(bm, p, n, back, size, lift=0.25, seg=6, rings=3, thick=0.32):
    """One overlapping scale: a flattened half-dome at p, normal n, long axis toward `back`
    (the free edge), tilted up by `lift` rad so it overlaps the next row like a roof tile."""
    n = Vector(n).normalized()
    b = (Vector(back) - n * Vector(back).dot(n)).normalized()
    s = n.cross(b).normalized()
    ez = (n * math.cos(lift) - b * math.sin(lift)).normalized()
    ey = ez.cross(s).normalized()      # along the scale, toward the free edge (roughly b)
    sx, sy = size
    sz = min(sx, sy) * thick
    M = Matrix((s * sx, ey * sy, ez * sz)).transposed().to_4x4()
    M.translation = Vector(p) + n * (sz * 0.15) + ey * (sy * 0.35)
    bmesh.ops.create_uvsphere(bm, u_segments=seg, v_segments=rings, radius=1.0, matrix=M)


def scales_on(loft, us, a_range, size_fn, n_fn, lift=0.28, seg=6, rings=3, jitter=0.0, skip=None, tints=2,
              back_sign=-1.0, thick=0.32):
    """Rows of shingles over a Loft. us: row positions along the spine; a_range: (a0, a1) arc;
    size_fn(u) -> (sx, sy); n_fn(u) -> scales per full turn. Returns `tints` bmeshes (alternating rows)."""
    import random
    rnd = random.Random(7)
    bms = [bmesh.new() for _ in range(tints)]
    a0, a1 = a_range
    for k, u in enumerate(us):
        c, t, ey, ez = loft.frame(u)
        n = max(3, int(n_fn(u)))
        da = 2 * math.pi / n
        a = a0 + (0.5 * da if k % 2 else 0.0)
        while a <= a1 + 1e-6:
            aj = a + (rnd.uniform(-1, 1) * jitter * da if jitter else 0.0)
            p = loft.S(u, aj)
            if skip is None or not skip(p, u, aj):
                nn = loft.N(u, aj)
                sx, sy = size_fn(u)
                shingle_bm(bms[(k + (1 if rnd.random() < 0.12 else 0)) % tints], p, nn, t * back_sign, (sx, sy),
                           lift, seg, rings, thick)
            a += da
    return bms


def bm_obj(ctx, name, bm, material):
    return _obj_from_bm(name, bm, material, (0, 0, 0), (0, 0, 0), ctx)


# ----------------------------------------------------------------------------- small parts
def claw(ctx, name, base, direction, down, length, r, material, curl=0.9, seg=8, n=5):
    """Curved tapering claw from `base` along `direction`, curling toward `down`."""
    d = Vector(direction).normalized()
    dn = (Vector(down) - d * Vector(down).dot(d)).normalized()
    pts = []
    for i in range(n + 1):
        t = i / n
        ang = curl * t
        pts.append(Vector(base) + (d * math.sin(ang) / max(curl, 1e-3) + dn * (1 - math.cos(ang)) / max(curl, 1e-3)) * length)
    L = Loft(pts, [r * (1 - 0.92 * i / n) for i in range(n + 1)], [r * (1 - 0.92 * i / n) for i in range(n + 1)],
             up=tuple(-dn) if abs(d.dot(Vector((0, 0, 1)))) > 0.9 else (0, 0, 1))
    return L.mesh(ctx, name, material, nu=n * 2, na=seg, caps=(True, True), cap_len=(r * 0.5, r * 0.3))


def strip(ctx, name, pts, w, h, material, normal=(0, 0, 1)):
    """Thin raised strip (light bar, trim) along a polyline lying on a surface with `normal`."""
    bm = bmesh.new()
    nrm = Vector(normal).normalized()
    prev = None
    for i, p in enumerate(pts):
        p = Vector(p)
        a = Vector(pts[max(i - 1, 0)])
        b = Vector(pts[min(i + 1, len(pts) - 1)])
        t = (b - a).normalized()
        s = nrm.cross(t).normalized()
        ring = [bm.verts.new(p + s * (w / 2) * sx + nrm * (h * sz)) for sx, sz in ((-1, 0), (1, 0), (1, 1), (-1, 1))]
        if prev:
            for j in range(4):
                k = (j + 1) % 4
                bm.faces.new((prev[j], prev[k], ring[k], ring[j]))
        else:
            bm.faces.new(ring[::-1])
        prev = ring
    bm.faces.new(prev)
    return _obj_from_bm(name, bm, material, (0, 0, 0), (0, 0, 0), ctx)


# ----------------------------------------------------------------------------- posing / export
def pose_q(rb, mode="walk", t=None):
    """Joint angles for the preview: 'stand' or a walking snapshot (forward trot, mid swing)."""
    if mode == "stand":
        return rb.stance_q()
    g = Gait(rb)
    v = (0.6 * float(rb.spec["max_vel_x"]), 0.0, 0.0)
    dt = 0.005
    T = t if t is not None else g.period * (2.0 + 0.25 * (1 - g.duty)) + g.period * g.duty * 0.0
    for _ in range(int(T / dt)):
        feet = g.update(dt, v)
    return [leg.ik(f)[0] for leg, f in zip(rb.legs, feet)]


def place_legs(ctx, rb, parts, qs, base_z):
    """Linked copies of the link meshes at the FK frames (ground coordinates). parts[part] is an object
    or {'l': obj, 'r': obj} for mirrored parts."""
    out = []
    T = Matrix.Translation((0, 0, base_z))
    for leg, q in zip(rb.legs, qs):
        fr = leg.frames(q)
        side = "l" if leg.side > 0 else "r"
        for k, part in enumerate(PARTS):
            src = parts[part][side] if isinstance(parts[part], dict) else parts[part]
            o = bpy.data.objects.new(f"{leg.name}_{part}", src.data)
            o.matrix_world = T @ Matrix(fr[k])
            ctx.coll.objects.link(o)
            out.append(o)
    return out


def export_links(name, base, parts):
    """Export base.obj and the link meshes (all modelled at their link frame origin)."""
    out = common.MESH_DIR / name
    common.export_obj(base, out / "base.obj")
    n = {"base": common.tri_count(base)}
    for part in PARTS:
        p = parts[part]
        if isinstance(p, dict):
            for s, o in p.items():
                common.export_obj(o, out / f"{part}_{s}.obj")
                n[f"{part}_{s}"] = common.tri_count(o)
        else:
            common.export_obj(p, out / f"{part}.obj")
            n[part] = common.tri_count(p)
    return n


def mirror_obj(ctx, obj, name):
    """Copy of obj mirrored in y (normals fixed)."""
    me = obj.data.copy()
    me.name = name
    me.transform(Matrix.Diagonal((1, -1, 1, 1)))
    me.flip_normals()
    o = bpy.data.objects.new(name, me)
    ctx.coll.objects.link(o)
    return o
