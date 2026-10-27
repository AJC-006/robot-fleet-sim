"""wyrm: dragon-hatchling quadruped with a sprawling salamander stance. Long scaled body that runs
into a tapering tail (visual only, held clear of the floor), a horned head on a short neck with a
snout and glowing violet eyes, wings folded along the back, a glowing crystal cradled by bone
prongs on the back (the lidar), clawed four-toed feet. Legs: shoulder ball, scaled upper leg,
forearm with a clawed hand (yaw legs, knee up)."""
import bisect
import math
import random

import bmesh
from mathutils import Matrix, Vector

from . import legkit as LK
from .common import apply_mods, cyl, join, mesh_from, sphere, torus, tube
from .common import _obj_from_bm

TINTS = ("wyrm_scale", "wyrm_scale2")


# ----------------------------------------------------------------------------- body
def body_loft():
    """One loft from the tail tip (u = 0) to the neck (u = 1), base_link frame. The tail sweeps back,
    then curls round to the right and dips; its tip stays ~8 cm above the floor while standing (it has
    no collision). Only the tail runs backwards in x (Arc.top_z relies on x rising elsewhere)."""
    spine = [(-0.300, -0.158, -0.060), (-0.326, -0.136, -0.057), (-0.342, -0.104, -0.052), (-0.340, -0.068, -0.046),
             (-0.322, -0.036, -0.038), (-0.290, -0.013, -0.029), (-0.248, -0.002, -0.020),
             (-0.200, 0.0, -0.010), (-0.160, 0.0, -0.003), (-0.110, 0.0, 0.004), (-0.050, 0.0, 0.009),
             (0.020, 0.0, 0.010), (0.080, 0.0, 0.008), (0.130, 0.0, 0.006), (0.170, 0.0, 0.012), (0.200, 0.0, 0.022)]
    ry = [0.0030, 0.0055, 0.0080, 0.0110, 0.0145, 0.0190, 0.0250,
          0.033, 0.047, 0.060, 0.067, 0.068, 0.065, 0.056, 0.040, 0.034]
    rz = [0.0030, 0.0052, 0.0075, 0.0100, 0.0130, 0.0165, 0.0200,
          0.026, 0.035, 0.040, 0.043, 0.043, 0.041, 0.037, 0.031, 0.029]
    sq = [2.0, 2.0, 2.0, 2.0, 2.05, 2.1, 2.2, 2.3, 2.4, 2.5, 2.5, 2.5, 2.5, 2.4, 2.2, 2.1]
    return LK.Loft(spine, ry, rz, sq=sq)


class Arc:
    """Arc-length table of a Loft spine: rows of scales at even spacing, x -> cross-section lookup."""

    def __init__(self, L, n=600):
        self.L = L
        self.us = [i / n for i in range(n + 1)]
        self.c = [LK._catmull(L.spine, u) for u in self.us]
        self.s = [0.0]
        for a, b in zip(self.c[:-1], self.c[1:]):
            self.s.append(self.s[-1] + (b - a).length)

    @property
    def length(self):
        return self.s[-1]

    def u(self, d):
        d = min(max(d, 0.0), self.s[-1])
        i = max(1, bisect.bisect_left(self.s, d))
        s0, s1 = self.s[i - 1], self.s[i]
        t = (d - s0) / (s1 - s0) if s1 > s0 else 0.0
        return self.us[i - 1] + t * (self.us[i] - self.us[i - 1])

    def r(self, u):
        return 0.5 * (LK._catmull(self.L.ry, u) + LK._catmull(self.L.rz, u))

    def rows(self, d0, d1, step_fn):
        out, d = [], d0
        while d <= d1:
            u = self.u(d)
            out.append(u)
            d += step_fn(u)
        return out

    def top_z(self, x, y):
        """Height of the upper surface at (x, y) (centre height outside the cross-section)."""
        xs = [c.x for c in self.c]
        i = min(max(bisect.bisect_left(xs, x), 1), len(xs) - 1)
        t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]) if xs[i] != xs[i - 1] else 0.0
        t = min(max(t, 0.0), 1.0)
        u = self.us[i - 1] + t * (self.us[i] - self.us[i - 1])
        c = LK._catmull(self.L.spine, u)
        ry, rz, p = (LK._catmull(v, u) for v in (self.L.ry, self.L.rz, self.L.sq))
        dy = abs(y - c.y)
        if dy >= ry:
            return c.z
        return c.z + rz * (1.0 - (dy / ry) ** p) ** (1.0 / p)


def in_poly(p, poly):
    x, y = p
    inside = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def wing_outline(s):
    """Folded wing membrane outline (xy), side s: shoulder, elbow, wrist, scalloped finger tips,
    back along the flank."""
    pts = [(0.088, 0.050), (0.060, 0.062), (0.032, 0.070), (-0.020, 0.071), (-0.066, 0.068), (-0.110, 0.071),
           (-0.146, 0.072), (-0.140, 0.060), (-0.178, 0.054), (-0.166, 0.044), (-0.184, 0.030), (-0.130, 0.032),
           (-0.070, 0.038), (-0.010, 0.040), (0.050, 0.042)]
    return [(x, s * y) for x, y in pts]


def torso(ctx):
    P = []
    L = body_loft()
    A = Arc(L)
    P.append(L.mesh(ctx, "body", "wyrm_skin", nu=72, na=36, caps=(True, True), cap_len=(0.006, 0.02)))
    # overlapping scales over the back and flanks (free edges toward the tail); skipped where the
    # folded wings cover the back
    wings = [wing_outline(1), wing_outline(-1)]

    def skip(p, u, a):
        if p.z > 0.02 and any(in_poly((p.x + 0.006, p.y * 1.06), w) for w in wings):
            return True
        return abs(p.x + 0.03) < 0.022 and abs(p.y) < 0.020 and p.z > 0.04      # crystal mound

    def size(u):
        r = A.r(u)
        return (min(0.0095, 0.0030 + 0.115 * r), min(0.0115, 0.0036 + 0.135 * r))

    def count(u):
        r = A.r(u)
        circ = 2 * math.pi * r
        return circ / (2 * size(u)[0] * 0.78)

    us = A.rows(0.012, A.length - 0.012, lambda u: 2 * size(u)[1] * 0.60)
    bms = LK.scales_on(L, us, (-0.55, math.pi + 0.55), size, count, lift=0.30, seg=5, rings=3, jitter=0.12,
                       skip=skip, tints=2, back_sign=-1.0, thick=0.34)
    for k, bm in enumerate(bms):
        P.append(LK.bm_obj(ctx, "scales", bm, TINTS[k]))
    # ventral scutes: broad transverse plates under the chest and belly
    for d in [A.length - 0.056 - 0.0143 * k for k in range(24)]:
        u = A.u(d)
        du = 0.010 / A.length
        P.append(L.patch(ctx, "scute", "deeppurple", u, u + du * 1.3, math.pi + 0.55, 2 * math.pi - 0.55, 0.0012,
                         0.0024, nu=1, na=6, chamfer=0.5))
    # dorsal ridge: small backward-raked spines from the neck to the tail tip
    bm = bmesh.new()
    for d in [0.022 + 0.0135 * k for k in range(int((A.length - 0.03) / 0.0135))]:
        u = A.u(d)
        c, t, ey, ez = L.frame(u)
        p = L.S(u, math.pi / 2)
        if abs(p.x + 0.03) < 0.040:
            continue
        r = A.r(u)
        h = 0.004 + 0.18 * r
        dr = (ez * 0.82 - t * 0.57).normalized()
        rot = Vector((0, 0, 1)).rotation_difference(dr).to_matrix().to_4x4()
        rot.translation = p + dr * (h * 0.45)
        bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=True, segments=5, radius1=0.0016 + 0.05 * r,
                              radius2=0.0003, depth=h, matrix=rot)
    P.append(LK.bm_obj(ctx, "ridge", bm, "wyrm_scale2"))
    return P, A


def _arm_z(p, segs, pts):
    """Height of the nearest wing bone above the xy point p."""
    S, E, W, T = pts
    bones = [(S, E), (E, W)] + [(W, t) for t in T]
    best, z = 1e9, 0.0
    for a, b in bones:
        a2, b2 = Vector((a.x, a.y)), Vector((b.x, b.y))
        t = max(0.0, min(1.0, (p - a2).dot(b2 - a2) / max((b2 - a2).length_squared, 1e-12)))
        d = (p - (a2 + t * (b2 - a2))).length
        if d < best:
            best, z = d, a.z + t * (b.z - a.z)
    return z


def wing(ctx, A, s):
    """Folded wing on side s: bone arm (shoulder, elbow, wrist), three fingers, membrane draped over the
    back with a scalloped trailing edge, and a thumb claw at the wrist."""
    P = []

    def on(x, y, dz):
        return Vector((x, y, A.top_z(x, y) + dz))

    S = on(0.088, s * 0.048, 0.012)
    E = on(0.030, s * 0.064, 0.032)
    W = on(-0.066, s * 0.062, 0.028)
    T = [on(-0.146, s * 0.068, 0.012), on(-0.178, s * 0.052, 0.011), on(-0.184, s * 0.031, 0.010)]
    outline = wing_outline(s)
    arm = [(S.x, S.y), (E.x, E.y), (W.x, W.y)] + [(t.x, t.y) for t in T]
    bm = bmesh.new()
    vs = [bm.verts.new((x, y, 0)) for x, y in outline]
    if s < 0:
        vs = vs[::-1]
    bm.faces.new(vs)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    bmesh.ops.subdivide_edges(bm, edges=bm.edges[:], cuts=4, use_grid_fill=True)
    segs = [(arm[0], arm[1]), (arm[1], arm[2])] + [(arm[2], a) for a in arm[3:]]
    for v in bm.verts:
        p = Vector((v.co.x, v.co.y))
        d = 1e9
        for a, b in segs:
            a, b = Vector(a), Vector(b)
            t = max(0.0, min(1.0, (p - a).dot(b - a) / max((b - a).length_squared, 1e-12)))
            d = min(d, (p - (a + t * (b - a))).length)
        w = max(0.0, 1.0 - d / 0.022)
        zb = A.top_z(v.co.x, v.co.y) + 0.0050
        za = _arm_z(p, segs, (S, E, W, T)) - 0.002
        v.co.z = zb + max(0.0, za - zb) * w * w * (3 - 2 * w)
    zmem = {}
    for v in bm.verts:
        zmem[(round(v.co.x, 4), round(v.co.y, 4))] = v.co.z
    o = _obj_from_bm("membrane", bm, "wyrm_wing", (0, 0, 0), (0, 0, 0), ctx)
    m = o.modifiers.new("sol", "SOLIDIFY")
    m.thickness = 0.0022
    m.offset = 0
    P.append(apply_mods(o))

    def mz(x, y):
        """Membrane height at (x, y): nearest sampled vertex (dense enough for veins)."""
        k = min(zmem, key=lambda q: (q[0] - x) ** 2 + (q[1] - y) ** 2)
        return zmem[k]

    # veins: fine raised ribs fanning from the wrist and forearm to the trailing edge along the flank
    def lerp(a, b, t):
        return (a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)

    veins = [((W.x, W.y), (-0.140, s * 0.060)), ((W.x, W.y), (-0.166, s * 0.044)), ((W.x, W.y), (-0.152, s * 0.033)),
             (lerp(E, W, 0.6), (-0.104, s * 0.034)), (lerp(E, W, 0.2), (-0.048, s * 0.039)),
             (lerp(S, E, 0.5), (0.012, s * 0.041))]
    vb = bmesh.new()
    for k, ((rx, ry_), (ex, ey)) in enumerate(veins):
        pts = []
        for i in range(7):
            t = i / 6
            x = rx + (ex - rx) * t
            y = ry_ + (ey - ry_) * t + s * 0.003 * math.sin(math.pi * t) * (1 if k % 2 else -1)
            pts.append(Vector((x, y, mz(x, y) + 0.0013)))
        for a, b in zip(pts[:-1], pts[1:]):
            d = b - a
            rot = Vector((0, 0, 1)).rotation_difference(d.normalized()).to_matrix().to_4x4()
            rot.translation = (a + b) * 0.5
            bmesh.ops.create_cone(vb, cap_ends=False, segments=5, radius1=0.0010, radius2=0.0008, depth=d.length,
                                  matrix=rot)
    P.append(LK.bm_obj(ctx, "veins", vb, "wyrm_vein"))
    # trailing edge: a slightly thicker rim from the outer finger tip round the scallops and along the flank
    rim = [Vector((x, y, mz(x, y) + 0.0004)) for x, y in outline[6:]]
    for a, b in zip(rim[:-1], rim[1:]):
        P.append(tube(ctx, "rim", a, b, 0.0015, "wyrm_vein", seg=6))
    # bones
    P.append(tube(ctx, "humerus", S, E, 0.0075, "wyrm_scale2", seg=10, r2=0.0062))
    P.append(tube(ctx, "forearm", E, W, 0.0062, "wyrm_scale2", seg=10, r2=0.0052))
    for p, r in ((S, 0.0095), (E, 0.0070), (W, 0.0060)):
        P.append(sphere(ctx, "wing_joint", r, p, "wyrm_scale", seg=12, rings=8))
    for t in T:
        P.append(tube(ctx, "finger", W, t, 0.0036, "wyrm_scale2", seg=8, r2=0.0012))
    P.append(LK.claw(ctx, "thumb", W + Vector((0.004, s * 0.003, 0.003)), (0.8, s * 0.25, 0.5), (0, s * 0.3, -1), 0.014,
                     0.0028, "wyrm_horn", curl=1.1))
    return P


def crystal(ctx, lid):
    """The lidar: a glowing crystal (equator on the sensor plane) on a stalk, cradled by bone prongs
    rising from a scaled mound on the back."""
    P = []
    lz = lid["z"] - lid["_bz"]
    x0 = lid["x"]
    P.append(sphere(ctx, "mound", 1.0, (x0, 0, 0.050), "wyrm_scale2", scale=(0.034, 0.030, 0.021), seg=28, rings=14))
    P.append(torus(ctx, "mound_ring", 0.022, 0.0022, (x0, 0, 0.066), "wyrm_scale", major=32, minor=6,
                   scale=(1.1, 1.0, 1.0)))
    P.append(cyl(ctx, "stalk", 0.011, 0.040, (x0, 0, 0.086), "wyrm_skin", seg=20, r2=0.007, bev=0.001))
    P.append(torus(ctx, "stalk_ring", 0.0085, 0.0018, (x0, 0, 0.098), "wyrm_core", major=20, minor=6))
    # hexagonal crystal: prism band + pyramids
    eq, band, top, bot = 0.021, 0.012, 0.034, 0.026
    n = 6
    verts = [(x0, 0, lz + top), (x0, 0, lz - bot)]
    for zz in (lz + band, lz - band):
        for i in range(n):
            a = 2 * math.pi * (i + 0.5) / n
            verts.append((x0 + eq * math.cos(a), eq * math.sin(a), zz))
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces.append((0, 2 + i, 2 + j))
        faces.append((2 + i, 2 + n + i, 2 + n + j, 2 + j))
        faces.append((1, 2 + n + j, 2 + n + i))
    P.append(mesh_from(ctx, "crystal", verts, faces, "wyrm_crystal"))
    P.append(cyl(ctx, "crystal_seat", 0.013, 0.006, (x0, 0, lz - bot + 0.001), "wyrm_scale2", seg=12, r2=0.009))
    # three obsidian talons curling up from the mound to grip the crystal's lower point
    for k in range(3):
        a = 2 * math.pi * k / 3 + math.pi / 3
        ca, sa = math.cos(a), math.sin(a)
        base = Vector((x0 + 0.020 * ca, 0.020 * sa, 0.066))
        P.append(LK.claw(ctx, "talon", base, (0.25 * ca, 0.25 * sa, 0.97), (-ca, -sa, 0.0), 0.040, 0.0034,
                         "wyrm_horn", curl=1.1, seg=8, n=6))
    # a cluster of small glowing shards breaking through the scales round the mound
    rnd = random.Random(11)
    for k in range(7):
        a = 2 * math.pi * k / 7 + rnd.uniform(-0.25, 0.25)
        ca, sa = math.cos(a), math.sin(a)
        r0 = 0.026 + rnd.uniform(-0.003, 0.004)
        h = rnd.uniform(0.012, 0.022)
        P.append(shard(ctx, Vector((x0 + r0 * ca * 1.1, r0 * sa, 0.054)), Vector((0.55 * ca, 0.55 * sa, 0.84)), h,
                       h * 0.28))
    return P


def shard(ctx, base, d, h, r):
    """Hexagonal crystal point from `base` along `d`."""
    d = d.normalized()
    rot = Vector((0, 0, 1)).rotation_difference(d).to_matrix()
    verts = [tuple(base + rot @ Vector((0, 0, h)))]
    for zz in (0.62 * h, -0.004):
        for i in range(6):
            a = 2 * math.pi * i / 6
            verts.append(tuple(base + rot @ Vector((r * math.cos(a), r * math.sin(a), zz))))
    faces = []
    for i in range(6):
        j = (i + 1) % 6
        faces.append((0, 1 + i, 1 + j))
        faces.append((1 + i, 7 + i, 7 + j, 1 + j))
    faces.append(tuple(7 + i for i in range(6))[::-1])
    return mesh_from(ctx, "shard", verts, faces, "wyrm_crystal")


HEAD_PIVOT = Vector((0.178, 0.0, 0.028))
HEAD_SCALE = Vector((1.06, 1.16, 1.16))      # skull is modelled small, then grown about the neck joint


def _head_pt(p):
    p = Vector(p) - HEAD_PIVOT
    return HEAD_PIVOT + Vector((p.x * HEAD_SCALE.x, p.y * HEAD_SCALE.y, p.z * HEAD_SCALE.z))


def _grow(objs):
    M = Matrix.Translation(HEAD_PIVOT) @ Matrix.Diagonal(HEAD_SCALE.to_4d()) @ Matrix.Translation(-HEAD_PIVOT)
    M[3][3] = 1.0
    for ob in objs:
        ob.data.transform(M @ ob.matrix_basis)
        ob.matrix_basis = Matrix.Identity(4)
    return objs


def head(ctx, cam):
    P = []
    cz = cam["z"] - cam["_bz"]
    cx = cam["x"]
    # skull-space position of the camera, so the lens lands on the sensor pose after _grow
    cxs = HEAD_PIVOT.x + (cx - HEAD_PIVOT.x) / HEAD_SCALE.x
    H = LK.Loft([(0.178, 0, 0.028), (0.210, 0, 0.037), (0.240, 0, 0.037), (0.268, 0, 0.027), (0.292, 0, 0.021),
                 (0.307, 0, 0.017)],
                ry=[0.036, 0.046, 0.042, 0.031, 0.022, 0.013], rz=[0.031, 0.037, 0.030, 0.021, 0.016, 0.010],
                sq=[2.2, 2.3, 2.4, 2.4, 2.3, 2.2])
    P.append(H.mesh(ctx, "skull", "wyrm_scale", nu=30, na=36, caps=(True, True), cap_len=(0.012, 0.007)))
    # small scales on the crown of the head and the snout bridge (clear of the eyes)
    Ah = Arc(H)

    def skip(p, u, a):
        return (0.218 < p.x < 0.256 and abs(p.y) > 0.020) or (abs(p.x - cxs) < 0.012 and abs(p.y) < 0.011)

    us = Ah.rows(0.004, Ah.length - 0.012, lambda u: 0.0062)
    bms = LK.scales_on(H, us, (0.35, math.pi - 0.35), lambda u: (0.0042, 0.0050),
                       lambda u: 2 * math.pi * Ah.r(u) / 0.0068, lift=0.25, seg=5, rings=3, skip=skip, tints=2,
                       back_sign=-1.0, thick=0.32)
    for k, bm in enumerate(bms):
        P.append(LK.bm_obj(ctx, "head_scales", bm, TINTS[(k + 1) % 2]))
    # lower jaw and mouth line
    J = LK.Loft([(0.196, 0, 0.010), (0.235, 0, 0.006), (0.270, 0, 0.004), (0.298, 0, 0.004)],
                ry=[0.030, 0.028, 0.021, 0.012], rz=[0.013, 0.011, 0.009, 0.006], sq=[2.2] * 4)
    P.append(J.mesh(ctx, "jaw", "wyrm_skin", nu=14, na=24, caps=(True, True), cap_len=(0.008, 0.006)))
    for s in (1, -1):
        # mouth seam and teeth
        P.append(tube(ctx, "lip", (0.205, s * 0.031, 0.013), (0.296, s * 0.011, 0.010), 0.0024, "charcoal", seg=6,
                      r2=0.0016))
        for k, xx in enumerate((0.250, 0.264, 0.278, 0.290)):
            yy = s * (0.022 - 0.0105 * (xx - 0.25) / 0.04)
            P.append(tube(ctx, "tooth", (xx, yy, 0.012), (xx + 0.001, yy, 0.0055), 0.0019, "bone", seg=6, r2=0.0002))
        # eye: almond socket following the side of the skull (which narrows toward the snout), a glowing
        # iris bulging from it, a vertical slit pupil and a heavy brow ridge
        e = Vector((0.238, s * 0.035, 0.049))
        rz_ = (0, 0, -s * 0.25)
        n = Vector((0.247, s * 0.969, 0.0))                       # outward axis of the rotated eye
        P.append(sphere(ctx, "socket", 1.0, e, "visor", scale=(0.0172, 0.0098, 0.0124), seg=20, rings=12, rot=rz_))
        ic = e + n * 0.0046
        P.append(sphere(ctx, "iris", 1.0, ic, "wyrm_iris", scale=(0.0136, 0.0072, 0.0100), seg=24, rings=14, rot=rz_))
        # bright core round the pupil: the deep violet iris then reads as a glow fading outward
        P.append(sphere(ctx, "iris_core", 1.0, ic + n * 0.0012, "wyrm_eyecore", scale=(0.0098, 0.0066, 0.0074), seg=20,
                        rings=12, rot=rz_))
        P.append(sphere(ctx, "pupil", 1.0, ic + n * 0.0064, "visor", scale=(0.0016, 0.0016, 0.0084), seg=10, rings=8,
                        rot=rz_))
        P.append(sphere(ctx, "brow", 1.0, (0.236, s * 0.036, 0.0605), "wyrm_scale2", scale=(0.021, 0.0085, 0.0046),
                        seg=16, rings=8, rot=(s * 0.25, -0.12, -s * 0.22)))
        P.append(sphere(ctx, "nostril", 1.0, (0.302, s * 0.0085, 0.0265), "visor", scale=(0.0042, 0.0026, 0.0028),
                        seg=10, rings=6, rot=(0, 0, s * 0.6)))
    # crest of small plates on the skull, running into the dorsal ridge
    for k in range(4):
        x = 0.214 - 0.012 * k
        P.append(sphere(ctx, "crest", 1.0, (x, 0, Ah.top_z(x, 0.0) + 0.002),
                        "wyrm_scale2", scale=(0.0062, 0.0022, 0.0060 - 0.0008 * k), seg=10, rings=6, rot=(0, -0.5, 0)))
    _grow(P)
    for s in (1, -1):
        # horns: a long pair swept back over the neck and a short lower pair, dark obsidian
        # (tips ~2 cm under the lidar plane: check_lidar_plane.py wyrm)
        P.append(LK.claw(ctx, "horn", _head_pt((0.206, s * 0.025, 0.058)), (-0.90, s * 0.30, 0.25), (0, s * 0.10, 1.0),
                         0.085, 0.0100, "wyrm_horn", curl=0.60, seg=12, n=9))
        P.append(LK.claw(ctx, "horn2", _head_pt((0.196, s * 0.039, 0.043)), (-0.90, s * 0.42, 0.04), (0, 0.0, 1.0),
                         0.044, 0.0060, "wyrm_horn", curl=0.6, seg=8, n=6))
        # cheek spines fanning back from the jaw hinge
        for k, (dz, ln) in enumerate(((0.010, 0.022), (0.0, 0.027), (-0.010, 0.020))):
            b = _head_pt((0.200, s * 0.033, 0.024 + dz))
            P.append(LK.claw(ctx, "cheek", b, (-0.85, s * 0.50, dz * 14), (0, s * 0.3, -0.4), ln, 0.0032, "wyrm_horn",
                             curl=0.4, seg=6, n=4))
    # camera: a dark lens jewel set in a bony socket on the snout bridge (at the sensor pose)
    P.append(sphere(ctx, "cam_socket", 1.0, (cx - 0.003, 0, cz - 0.003), "wyrm_scale2", scale=(0.013, 0.010, 0.0068),
                    seg=16, rings=8))
    P.append(sphere(ctx, "cam_lens", 0.0056, (cx + 0.0005, 0, cz + 0.0002), "lens", seg=16, rings=10))
    return P


# ----------------------------------------------------------------------------- legs
def coxa(ctx, lc, rc):
    """Shoulder/hip ball at the yaw joint and a short scaled segment to the femur joint at (lc, 0, 0)."""
    P = []
    P.append(sphere(ctx, "shoulder", rc * 1.12, (0, 0, 0.002), "wyrm_skin", seg=20, rings=12))
    L = LK.Loft([(-0.004, 0, 0.002), (lc * 0.5, 0, 0.002), (lc + 0.004, 0, 0)], ry=[rc * 1.0, rc * 0.92, rc * 0.84],
                rz=[rc * 1.05, rc * 0.98, rc * 0.88])
    P.append(L.mesh(ctx, "coxa_seg", "wyrm_skin", nu=6, na=18, caps=(False, False)))
    us = [0.12, 0.40, 0.68]
    bms = LK.scales_on(L, us, (-0.5, math.pi + 0.5), lambda u: (0.0072, 0.0082), lambda u: 12, lift=0.2, seg=5,
                       rings=3, tints=2, back_sign=1.0, thick=0.26)
    for k, bm in enumerate(bms):
        P.append(LK.bm_obj(ctx, "coxa_scales", bm, TINTS[k]))
    return join(ctx, P, "coxa")


def femur(ctx, l1, r1):
    """Muscular scaled upper leg to the knee at (l1, 0, 0) with an elbow knob and spur."""
    P = []
    L = LK.Loft([(-0.010, 0, 0.0), (0.024, 0, 0.005), (0.060, 0, 0.005), (l1, 0, 0.0)],
                ry=[r1 * 1.10, r1 * 1.28, r1 * 1.12, r1 * 0.86], rz=[r1 * 1.20, r1 * 1.42, r1 * 1.22, r1 * 0.92],
                sq=[2.2] * 4)
    P.append(L.mesh(ctx, "femur_seg", "wyrm_skin", nu=14, na=20, caps=(True, False), cap_len=(0.010, None)))
    A = Arc(L)
    us = A.rows(0.004, A.length - 0.008, lambda u: 0.0092)
    bms = LK.scales_on(L, us, (-0.55, math.pi + 0.55), lambda u: (0.0072, 0.0080),
                       lambda u: 2 * math.pi * A.r(u) / 0.0112, lift=0.20, seg=5, rings=3, tints=2, back_sign=1.0,
                       thick=0.26)
    for k, bm in enumerate(bms):
        P.append(LK.bm_obj(ctx, "femur_scales", bm, TINTS[k]))
    # underside: smooth pale-dark skin plates
    P.append(L.patch(ctx, "femur_belly", "deeppurple", 0.12, 0.82, math.pi + 0.7, 2 * math.pi - 0.7, 0.0008, 0.0018,
                     nu=6, na=3))
    P.append(sphere(ctx, "knee", r1 * 0.98, (l1, 0, 0.002), "wyrm_scale", seg=16, rings=10))
    P.append(LK.claw(ctx, "spur", (l1 + 0.002, 0, r1 * 0.80), (0.55, 0, 0.83), (1, 0, 0), 0.014, 0.0032, "wyrm_horn",
                     curl=0.6, seg=6, n=4))
    return join(ctx, P, "femur")


def tibia(ctx, l2, r2, rf, theta):
    """Forearm from the knee down to the wrist, then a hand flat on the floor (stance pitch theta) with
    four splayed clawed toes."""
    P = []
    up = Vector((-math.sin(theta), 0, math.cos(theta)))       # world up in the link frame at stance
    out = Vector((math.cos(theta), 0, math.sin(theta)))       # outward horizontal
    side = Vector((0, 1, 0))
    wrist = Vector((l2, 0, 0)) + up * (rf * 0.10)
    L = LK.Loft([(0.0, 0, 0), (0.030, 0, 0.004), (0.068, 0, 0.003), (wrist.x - 0.004, 0, wrist.z * 0.6)],
                ry=[r2 * 1.40, r2 * 1.32, r2 * 1.05, r2 * 0.82], rz=[r2 * 1.50, r2 * 1.42, r2 * 1.12, r2 * 0.86],
                sq=[2.2] * 4)
    P.append(L.mesh(ctx, "tibia_seg", "wyrm_skin", nu=12, na=18, caps=(True, False), cap_len=(0.008, None)))
    A = Arc(L)
    us = A.rows(0.006, A.length - 0.010, lambda u: 0.0088)
    bms = LK.scales_on(L, us, (-1.0, math.pi + 1.0), lambda u: (0.0064, 0.0074),
                       lambda u: 2 * math.pi * A.r(u) / 0.0104, lift=0.20, seg=5, rings=3, tints=2, back_sign=1.0,
                       thick=0.26)
    for k, bm in enumerate(bms):
        P.append(LK.bm_obj(ctx, "tibia_scales", bm, TINTS[k]))
    # hand: palm pad on the floor, four toes splayed around the outward direction, curved claws
    floor = Vector((l2, 0, 0)) - up * rf
    palm = floor + up * 0.0065 + out * 0.004
    P.append(sphere(ctx, "wrist", r2 * 0.85, wrist, "wyrm_skin", seg=14, rings=8))
    P.append(sphere(ctx, "palm", 1.0, palm, "wyrm_skin", scale=(0.0135, 0.0125, 0.0065), seg=18, rings=10,
                    rot=(0, -theta, 0)))
    for k, ang in enumerate((-0.95, -0.32, 0.32, 0.95)):
        d = (out * math.cos(ang) + side * math.sin(ang)).normalized()
        ln = 0.020 if abs(ang) > 0.5 else 0.025
        a = palm + d * 0.008 + up * 0.002
        b = palm + d * (0.008 + ln * 0.55) + up * 0.0045
        c = palm + d * (0.008 + ln) + up * 0.0018
        P.append(tube(ctx, "toe", a, b, 0.0042, "wyrm_skin", seg=8, r2=0.0034))
        P.append(tube(ctx, "toe2", b, c, 0.0034, "wyrm_skin", seg=8, r2=0.0027))
        P.append(sphere(ctx, "knuckle", 0.0036, b, "wyrm_scale2", seg=8, rings=6))
        P.append(LK.claw(ctx, "claw", c + up * 0.0008, d * 0.8 + up * 0.2, -up, 0.011, 0.0024, "wyrm_claw", curl=1.0,
                         seg=6, n=5))
    return join(ctx, P, "tibia")


def build(cfg, ctx):
    bz = cfg["standing_height"]
    lid = dict(cfg["lidar"], _bz=bz)
    cam = dict(cfg["camera"], _bz=bz)
    lk = cfg["link"]
    P, A = torso(ctx)
    P += wing(ctx, A, 1) + wing(ctx, A, -1)
    P += crystal(ctx, lid)
    P += head(ctx, cam)
    base = join(ctx, P, "base")
    rb = LK.robot("wyrm", {"wyrm": cfg})
    _, th = LK.stance_theta(rb, 0)
    parts_ = {"coxa": coxa(ctx, lk["coxa"], lk["coxa_radius"]),
              "femur": femur(ctx, lk["femur"], lk["femur_radius"]),
              "tibia": tibia(ctx, lk["tibia"], lk["tibia_radius"], lk["foot_radius"], th)}
    return base, parts_
