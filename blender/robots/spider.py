"""spider: octopod. Glossy dark chitin carapace with an eye cluster (the camera is the central eye),
chelicerae and palps, a separate abdomen with glowing violet markings, and the lidar on a short
segmented mast above knee height. Long legs with knees above the body (knee-up yaw legs)."""
import math
import random

import bmesh
from mathutils import Vector

from . import legkit as LK
from . import parts
from .common import cyl, join, sphere, torus, tube


def carapace(ctx):
    P = []
    C = LK.Loft([(-0.070, 0, 0.002), (-0.045, 0, 0.004), (0.0, 0, 0.006), (0.045, 0, 0.006), (0.075, 0, 0.004),
                 (0.092, 0, 0.000)],
                ry=[0.030, 0.052, 0.064, 0.062, 0.050, 0.030], rz=[0.016, 0.026, 0.031, 0.030, 0.025, 0.016],
                sq=[2.3] * 6)
    P.append(C.mesh(ctx, "ceph", "carapace", nu=28, na=40, cap_len=(0.012, 0.010)))
    # dorsal shield with a darker rim, central groove (fovea) and radial grooves
    P.append(C.patch(ctx, "shield", "carapace_hi", 0.08, 0.90, 0.55, math.pi - 0.55, 0.0016, 0.003, nu=20, na=14))
    P.append(sphere(ctx, "fovea", 1.0, (-0.012, 0, 0.040), "carapace", scale=(0.010, 0.004, 0.003), seg=12, rings=6))
    for k in range(-3, 4):
        if k == 0:
            continue
        a = k * 0.42
        p0 = Vector((-0.012 + 0.012 * math.cos(a), 0.012 * math.sin(a), 0.039))
        p1 = Vector((-0.012 + 0.042 * math.cos(a), 0.046 * math.sin(a), 0.031 - 0.004 * abs(math.sin(a))))
        P.append(tube(ctx, "groove", p0, p1, 0.0018, "carapace", seg=6, r2=0.001))
    # sternum below
    P.append(sphere(ctx, "sternum", 1.0, (0.005, 0, -0.022), "chitin", scale=(0.055, 0.042, 0.010), seg=24, rings=10))
    return P


def eyes(ctx, cam):
    P = []
    cz = cam["z"] - cam["_bz"]
    cx = cam["x"]
    # eye mound
    P.append(sphere(ctx, "mound", 1.0, (0.074, 0, cz + 0.006), "carapace", scale=(0.022, 0.030, 0.016), seg=24, rings=12))
    # central eye = camera lens
    P.append(sphere(ctx, "eye_c", 0.0105, (cx + 0.003, 0, cz), "spider_eye", seg=24, rings=14))
    P.append(torus(ctx, "eye_c_ring", 0.0108, 0.0012, (cx + 0.004, 0, cz), "darksteel", major=24, minor=6, axis="x"))
    spec = [  # (x, y, z, r)
        (0.084, 0.0175, 0.003, 0.0078), (0.081, 0.0105, 0.015, 0.0052),
        (0.072, 0.030, 0.008, 0.0055), (0.062, 0.034, 0.014, 0.0046),
    ]
    for s in (1, -1):
        for x, y, z, r in spec:
            P.append(sphere(ctx, "eye", r, (x, s * y, cz + z), "spider_eye", seg=16, rings=10))
            P.append(torus(ctx, "eye_rim", r * 1.02, 0.0009, (x + 0.001, s * y, cz + z), "carapace_hi", major=16, minor=4,
                           axis="x"))
    return P


def mouth(ctx):
    P = []
    for s in (1, -1):
        # chelicerae: bulbous bases, curved fangs
        P.append(sphere(ctx, "chel", 1.0, (0.088, s * 0.012, -0.010), "carapace_hi", scale=(0.012, 0.0095, 0.016), seg=16,
                        rings=10, rot=(0, 0.35, 0)))
        P.append(LK.claw(ctx, "fang", (0.093, s * 0.012, -0.024), (0.25, -s * 0.25, -1.0), (-1, 0, 0), 0.014, 0.0032,
                         "wyrm_claw", curl=1.1))
        # palps: short three-segment feelers
        a = Vector((0.084, s * 0.026, -0.006))
        b = Vector((0.098, s * 0.036, -0.004))
        c = Vector((0.103, s * 0.034, -0.022))
        P.append(tube(ctx, "palp", a, b, 0.0042, "chitin", seg=10, r2=0.0038))
        P.append(sphere(ctx, "palpj", 0.0042, b, "carapace_hi", seg=10, rings=6))
        P.append(tube(ctx, "palp2", b, c, 0.0038, "chitin", seg=10, r2=0.0030))
        P.append(sphere(ctx, "palp_tip", 0.0034, c, "carapace_hi", seg=10, rings=6))
    return P


def abdomen(ctx):
    P = []
    A = LK.Loft([(-0.050, 0, 0.012), (-0.070, 0, 0.022), (-0.110, 0, 0.030), (-0.150, 0, 0.028), (-0.172, 0, 0.020)],
                ry=[0.020, 0.044, 0.056, 0.048, 0.022], rz=[0.016, 0.040, 0.050, 0.042, 0.020], sq=[2.0] * 5)
    P.append(A.mesh(ctx, "abdomen", "carapace", nu=28, na=36, cap_len=(0.010, 0.010)))
    P.append(tube(ctx, "pedicel", (-0.066, 0, 0.004), (-0.046, 0, 0.012), 0.012, "chitin", seg=14, r2=0.011))
    # dorsal chevrons + hourglass (glowing markings) and fine ridge lines
    for k in range(4):
        u = 0.30 + 0.15 * k
        w = 0.42 - 0.07 * k
        for s in (1, -1):
            a_c = math.pi / 2 + s * (0.18 + w * 0.5)
            P.append(A.patch(ctx, "chevron", "spider_mark", u, u + 0.045, a_c - w * 0.5, a_c + w * 0.5, 0.0012, 0.0016,
                             nu=2, na=4, chamfer=0.3))
    P.append(A.patch(ctx, "spot", "spider_mark", 0.22, 0.27, math.pi / 2 - 0.12, math.pi / 2 + 0.12, 0.0012, 0.0016,
                     nu=2, na=3, chamfer=0.3))
    P.append(A.patch(ctx, "hourglass", "spider_mark", 0.42, 0.62, 3 * math.pi / 2 - 0.18, 3 * math.pi / 2 + 0.18, 0.0012,
                     0.0016, nu=4, na=3, chamfer=0.3))
    for s in (1, -1):
        P.append(A.patch(ctx, "flank", "carapace_hi", 0.12, 0.88, s * 0.25 + (0 if s > 0 else math.pi) - 0.32,
                         s * 0.25 + (0 if s > 0 else math.pi) + 0.32, 0.0010, 0.0018, nu=14, na=4))
    # spinnerets
    for s in (1, -1):
        P.append(sphere(ctx, "spinneret", 1.0, (-0.178, s * 0.005, 0.012), "chitin", scale=(0.007, 0.0035, 0.0035),
                        seg=10, rings=6, rot=(0, 0.4, s * 0.25)))
    return P


def mast(ctx, lid):
    P = []
    lz = lid["z"] - lid["_bz"]
    x = lid["x"]
    z0 = 0.030
    P.append(cyl(ctx, "mast_socket", 0.016, 0.010, (x, 0, z0 + 0.004), "carapace_hi", seg=24, r2=0.012, bev=0.0015))
    top = lz - 0.024
    P.append(cyl(ctx, "mast", 0.0065, top - z0, (x, 0, (top + z0) / 2), "darksteel", seg=16))
    n = 4
    for k in range(1, n + 1):
        z = z0 + (top - z0) * k / (n + 1)
        P.append(torus(ctx, "mast_ring", 0.0075, 0.0022, (x, 0, z), "chitin", major=16, minor=6))
    P.append(torus(ctx, "mast_led", 0.0078, 0.0014, (x, 0, z0 + (top - z0) * 0.5 + 0.012), "spider_mark", major=16, minor=6))
    P.append(cyl(ctx, "mast_head", 0.010, 0.006, (x, 0, top + 0.001), "carapace_hi", seg=20, r2=0.020, bev=0.001))
    P += parts.lidar_puck(ctx, (x, lid["y"], lz), r=0.032, h=0.040, accent="spider_mark")
    return P


# ----------------------------------------------------------------------------- legs
def spines_bm(bm, loft, us, n_per, length, r, seed, a_center=math.pi / 2, spread=1.2):
    rnd = random.Random(seed)
    for u in us:
        for _ in range(n_per):
            a = a_center + rnd.uniform(-spread, spread)
            p = loft.S(u, a)
            nn = loft.N(u, a)
            c, t, ey, ez = loft.frame(u)
            d = (nn * 0.55 + t * 0.85).normalized()
            q = p + d * length * rnd.uniform(0.7, 1.15)
            D = q - p
            rot = Vector((0, 0, 1)).rotation_difference(D.normalized()).to_matrix().to_4x4()
            rot.translation = (p + q) / 2
            bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=True, segments=4, radius1=r, radius2=r * 0.12,
                                  depth=D.length, matrix=rot)


def coxa(ctx, lc, rc):
    P = []
    P.append(sphere(ctx, "socket", rc * 1.05, (0, 0, 0), "carapace_hi", seg=20, rings=12))
    L = LK.Loft([(0.0, 0, 0), (lc * 0.5, 0, 0.001), (lc, 0, 0)], ry=[rc * 0.95, rc * 0.9, rc * 0.78],
                rz=[rc * 1.0, rc * 0.95, rc * 0.8])
    P.append(L.mesh(ctx, "coxa_seg", "chitin", nu=6, na=18, caps=(False, False)))
    P.append(cyl(ctx, "knuckle", rc * 0.8, rc * 1.5, (lc, 0, 0), "carapace_hi", axis="y", seg=18, bev=0.001))
    P.append(torus(ctx, "coxa_ring", rc * 0.9, 0.0012, (lc * 0.55, 0, 0.0005), "spider_mark", major=18, minor=4, axis="x"))
    return join(ctx, P, "coxa")


def femur(ctx, l1, r1):
    P = []
    L = LK.Loft([(0.0, 0, 0), (0.03, 0, 0.002), (0.07, 0, 0.0025), (l1 - 0.012, 0, 0.001), (l1, 0, 0)],
                ry=[r1 * 0.75, r1 * 0.85, r1 * 0.80, r1 * 0.68, r1 * 0.62],
                rz=[r1 * 0.85, r1 * 1.0, r1 * 0.95, r1 * 0.80, r1 * 0.70], sq=[2.2] * 5)
    P.append(L.mesh(ctx, "femur_seg", "chitin", nu=18, na=18, caps=(True, False), cap_len=(0.005, None)))
    P.append(L.patch(ctx, "femur_plate", "carapace_hi", 0.06, 0.88, 0.75, math.pi - 0.75, 0.0008, 0.0016, nu=10, na=4))
    P.append(torus(ctx, "femur_band", r1 * 0.88, 0.0014, (0.072, 0, 0.0025), "carapace", major=18, minor=4, axis="x"))
    bm = bmesh.new()
    spines_bm(bm, L, [0.18, 0.32, 0.46, 0.60, 0.74, 0.86], 2, 0.010, 0.0011, 3)
    P.append(LK.bm_obj(ctx, "spines", bm, "carapace"))
    # knee: knuckle + patella cap + violet joint light
    P.append(sphere(ctx, "knee", r1 * 0.78, (l1, 0, 0), "carapace_hi", seg=18, rings=10))
    P.append(cyl(ctx, "knee_pin", r1 * 0.55, r1 * 1.75, (l1, 0, 0), "darksteel", axis="y", seg=14))
    P.append(torus(ctx, "knee_led", r1 * 0.72, 0.0013, (l1 - 0.008, 0, 0), "spider_mark", major=18, minor=4, axis="x"))
    return join(ctx, P, "femur")


def tibia(ctx, l2, r2, rf, theta):
    """Long tibia+tarsus to the foot at x = l2. theta = stance pitch of the link (down > 0), so the tip
    pad meets the floor."""
    P = []
    L = LK.Loft([(0.0, 0, 0), (0.05, 0, 0.003), (0.11, 0, 0.004), (0.15, 0, 0.002), (l2 - 0.006, 0, 0)],
                ry=[r2 * 0.95, r2 * 0.90, r2 * 0.72, r2 * 0.55, r2 * 0.42],
                rz=[r2 * 1.05, r2 * 1.0, r2 * 0.80, r2 * 0.62, r2 * 0.46], sq=[2.1] * 5)
    P.append(L.mesh(ctx, "tibia_seg", "chitin", nu=22, na=16, caps=(True, False), cap_len=(0.004, None)))
    P.append(torus(ctx, "metatarsus_joint", r2 * 0.78, 0.0013, (0.112, 0, 0.004), "carapace_hi", major=16, minor=4, axis="x"))
    P.append(torus(ctx, "tarsus_joint", r2 * 0.58, 0.0011, (0.152, 0, 0.002), "carapace_hi", major=16, minor=4, axis="x"))
    bm = bmesh.new()
    spines_bm(bm, L, [0.10, 0.22, 0.34, 0.46, 0.58, 0.70], 2, 0.009, 0.0009, 11)
    P.append(LK.bm_obj(ctx, "spines", bm, "carapace"))
    # tip: a dark pad on the floor-facing side of the foot sphere and a small claw
    up = Vector((-math.sin(theta), 0, math.cos(theta)))
    floor = Vector((l2, 0, 0)) - up * (rf * 0.62)
    P.append(tube(ctx, "tip", (l2 - 0.012, 0, 0), floor, r2 * 0.42, "chitin", seg=12, r2=r2 * 0.30))
    P.append(sphere(ctx, "pad", rf * 0.42, floor, "rubber", seg=14, rings=8))
    P.append(LK.claw(ctx, "claw", floor + up * 0.002, Vector((1, 0, 0)) * 0.7 - up * 0.3, -up, 0.007, 0.0016, "wyrm_claw",
                     curl=0.9))
    return join(ctx, P, "tibia")


def build(cfg, ctx):
    bz = cfg["standing_height"]
    lid = dict(cfg["lidar"], _bz=bz)
    cam = dict(cfg["camera"], _bz=bz)
    lk = cfg["link"]
    P = carapace(ctx) + eyes(ctx, cam) + mouth(ctx) + abdomen(ctx) + mast(ctx, lid)
    base = join(ctx, P, "base")
    rb = LK.robot("spider", {"spider": cfg})
    _, th = LK.stance_theta(rb, 1)
    parts_ = {"coxa": coxa(ctx, lk["coxa"], lk["coxa_radius"]),
              "femur": femur(ctx, lk["femur"], lk["femur_radius"]),
              "tibia": tibia(ctx, lk["tibia"], lk["tibia_radius"], lk["foot_radius"], th)}
    return base, parts_
