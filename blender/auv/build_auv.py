"""Build the knucker AUV (a hybrid torpedo/hover hull with a sea-dragon flavour) in headless Blender.

    blender -b --factory-startup -P blender/auv/build_auv.py -- [--no-render] [--hero AZ,EL,FIT] [--out PNG]

Writes src/fleet_auv/meshes/knucker/{hull.obj,prop.obj}(+mtl) (forward +X in this robot's own frame:
base_link x forward, y left, z up; OBJ exported with forward_axis=X so no conversion happens in
Gazebo/RViz) and a preview render blender/previews/robot_knucker*.png. Numbers come from
src/fleet_auv/config/knucker.yaml and fleet_auv/shape.py.
"""
import math
import os
import random
import sys
from pathlib import Path

import bmesh
import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "blender"))
sys.path.insert(0, str(ROOT / "src" / "fleet_auv"))
from robots import common as C  # noqa: E402
from robots import preview as PV  # noqa: E402
from fleet_auv import shape  # noqa: E402

OUT = ROOT / "src" / "fleet_auv" / "meshes" / "knucker"

C.PALETTE.update({
    "k_hull":   ("#150d26", 0.55, 0.20, None, 0, 1),
    "k_hull2":  ("#241240", 0.60, 0.22, None, 0, 1),
    "k_scale":  ("#3a1870", 0.80, 0.24, None, 0, 1),
    "k_scale2": ("#5f2fae", 0.85, 0.20, None, 0, 1),
    "k_scale3": ("#2a1050", 0.75, 0.28, None, 0, 1),
    "k_belly":  ("#241f35", 0.60, 0.30, None, 0, 1),
    "k_fin":    ("#2b0c4c", 0.15, 0.45, "#3a0a60", 0.12, 1),
    "k_spine":  ("#a99cc0", 0.90, 0.28, None, 0, 1),
    "k_duct":   ("#17131f", 0.55, 0.32, None, 0, 1),
    "k_brass":  ("#b8934a", 1.00, 0.28, None, 0, 1),
    "k_prop":   ("#b9b2c9", 0.90, 0.25, None, 0, 1),
    "k_led":    ("#c79bff", 0.0, 0.3, "#a050ff", 7.0, 1),
    "k_eye":    ("#ff90f0", 0.0, 0.2, "#ff30e0", 6.0, 1),
    "k_glass":  ("#05040c", 0.0, 0.04, "#101040", 0.2, 1),
    "k_black":  ("#050508", 0.2, 0.5, None, 0, 1),
})
C.FX.update({
    "k_hull": dict(bump=0.08, scale=900.0, var=0.12),
    "k_scale": dict(bump=0.12, scale=1500.0, var=0.2),
    "k_scale2": dict(bump=0.12, scale=1500.0, var=0.2),
    "k_fin": dict(bump=0.25, scale=500.0, var=0.3, sss=0.3),
})

CFG = shape.load_cfg()
H = CFG["hull"]
TH = CFG["thrusters"]
R0, L = H["radius"], H["length"]
XN = L / 2
rng = random.Random(4)


# ----------------------------------------------------------------------------- helpers
def hr(x):
    return shape.hull_r(x, H)


def M(loc=(0, 0, 0), rx=0, ry=0, rz=0):
    return Matrix.Translation(loc) @ Matrix.Rotation(rz, 4, "Z") @ Matrix.Rotation(ry, 4, "Y") @ Matrix.Rotation(rx, 4, "X")


def place(o, mtx):
    o.matrix_world = mtx @ o.matrix_world
    return o


def plate(ctx, name, pts, th, material, bev=0.002):
    """Flat extruded polygon in the xy plane (centred on z = 0), pts = [(x, y), ...] counter-clockwise."""
    bm = bmesh.new()
    top = [bm.verts.new((x, y, th / 2)) for x, y in pts]
    bot = [bm.verts.new((x, y, -th / 2)) for x, y in pts]
    bm.faces.new(top)
    bm.faces.new(list(reversed(bot)))
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((bot[i], bot[j], top[j], top[i]))
    o = C._obj_from_bm(name, bm, material, ctx=ctx)
    return C.bevel(o, bev, 1, 60)


def fin_outline(span, root, tip, sweep, scallop=4, depth=0.35):
    """Dragon-fin outline in a local frame: root edge along +x at y=0 (length `root`), tip at y=span.
    Leading edge straight, trailing edge scalloped between finger tips."""
    lead_tip = (sweep + tip * 0.2, span)
    pts = [(0.0, 0.0), (root, 0.0)]
    # trailing edge, from the root rear to the tip, scalloped
    n = scallop
    te_root = (root, 0.0)
    te_tip = (sweep + tip, span)
    prev = te_root
    for i in range(1, n + 1):
        t = i / n
        px = te_root[0] + (te_tip[0] - te_root[0]) * t
        py = te_root[1] + (te_tip[1] - te_root[1]) * t
        mx = (prev[0] + px) / 2 - depth * tip * 0.5
        my = (prev[1] + py) / 2 - 0.0
        pts.append((mx, my))
        pts.append((px, py))
        prev = (px, py)
    pts.append(lead_tip)
    return pts


def spines(ctx, name, base_pts, tip_pts, r0, material, seg=6):
    out = []
    for a, b in zip(base_pts, tip_pts):
        out.append(C.tube(ctx, name, a, b, r0, material, seg=seg, r2=r0 * 0.35))
    return out


# ----------------------------------------------------------------------------- hull
def hull(ctx):
    prof = []
    xs = sorted(set([XN - 1e-4 - 0.0008 * k ** 2 for k in range(0, 14)] +
                    [-XN + 1e-4 + 0.0008 * k ** 2 for k in range(0, 14)] +
                    [-XN + L * i / 70 for i in range(71)]))
    for x in xs:
        r = hr(x) * 0.985
        prof.append((max(r, 0.0), x))
    prof[0] = (0.0, prof[0][1])   # tail pole (the tail closes at a rounded tip)
    prof[-1] = (0.0, prof[-1][1])
    # tail: the real hull ends at r = tail_radius with a cap; fade to a pole over the last 15 mm
    prof.insert(1, (H["tail_radius"] * 0.9, -XN + 0.004))
    o = C.lathe(ctx, "hull", prof, (0, 0, 0), "k_hull", seg=64, axis="x")
    return o


def tunnels(ctx, hull_obj):
    parts = []
    for key in ("vfore", "vaft"):
        x = TH[key]["pos"][0]
        r_in, r_out, hh = 0.052, 0.058, 0.100  # T200 OD 100 mm + 2 mm clearance, 6 mm PETG wall (docs/notes/sea.md)
        cutter = C.cyl(ctx, "tcut", r_out, 0.4, (x, 0, 0), "k_black", seg=48)
        C.cut(hull_obj, cutter)
        # tube wall with flared lips, flush ring lights and a brass rim
        prof = [(r_in + 0.005, -hh), (r_out + 0.004, -hh), (r_out + 0.004, -hh + 0.010), (r_out, -hh + 0.016),
                (r_out, hh - 0.016), (r_out + 0.004, hh - 0.010), (r_out + 0.004, hh), (r_in + 0.005, hh),
                (r_in + 0.004, hh - 0.012), (r_in, hh - 0.025), (r_in, -hh + 0.025), (r_in + 0.004, -hh + 0.012)]
        t = C.lathe(ctx, "tube_" + key, prof, (x, 0, 0), "k_duct", seg=48, loop=True, bev=0.0015)
        parts.append(t)
        for s in (-1, 1):
            parts.append(C.torus(ctx, "rim", r_out + 0.004, 0.0035, (x, 0, s * (hh - 0.003)), "k_brass", major=48, minor=8))
            parts.append(C.torus(ctx, "tled", r_in + 0.005, 0.0025, (x, 0, s * (hh + 0.0005)), "k_led", major=48, minor=6))
        # guard grille on the top lip: ring + crossing ribs + lower guard
        for z in (hh - 0.012, -hh + 0.012):
            parts.append(C.torus(ctx, "gr", r_in - 0.002, 0.0022, (x, 0, z), "k_spine", major=40, minor=6))
            for k in range(4):
                a = k * math.pi / 4
                parts.append(C.box(ctx, "rib", (2 * r_in, 0.004, 0.004), (x, 0, z), "k_spine", bev=0.001, rot=(0, 0, a)))
        # shroud blending the tunnel into the hull: a pair of short brass-banded saddle bumps
        parts.append(C.sphere(ctx, "saddle", 0.085, (x, 0, 0), "k_hull2", scale=(0.95, 0.62, 0.0 + 1.0 * 0.001)) if False else None)
    return [p for p in parts if p]


def horizontal_ducts(ctx):
    parts = []
    for key in ("port", "stbd"):
        x, y, z = TH[key]["pos"]
        r_in, r_out, hl = 0.052, 0.058, 0.085
        prof = [(r_in + 0.005, -hl), (r_out + 0.003, -hl), (r_out + 0.003, -hl + 0.012), (r_out, -hl + 0.02),
                (r_out, hl - 0.02), (r_out + 0.003, hl - 0.012), (r_out + 0.003, hl), (r_in + 0.005, hl),
                (r_in + 0.004, hl - 0.012), (r_in, hl - 0.03), (r_in, -hl + 0.03), (r_in + 0.004, -hl + 0.012)]
        d = C.lathe(ctx, "duct_" + key, prof, (x, y, z), "k_duct", seg=48, loop=True, axis="x", bev=0.0015)
        parts.append(d)
        for s in (-1, 1):
            parts.append(C.torus(ctx, "drim", r_out + 0.003, 0.0035, (x + s * (hl - 0.003), y, z), "k_brass", axis="x", major=48, minor=8))
        parts.append(C.torus(ctx, "dled", r_out + 0.0005, 0.0028, (x + 0.02, y, z), "k_led", axis="x", major=48, minor=6))
        # rear guard: ring + three spokes + hub cone (stator)
        parts.append(C.torus(ctx, "dgr", r_in - 0.002, 0.0022, (x - hl + 0.012, y, z), "k_spine", axis="x", major=40, minor=6))
        for k in range(3):
            a = k * 2 * math.pi / 3 + math.pi / 6
            parts.append(C.box(ctx, "spoke", (0.004, 2 * r_in, 0.004), (x - hl + 0.012, y, z), "k_spine", bev=0.001, rot=(a, 0, 0)))
        parts.append(C.cyl(ctx, "stator", 0.014, 0.05, (x - hl + 0.025, y, z), "k_duct", axis="x", r2=0.004, seg=16))
        # pylon: swept strut from the hull to the duct
        s = 1 if y > 0 else -1
        hx = x + 0.02
        r_hull = hr(hx)
        pts = [(x + 0.07, 0.0), (x - 0.06, 0.0), (x - 0.075, 0.0), (x - 0.045, 0.0)]
        strut = plate(ctx, "pylon", [(0.075, -0.012), (-0.075, -0.012), (-0.085, 0.012), (0.04, 0.012)], 0.012, "k_hull2", bev=0.003)
        # strut lies in the xy plane at z = 0, spanning hull side to duct: stretch along y by scaling after rotation
        strut.rotation_euler = (math.pi / 2, 0, 0)
        strut.scale = (1.0, 1.0, 1.0)
        strut.location = (x, s * (r_hull * 0.6 + (abs(y) - r_in - 0.002) * 0.5), z)
        # make the strut span: use a thin box instead (plate with width = gap)
        gap = abs(y) - r_out - r_hull * 0.7
        bpy.data.objects.remove(strut)
        strut = C.box(ctx, "pylon", (0.17, gap + 0.02, 0.016), (x + 0.0, s * (r_hull * 0.7 + gap / 2 + 0.002), z), "k_hull2", bev=0.004)
        parts.append(strut)
    return parts


# ----------------------------------------------------------------------------- scales
def scales(ctx):
    parts = []
    tun = [TH["vfore"]["pos"][0], TH["vaft"]["pos"][0]]
    x = XN - 0.075
    row = 0
    while x > -XN + 0.10:
        r = hr(x)
        dx = 0.002
        slope = (hr(x + dx) - hr(x - dx)) / (2 * dx)
        n = max(8, int(round(2 * math.pi * r / 0.052)))
        sl = 0.056 if r > 0.06 else 0.045
        for i in range(n):
            th = 2 * math.pi * (i + 0.5 * (row % 2)) / n
            # keep the camera window, the eye lights, tunnels and the side light strip clear
            y, z = r * math.cos(th), r * math.sin(th)
            if any(abs(x - tx) < 0.075 and abs(y) < 0.075 for tx in tun) and abs(z) > 0.03:
                continue
            if abs(z) < 0.0 and False:
                continue
            nrm = Vector((-slope, math.cos(th), math.sin(th))).normalized()
            tx_ = Vector((1.0, slope * math.cos(th), slope * math.sin(th))).normalized()
            ty_ = nrm.cross(tx_).normalized()
            tx_ = ty_.cross(nrm).normalized()
            rot = Matrix(((tx_.x, ty_.x, nrm.x), (tx_.y, ty_.y, nrm.y), (tx_.z, ty_.z, nrm.z))).to_4x4()
            wid = 2 * math.pi * r / n * 0.55
            m = Matrix.Translation((x, y * 0.99, z * 0.99)) @ rot
            if z > 0.02:
                mat = "k_scale2" if (i + row) % 5 == 0 else "k_scale"
            elif z < -0.045:
                mat = "k_belly"
            else:
                mat = "k_scale3" if (i + row) % 4 else "k_scale"
            o = C.sphere(ctx, "scale", 1.0, (0, 0, 0), mat, scale=(sl / 2, wid, 0.0042), seg=8, rings=4)
            o.matrix_world = m
            parts.append(o)
        x -= 0.047 if r > 0.07 else 0.040
        row += 1
    return parts


def bands(ctx):
    parts = []
    for x, w in ((0.265, 0.010), (-0.205, 0.010), (0.50, 0.006), (-0.42, 0.006)):
        parts.append(C.torus(ctx, "band", hr(x) * 0.99 + 0.003, w * 0.5, (x, 0, 0), "k_brass", axis="x", major=64, minor=8))
    return parts


# ----------------------------------------------------------------------------- dragon details
def head_details(ctx):
    parts = []
    # camera window at the nose, bezel + glass
    xn = XN - 0.012
    parts.append(C.sphere(ctx, "lens", 0.026, (xn, 0, 0.012), "k_glass", scale=(0.55, 1, 1), seg=24, rings=12))
    parts.append(C.torus(ctx, "bezel", 0.027, 0.0035, (xn - 0.006, 0, 0.012), "k_brass", axis="x", major=40, minor=8))
    for s in (-1, 1):
        # glowing eye in a swept brow socket
        ex, ey, ez = 0.50, s * 0.073, 0.034
        brow = C.sphere(ctx, "brow", 1.0, (0, 0, 0), "k_hull2", scale=(0.050, 0.020, 0.012), seg=12, rings=6)
        brow.matrix_world = M((ex - 0.002, ey * 1.02, ez + 0.017), 0.0, 0.0, -s * 0.25) @ M(rx=s * 0.55)
        parts.append(brow)
        eye = C.sphere(ctx, "eye", 1.0, (0, 0, 0), "k_eye", scale=(0.020, 0.010, 0.012), seg=16, rings=8)
        eye.matrix_world = M((ex + 0.004, ey, ez), 0, 0, -s * 0.15) @ M(rx=s * 0.45)
        parts.append(eye)
        pup = C.sphere(ctx, "pupil", 1.0, (0, 0, 0), "k_black", scale=(0.0045, 0.0035, 0.011), seg=8, rings=5)
        pup.matrix_world = M((ex + 0.008, ey * 1.045, ez - 0.001), 0, 0, -s * 0.15) @ M(rx=s * 0.45)
        parts.append(pup)
        # swept-back horn (a tapered, three-segment tube): doubles as an acoustic-modem antenna
        p = [(0.455, s * 0.040, 0.081), (0.405, s * 0.048, 0.116), (0.345, s * 0.056, 0.134), (0.290, s * 0.062, 0.130)]
        for k in range(3):
            r0 = 0.0125 * (1 - k / 3.4)
            r1 = 0.0125 * (1 - (k + 1) / 3.4)
            parts.append(C.tube(ctx, "horn", p[k], p[k + 1], r0, "k_spine", seg=10, r2=r1))
        # cheek plates (jaw line)
        for j in range(3):
            x = 0.58 - 0.07 * j
            ch = C.sphere(ctx, "cheek", 1.0, (0, 0, 0), "k_scale2", scale=(0.032, 0.008, 0.013), seg=8, rings=4)
            ch.matrix_world = M((x, s * (hr(x) - 0.006), -0.032 - 0.004 * j), 0, 0, -s * 0.0) @ M(rx=-s * 0.9)
            parts.append(ch)
    # chin: forward sonar plate under the nose
    parts.append(C.box(ctx, "sonar", (0.07, 0.07, 0.014), (0.575, 0, -0.058), "k_black", bev=0.004, rot=(0, 0.22, 0)))
    parts.append(C.box(ctx, "sonarf", (0.078, 0.078, 0.006), (0.575, 0, -0.056), "k_brass", bev=0.002, rot=(0, 0.22, 0)))
    return parts


def dorsal(ctx):
    parts = []
    # dorsal sail fin with spiny rays and a scalloped membrane
    base_x = [-0.04, -0.12, -0.20, -0.28, -0.36]
    tops = [0.115, 0.165, 0.150, 0.115, 0.075]
    pts = [(base_x[0] + 0.08, hr(0.0) * 0.95)]
    for bx, t in zip(base_x, tops):
        pts.append((bx - t * 0.55, hr(bx) * 0.9 + t))
        pts.append((bx - t * 0.55 - 0.03, hr(bx) * 0.9 + t * 0.55))
    pts.append((base_x[-1] - 0.10, hr(-0.46) * 0.9))
    pts = [(p[0], p[1]) for p in pts]
    mem = plate(ctx, "dfin", pts, 0.005, "k_fin", bev=0.001)
    mem.matrix_world = M(rx=math.pi / 2) @ mem.matrix_world
    mem.matrix_world = Matrix.Translation((0.0, 0.0, 0.0)) @ mem.matrix_world
    parts.append(mem)
    for bx, t in zip(base_x, tops):
        a = (bx, 0, hr(bx) * 0.9)
        b = (bx - t * 0.55, 0, hr(bx) * 0.9 + t + 0.004)
        parts.append(C.tube(ctx, "dray", a, b, 0.0045, "k_spine", seg=6, r2=0.0015))
        parts.append(C.sphere(ctx, "dtip", 0.006, b, "k_led", seg=8, rings=4)) if t > 0.12 else None
    # ridge scutes down the back, forward of the sail and aft of it
    for i in range(12):
        x = 0.42 - 0.052 * i if i < 6 else -0.44 - 0.0 * i
        if i >= 6:
            x = -0.44 + 0.0
            continue
        s = 0.020 - 0.002 * i
        o = C.cyl(ctx, "scute", s * 0.8, s * 1.6, (x, 0, hr(x) * 0.99 + s * 0.35), "k_scale2", r2=0.002, seg=8, bev=0.0)
        o.scale = (1.4, 0.45, 1.0)
        o.rotation_euler = (0, -0.35, 0)
        parts.append(o)
    # lift mast: strobe + loop
    parts.append(C.cyl(ctx, "mast", 0.006, 0.06, (0.06, 0, hr(0.06) + 0.028), "k_spine", seg=10))
    parts.append(C.sphere(ctx, "strobe", 0.011, (0.06, 0, hr(0.06) + 0.062), "k_led", seg=12, rings=6))
    parts.append(C.torus(ctx, "lift", 0.022, 0.0035, (0.14, 0, hr(0.14) + 0.012), "k_brass", axis="y", major=24, minor=6))
    return parts


def pectoral(ctx):
    parts = []
    for s in (-1, 1):
        root_x = 0.205
        r = hr(root_x)
        out = fin_outline(0.260, 0.150, 0.090, 0.150, scallop=4, depth=0.42)
        fin = plate(ctx, "pfin", out, 0.005, "k_fin", bev=0.001)
        # fin local: root along +x, span along +y -> rotate so the span is +/-y with the root edge on the hull
        mtx = M((root_x - 0.105, s * (r * 0.92), -0.008), rx=-s * 0.14) @ M(rz=0.0)
        if s < 0:
            fin.scale = (1, -1, 1)
            bpy.context.view_layer.update()
            bm = bmesh.new()
            bm.from_mesh(fin.data)
            bmesh.ops.reverse_faces(bm, faces=bm.faces)
            bm.to_mesh(fin.data)
            bm.free()
        fin.matrix_world = mtx @ fin.matrix_world
        parts.append(fin)
        # finger spines
        rays = [(0.015 + 0.027 * k, 0.0) for k in range(4)]
        tips = [(0.115 + 0.012 + 0.030 * k * 0.0 + 0.040 - 0.012 * k, 0.200 - 0.018 * k) for k in range(4)]
        for (bx, by), (tx, ty) in zip(rays, tips):
            a = mtx @ Vector((bx, s * 0.0 + by * 0, 0.0)) if False else None
            pa = mtx @ Vector((bx, 0.0, 0.0)) if s > 0 else mtx @ Vector((bx, 0.0, 0.0))
            pb = mtx @ Vector((tx, ty * s, 0.0))
            parts.append(C.tube(ctx, "pray", pa, pb, 0.0042, "k_spine", seg=6, r2=0.0014))
        tip = mtx @ Vector((0.115 + 0.040, 0.200 * s, 0.0))
        parts.append(C.sphere(ctx, "ptip", 0.0065, tip, "k_led", seg=8, rings=4))
    return parts


def lights_strip(ctx):
    parts = []
    for s in (-1, 1):
        for i in range(9):
            x = 0.22 - 0.06 * i
            if abs(x - TH["vfore"]["pos"][0]) < 0.0 or x < -0.40:
                continue
            o = C.sphere(ctx, "dash", 1.0, (0, 0, 0), "k_led", scale=(0.017, 0.0035, 0.0035), seg=8, rings=4)
            o.location = (x, s * (hr(x) + 0.0005), 0.0)
            parts.append(o)
    # sensor details: DVL transducer pad (looks down), depth-sensor port, battery access hatch
    dv = TH_S = CFG["sensors"]["dvl"]["pos"]
    parts.append(C.box(ctx, "dvl", (0.085, 0.085, 0.014), (dv[0], dv[1], dv[2] + 0.006), "k_black", bev=0.004))
    for a in (0, 1, 2, 3):
        sx, sy = (-1) ** a * 0.02, (-1) ** (a // 2) * 0.02
        parts.append(C.cyl(ctx, "dvlb", 0.0125, 0.004, (dv[0] + sx, dv[1] + sy, dv[2] - 0.002), "k_brass", seg=20))
    parts.append(C.cyl(ctx, "depthport", 0.012, 0.005, (0.20, 0, hr(0.20) + 0.001), "k_steel" if "k_steel" in C.PALETTE else "steel", seg=20))
    parts.append(C.box(ctx, "hatch", (0.10, 0.10, 0.004), (-0.10, 0, hr(-0.10) + 0.0), "k_hull2", bev=0.002)) if False else None
    return parts


# ----------------------------------------------------------------------------- propeller
def prop(ctx, blades=3, r=0.038):   # T200 propeller diameter 76 mm
    parts = []
    parts.append(C.cyl(ctx, "hub", 0.011, 0.022, (0, 0, 0), "k_prop", seg=20, r2=0.008))
    parts.append(C.cyl(ctx, "cone", 0.011, 0.016, (0, 0, 0.018), "k_prop", seg=20, r2=0.002))
    for k in range(blades):
        a = 2 * math.pi * k / blades
        # blade: flattened ellipsoid with pitch, roots on the hub
        b = C.sphere(ctx, "blade", 1.0, (0, 0, 0), "k_prop", scale=(0.0145, r * 0.55, 0.0016), seg=14, rings=6)
        b.matrix_world = M(rz=a) @ M((0, r * 0.5 + 0.004, 0.0)) @ M(ry=0.0) @ Matrix.Rotation(0.55, 4, "Y") @ b.matrix_world
        parts.append(b)
    o = C.join(ctx, parts, "prop")
    return o


# ----------------------------------------------------------------------------- build / preview
def build(render=True, hero=(38, 14, 1.5), out_png=None):
    C.reset_scene()
    C.reset_mat_cache()
    ctx = C.Ctx("knucker")
    h = hull(ctx)
    parts = [h]
    tun = tunnels(ctx, h)
    parts += tun
    parts += horizontal_ducts(ctx)
    parts += scales(ctx)
    parts += bands(ctx)
    parts += head_details(ctx)
    parts += dorsal(ctx)
    parts += pectoral(ctx)
    parts += lights_strip(ctx)
    body = C.join(ctx, parts, "knucker_hull", smooth_deg=40)
    pr = prop(ctx)
    pr.location = (0.0, 0.0, -2.0)           # keep it out of the hull preview, exported at its own origin
    # exports (base_link frame = origin of the hull, prop mesh around its own origin, axis +z)
    pr.location = (0, 0, 0)
    C.export_obj(body, OUT / "hull.obj")
    C.export_obj(pr, OUT / "prop.obj")
    print("[knucker] hull tris", C.tri_count(body), "prop tris", C.tri_count(pr), flush=True)
    import mathutils
    pts = [body.matrix_world @ mathutils.Vector(c) for c in body.bound_box]
    lo = [min(p[k] for p in pts) for k in range(3)]
    hi = [max(p[k] for p in pts) for k in range(3)]
    print("[knucker] bbox x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f" % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]), flush=True)
    # preview: props on their thrusters, raised above the studio floor
    for key, th in TH.items():
        d = th["dir"]
        p = bpy.data.objects.new("prop_" + key, pr.data)
        ctx.coll.objects.link(p)
        ry = math.pi / 2 if d[0] else 0.0
        p.matrix_world = M(tuple(th["pos"]), 0, ry, 0)
    bpy.data.objects.remove(pr)
    for o in ctx.coll.objects:
        o.location.z += 0.42
    bpy.context.view_layer.update()
    if render:
        base = ROOT / "blender" / "previews"
        base.mkdir(parents=True, exist_ok=True)
        az, el, fit = hero
        PV.hero((0.0, 0.0, 0.42), 1.3, out_png or base / "robot_knucker.png", az=az, el=el, fit=fit)
    (ROOT / "blender" / "blend").mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(ROOT / "blender" / "blend" / "robot_knucker.blend"))


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    hero, outp, render = (38, 14, 1.5), None, "--no-render" not in argv
    if "--hero" in argv:
        hero = tuple(float(v) for v in argv[argv.index("--hero") + 1].split(","))
    if "--out" in argv:
        outp = argv[argv.index("--out") + 1]
    build(render, hero, outp)
