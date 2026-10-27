"""Dragon statue for the keep great hall: skin-modifier skeleton + subdivision + noise weathering,
decimated, plus half-spread membrane wings. Built inside Blender (bpy) and injected into the
WorldBuilder as shared-vertex smooth meshes (visual only, the plinth carries the collision).

Local frame: origin = centre of the plinth top, +X = facing direction, +Y left, +Z up.
"""
import math

import bpy
import bmesh
from mathutils import Vector

TARGET_BODY_TRIS = 34000


def V(x, y, z):
    return Vector((x, y, z))


class Skel:
    def __init__(self):
        self.v, self.r, self.e = [], [], []

    def add(self, p, r, parent=None):
        self.v.append(Vector(p))
        self.r.append(r)
        i = len(self.v) - 1
        if parent is not None:
            self.e.append((parent, i))
        return i

    def chain(self, parent, pts):
        """pts = [(pos, radius), ...] hanging off `parent`; returns the last index."""
        cur = parent
        for p, r in pts:
            cur = self.add(p, r, cur)
        return cur


def skeleton():
    S = Skel()
    # ---- spine: tail base -> hips -> belly -> chest -> neck -> head
    hips = S.add(V(-0.62, 0, 0.50), 0.27)
    belly = S.add(V(-0.18, 0, 0.42), 0.29, hips)
    chest = S.add(V(0.30, 0, 0.60), 0.31, belly)
    n0 = S.add(V(0.62, 0, 0.95), 0.235, chest)
    n1 = S.add(V(0.80, 0, 1.33), 0.185, n0)
    n2 = S.add(V(0.76, 0, 1.68), 0.16, n1)
    skull = S.add(V(0.95, 0, 1.98), 0.185, n2)
    snout = S.add(V(1.32, 0, 1.92), 0.105, skull)
    nose = S.add(V(1.62, 0, 1.88), 0.075, snout)
    # lower jaw
    j0 = S.add(V(1.05, 0, 1.78), 0.07, skull)
    j1 = S.add(V(1.36, 0, 1.72), 0.055, j0)
    S.add(V(1.58, 0, 1.74), 0.04, j1)
    # horns, brow ridges, cheek frills
    for s in (-1, 1):
        S.chain(skull, [(V(0.88, s * 0.12, 2.12), 0.06), (V(0.66, s * 0.20, 2.38), 0.045),
                        (V(0.40, s * 0.26, 2.58), 0.025), (V(0.22, s * 0.28, 2.62), 0.006)])
        S.chain(skull, [(V(1.12, s * 0.13, 2.13), 0.06), (V(1.34, s * 0.10, 2.06), 0.04)])
        S.chain(skull, [(V(0.86, s * 0.20, 1.90), 0.05), (V(0.66, s * 0.34, 1.86), 0.03), (V(0.56, s * 0.40, 1.84), 0.008)])
        S.chain(snout, [(V(1.55, s * 0.06, 2.0), 0.03)])
    # dorsal spikes up the neck and back
    for (p, d, L, r) in [(n1, (-0.5, 0, 0.85), 0.20, 0.035), (n2, (-0.6, 0, 0.8), 0.18, 0.032),
                         (n0, (-0.5, 0, 0.85), 0.2, 0.04), (chest, (-0.3, 0, 1.0), 0.2, 0.045),
                         (belly, (-0.2, 0, 1.0), 0.18, 0.045), (hips, (-0.1, 0, 1.0), 0.16, 0.04)]:
        base = S.v[p] + V(0, 0, S.r[p] * 0.8)
        d = Vector(d).normalized()
        a = S.add(base, r, p)
        S.add(base + d * L, r * 0.12, a)
    # ---- tail: back along the plinth, over the edge, curling down its face (tip stays above z=0 local -> 1.0 world)
    tail = [(V(-1.05, 0.03, 0.44), 0.21), (V(-1.45, 0.10, 0.40), 0.16), (V(-1.72, 0.20, 0.30), 0.115),
            (V(-1.78, 0.31, 0.06), 0.085), (V(-1.68, 0.42, -0.17), 0.06), (V(-1.50, 0.46, -0.27), 0.04),
            (V(-1.36, 0.42, -0.24), 0.02), (V(-1.30, 0.38, -0.17), 0.006)]
    prev = hips
    ti = []
    for p, r in tail:
        prev = S.add(p, r, prev)
        ti.append(prev)
    for k in (1, 2, 3, 4):  # tail spikes
        p = S.v[ti[k]]
        a = S.add(p + V(0, 0, S.r[ti[k]] * 0.8), 0.028, ti[k])
        S.add(p + V(-0.05, 0, S.r[ti[k]] * 0.8 + 0.14 - 0.02 * k), 0.004, a)
    # ---- legs
    for s in (-1, 1):
        sh = S.add(V(0.38, s * 0.26, 0.62), 0.17, chest)
        el = S.add(V(0.22, s * 0.40, 0.30), 0.10, sh)
        wr = S.add(V(0.55, s * 0.40, 0.11), 0.07, el)
        S.add(V(0.66, s * 0.40, 0.05), 0.075, wr)
        for k in (-1, 0, 1):
            t = S.add(V(0.78, s * (0.40 + 0.065 * k), 0.035), 0.03, wr)
            S.add(V(0.89, s * (0.40 + 0.08 * k), 0.015), 0.007, t)
        hp = S.add(V(-0.62, s * 0.34, 0.55), 0.24, hips)
        kn = S.add(V(-0.30, s * 0.47, 0.32), 0.13, hp)
        hk = S.add(V(-0.78, s * 0.46, 0.20), 0.085, kn)
        ft = S.add(V(-0.46, s * 0.43, 0.06), 0.07, hk)
        for k in (-1, 0, 1):
            t = S.add(V(-0.26, s * (0.43 + 0.065 * k), 0.035), 0.03, ft)
            S.add(V(-0.15, s * (0.43 + 0.08 * k), 0.015), 0.007, t)
    # ---- wing bones (half-spread, swept back), roots on the shoulders
    for s in (-1, 1):
        sh = S.add(V(0.34, s * 0.20, 0.98), 0.12, chest)
        el = S.add(V(0.02, s * 0.78, 1.78), 0.075, sh)
        wr = S.add(V(-0.38, s * 1.38, 2.22), 0.055, el)
        S.add(V(-0.50, s * 1.42, 2.46), 0.012, wr)  # thumb claw
        for tip in (V(-0.62, s * 2.05, 1.82), V(-1.05, s * 1.78, 1.28), V(-1.12, s * 1.22, 0.92)):
            mid = (S.v[wr] + tip) * 0.5
            m = S.add(mid, 0.03, wr)
            S.add(tip, 0.008, m)
    return S


def wing_membrane(s):
    """Triangulated, scalloped, slightly sagging membrane (verts, faces), one wing."""
    sh, el, wr = V(0.34, s * 0.20, 0.98), V(0.02, s * 0.78, 1.78), V(-0.38, s * 1.38, 2.22)
    tips = [V(-0.62, s * 2.05, 1.82), V(-1.05, s * 1.78, 1.28), V(-1.12, s * 1.22, 0.92), V(-0.92, s * 0.40, 0.66)]
    verts, faces = [], []
    N = 8
    th = 0.012

    def panel(A, B, C, scallop, sag):
        base = len(verts)
        idx = {}
        for i in range(N + 1):
            for j in range(N + 1 - i):
                u, v = i / N, j / N
                t_ = v / (u + v) if (u + v) > 1e-9 else 0.5
                sc = 1.0 - scallop * 4 * t_ * (1 - t_) * (u + v) ** 3
                u2, v2 = u * sc, v * sc
                p = A + (B - A) * u2 + (C - A) * v2
                p = p + V(0, 0, -sag * math.sin(math.pi * min(1.0, (u + v))) * 0.6 * (u + v))
                idx[(i, j)] = len(verts)
                verts.append(p)
        for i in range(N):
            for j in range(N - i):
                faces.append((idx[(i, j)], idx[(i + 1, j)], idx[(i, j + 1)]))
                if i + j < N - 1:
                    faces.append((idx[(i + 1, j)], idx[(i + 1, j + 1)], idx[(i, j + 1)]))

    panel(wr, tips[0], tips[1], 0.22, 0.10)
    panel(wr, tips[1], tips[2], 0.22, 0.09)
    panel(wr, tips[2], tips[3], 0.22, 0.08)
    # arm membrane: shoulder-elbow-wrist down to the back
    back = V(-0.35, s * 0.28, 0.72)
    panel(sh, el, back, 0.0, 0.03)
    panel(el, wr, back, 0.0, 0.03)
    panel(wr, tips[3], back, 0.1, 0.04)
    return verts, faces


def solidify(verts, faces, thick):
    bm = bmesh.new()
    vs = [bm.verts.new(v) for v in verts]
    for f in faces:
        try:
            bm.faces.new([vs[i] for i in f])
        except ValueError:
            pass
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=1e-5)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    r = bmesh.ops.solidify(bm, geom=bm.faces[:] + bm.edges[:] + bm.verts[:], thickness=thick)
    bm.normal_update()
    out_v = [tuple(v.co) for v in bm.verts]
    out_f = [tuple(v.index for v in f.verts) for f in bm.faces]
    bm.verts.index_update()
    out_f = [tuple(v.index for v in f.verts) for f in bm.faces]
    bm.free()
    return out_v, out_f


def build_body():
    S = skeleton()
    me = bpy.data.meshes.new("dragon_skel")
    me.from_pydata([tuple(v) for v in S.v], S.e, [])
    ob = bpy.data.objects.new("dragon_skel", me)
    bpy.context.scene.collection.objects.link(ob)
    bpy.context.view_layer.objects.active = ob
    ob.modifiers.new("skin", "SKIN")
    ob.modifiers["skin"].branch_smoothing = 0.6
    ob.modifiers["skin"].use_smooth_shade = True
    sv = me.skin_vertices[0].data
    for i, r in enumerate(S.r):
        sv[i].radius = (r, r * 0.92)
    sv[0].use_root = True
    sub = ob.modifiers.new("sub", "SUBSURF")
    sub.levels = 3
    sub.render_levels = 3
    tex = bpy.data.textures.new("dragon_noise", "CLOUDS")
    tex.noise_scale = 0.045
    tex.noise_depth = 3
    d = ob.modifiers.new("weather", "DISPLACE")
    d.texture = tex
    d.strength = 0.03
    d.mid_level = 0.5
    d.direction = "NORMAL"
    dg = bpy.context.evaluated_depsgraph_get()
    tmp = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    n_tri = sum(len(p.vertices) - 2 for p in tmp.polygons)
    bpy.data.meshes.remove(tmp)
    ratio = min(1.0, TARGET_BODY_TRIS / max(n_tri, 1))
    dec = ob.modifiers.new("dec", "DECIMATE")
    dec.ratio = ratio
    tri = ob.modifiers.new("tri", "TRIANGULATE")
    dg = bpy.context.evaluated_depsgraph_get()
    out = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    verts = [tuple(v.co) for v in out.vertices]
    faces = [tuple(p.vertices) for p in out.polygons]
    bpy.data.meshes.remove(out)
    bpy.data.objects.remove(ob)
    bpy.data.meshes.remove(me)
    print("DRAGON body tris pre-decimate %d ratio %.2f" % (n_tri, ratio))
    return verts, faces, S


def xform(verts, cx, cy, z0, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + x * c - y * s, cy + x * s + y * c, z0 + z) for (x, y, z) in verts]


def build(W, cx, cy, z0, yaw=math.pi):
    verts, faces, S = build_body()
    W.add_mesh("dragon", xform(verts, cx, cy, z0, yaw), faces, smooth=True)
    # membrane wings (both sides, thin solid)
    for s in (-1, 1):
        mv, mf = wing_membrane(s)
        mv, mf = solidify(mv, mf, 0.014)
        W.add_mesh("wing", xform(mv, cx, cy, z0, yaw), mf, smooth=False)
    # glowing eyes (tiny emissive spheres on the skull)
    for s in (-1, 1):
        p = xform([(1.14, s * 0.15, 2.04)], cx, cy, z0, yaw)[0]
        W.ellipsoid("eye", p, (0.04, 0.025, 0.025), seg=10, rings=6, yaw=yaw + s * 0.5)
    return S
