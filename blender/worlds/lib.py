"""Shared procedural-geometry builder for the robot-fleet-sim worlds.

Geometry is accumulated as plain python lists, one mesh per material (cheap, deterministic, UVs
are box-projected in metres so textures tile at a real-world scale), then turned into bpy objects.
Every solid piece also registers a collision primitive (box / cylinder) that tools/make_map.py and
the generated model.sdf use.
"""
import contextlib
import math
import os
import shutil

import bpy

TEX_SRC = os.path.expanduser("~/Assets/textures")


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    l = math.sqrt(_dot(a, a)) or 1.0
    return (a[0] / l, a[1] / l, a[2] / l)


def _normal(pts):
    # Newell's method, robust for n-gons
    nx = ny = nz = 0.0
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        nx += (a[1] - b[1]) * (a[2] + b[2])
        ny += (a[2] - b[2]) * (a[0] + b[0])
        nz += (a[0] - b[0]) * (a[1] + b[1])
    return (nx, ny, nz)


class WorldBuilder:
    def __init__(self, name, bounds, materials):
        """bounds = (xmin, ymin, xmax, ymax) of the interior floor."""
        self.name = name
        self.bounds = bounds
        self.mats = materials
        self.acc = {}  # mat -> {"v": [], "f": [], "uv": []}
        self.tags = {}  # tag -> [(mat, v0, v1)]
        self.prims = []
        self.lights = []
        self.tri_count = 0
        self.cur_tag = None
        self.meshes = []  # (mat, verts, faces, smooth): ready-made shared-vertex meshes (e.g. the keep dragon statue)
        # chunking (the keep): geometry is binned per (chunk, material); chunk = square tile of chunk_size metres
        self.chunk_size = None
        self.chunk_origin = (0.0, 0.0)
        self.cur_chunk = None  # forced chunk key (e.g. "lake", "sky")
        self.chunk_names = {}

    def add_mesh(self, mat, verts, faces, smooth=True, uv=None):
        """Inject a ready-made mesh (visual only). Faces are tris/quads over shared vertices."""
        if mat not in self.mats:
            raise KeyError("unknown material " + mat)
        vs = [tuple(v) for v in verts]
        ck = None
        if self.chunk_size:
            ck = self._ck(sum(v[0] for v in vs) / len(vs), sum(v[1] for v in vs) / len(vs))
        self.meshes.append((mat, vs, [tuple(f) for f in faces], smooth, ck, uv))
        self.tri_count += sum(len(f) - 2 for f in faces)

    # ------------------------------------------------------------------ low level
    def _ck(self, x, y):
        if self.cur_chunk is not None:
            return self.cur_chunk
        cs = self.chunk_size
        return (int(math.floor((x - self.chunk_origin[0]) / cs)), int(math.floor((y - self.chunk_origin[1]) / cs)))

    @contextlib.contextmanager
    def chunk(self, key):
        """Force every piece built inside the block into chunk `key` (e.g. "lake")."""
        old, self.cur_chunk = self.cur_chunk, key
        try:
            yield
        finally:
            self.cur_chunk = old

    def _a(self, mat, ck=None):
        if mat not in self.mats:
            raise KeyError("unknown material " + mat)
        key = mat if not self.chunk_size else (ck, mat)
        return self.acc.setdefault(key, {"v": [], "f": [], "uv": []})

    def _face(self, mat, pts, uvpts, cen, tile=None, ck=None):
        """Add one convex polygon, oriented away from `cen`, box-projected UVs from uvpts."""
        n = _normal(pts)
        fc = (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts), sum(p[2] for p in pts) / len(pts))
        if _dot(n, _sub(fc, cen)) < 0:
            pts = pts[::-1]
            uvpts = uvpts[::-1]
        if self.chunk_size and ck is None:
            ck = self._ck(fc[0], fc[1])
        a = self._a(mat, ck)
        t = tile or self.mats[mat].get("tile", 1.0)
        nl = _normal(uvpts)
        ax = max(range(3), key=lambda i: abs(nl[i]))
        base = len(a["v"])
        a["v"].extend(pts)
        a["f"].append(tuple(range(base, base + len(pts))))
        self._rec(mat, base, base + len(pts))
        for p in uvpts:
            if ax == 2:
                a["uv"].append((p[0] / t, p[1] / t))
            elif ax == 0:
                a["uv"].append((p[1] / t, p[2] / t))
            else:
                a["uv"].append((p[0] / t, p[2] / t))
        self.tri_count += len(pts) - 2

    def _rec(self, mat, v0, v1):
        if self.cur_tag and not self.chunk_size:
            lst = self.tags.setdefault(self.cur_tag, [])
            if lst and lst[-1][0] == mat and lst[-1][2] == v0:
                lst[-1] = (mat, lst[-1][1], v1)
            else:
                lst.append((mat, v0, v1))

    @contextlib.contextmanager
    def tagged(self, tag):
        """Everything built inside the block is clamped by the preview cutaway for `tag`."""
        old, self.cur_tag = self.cur_tag, tag
        try:
            yield
        finally:
            self.cur_tag = old

    def _tag(self, tag, mat, v0):
        if tag and not self.chunk_size:
            self.tags.setdefault(tag, []).append((mat, v0, len(self.acc[mat]["v"])))

    # ------------------------------------------------------------------ primitives
    def box(self, mat, cx, cy, z0, sx, sy, h, yaw=0.0, solid=True, tag=None, tile=None, sides=True):
        cs = self.chunk_size
        if cs and yaw == 0.0 and self.cur_chunk is None and (sx > cs * 0.6 or sy > cs * 0.6):
            self._split_box(mat, cx, cy, z0, sx, sy, h, tile)
            if solid:
                self.prims.append({"type": "box", "size": [sx, sy, h], "pose": [cx, cy, z0 + h / 2, yaw]})
            return
        ck = self._ck(cx, cy) if cs else None
        a = self._a(mat, ck)
        v0 = len(a["v"])
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = sx / 2, sy / 2
        loc = [(x, y, z) for z in (0.0, h) for (x, y) in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
        wp = [(cx + x * c - y * s, cy + x * s + y * c, z0 + z) for (x, y, z) in loc]
        up = [(cx + x, cy + y, z0 + z) for (x, y, z) in loc]
        cen = (cx, cy, z0 + h / 2)
        quads = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
        for q in quads:
            self._face(mat, [wp[i] for i in q], [up[i] for i in q], cen, tile, ck)
        self._tag(tag, mat, v0)
        if solid:
            self.prims.append({"type": "box", "size": [sx, sy, h], "pose": [cx, cy, z0 + h / 2, yaw]})

    def _split_box(self, mat, cx, cy, z0, sx, sy, h, tile):
        """Visual-only: cut a big axis-aligned box at the chunk grid lines so each piece lands in its own tile."""
        cs, (ox, oy) = self.chunk_size, self.chunk_origin
        xs = [cx - sx / 2] + [ox + k * cs for k in range(int(math.floor((cx - sx / 2 - ox) / cs)) + 1, int(math.floor((cx + sx / 2 - ox) / cs)) + 1)
                              if cx - sx / 2 < ox + k * cs < cx + sx / 2] + [cx + sx / 2]
        ys = [cy - sy / 2] + [oy + k * cs for k in range(int(math.floor((cy - sy / 2 - oy) / cs)) + 1, int(math.floor((cy + sy / 2 - oy) / cs)) + 1)
                              if cy - sy / 2 < oy + k * cs < cy + sy / 2] + [cy + sy / 2]
        for i in range(len(xs) - 1):
            for j in range(len(ys) - 1):
                px, py = (xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2
                old = self.cur_chunk
                self.cur_chunk = self._ck(px, py)
                try:
                    self.box(mat, px, py, z0, xs[i + 1] - xs[i], ys[j + 1] - ys[j], h, solid=False, tile=tile)
                finally:
                    self.cur_chunk = old

    def col_box(self, cx, cy, z0, sx, sy, h, yaw=0.0, rp=None):
        """Collision-only box (visual built from several non-solid pieces). rp = (roll, pitch) for tilted
        underwater pieces (never used inside the 0.05-0.60 m band, the map ignores tilted primitives)."""
        p = {"type": "box", "size": [sx, sy, h], "pose": [cx, cy, z0 + h / 2, yaw]}
        if rp:
            p["rp"] = list(rp)
        self.prims.append(p)

    def col_cyl(self, cx, cy, z0, r, h):
        self.prims.append({"type": "cylinder", "radius": r, "length": h, "pose": [cx, cy, z0 + h / 2, 0.0]})

    def boxp(self, mat, x0, y0, x1, y1, z0, h, **kw):
        """Box from two corners."""
        self.box(mat, (x0 + x1) / 2, (y0 + y1) / 2, z0, abs(x1 - x0), abs(y1 - y0), h, **kw)

    def cyl(self, mat, cx, cy, z0, r, h, seg=20, solid=True, tag=None, tile=None, r_top=None):
        ck = self._ck(cx, cy) if self.chunk_size else None
        a = self._a(mat, ck)
        v0 = len(a["v"])
        t = tile or self.mats[mat].get("tile", 1.0)
        rt = r if r_top is None else r_top
        bot = [(cx + r * math.cos(2 * math.pi * i / seg), cy + r * math.sin(2 * math.pi * i / seg), z0) for i in range(seg)]
        top = [(cx + rt * math.cos(2 * math.pi * i / seg), cy + rt * math.sin(2 * math.pi * i / seg), z0 + h) for i in range(seg)]
        cen = (cx, cy, z0 + h / 2)
        circ = 2 * math.pi * r
        for i in range(seg):
            j = (i + 1) % seg
            pts = [bot[i], bot[j], top[j], top[i]]
            fc = tuple(sum(p[k] for p in pts) / 4 for k in range(3))
            n = _normal(pts)
            if _dot(n, _sub(fc, cen)) < 0:
                pts = pts[::-1]
            base = len(a["v"])
            a["v"].extend(pts)
            a["f"].append((base, base + 1, base + 2, base + 3))
            self._rec(mat, base, base + 4)
            us = [(math.atan2(p[1] - cy, p[0] - cx) % (2 * math.pi)) * r / t for p in pts]
            if max(us) - min(us) > circ / t * 0.5:  # face straddles the seam
                us = [u + circ / t if u < circ / t * 0.25 else u for u in us]
            for p, u in zip(pts, us):
                a["uv"].append((u, (p[2] - z0) / t))
            self.tri_count += 2
        for pts in (bot, top):
            if pts is top and rt < 1e-6:
                continue
            self._face(mat, pts, [(p[0], p[1], p[2]) for p in pts], cen, tile, ck)
        self._tag(tag, mat, v0)
        if solid:
            self.prims.append({"type": "cylinder", "radius": max(r, rt), "length": h, "pose": [cx, cy, z0 + h / 2, 0.0]})

    def frustum(self, mat, p0, p1, r0, r1, seg=10, tile=None):
        """Tapered tube between two 3D points (visual only)."""
        axis = _norm(_sub(p1, p0))
        ref = (0, 0, 1) if abs(axis[2]) < 0.9 else (1, 0, 0)
        u = _norm(_cross(axis, ref))
        v = _cross(axis, u)
        cen = tuple((p0[i] + p1[i]) / 2 for i in range(3))
        ring0 = [tuple(p0[k] + r0 * (math.cos(2 * math.pi * i / seg) * u[k] + math.sin(2 * math.pi * i / seg) * v[k]) for k in range(3)) for i in range(seg)]
        ring1 = [tuple(p1[k] + r1 * (math.cos(2 * math.pi * i / seg) * u[k] + math.sin(2 * math.pi * i / seg) * v[k]) for k in range(3)) for i in range(seg)]
        for i in range(seg):
            j = (i + 1) % seg
            q = [ring0[i], ring0[j], ring1[j], ring1[i]]
            self._face(mat, q, q, cen, tile)
        self._face(mat, ring0, ring0, cen, tile)
        if r1 > 1e-4:
            self._face(mat, ring1, ring1, cen, tile)

    def ellipsoid(self, mat, c, radii, seg=12, rings=8, yaw=0.0, pitch=0.0, tile=None):
        rx, ry, rz = radii
        cp, sp = math.cos(pitch), math.sin(pitch)
        cyw, syw = math.cos(yaw), math.sin(yaw)

        def P(i, j):
            th = math.pi * i / rings
            ph = 2 * math.pi * j / seg
            x, y, z = rx * math.sin(th) * math.cos(ph), ry * math.sin(th) * math.sin(ph), rz * math.cos(th)
            x, z = x * cp + z * sp, -x * sp + z * cp
            x, y = x * cyw - y * syw, x * syw + y * cyw
            return (c[0] + x, c[1] + y, c[2] + z)

        for i in range(rings):
            for j in range(seg):
                j2 = (j + 1) % seg
                pts = [P(i, j), P(i, j2), P(i + 1, j2), P(i + 1, j)]
                uniq = []
                for p in pts:
                    if not any(max(abs(p[k] - q[k]) for k in range(3)) < 1e-9 for q in uniq):
                        uniq.append(p)
                if len(uniq) >= 3:
                    self._face(mat, uniq, uniq, c, tile)

    def tri_panel(self, mat, p0, p1, p2, th=0.03, tile=None):
        n = _norm(_cross(_sub(p1, p0), _sub(p2, p0)))
        o = (n[0] * th / 2, n[1] * th / 2, n[2] * th / 2)
        A = [(p[0] + o[0], p[1] + o[1], p[2] + o[2]) for p in (p0, p1, p2)]
        B = [(p[0] - o[0], p[1] - o[1], p[2] - o[2]) for p in (p0, p1, p2)]
        cen = tuple(sum(p[k] for p in (p0, p1, p2)) / 3 for k in range(3))
        self._face(mat, A, A, cen, tile)
        self._face(mat, B, B, cen, tile)
        for i in range(3):
            j = (i + 1) % 3
            q = [A[i], A[j], B[j], B[i]]
            self._face(mat, q, q, cen, tile)

    def quad_panel(self, mat, p0, p1, p2, p3, th=0.03, tile=None):
        self.tri_panel(mat, p0, p1, p2, th, tile)
        self.tri_panel(mat, p0, p2, p3, th, tile)

    def decal(self, mat, cx, cy, sx, sy, yaw=0.0, z=0.004):
        self.box(mat, cx, cy, z, sx, sy, 0.004, yaw=yaw, solid=False)

    def stripes(self, mat_a, mat_b, x0, y0, x1, y1, step=0.3, along="x"):
        """Alternating hazard stripes filling a rectangle (axis-aligned bars)."""
        n = int(round(((x1 - x0) if along == "x" else (y1 - y0)) / step))
        for i in range(n):
            m = mat_a if i % 2 == 0 else mat_b
            if along == "x":
                self.decal(m, x0 + (i + 0.5) * step, (y0 + y1) / 2, step, y1 - y0)
            else:
                self.decal(m, (x0 + x1) / 2, y0 + (i + 0.5) * step, x1 - x0, step)

    def line(self, mat, x0, y0, x1, y1, w=0.1, z=0.004):
        L = math.hypot(x1 - x0, y1 - y0)
        self.decal(mat, (x0 + x1) / 2, (y0 + y1) / 2, L, w, yaw=math.atan2(y1 - y0, x1 - x0), z=z)

    def light(self, name, x, y, z, color, intensity=1.0, rng=8.0, watts=100.0, atten=None):
        d = {"name": name, "pos": [x, y, z], "color": list(color), "intensity": intensity, "range": rng, "watts": watts}
        if atten:
            d["atten"] = list(atten)
        self.lights.append(d)

    # ------------------------------------------------------------------ walls
    def wall(self, a, b, t, h, mat, openings=(), tag="wall", ext=True, arch_mat=None):
        """Axis-aligned wall centreline a->b. openings: dicts {c: centre along wall from a, w, h, arch}.
        An arch opening's `h` is the height of the arch apex; the spring line is h - w/2."""
        (x0, y0), (x1, y1) = a, b
        if abs(y1 - y0) < 1e-9:
            ax = "x"
            if x1 < x0:
                x0, x1 = x1, x0
                openings = [dict(o, c=(x1 - x0) - o["c"]) for o in openings]
            L = x1 - x0
            to = lambda u, v, z: (x0 + u, y0 + v, z)
        else:
            ax = "y"
            if y1 < y0:
                y0, y1 = y1, y0
                openings = [dict(o, c=(y1 - y0) - o["c"]) for o in openings]
            L = y1 - y0
            to = lambda u, v, z: (x0 + v, y0 + u, z)
        e = t / 2 if ext else 0.0
        ops = sorted(openings, key=lambda o: o["c"])
        segs = []
        cur = -e
        for o in ops:
            segs.append((cur, o["c"] - o["w"] / 2))
            cur = o["c"] + o["w"] / 2
        segs.append((cur, L + e))
        am = arch_mat or mat

        def put(u0, u1, z0, hh, solid):
            if u1 - u0 < 1e-6:
                return
            if ax == "x":
                self.boxp(mat, x0 + u0, y0 - t / 2, x0 + u1, y0 + t / 2, z0, hh, solid=solid, tag=tag)
            else:
                self.boxp(mat, x0 - t / 2, y0 + u0, x0 + t / 2, y0 + u1, z0, hh, solid=solid, tag=tag)

        for (u0, u1) in segs:
            put(u0, u1, 0.0, h, True)
        for o in ops:
            c, w, oh = o["c"], o["w"], o["h"]
            if not o.get("arch"):
                put(c - w / 2, c + w / 2, oh, h - oh, False)
            else:
                r = w / 2
                s = oh - r
                n = 14
                a_ = self._a(mat, self._ck(*to(c, 0, 0)[:2]) if self.chunk_size else None)
                v0 = len(a_["v"])
                for i in range(n):
                    t0, t1 = math.pi * (1 - i / n), math.pi * (1 - (i + 1) / n)
                    ua, za = c + r * math.cos(t0), s + r * math.sin(t0)
                    ub, zb = c + r * math.cos(t1), s + r * math.sin(t1)
                    pts = [to(u, v, z) for v in (-t / 2, t / 2) for (u, z) in ((ua, za), (ub, zb), (ub, h), (ua, h))]
                    cen = tuple(sum(p[k] for p in pts) / 8 for k in range(3))
                    for q in ((0, 1, 2, 3), (4, 5, 6, 7), (2, 3, 7, 6), (0, 1, 5, 4)):
                        qq = [pts[k] for k in q]
                        self._face(mat, qq, qq, cen)
                self._tag(tag, mat, v0)

    def roof(self, mat, x0, y0, x1, y1, z, th=0.3, tag="roof", beam_mat=None, beams=0, beam_axis="x", beam_w=0.35, beam_h=0.5):
        """Flat roof slab (top at z+th, underside at z) with collision; optional exposed beams under it."""
        self.boxp(mat, x0, y0, x1, y1, z, th, solid=True, tag=tag)
        if beams:
            L = (y1 - y0) if beam_axis == "x" else (x1 - x0)
            for i in range(beams):
                c = ((y0 if beam_axis == "x" else x0) + (i + 0.5) * L / beams)
                if beam_axis == "x":
                    self.boxp(beam_mat or mat, x0, c - beam_w / 2, x1, c + beam_w / 2, z - beam_h, beam_h, solid=False, tag=tag)
                else:
                    self.boxp(beam_mat or mat, c - beam_w / 2, y0, c + beam_w / 2, y1, z - beam_h, beam_h, solid=False, tag=tag)

    def perimeter(self, mat, t, h, openings=None):
        """Four outer walls hugging the interior bounds from outside (interior stays exactly `bounds`)."""
        x0, y0, x1, y1 = self.bounds
        op = openings or {}
        self.wall((x0 - t, y0 - t / 2), (x1 + t, y0 - t / 2), t, h, mat, op.get("s", ()), tag="perim_s", ext=False)
        self.wall((x0 - t, y1 + t / 2), (x1 + t, y1 + t / 2), t, h, mat, op.get("n", ()), tag="perim_n", ext=False)
        self.wall((x1 + t / 2, y0), (x1 + t / 2, y1), t, h, mat, op.get("e", ()), tag="perim_e", ext=False)
        self.wall((x0 - t / 2, y0), (x0 - t / 2, y1), t, h, mat, op.get("w", ()), tag="perim_w", ext=False)

    def ring(self, mat, cx, cy, r, w, n=32, z=0.004):
        for i in range(n):
            a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
            self.line(mat, cx + r * math.cos(a0), cy + r * math.sin(a0), cx + r * math.cos(a1), cy + r * math.sin(a1), w, z)

    # ------------------------------------------------------------------ finalisation
    def floor(self, mat, margin=0.0, thick=0.2, tag=None, surface=None):
        """surface: key of sdfgen.FLOOR_SURFACES (rubber/floor friction of the collision floor)."""
        x0, y0, x1, y1 = self.bounds
        self.boxp(mat, x0 - margin, y0 - margin, x1 + margin, y1 + margin, -0.1, 0.1, solid=False, tag=tag)
        # collision floor: top at z=0
        prim = {"type": "box", "size": [x1 - x0 + 2 * margin, y1 - y0 + 2 * margin, thick],
                "pose": [(x0 + x1) / 2, (y0 + y1) / 2, -thick / 2, 0.0]}
        if surface:
            prim["surface"] = surface
        self.prims.append(prim)

    def refill_plain(self, mat, me):
        """Put the un-bevelled geometry back into `me` (the preview cutaway clamps vertices by their original index)."""
        a = self.acc[mat]
        me.clear_geometry()
        me.from_pydata(a["v"], [], a["f"])
        uvl = me.uv_layers.new(name="UVMap")
        flat = []
        for (u, v) in a["uv"]:
            flat.extend((u, v))
        uvl.data.foreach_set("uv", flat)
        me.update()

    def chunk_keys(self):
        ks = {k[0] for k in self.acc} | {m[4] for m in self.meshes}
        return sorted(ks, key=lambda k: (isinstance(k, str), str(k)))

    def make_chunk(self, ck, collection):
        """bpy objects for one chunk (chunk mode); returns a list."""
        out = []
        for (c, mat), a in self.acc.items():
            if c != ck:
                continue
            nm = "%s_%s_%s" % (self.name, self.chunk_name(ck), mat)
            me = bpy.data.meshes.new(nm)
            me.from_pydata(a["v"], [], a["f"])
            uvl = me.uv_layers.new(name="UVMap")
            flat = []
            for (u, v) in a["uv"]:
                flat.extend((u, v))
            uvl.data.foreach_set("uv", flat)
            me.update()
            bw = self.mats[mat].get("bevel")
            if bw:
                _bevel_mesh(me, bw)
            me.materials.append(bpy.data.materials[self.name + "_" + mat])
            ob = bpy.data.objects.new(nm, me)
            collection.objects.link(ob)
            out.append(ob)
        for i, (mat, vs, fs, smooth, mck, uv) in enumerate(self.meshes):
            if mck != ck:
                continue
            nm = "%s_%s_%s_m%d" % (self.name, self.chunk_name(ck), mat, i)
            me = bpy.data.meshes.new(nm)
            me.from_pydata(vs, [], fs)
            me.update()
            if uv is not None:
                uvl = me.uv_layers.new(name="UVMap")
                flat = []
                for lp in me.loops:
                    flat.extend(uv[lp.vertex_index])
                uvl.data.foreach_set("uv", flat)
            if smooth:
                me.polygons.foreach_set("use_smooth", [True] * len(me.polygons))
            me.materials.append(bpy.data.materials[self.name + "_" + mat])
            ob = bpy.data.objects.new(nm, me)
            collection.objects.link(ob)
            out.append(ob)
        return out

    @staticmethod
    def chunk_name(ck):
        if isinstance(ck, str):
            return ck
        return "t%s_%s" % (("m%d" % -ck[0]) if ck[0] < 0 else str(ck[0]), ("m%d" % -ck[1]) if ck[1] < 0 else str(ck[1]))

    def make_objects(self, collection):
        objs = {}
        for mat, a in self.acc.items():
            me = bpy.data.meshes.new(self.name + "_" + mat)
            me.from_pydata(a["v"], [], a["f"])
            uvl = me.uv_layers.new(name="UVMap")
            flat = []
            for (u, v) in a["uv"]:
                flat.extend((u, v))
            uvl.data.foreach_set("uv", flat)
            me.update()
            bw = self.mats[mat].get("bevel")
            if bw:
                _bevel_mesh(me, bw)
            me.materials.append(bpy.data.materials[self.name + "_" + mat])
            ob = bpy.data.objects.new(self.name + "_" + mat, me)
            collection.objects.link(ob)
            objs[mat] = ob
        for i, (mat, vs, fs, smooth, _ck, uv) in enumerate(self.meshes):
            me = bpy.data.meshes.new("%s_%s_m%d" % (self.name, mat, i))
            me.from_pydata(vs, [], fs)
            me.update()
            if uv is not None:
                uvl = me.uv_layers.new(name="UVMap")
                flat = []
                for lp in me.loops:
                    flat.extend(uv[lp.vertex_index])
                uvl.data.foreach_set("uv", flat)
            if smooth:
                me.polygons.foreach_set("use_smooth", [True] * len(me.polygons))
            me.materials.append(bpy.data.materials[self.name + "_" + mat])
            ob = bpy.data.objects.new("%s_%s_m%d" % (self.name, mat, i), me)
            collection.objects.link(ob)
            objs[(mat, i)] = ob
        return objs


def _bevel_mesh(me, width):
    """Chamfer/round the hard edges of a mesh built from separate boxes (visual only, never grows a
    box: the bevel only removes material inside the original silhouette)."""
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=1e-5)
    edges = [e for e in bm.edges if len(e.link_faces) == 2 and e.calc_face_angle(0.0) > 0.6]
    bmesh.ops.bevel(bm, geom=edges, offset=width, offset_type="OFFSET", segments=2, profile=0.6,
                    affect="EDGES", clamp_overlap=True)
    bm.to_mesh(me)
    bm.free()
    me.update()


# ---------------------------------------------------------------------- materials
_IMG = {}


def load_image(path, data):
    if path not in _IMG:
        img = bpy.data.images.load(path)
        img.colorspace_settings.name = "sRGB" if data == "color" else "Non-Color"
        _IMG[path] = img
    return _IMG[path]


def make_materials(world, specs, tex_dir):
    """Create bpy materials (name = world_matname). Copies used texture maps into tex_dir."""
    used = set()
    for name, sp in specs.items():
        m = bpy.data.materials.new(world + "_" + name)
        m.use_nodes = True
        nt = m.node_tree
        for n in list(nt.nodes):
            if n.type != "OUTPUT_MATERIAL":
                nt.nodes.remove(n)
        out = [n for n in nt.nodes if n.type == "OUTPUT_MATERIAL"][0]
        bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
        nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
        col = sp.get("color", (0.5, 0.5, 0.5))
        bsdf.inputs["Base Color"].default_value = (col[0], col[1], col[2], 1.0)
        bsdf.inputs["Roughness"].default_value = sp.get("rough", 0.6)
        bsdf.inputs["Metallic"].default_value = sp.get("metal", 0.0)
        if "alpha" in sp:
            bsdf.inputs["Alpha"].default_value = sp["alpha"]
            try:
                m.surface_render_method = "BLENDED"
            except Exception:
                pass
            m.use_backface_culling = False
        if "emit" in sp:
            ec, es = sp["emit"]
            bsdf.inputs["Emission Color"].default_value = (ec[0], ec[1], ec[2], 1.0)
            bsdf.inputs["Emission Strength"].default_value = es
        tid = sp.get("tex")
        if tid:
            os.makedirs(tex_dir, exist_ok=True)
            maps = {"diff": "color", "nor_gl": "data", "rough": "data"}
            nodes = {}
            for k, kind in maps.items():
                fn = "%s_%s_1k.jpg" % (tid, k)
                src = os.path.join(TEX_SRC, tid, fn)
                if not os.path.exists(src):
                    continue
                dst = os.path.join(tex_dir, fn)
                if not os.path.exists(dst):
                    shutil.copy(src, dst)
                used.add(fn)
                n = nt.nodes.new("ShaderNodeTexImage")
                n.image = load_image(dst, kind)
                nodes[k] = n
            if "diff" in nodes:
                tint = sp.get("tint")
                if tint:
                    mix = nt.nodes.new("ShaderNodeMix")
                    mix.data_type = "RGBA"
                    mix.blend_type = "MULTIPLY"
                    mix.inputs["Factor"].default_value = 1.0
                    mix.inputs["B"].default_value = (tint[0], tint[1], tint[2], 1.0)
                    nt.links.new(nodes["diff"].outputs["Color"], mix.inputs["A"])
                    nt.links.new(mix.outputs["Result"], bsdf.inputs["Base Color"])
                else:
                    nt.links.new(nodes["diff"].outputs["Color"], bsdf.inputs["Base Color"])
            if "nor_gl" in nodes:
                nm = nt.nodes.new("ShaderNodeNormalMap")
                nt.links.new(nodes["nor_gl"].outputs["Color"], nm.inputs["Color"])
                nt.links.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
            if "rough" in nodes:
                nt.links.new(nodes["rough"].outputs["Color"], bsdf.inputs["Roughness"])
    return sorted(used)
