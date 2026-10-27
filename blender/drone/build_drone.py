"""Build the wyvern quadcopter visuals (a dragon-flavoured carbon X-quad on the PX4 x500 airframe) in headless Blender.

    blender -b --factory-startup -P blender/drone/build_drone.py -- [--no-render] [--review DIR] [--hero PNG]
                                                                     [--res 1600x900] [--samples 96]

Writes (base_link frame: x forward, y left, z up, metres; glTF exported Z-up, i.e. no axis conversion, so Gazebo
and RViz read the coordinates as they are):
  src/fleet_drone/models/wyvern/meshes/wyvern_body.glb       everything fixed to base_link
  src/fleet_drone/models/wyvern/meshes/wyvern_rotor_ccw.glb  motor bell + 2-blade prop + nut, rotor-link frame
  src/fleet_drone/models/wyvern/meshes/wyvern_rotor_cw.glb   mirrored prop (cw)
  blender/drone/textures/*.png                                  procedural PBR maps (embedded in the GLBs)
  docs/img/wyvern_hero.png                                      cinematic EEVEE render
Geometry that has to match the simulation comes from PX4's x500 (rotor positions, prop radius from the rotor
collision box, landing-gear collision boxes); everything else is decoration below/around the prop discs.
All meshes and textures are generated here (self-made, no external assets).
"""
import argparse
import math
import sys
from pathlib import Path

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "blender"))
from robots import common as C  # noqa: E402

OUT = ROOT / "src" / "fleet_drone" / "models" / "wyvern" / "meshes"
TEX = ROOT / "blender" / "drone" / "textures"
IMG = ROOT / "docs" / "img"

# ---------------------------------------------------------------- x500 numbers (PX4 x500_base/model.sdf)
ROTORS = [((0.174, -0.174), "ccw"), ((-0.174, 0.174), "ccw"), ((0.174, 0.174), "cw"), ((-0.174, -0.174), "cw")]
ROTOR_Z = 0.06
PROP_R = 0.1396           # rotor collision box 0.2792 long
ARM_Z, ARM_R = 0.012, 0.008
LEG_TOP, LEG_BOT = (0.062, -0.0244), (0.134, -0.2216)   # (|y|, z) ends of the leg collision boxes
SKID_Y, SKID_Z, SKID_R = 0.132, -0.2195, 0.0075

TILE = {}                 # material name -> texture tile size in metres (UV = metres / tile)


# ================================================================= procedural textures (numpy)
def _blur(a, s_rows, s_cols):
    f0 = np.fft.fftfreq(a.shape[0])[:, None]
    f1 = np.fft.fftfreq(a.shape[1])[None, :]
    g = np.exp(-2 * np.pi ** 2 * ((s_rows * f0) ** 2 + (s_cols * f1) ** 2))
    return np.real(np.fft.ifft2(np.fft.fft2(a) * g))


def _norm01(a):
    a = a - a.min()
    return a / (a.max() + 1e-9)


def tnoise(n, s_rows, s_cols, seed):
    return _norm01(_blur(np.random.default_rng(seed).standard_normal((n, n)), s_rows, s_cols))


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


def srgb(lin):
    lin = np.clip(lin, 0, 1)
    return np.where(lin <= 0.0031308, lin * 12.92, 1.055 * np.power(lin, 1 / 2.4) - 0.055)


def hexlin(h):
    return np.array(C._rgb(h))


def normal_map(h, k):
    """Height (rows = +v, cols = +u, periodic) -> OpenGL tangent-space normal map in 0..1."""
    dx = (np.roll(h, -1, 1) - np.roll(h, 1, 1)) * 0.5 * k
    dy = (np.roll(h, -1, 0) - np.roll(h, 1, 0)) * 0.5 * k
    n = np.dstack([-dx, -dy, np.ones_like(h)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return n * 0.5 + 0.5


def save_png(path, arr):
    """arr rows run along +v (row 0 = v 0, Blender's pixel order), values already encoded."""
    if arr.ndim == 2:
        arr = np.repeat(arr[..., None], 3, 2)
    h, w = arr.shape[:2]
    rgba = np.ones((h, w, 4), np.float32)
    rgba[..., :3] = arr[..., :3]
    im = bpy.data.images.new("tmp_" + path.stem, w, h, alpha=False)
    im.pixels.foreach_set(rgba.ravel())
    im.filepath_raw = str(path)
    im.file_format = "PNG"
    im.save()
    bpy.data.images.remove(im)
    return path


def worley(n, npts, seed):
    p = np.random.default_rng(seed).random((npts, 2))
    c = (np.arange(n) + 0.5) / n
    u, v = np.meshgrid(c, c)
    f1 = np.full((n, n), 9.0)
    f2 = f1.copy()
    for px, py in p:
        du = np.abs(u - px)
        du = np.minimum(du, 1 - du)
        dv = np.abs(v - py)
        dv = np.minimum(dv, 1 - dv)
        d = np.sqrt(du * du + dv * dv)
        f2 = np.minimum(f2, np.maximum(f1, d))
        f1 = np.minimum(f1, d)
    return f1, f2


def tex_carbon(n=512, tows=8):
    """2/2 twill 3K weave with a violet-tinted clear coat: 8 tows per tile (tile 24 mm -> 3 mm tows)."""
    c = (np.arange(n) + 0.5) / n
    u, v = np.meshgrid(c, c)
    a, b = np.floor(u * tows), np.floor(v * tows)
    fu, fv = u * tows - a, v * tows - b
    warp = ((a + b) % 4) < 2
    across = np.where(warp, fu, fv)
    along = np.where(warp, fv, fu)
    crown = np.clip(np.sin(np.pi * across), 0, 1) ** 0.6
    h = crown * (0.82 + 0.18 * np.sin(np.pi * along))
    sw = tnoise(n, 30.0, 0.7, 7)        # warp tows run along v: streaks vary across u
    sf = tnoise(n, 0.7, 30.0, 8)
    streak = np.where(warp, sw, sf)
    lum = np.where(warp, 0.036, 0.017) * (0.7 + 0.6 * streak) * (0.28 + 0.72 * h)
    alb = srgb(lum[..., None] * np.array([0.92, 0.78, 1.25]))
    rough = 0.20 + 0.10 * (1 - h) + 0.06 * (streak - 0.5)
    orm = np.dstack([np.ones_like(h), rough, np.zeros_like(h)])
    return dict(albedo=save_png(TEX / "carbon_albedo.png", alb), orm=save_png(TEX / "carbon_orm.png", orm),
                normal=save_png(TEX / "carbon_normal.png", normal_map(h + 0.12 * streak, 9.0)))


def tex_membrane(n=512):
    """Wing membrane: dark plum skin, branching veins (cell edges) that glow faintly."""
    f1, f2 = worley(n, 9, 11)
    big = 1 - smoothstep(0.0, 0.022, f2 - f1)
    g1, g2 = worley(n, 42, 12)
    fine = 1 - smoothstep(0.0, 0.007, g2 - g1)
    mott = tnoise(n, 14, 14, 13)
    vein = np.clip(big + 0.45 * fine, 0, 1)
    base = hexlin("#12031a")[None, None] * (1 - mott[..., None]) + hexlin("#2c0a3a")[None, None] * mott[..., None]
    alb = base * (1 - 0.85 * vein[..., None]) + hexlin("#6a2280")[None, None] * 0.85 * vein[..., None]
    rough = 0.74 - 0.16 * vein
    emi = (big * 0.9 + fine * 0.25)[..., None] * hexlin("#b02ad8")[None, None] * 0.55
    h = 0.7 * big + 0.35 * fine + 0.25 * mott
    orm = np.dstack([np.ones_like(h), rough, np.full_like(h, 0.05)])
    return dict(albedo=save_png(TEX / "membrane_albedo.png", srgb(alb)),
                orm=save_png(TEX / "membrane_orm.png", orm),
                normal=save_png(TEX / "membrane_normal.png", normal_map(h, 5.0)),
                emission=save_png(TEX / "membrane_emission.png", srgb(emi)))


def tex_scales(n=512, rows=8, cols=4):
    """Overlapping dragon scales (free edges face -v = towards the tail); metallic violet."""
    c = (np.arange(n) + 0.5) / n
    u, v = np.meshgrid(c, c)
    rad = 0.62 / cols
    dist = np.full((n, n), rad)
    sk = np.zeros((n, n), int)
    si = np.zeros((n, n), int)
    base = np.floor(v * rows).astype(int)
    for off in range(-3, 4):
        k = base + off
        shift = 0.5 * (k % 2)
        ic = np.round(u * cols - shift - 0.5)
        du = u - (ic + 0.5 + shift) / cols
        du -= np.round(du)
        dv = v - (k + 0.5) / rows
        d = np.sqrt(du * du + dv * dv)
        m = d < rad
        dist = np.where(m, d, dist)
        sk = np.where(m, k % rows, sk)
        si = np.where(m, ic.astype(int) % cols, si)
    r = dist / rad
    h = 0.2 + 0.8 * r
    rnd = np.random.default_rng(5).random((rows, cols, 3))[sk, si]
    col = hexlin("#48146e")[None, None] * (0.8 + 0.45 * rnd[..., :1])
    col = col + hexlin("#7a0f62")[None, None] * 0.45 * rnd[..., 1:2]
    col = col * (0.45 + 0.55 * h[..., None]) * np.where(r[..., None] > 0.9, 1.35, 1.0)
    rough = 0.44 - 0.16 * h + 0.06 * rnd[..., 2]
    orm = np.dstack([np.ones_like(h), rough, np.full_like(h, 0.6)])
    return dict(albedo=save_png(TEX / "scales_albedo.png", srgb(col)), orm=save_png(TEX / "scales_orm.png", orm),
                normal=save_png(TEX / "scales_normal.png", normal_map(h, 7.0)))


# ================================================================= materials
def pbr(name, base, metal=0.0, rough=0.5, emit=None, estr=0.0, tex=None, coat=0.0, nstr=1.0, tile=0.05):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    b = nt.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = list(C._rgb(base)) + [1]
    m.diffuse_color = list(C._rgb(base)) + [1]
    b.inputs["Metallic"].default_value = metal
    b.inputs["Roughness"].default_value = rough
    if coat:
        b.inputs["Coat Weight"].default_value = coat
        b.inputs["Coat Roughness"].default_value = 0.06
    if emit:
        b.inputs["Emission Color"].default_value = list(C._rgb(emit)) + [1]
        b.inputs["Emission Strength"].default_value = estr
    if tex:
        def img(key, data):
            im = bpy.data.images.load(str(tex[key]), check_existing=True)
            if data:
                im.colorspace_settings.name = "Non-Color"
            node = nt.nodes.new("ShaderNodeTexImage")
            node.image = im
            return node
        if "albedo" in tex:
            nt.links.new(img("albedo", False).outputs["Color"], b.inputs["Base Color"])
        if "orm" in tex:
            node = img("orm", True)
            sep = nt.nodes.new("ShaderNodeSeparateColor")
            nt.links.new(node.outputs["Color"], sep.inputs["Color"])
            nt.links.new(sep.outputs["Green"], b.inputs["Roughness"])
            nt.links.new(sep.outputs["Blue"], b.inputs["Metallic"])
        if "normal" in tex:
            node = img("normal", True)
            nm = nt.nodes.new("ShaderNodeNormalMap")
            nm.inputs["Strength"].default_value = nstr
            nt.links.new(node.outputs["Color"], nm.inputs["Color"])
            nt.links.new(nm.outputs["Normal"], b.inputs["Normal"])
        if "emission" in tex:
            nt.links.new(img("emission", False).outputs["Color"], b.inputs["Emission Color"])
            b.inputs["Emission Strength"].default_value = estr or 1.0
    TILE[name] = tile
    return m


def make_materials():
    TEX.mkdir(parents=True, exist_ok=True)
    tc, tm, ts = tex_carbon(), tex_membrane(), tex_scales()
    M = dict(
        carbon=pbr("wy_carbon", "#141218", 0.0, 0.25, tex=tc, coat=0.7, tile=0.024),
        membrane=pbr("wy_membrane", "#2a0a36", 0.05, 0.7, tex=tm, estr=2.2, tile=0.08),
        scales=pbr("wy_scales", "#48146e", 0.6, 0.32, tex=ts, coat=0.3, tile=0.032),
        gun=pbr("wy_gunmetal", "#2b2833", 0.85, 0.34),
        anod=pbr("wy_anodized_violet", "#6a2aa6", 0.9, 0.27),
        anodm=pbr("wy_anodized_magenta", "#b0207e", 0.9, 0.27),
        steel=pbr("wy_steel", "#8d8a98", 0.95, 0.30),
        dark=pbr("wy_dark_plastic", "#121016", 0.0, 0.55),
        gloss=pbr("wy_gloss_black", "#09080c", 0.0, 0.12, coat=0.8),
        horn=pbr("wy_horn", "#1d1622", 0.35, 0.22, coat=0.5),
        ivory=pbr("wy_horn_ivory", "#a397b0", 0.1, 0.30, coat=0.5),
        bone=pbr("wy_fang", "#cbc1d3", 0.0, 0.38),
        rubber=pbr("wy_rubber", "#0e0d10", 0.0, 0.85),
        battery=pbr("wy_battery", "#18151f", 0.0, 0.45),
        label=pbr("wy_label", "#3d1d70", 0.2, 0.40),
        strap=pbr("wy_strap", "#1b1a20", 0.0, 0.92),
        glass=pbr("wy_lens", "#04040a", 0.4, 0.04, coat=1.0),
        red=pbr("wy_wire_red", "#7a1020", 0.0, 0.5),
        arrow=pbr("wy_arrow", "#d8ccff", 0.0, 0.4, emit="#b89cff", estr=0.6),
        led_v=pbr("wy_led_violet", "#c08cff", 0.0, 0.3, emit="#8f3dff", estr=9.0),
        led_m=pbr("wy_led_magenta", "#ff8ce0", 0.0, 0.3, emit="#ff1fb8", estr=9.0),
        eye=pbr("wy_eye", "#e0109a", 0.0, 0.2, emit="#ff1fb0", estr=14.0),
        maw=pbr("wy_maw", "#1a0410", 0.0, 0.6, emit="#b0106e", estr=0.9),
        heart=pbr("wy_heart", "#ff70d0", 0.0, 0.3, emit="#e01cff", estr=10.0),
        tip=pbr("wy_prop_tip", "#c0208a", 0.1, 0.35, emit="#ff1fb8", estr=0.4),
    )
    return M


# ================================================================= geometry helpers
def pchip(xs, ys, x):
    xs, ys = np.asarray(xs, float), np.asarray(ys, float)
    h = np.diff(xs)
    d = np.diff(ys) / h
    n = len(xs)
    m = np.zeros(n)
    for k in range(1, n - 1):
        if d[k - 1] * d[k] > 0:
            w1, w2 = 2 * h[k] + h[k - 1], h[k] + 2 * h[k - 1]
            m[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k])
    m[0], m[-1] = d[0], d[-1]
    x = np.clip(x, xs[0], xs[-1])
    k = np.clip(np.searchsorted(xs, x) - 1, 0, n - 2)
    t = (x - xs[k]) / h[k]
    return ((2 * t ** 3 - 3 * t ** 2 + 1) * ys[k] + (t ** 3 - 2 * t ** 2 + t) * h[k] * m[k]
            + (-2 * t ** 3 + 3 * t ** 2) * ys[k + 1] + (t ** 3 - t ** 2) * h[k] * m[k + 1])


class Shell:
    """Superellipse-section loft along x. Station = (x, half_width, z_centre, half_height_top, half_height_bottom,
    exponent, dorsal_ridge). Angle t: 0 = +y (left), pi/2 = top, pi = -y, -pi/2 = bottom."""

    def __init__(self, stations, nfine=1500):
        st = np.array(sorted(stations), float)
        self.x0, self.x1 = st[0, 0], st[-1, 0]
        self.xf = np.linspace(self.x0, self.x1, nfine)
        self.pf = np.stack([pchip(st[:, 0], st[:, i], self.xf) for i in range(1, st.shape[1])], 1)

    def par(self, x):
        return [float(np.interp(x, self.xf, self.pf[:, i])) for i in range(self.pf.shape[1])]

    def point(self, x, t):
        hw, zc, ht, hb, n, ridge = self.par(x)
        c, s = math.cos(t), math.sin(t)
        e = 2.0 / n
        y = hw * math.copysign(abs(c) ** e, c)
        z = zc + ht * abs(s) ** e + ridge * s ** 12 if s >= 0 else zc - hb * abs(s) ** e
        return np.array([x, y, z])

    def normal(self, x, t, e=2e-4):
        n = np.cross(self.point(x + e, t) - self.point(x - e, t), self.point(x, t + e) - self.point(x, t - e))
        n /= np.linalg.norm(n) + 1e-12
        p = self.point(x, t)
        if np.dot(n, [0.0, p[1], p[2] - self.par(x)[1]]) < 0:
            n = -n
        return n

    def t_at(self, x, y):
        """Angle on the top half where the section passes through lateral position y."""
        hw, zc, ht, hb, n, ridge = self.par(x)
        c = min(abs(y) / hw, 1.0) ** (n / 2)
        t = math.acos(c)
        return t if y >= 0 else math.pi - t

    def top_z(self, x, y=0.0):
        return float(self.point(x, self.t_at(x, y))[2])


def new_bm():
    bm = bmesh.new()
    return bm, bm.loops.layers.uv.new("UVMap")


def grid_obj(ctx, name, P, UV, material):
    """P (nu, nv, 3) points, UV (nu, nv, 2) -> quad grid object."""
    bm, uvl = new_bm()
    nu, nv = P.shape[:2]
    vs = [[bm.verts.new(tuple(P[i, j])) for j in range(nv)] for i in range(nu)]
    for i in range(nu - 1):
        for j in range(nv - 1):
            idx = ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1))
            f = bm.faces.new([vs[a][b] for a, b in idx])
            for lp, (a, b) in zip(f.loops, idx):
                lp[uvl].uv = tuple(UV[a, b])
    return C._obj_from_bm(name, bm, material, ctx=ctx)


def solidify(obj, th, offset=0.0):
    m = obj.modifiers.new("solid", "SOLIDIFY")
    m.thickness = th
    m.offset = offset
    m.use_even_offset = True
    m.use_quality_normals = True
    return C.apply_mods(obj)


def sweep(ctx, name, pts, radii, material, seg=12, closed=False, caps=True, bev=0.0):
    """Tube along a polyline with per-point radius (0 = pointed tip) and parallel-transport frames. UVs follow
    the tube (u around, v along) in metres / tile of the material."""
    pts = [Vector(p) for p in pts]
    n = len(pts)
    radii = [float(radii)] * n if np.isscalar(radii) else [float(r) for r in radii]
    tang = []
    for i in range(n):
        if closed:
            t = pts[(i + 1) % n] - pts[i - 1]
        elif i == 0:
            t = pts[1] - pts[0]
        elif i == n - 1:
            t = pts[-1] - pts[-2]
        else:
            t = pts[i + 1] - pts[i - 1]
        tang.append(t.normalized())
    ref = Vector((0, 0, 1)) if abs(tang[0].z) < 0.9 else Vector((1, 0, 0))
    nrm = (ref - tang[0] * ref.dot(tang[0])).normalized()
    frames = []
    for i in range(n):
        if i:
            nrm = tang[i - 1].rotation_difference(tang[i]) @ nrm
            nrm = (nrm - tang[i] * nrm.dot(tang[i])).normalized()
        frames.append((nrm.copy(), tang[i].cross(nrm)))
    tile = TILE.get(material.name, 0.05) if not isinstance(material, str) else 0.05
    s = [0.0]
    for i in range(1, n):
        s.append(s[-1] + (pts[i] - pts[i - 1]).length)
    bm, uvl = new_bm()
    rings = []
    for i in range(n):
        a, b = frames[i]
        r = radii[i]
        if r < 1e-6:
            rings.append([bm.verts.new(pts[i])])
        else:
            rings.append([bm.verts.new(pts[i] + r * (math.cos(2 * math.pi * j / seg) * a
                                                     + math.sin(2 * math.pi * j / seg) * b)) for j in range(seg)])
    pairs = list(zip(range(n - 1), range(1, n)))
    if closed:
        pairs.append((n - 1, 0))
    for i0, i1 in pairs:
        ra, rb = rings[i0], rings[i1]
        va, vb = s[i0] / tile, (s[i1] if i1 else s[-1] + (pts[0] - pts[-1]).length) / tile
        circ = 2 * math.pi * max(radii[i0], radii[i1], 1e-4) / tile
        for j in range(seg):
            j2 = (j + 1) % seg
            ua, ub = j / seg * circ, (j + 1) / seg * circ
            if len(ra) == 1 and len(rb) == 1:
                continue
            if len(ra) == 1:
                f = bm.faces.new((ra[0], rb[j], rb[j2]))
                uvs = ((ua, va), (ua, vb), (ub, vb))
            elif len(rb) == 1:
                f = bm.faces.new((ra[j], ra[j2], rb[0]))
                uvs = ((ua, va), (ub, va), (ua, vb))
            else:
                f = bm.faces.new((ra[j], ra[j2], rb[j2], rb[j]))
                uvs = ((ua, va), (ub, va), (ub, vb), (ua, vb))
            for lp, uv in zip(f.loops, uvs):
                lp[uvl].uv = uv
    if caps and not closed:
        for ring in (rings[0], rings[-1]):
            if len(ring) > 2:
                bm.faces.new(ring)
    o = C._obj_from_bm(name, bm, material, ctx=ctx)
    return C.bevel(o, bev, 1) if bev else o


def bez(p0, c, p1, n):
    t = np.linspace(0, 1, n)[:, None]
    p0, c, p1 = (np.asarray(q, float) for q in (p0, c, p1))
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * c + t ** 2 * p1


def ensure_uv(obj, tile=None):
    """Box-project UVs (metres / tile) on meshes made without them."""
    me = obj.data
    if me.uv_layers:
        return
    me.uv_layers.new(name="UVMap")
    if tile is None:
        tile = TILE.get(me.materials[0].name, 0.05) if me.materials else 0.05
    bm = bmesh.new()
    bm.from_mesh(me)
    uvl = bm.loops.layers.uv.active
    mw = obj.matrix_world
    r3 = mw.to_3x3()
    for f in bm.faces:
        nn = r3 @ f.normal
        ax = max(range(3), key=lambda i: abs(nn[i]))
        a, b = ((1, 2), (0, 2), (0, 1))[ax]
        for lp in f.loops:
            p = mw @ lp.vert.co
            lp[uvl].uv = (p[a] / tile, p[b] / tile)
    bm.to_mesh(me)
    bm.free()


def boolean(obj, cutter, op="DIFFERENCE"):
    m = obj.modifiers.new("bool", "BOOLEAN")
    m.operation = op
    m.object = cutter
    m.solver = "EXACT"
    try:
        m.material_mode = "TRANSFER"
    except Exception:
        pass
    C.apply_mods(obj)
    bpy.data.objects.remove(cutter)
    return obj


def axis_rot(d):
    return Vector((0, 0, 1)).rotation_difference(Vector(d).normalized()).to_euler()


def screw(ctx, parts, M, p, axis=(0, 0, 1), r=0.0021, h=0.0016, mat=None):
    p, ax = Vector(p), Vector(axis).normalized()
    parts.append(C.tube(ctx, "screw", p, p + ax * h, r, mat or M["steel"], seg=12))
    parts.append(C.tube(ctx, "socket", p + ax * (h - 0.0004), p + ax * (h + 0.0001), r * 0.5, M["dark"], seg=6))


# ================================================================= the body
STATIONS = [
    # x       hw      zc      ht      hb      n    ridge
    (-0.345, 0.0030, 0.0060, 0.0022, 0.0020, 2.2, 0.0000),
    (-0.320, 0.0062, 0.0062, 0.0042, 0.0038, 2.2, 0.0004),
    (-0.280, 0.0092, 0.0068, 0.0068, 0.0058, 2.3, 0.0010),
    (-0.230, 0.0132, 0.0078, 0.0100, 0.0080, 2.4, 0.0015),
    (-0.185, 0.0195, 0.0088, 0.0150, 0.0112, 2.5, 0.0020),
    (-0.150, 0.0320, 0.0080, 0.0240, 0.0180, 2.6, 0.0030),
    (-0.115, 0.0500, 0.0065, 0.0340, 0.0300, 2.8, 0.0040),
    (-0.060, 0.0610, 0.0060, 0.0400, 0.0380, 3.0, 0.0040),
    (0.000, 0.0640, 0.0060, 0.0420, 0.0400, 3.0, 0.0040),
    (0.060, 0.0580, 0.0060, 0.0400, 0.0360, 3.0, 0.0040),
    (0.105, 0.0440, 0.0050, 0.0330, 0.0280, 2.8, 0.0040),
    (0.140, 0.0270, 0.0040, 0.0220, 0.0180, 2.5, 0.0030),
    (0.175, 0.0200, 0.0040, 0.0170, 0.0150, 2.4, 0.0030),
    (0.205, 0.0215, 0.0045, 0.0175, 0.0150, 2.4, 0.0030),
    (0.232, 0.0305, 0.0065, 0.0215, 0.0195, 2.7, 0.0040),
    (0.258, 0.0300, 0.0045, 0.0205, 0.0205, 2.7, 0.0030),
    (0.282, 0.0235, 0.0015, 0.0170, 0.0190, 2.6, 0.0020),
    (0.300, 0.0175, -0.0005, 0.0130, 0.0160, 2.5, 0.0010),
    (0.312, 0.0105, -0.0015, 0.0085, 0.0105, 2.3, 0.0000),
    (0.318, 0.0040, -0.0025, 0.0040, 0.0050, 2.2, 0.0000),
]
X_TAIL, X_NECK = -0.150, 0.130


def loft_body(ctx, S, M, nx=130, nt=48):
    mats = [M["carbon"], M["scales"]]
    region = lambda x: 0 if X_TAIL <= x <= X_NECK else 1   # noqa: E731
    u = np.linspace(0, 1, nx)
    xs = S.x0 + (S.x1 - S.x0) * (0.5 - 0.5 * np.cos(np.pi * u))
    ts = [-math.pi / 2 + 2 * math.pi * j / nt for j in range(nt)]
    bm, uvl = new_bm()
    rings, perim = [], []
    for x in xs:
        pts = [S.point(x, t) for t in ts]
        rings.append([bm.verts.new(tuple(p)) for p in pts])
        acc = [0.0]
        for j in range(1, nt + 1):
            acc.append(acc[-1] + float(np.linalg.norm(pts[j % nt] - pts[j - 1])))
        perim.append(acc)
    for i in range(nx - 1):
        mi = region(0.5 * (xs[i] + xs[i + 1]))
        tile = TILE[mats[mi].name]
        for j in range(nt):
            j2 = (j + 1) % nt
            f = bm.faces.new((rings[i][j], rings[i][j2], rings[i + 1][j2], rings[i + 1][j]))
            f.material_index = mi
            uvs = ((perim[i][j], xs[i]), (perim[i][j + 1], xs[i]), (perim[i + 1][j + 1], xs[i + 1]),
                   (perim[i + 1][j], xs[i + 1]))
            for lp, (a, b) in zip(f.loops, uvs):
                lp[uvl].uv = (a / tile, b / tile)
    for ring, xe, xr in ((rings[0], xs[0] - 0.003, xs[0]), (rings[-1], xs[-1] + 0.003, xs[-1])):
        pv = bm.verts.new((xe, 0.0, S.par(xr)[1]))
        for j in range(nt):
            f = bm.faces.new((ring[j], ring[(j + 1) % nt], pv))
            f.material_index = 1
            for lp in f.loops:
                lp[uvl].uv = (lp.vert.co.y / 0.032, lp.vert.co.x / 0.032)
    o = C._obj_from_bm("shell", bm, mats[0], ctx=ctx)
    o.data.materials.append(mats[1])
    return o


def patch(ctx, S, name, xf, xb, t0, t1, off_f, off_b, th, material, nx=8, nt=8, shape=0.0, bev=0.0005):
    """Curved plate following the shell between x=xf (front) and x=xb (back), angles t0..t1, lifted off the
    surface by off_f..off_b (shingled scales lift at the back). shape > 0 rounds the back edge into a point."""
    P = np.zeros((nx, nt, 3))
    UV = np.zeros((nx, nt, 2))
    tile = TILE.get(material.name, 0.05)
    tm, hw = 0.5 * (t0 + t1), 0.5 * (t1 - t0)
    for i in range(nx):
        u = i / (nx - 1)
        x = xf + (xb - xf) * u
        w = 1.0
        if shape and u > 1 - shape:
            q = (u - (1 - shape)) / shape
            w = max(math.sqrt(max(1 - q * q, 0.0)), 0.12)
        off = off_f + (off_b - off_f) * u
        for j in range(nt):
            t = tm + hw * w * (2 * j / (nt - 1) - 1)
            p = S.point(x, t) + S.normal(x, t) * off
            P[i, j] = p
            UV[i, j] = (p[0] / tile, (t - tm) * 0.05 / tile)
    o = grid_obj(ctx, name, P, UV, material)
    solidify(o, th, offset=-1.0)
    return C.bevel(o, bev, 2, angle=50) if bev else o


def body_frame(ctx, M, parts):
    """Carbon centre plates, arm tubes, clamps, motor mounts, stators, LEDs, ESCs."""
    # X hub plates (2 mm carbon), lobes along the arm diagonals reach r = 0.112
    th = np.linspace(0, 2 * np.pi, 97)[:-1]
    R = 0.060 + 0.052 * np.maximum(0, np.cos(4 * (th - np.pi / 4))) ** 3
    for nm, z in (("plate_bottom", 0.002), ("plate_top", 0.020)):
        verts = [(r * math.cos(a), r * math.sin(a), z) for r, a in zip(R, th)]
        verts += [(r * math.cos(a), r * math.sin(a), z + 0.002) for r, a in zip(R, th)]
        n = len(th)
        faces = [list(range(n)), list(range(2 * n - 1, n - 1, -1))]
        faces += [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
        parts.append(C.mesh_from(ctx, nm, verts, faces, M["carbon"], bev=0.0006))
    for (rx, ry), spin in ROTORS:
        d = Vector((rx, ry, 0)).normalized()
        p = Vector((-d.y, d.x, 0))                       # left of the arm direction
        rot = (0, 0, math.atan2(d.y, d.x))
        front = rx > 0
        a0, a1 = d * 0.050 + Vector((0, 0, ARM_Z)), d * 0.252 + Vector((0, 0, ARM_Z))
        parts.append(sweep(ctx, "arm", [a0, a1], ARM_R, M["carbon"], seg=24))
        # root clamp between the plates, with bolts through the top plate
        c = d * 0.098 + Vector((0, 0, 0.012))
        parts.append(C.box(ctx, "root_clamp", (0.024, 0.026, 0.016), c, M["gun"], bev=0.0012, rot=rot))
        for sgn in (-1, 1):
            screw(ctx, parts, M, d * 0.098 + p * (0.0085 * sgn) + Vector((0, 0, 0.022)))
        # motor-mount clamp at the arm end + pad + stator
        mc = d * 0.240 + Vector((0, 0, ARM_Z))
        parts.append(C.box(ctx, "mount_clamp", (0.034, 0.024, 0.019), mc, M["anod"], bev=0.0018, rot=rot))
        for sgn in (-1, 1):
            q = mc + p * (0.0122 * sgn) + d * 0.008
            parts.append(C.tube(ctx, "clamp_bolt", q, q + p * (0.002 * sgn), 0.0024, M["steel"], seg=12))
            q2 = mc + p * (0.0122 * sgn) - d * 0.008
            parts.append(C.tube(ctx, "clamp_bolt", q2, q2 + p * (0.002 * sgn), 0.0024, M["steel"], seg=12))
        rc = Vector((rx, ry, 0))
        parts.append(C.cyl(ctx, "mount_pad", 0.0235, 0.005, rc + Vector((0, 0, 0.0235)), M["anod"], seg=40,
                           bev=0.0008))
        parts.append(C.lathe(ctx, "stator", [(0, 0.026), (0.0205, 0.026), (0.0212, 0.027), (0.0212, 0.0335),
                                             (0.0195, 0.0340), (0, 0.0340)], rc, M["gun"], seg=40))
        parts.append(C.cyl(ctx, "stator_ring", 0.0216, 0.0022, rc + Vector((0, 0, 0.0300)), M["dark"], seg=40))
        # LED pod under each motor: violet front, magenta rear (navigation colours, dragon style)
        led = M["led_v"] if front else M["led_m"]
        parts.append(C.lathe(ctx, "led_housing", [(0, 0.0035), (0.0115, 0.0035), (0.0105, -0.0025), (0, -0.0025)],
                             rc, M["gun"], seg=32))
        parts.append(C.lathe(ctx, "led_lens", [(0.0098, -0.0024), (0.0085, -0.0070), (0.0055, -0.0115),
                                               (0.0022, -0.0140), (0, -0.0146)], rc, led, seg=32))
        # LED strip on the free side of the arm (the wing membrane hangs on the other side)
        wing_side = p if ((rx > 0) == (ry > 0)) else -p
        free = -wing_side
        sc = d * 0.150 + free * 0.0062 + Vector((0, 0, ARM_Z - 0.004))
        parts.append(C.box(ctx, "arm_led", (0.115, 0.0016, 0.0034), sc, led, bev=0.0004, rot=rot))
        # ESC under the arm, offset to the free side, with a finned heat sink
        ec = d * 0.135 + free * 0.003 + Vector((0, 0, ARM_Z - ARM_R - 0.0035))
        parts.append(C.box(ctx, "esc", (0.034, 0.015, 0.006), ec, M["dark"], bev=0.0012, rot=rot))
        for k in range(5):
            fc = ec + d * (-0.012 + 0.006 * k) + Vector((0, 0, -0.004))
            parts.append(C.box(ctx, "esc_fin", (0.0012, 0.013, 0.003), fc, M["gun"], bev=0.0, rot=rot))
        # motor cable bundle from the ESC to the mount
        parts.append(sweep(ctx, "motor_cable", [ec + d * 0.017 + Vector((0, 0, 0.001)),
                                                d * 0.200 + free * 0.003 + Vector((0, 0, ARM_Z - ARM_R - 0.0012)),
                                                d * 0.226 + Vector((0, 0, ARM_Z - ARM_R + 0.001))],
                           0.0016, M["dark"], seg=8))


def body_shell_details(ctx, S, M, parts):
    sym = (1, -1)
    # collars at the carbon/scale seams
    for xc in (X_TAIL, X_NECK):
        loop = [S.point(xc, -math.pi / 2 + 2 * math.pi * j / 64) + S.normal(xc, -math.pi / 2 + 2 * math.pi * j / 64)
                * 0.0005 for j in range(64)]
        parts.append(sweep(ctx, "collar", loop, 0.0021, M["gun"], seg=10, closed=True))
    # flight-controller hatch on top + the glowing "heart" vent under slats
    parts.append(patch(ctx, S, "hatch", 0.084, -0.030, 1.00, math.pi - 1.00, 0.0012, 0.0012, 0.0022, M["gun"],
                       nx=14, nt=12, bev=0.0007))
    parts.append(patch(ctx, S, "heart_glow", 0.056, 0.006, 1.30, math.pi - 1.30, 0.0016, 0.0016, 0.0005,
                       M["heart"], nx=6, nt=6, bev=0.0))
    for k in range(6):
        xk = 0.052 - 0.0084 * k
        parts.append(patch(ctx, S, "heart_slat", xk + 0.0021, xk - 0.0021, 1.26, math.pi - 1.26, 0.0046, 0.0046,
                           0.0026, M["dark"], nx=3, nt=6, bev=0.0004))
    for xs_, ys_ in ((0.074, 0.030), (0.074, -0.030), (-0.022, 0.030), (-0.022, -0.030)):
        z = S.top_z(xs_, ys_)
        n = S.normal(xs_, S.t_at(xs_, ys_))
        screw(ctx, parts, M, Vector((xs_, ys_, z)) + Vector(n) * 0.0028, axis=n, r=0.0018, h=0.0012)
    # safety button + buzzer (Pixhawk-style hardware)
    for (xs_, ys_), m_ring in (((-0.014, -0.020), M["led_m"]),):
        z = S.top_z(xs_, ys_) + 0.0030
        parts.append(C.cyl(ctx, "safety_btn", 0.0044, 0.0026, (xs_, ys_, z), M["dark"], seg=24, bev=0.0006))
        parts.append(C.torus(ctx, "safety_ring", 0.0046, 0.0007, (xs_, ys_, z + 0.0006), m_ring, major=24, minor=6))
    z = S.top_z(-0.014, 0.020) + 0.0028
    parts.append(C.cyl(ctx, "buzzer", 0.0050, 0.0030, (-0.014, 0.020, z), M["gloss"], seg=24, bev=0.0006))
    parts.append(C.cyl(ctx, "buzzer_hole", 0.0012, 0.0008, (-0.014, 0.020, z + 0.0015), M["dark"], seg=12))
    # shingled scale plates on the upper flanks
    for sy in sym:
        rows = [(0.30, 0.62, 0.096, -0.128, 0.0), (0.62, 0.96, 0.085, -0.115, 0.5)]
        for t0, t1, xa, xb, stag in rows:
            L, step = 0.028, 0.0235
            x = xa - stag * step
            while x - L >= xb - 0.001:
                if not (t0 >= 0.62 and -0.034 < x - L / 2 < 0.090):     # keep the hatch area clear
                    tt0, tt1 = (t0, t1) if sy > 0 else (math.pi - t1, math.pi - t0)
                    parts.append(patch(ctx, S, "scale_plate", x, x - L, tt0, tt1, 0.0007, 0.0024, 0.0016,
                                       M["anod"], nx=7, nt=6, shape=0.45, bev=0.0004))
                x -= step
    # fin root plate (the tail fin is mounted on it)
    parts.append(patch(ctx, S, "fin_root", -0.024, -0.190, math.pi / 2 - 0.22, math.pi / 2 + 0.22, 0.0008, 0.0008,
                       0.0022, M["gun"], nx=16, nt=6, bev=0.0006))
    # dorsal spines: neck, shoulders, tail
    spines = [(0.214, 0.006), (0.196, 0.008), (0.178, 0.010), (0.158, 0.011), (0.118, 0.010), (0.100, 0.008),
              (-0.196, 0.011), (-0.218, 0.010), (-0.240, 0.009), (-0.262, 0.0075), (-0.284, 0.006), (-0.304, 0.0045)]
    for xs_, hgt in spines:
        parts.append(spine(ctx, S, xs_, hgt, M["horn"]))


def spine(ctx, S, x, h, material):
    L = h * 1.5
    zb = S.top_z(x) - 0.0015
    zb2 = S.top_z(x - L) - 0.0015
    tb = 0.0011 + 0.08 * h
    apex = (x - 0.85 * L, 0.0, max(zb, zb2) + h)
    verts = [(x, -tb, zb), (x - L, -tb, zb2), (x - L, tb, zb2), (x, tb, zb),
             (apex[0], -0.0003, apex[2]), (apex[0], 0.0003, apex[2])]
    faces = [(0, 3, 2, 1), (0, 1, 4), (3, 5, 2), (0, 4, 5, 3), (1, 2, 5, 4)]
    return C.mesh_from(ctx, "spine", verts, faces, material, bev=0.0003)


def body_head(ctx, S, M, shell, parts):
    """Mouth (camera in the maw), fangs, eyes, brows, horn antennas, nostrils."""
    cutter = C.box(ctx, "mouth_cut", (0.060, 0.025, 0.0105), (0.3135, 0.0, -0.0058), M["maw"], bev=0.0015)
    boolean(shell, cutter)
    # camera: barrel + glass + violet iris ring, sitting in the maw
    parts.append(C.cyl(ctx, "cam_barrel", 0.0047, 0.016, (0.2915, 0.0, -0.0058), M["gun"], axis="x", seg=24,
                       bev=0.0005))
    parts.append(C.cyl(ctx, "cam_bezel", 0.0051, 0.0024, (0.2985, 0.0, -0.0058), M["dark"], axis="x", seg=24,
                       bev=0.0004))
    parts.append(C.sphere(ctx, "cam_glass", 0.0036, (0.2995, 0.0, -0.0058), M["glass"], scale=(0.55, 1, 1), seg=20,
                          rings=10))
    t = C.torus(ctx, "cam_iris", 0.0041, 0.0005, (0.2999, 0.0, -0.0058), M["led_v"], major=24, minor=6, axis="x")
    parts.append(t)
    # fangs: upper pair at the mouth corners + small teeth, lower pair pointing up
    for sy in (1, -1):
        parts.append(sweep(ctx, "fang", [(0.3020, 0.0105 * sy, 0.0004), (0.3030, 0.0102 * sy, -0.0035),
                                         (0.3045, 0.0094 * sy, -0.0072)], [0.0017, 0.0011, 0.0], M["bone"], seg=10))
        for xt in (0.2880, 0.2950):
            parts.append(sweep(ctx, "tooth", [(xt, 0.0118 * sy, 0.0002), (xt + 0.0005, 0.0116 * sy, -0.0030)],
                               [0.0010, 0.0], M["bone"], seg=8))
        parts.append(sweep(ctx, "fang_low", [(0.3005, 0.0085 * sy, -0.0112), (0.3015, 0.0082 * sy, -0.0080),
                                             (0.3025, 0.0078 * sy, -0.0058)], [0.0013, 0.0008, 0.0], M["bone"], seg=10))
        # eye: emissive almond on the skull side, slit pupil, brow ridge above
        xe, te = 0.249, (0.50 if sy > 0 else math.pi - 0.50)
        pe = S.point(xe, te)
        ne = S.normal(xe, te)
        tx = np.array([1.0, 0.0, 0.0])
        bx = np.cross(ne, tx)
        bx /= np.linalg.norm(bx)
        tx = np.cross(bx, ne)
        rotm = Matrix(((tx[0], bx[0], ne[0]), (tx[1], bx[1], ne[1]), (tx[2], bx[2], ne[2])))
        eye = C.sphere(ctx, "eye", 1.0, tuple(pe + ne * 0.0004), M["eye"], scale=(0.0112, 0.0052, 0.0028), seg=20,
                       rings=10)
        eye.rotation_euler = rotm.to_euler()
        parts.append(eye)
        pupil = C.sphere(ctx, "pupil", 1.0, tuple(pe + ne * 0.0010), M["dark"], scale=(0.0011, 0.0041, 0.0026), seg=16,
                         rings=8)
        pupil.rotation_euler = rotm.to_euler()
        parts.append(pupil)
        tb0, tb1 = (0.62, 1.02) if sy > 0 else (math.pi - 1.02, math.pi - 0.62)
        parts.append(patch(ctx, S, "brow", 0.266, 0.230, tb0, tb1, 0.0016, 0.0030, 0.0030, M["gun"], nx=7, nt=5,
                           shape=0.3, bev=0.0006))
        # swept-back horns = RC receiver antennas (tip glows)
        # (kept inside |y| < 0.027 so they clear the front prop discs by > 4 mm)
        hp = [(0.244, 0.0135 * sy, 0.0215), (0.230, 0.0170 * sy, 0.0340), (0.214, 0.0200 * sy, 0.0465),
              (0.194, 0.0225 * sy, 0.0575), (0.170, 0.0240 * sy, 0.0660), (0.146, 0.0245 * sy, 0.0715),
              (0.124, 0.0240 * sy, 0.0745)]
        parts.append(sweep(ctx, "horn", hp, [0.0062, 0.0055, 0.0046, 0.0037, 0.0027, 0.0016, 0.0006], M["ivory"],
                           seg=16))
        for k, (xr, rr) in enumerate(((0.214, 0.0058), (0.190, 0.0049), (0.166, 0.0040))):     # growth rings
            j = 1 + k
            q = np.array(hp[j]) * 0.5 + np.array(hp[j + 1]) * 0.5
            d = np.array(hp[j + 1]) - np.array(hp[j])
            ring = C.torus(ctx, "horn_ring", rr * 0.92, 0.0007, tuple(q), M["gun"], major=20, minor=5)
            ring.rotation_euler = axis_rot(tuple(d))
            parts.append(ring)
        parts.append(C.sphere(ctx, "horn_tip", 0.0014, hp[-1], M["led_m"], seg=10, rings=6))
        # cheek spikes swept back from the jaw hinge
        pb = S.point(0.268, 0.22 if sy > 0 else math.pi - 0.22) - np.array([0.0, 0.004 * sy, 0.0])
        for dz, ln, r0 in ((0.0, 1.0, 0.0032), (-0.007, 0.7, 0.0024)):
            b = pb + np.array([0.0, 0.0, dz])
            parts.append(sweep(ctx, "cheek_spike", [tuple(b), tuple(b + ln * np.array([-0.020, 0.0085 * sy, 0.004])),
                                                     tuple(b + ln * np.array([-0.042, 0.0125 * sy, 0.010]))],
                               [r0, r0 * 0.62, 0.0003], M["ivory"], seg=10))
        # nostrils
        pn = S.point(0.306, 1.15 if sy > 0 else math.pi - 1.15)
        parts.append(C.sphere(ctx, "nostril", 1.0, tuple(pn), M["maw"], scale=(0.0028, 0.0012, 0.0010), seg=12,
                              rings=6))
    # jaw-line plates (gunmetal) along the upper jaw
    for sy in (1, -1):
        tj0, tj1 = (0.02, 0.40) if sy > 0 else (math.pi - 0.40, math.pi - 0.02)
        parts.append(patch(ctx, S, "jaw_plate", 0.296, 0.262, tj0, tj1, 0.0010, 0.0022, 0.0020, M["gun"], nx=7,
                           nt=5, shape=0.35, bev=0.0005))


def body_wings(ctx, S, M, parts):
    """Bat-wing membranes between the front and rear arm on each side: arm spars, one finger spar, scalloped
    trailing edges, billowing below the arms (well under the prop discs)."""
    tile = TILE[M["membrane"].name]
    for sy in (1, -1):
        d_f, p_f = np.array([0.7071, 0.7071 * sy]), np.array([-0.7071, 0.7071 * sy])
        d_r, p_r = np.array([-0.7071, 0.7071 * sy]), np.array([0.7071, 0.7071 * sy])
        off = 0.0042
        Pf, Pr = d_f * 0.214 + p_f * off, d_r * 0.214 + p_r * off
        If, Ir = d_f * 0.072 + p_f * off, d_r * 0.072 + p_r * off
        F = np.array([0.0, 0.200 * sy])

        def ctrl(a, b):
            t = b - a
            n = np.array([t[1], -t[0]]) / np.linalg.norm(t)
            if np.dot(n, (a + b) / 2) < 0:
                n = -n
            return (a + b) / 2 - 0.042 * n
        ns, nr = 22, 10
        bnd = np.vstack([bez(Pf, ctrl(Pf, F), F, ns + 1), bez(F, ctrl(F, Pr), Pr, ns + 1)[1:]])
        nb = len(bnd)
        sv = np.linspace(0, 1, nb)
        inner = If[None] * (1 - sv[:, None]) + Ir[None] * sv[:, None]
        P = np.zeros((nb, nr + 1, 3))
        UV = np.zeros((nb, nr + 1, 2))
        for i in range(nb):
            sl = i / ns if i <= ns else (i - ns) / ns
            for k in range(nr + 1):
                r = k / nr
                xy = inner[i] * (1 - r) + bnd[i] * r
                z = 0.0065 - 0.0080 * math.sin(math.pi * sl) * r ** 0.8
                P[i, k] = (xy[0], xy[1], z)
                UV[i, k] = (xy[0] / tile, xy[1] / tile)
        w = grid_obj(ctx, "wing", P, UV, M["membrane"])
        parts.append(solidify(w, 0.0013))
        parts.append(sweep(ctx, "wing_edge", [tuple(P[i, nr]) for i in range(nb)], 0.0011, M["gun"], seg=8))
        fp = [(0.0, 0.056 * sy, 0.0075), (0.004, 0.100 * sy, 0.0072), (0.003, 0.150 * sy, 0.0068),
              (0.0, 0.200 * sy, 0.0065)]
        parts.append(sweep(ctx, "finger", fp, [0.0044, 0.0036, 0.0027, 0.0019], M["gun"], seg=14))
        parts.append(C.sphere(ctx, "finger_knuckle", 0.0060, fp[0], M["gun"], seg=16, rings=8))
        parts.append(C.sphere(ctx, "finger_joint", 0.0026, fp[-1], M["gun"], seg=12, rings=6))
        parts.append(sweep(ctx, "wing_claw", [(0.0, 0.200 * sy, 0.0065), (0.001, 0.209 * sy, 0.0045),
                                              (-0.002, 0.216 * sy, -0.0015), (-0.006, 0.2185 * sy, -0.0080)],
                           [0.0022, 0.0017, 0.0010, 0.0], M["horn"], seg=10))


def body_tail_fin(ctx, S, M, parts):
    """Swept dorsal fin with the GPS mast as its leading spar; GPS puck with arrow and status ring."""
    tile = TILE[M["membrane"].name]
    xa, xt = -0.030, -0.182
    A = np.array([xa, S.top_z(xa) - 0.003])
    Mt = np.array([-0.100, 0.1475])
    S1 = np.array([-0.142, 0.094])
    T = np.array([xt, S.top_z(xt) - 0.003])
    cen = np.array([-0.11, 0.06])

    def ctrl(a, b, k):
        t = b - a
        n = np.array([t[1], -t[0]]) / np.linalg.norm(t)
        if np.dot(n, (a + b) / 2 - cen) < 0:
            n = -n
        return (a + b) / 2 - k * n
    lead = bez(A, np.array([-0.036, 0.112]), Mt, 18)
    tr = np.vstack([bez(Mt, ctrl(Mt, S1, 0.016), S1, 12)[1:], bez(S1, ctrl(S1, T, 0.020), T, 12)[1:]])
    bnd = np.vstack([lead, tr])
    nb, nr = len(bnd), 10
    sv = np.linspace(0, 1, nb)
    P = np.zeros((nb, nr + 1, 3))
    UV = np.zeros((nb, nr + 1, 2))
    for i in range(nb):
        xr = xa + (xt - xa) * sv[i]
        root = np.array([xr, S.top_z(xr) - 0.003])
        for k in range(nr + 1):
            q = root * (1 - k / nr) + bnd[i] * (k / nr)
            P[i, k] = (q[0], 0.0, q[1])
            UV[i, k] = (q[0] / tile, q[1] / tile)
    fin = grid_obj(ctx, "fin", P, UV, M["membrane"])
    parts.append(solidify(fin, 0.0024))
    parts.append(sweep(ctx, "fin_lead", [(p[0], 0.0, p[1]) for p in lead], np.linspace(0.0034, 0.0026, len(lead)),
                       M["gun"], seg=12))
    parts.append(sweep(ctx, "fin_edge", [(p[0], 0.0, p[1]) for p in tr], 0.0011, M["gun"], seg=8))
    for xr, tip in ((-0.122, S1), (-0.158, bez(S1, ctrl(S1, T, 0.020), T, 12)[5])):
        parts.append(sweep(ctx, "fin_rib", [(xr, 0.0, S.top_z(xr) - 0.002), (tip[0], 0.0, tip[1])], [0.0021, 0.0012],
                           M["gun"], seg=8))
    # GPS mast (carbon) + clamp + puck
    xm = -0.100
    parts.append(sweep(ctx, "gps_mast", [(xm, 0.0, S.top_z(xm) - 0.004), (xm, 0.0, 0.1500)], 0.0046,
                       M["carbon"], seg=16))
    parts.append(C.cyl(ctx, "mast_clamp", 0.0070, 0.0060, (xm, 0.0, 0.1480), M["gun"], seg=24, bev=0.0008))
    parts.append(C.cyl(ctx, "mast_base", 0.0080, 0.0050, (xm, 0.0, S.top_z(xm) + 0.0005), M["gun"], seg=24,
                       bev=0.0008))
    parts.append(C.lathe(ctx, "gps_puck", [(0, 0.1508), (0.0270, 0.1508), (0.0305, 0.1535), (0.0305, 0.1585),
                                           (0.0282, 0.1625), (0.0200, 0.1656), (0.0100, 0.1666), (0, 0.1668)],
                         (xm, 0.0, 0.0), M["gloss"], seg=48))
    parts.append(C.torus(ctx, "gps_ring", 0.0307, 0.0009, (xm, 0.0, 0.1560), M["led_m"], major=48, minor=6))
    arrow = [(0.0150, 0.0), (0.0030, 0.0075), (0.0030, 0.0028), (-0.0110, 0.0028), (-0.0110, -0.0028),
             (0.0030, -0.0028), (0.0030, -0.0075)]
    z0, z1 = 0.1662, 0.1670
    verts = [(xm + a, b, z0) for a, b in arrow] + [(xm + a, b, z1) for a, b in arrow]
    n = len(arrow)
    faces = [list(range(n)), list(range(2 * n - 1, n - 1, -1))] + [(i, (i + 1) % n, n + (i + 1) % n, n + i)
                                                                   for i in range(n)]
    parts.append(C.mesh_from(ctx, "gps_arrow", verts, faces, M["arrow"]))
    # tail spade: horizontal arrowhead of membrane with a gunmetal rim
    sp = [(-0.300, 0.0040), (-0.322, 0.0215), (-0.333, 0.0245), (-0.352, 0.0130), (-0.374, 0.0)]
    outline = [np.array(p) for p in sp] + [np.array((p[0], -p[1])) for p in reversed(sp[:-1])]
    cen2 = np.array([-0.335, 0.0])
    nr = 6
    P = np.zeros((len(outline) + 1, nr + 1, 3))
    UV = np.zeros_like(P[..., :2])
    loop = outline + [outline[0]]
    for i, q in enumerate(loop):
        for k in range(nr + 1):
            xy = cen2 * (1 - k / nr) + q * (k / nr)
            P[i, k] = (xy[0], xy[1], 0.0062 + 0.004 * (xy[0] + 0.30))
            UV[i, k] = xy / tile
    spade = grid_obj(ctx, "spade", P, UV, M["membrane"])
    bm = bmesh.new()
    bm.from_mesh(spade.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
    bm.to_mesh(spade.data)
    bm.free()
    parts.append(solidify(spade, 0.0020))
    parts.append(sweep(ctx, "spade_rim", [tuple(P[i, nr]) for i in range(len(loop))], 0.0012, M["gun"], seg=8))


def body_belly(ctx, S, M, parts):
    """Battery with straps and lead, landing-gear brackets, struts, T-joints, skids with talon feet."""
    zt = -0.0335
    bat = (0.150, 0.062, 0.034)
    bc = Vector((-0.004, 0.0, zt - bat[2] / 2))
    parts.append(C.box(ctx, "battery", bat, bc, M["battery"], bev=0.004, seg=3))
    for sy in (1, -1):
        parts.append(C.box(ctx, "battery_label", (0.110, 0.0008, 0.016), bc + Vector((0.004, sy * 0.0312, 0.002)),
                           M["label"], bev=0.0))
    for xs_ in (-0.048, 0.040):
        parts.append(C.box(ctx, "strap_bottom", (0.020, 0.066, 0.0016), (xs_, 0, zt - bat[2] - 0.0008), M["strap"],
                           bev=0.0004))
        for sy in (1, -1):
            parts.append(C.box(ctx, "strap_side", (0.020, 0.0016, bat[2] + 0.004), (xs_, sy * 0.0318, bc.z),
                               M["strap"], bev=0.0004))
        parts.append(C.box(ctx, "buckle", (0.024, 0.016, 0.003), (xs_, -0.012, zt - bat[2] - 0.0022), M["gun"],
                           bev=0.0006))
    xc = Vector((-0.083, 0.0, bc.z))
    parts.append(C.box(ctx, "xt60", (0.010, 0.016, 0.008), xc, M["dark"], bev=0.0008))
    for sy, m in ((1, M["red"]), (-1, M["dark"])):
        parts.append(sweep(ctx, "bat_lead", [xc + Vector((0.004, 0.004 * sy, 0.004)),
                                             Vector((-0.090, 0.005 * sy, zt + 0.004)),
                                             Vector((-0.078, 0.006 * sy, zt + 0.012))], 0.0018, m, seg=8))
    for sy in (1, -1):
        roll = 0.35 * sy
        top = Vector((0.0, LEG_TOP[0] * sy, LEG_TOP[1]))
        bot = Vector((0.0, LEG_BOT[0] * sy, LEG_BOT[1]))
        ax = (bot - top).normalized()
        parts.append(C.box(ctx, "leg_bracket", (0.030, 0.022, 0.016), top + ax * 0.004, M["gun"], bev=0.0015,
                           rot=(roll, 0, 0)))
        parts.append(C.tube(ctx, "leg_pivot", top + Vector((0.0, 0, 0.0)) + Vector((-0.017, 0, 0)),
                            top + Vector((0.017, 0, 0)), 0.0032, M["steel"], seg=16))
        parts.append(sweep(ctx, "leg_strut", [top + ax * 0.004, bot - ax * 0.008], 0.0075, M["carbon"], seg=20))
        mid = top + (bot - top) * 0.55
        parts.append(sweep(ctx, "leg_knee", [mid - ax * 0.006, mid + ax * 0.006], 0.0098, M["anod"], seg=20,
                           bev=0.0006))
        parts.append(sweep(ctx, "knee_spur", [mid + Vector((-0.004, 0, 0)), mid + Vector((-0.016, 0, -0.006)),
                                              mid + Vector((-0.024, 0, -0.016))], [0.0034, 0.0020, 0.0],
                           M["horn"], seg=10))
        sk = Vector((0.0, SKID_Y * sy, SKID_Z))
        up = lambda r: Vector((0, 0, r - SKID_R))   # noqa: E731  (keep the bottom flush with the skid collision box)
        parts.append(sweep(ctx, "tee", [sk + up(0.0105) - Vector((0.014, 0, 0)), sk + up(0.0105) + Vector((0.014, 0, 0))], 0.0105, M["gun"],
                           seg=20, bev=0.0008))
        parts.append(sweep(ctx, "tee_socket", [sk, sk - ax * 0.024], 0.0098, M["gun"], seg=20))
        parts.append(sweep(ctx, "skid", [sk - Vector((0.118, 0, 0)), sk + Vector((0.118, 0, 0))], SKID_R,
                           M["carbon"], seg=20))
        for xs_ in (-0.070, 0.070):
            parts.append(sweep(ctx, "skid_grip", [sk + up(SKID_R + 0.0012) + Vector((xs_ - 0.016, 0, 0)),
                                                   sk + up(SKID_R + 0.0012) + Vector((xs_ + 0.016, 0, 0))],
                               SKID_R + 0.0012, M["rubber"], seg=20, bev=0.0006))
        for end, dirs, scale in ((1, (-28, 0, 28), (0.8, 1.0, 0.8)), (-1, (-22, 22), (0.75, 0.75))):
            k = sk + Vector((0.118 * end, 0, 0))
            parts.append(C.sphere(ctx, "foot_knuckle", 0.0098, k + up(0.0098), M["gun"], seg=20, rings=10))
            for a, sc in zip(dirs, scale):
                dv = Vector((math.cos(math.radians(a)) * end, math.sin(math.radians(a)), 0))
                pts = [k + dv * 0.004 * sc + Vector((0, 0, 0.002)), k + dv * 0.016 * sc + Vector((0, 0, 0.0005)),
                       k + dv * 0.025 * sc + Vector((0, 0, -0.0022)), k + dv * 0.030 * sc + Vector((0, 0, -0.0055))]
                parts.append(sweep(ctx, "talon", pts, [0.0040, 0.0033, 0.0020, 0.0], M["horn"], seg=12))


def build_body(M):
    ctx = C.Ctx("wyvern_body")
    S = Shell(STATIONS)
    parts = []
    shell = loft_body(ctx, S, M)
    body_head(ctx, S, M, shell, parts)
    parts.append(shell)
    body_frame(ctx, M, parts)
    body_shell_details(ctx, S, M, parts)
    body_wings(ctx, S, M, parts)
    body_tail_fin(ctx, S, M, parts)
    body_belly(ctx, S, M, parts)
    for o in parts:
        ensure_uv(o)
    return C.join(ctx, parts, "wyvern_body", smooth_deg=40), S


# ================================================================= rotors (rotor-link frame)
BLADE = dict(  # r/R, chord (m), pitch (deg), thickness (m)
    rn=[0.07, 0.15, 0.25, 0.35, 0.50, 0.65, 0.80, 0.90, 0.97, 1.00],
    c=[0.0130, 0.0185, 0.0235, 0.0245, 0.0228, 0.0198, 0.0160, 0.0128, 0.0090, 0.0040],
    beta=[30, 29, 25, 21, 17, 14, 12, 11, 10, 10],
    t=[0.0050, 0.0040, 0.0030, 0.0025, 0.0020, 0.0017, 0.0014, 0.0012, 0.0010, 0.0008],
)


def blade_mesh(ctx, M, sign, z0=-0.0005, nrad=34, nch=14):
    """One blade along +x. sign +1: ccw rotor (leading edge on +y, raised); -1 mirrors it for cw."""
    rn = 0.07 + (1.0 - 0.07) * (0.5 - 0.5 * np.cos(np.pi * np.linspace(0, 1, nrad)))
    rn[-1] = 0.9999
    c = pchip(BLADE["rn"], BLADE["c"], rn)
    beta = np.radians(pchip(BLADE["rn"], BLADE["beta"], rn))
    th = pchip(BLADE["rn"], BLADE["t"], rn)
    xi = 0.5 - 0.5 * np.cos(np.pi * np.linspace(0, 1, nch))
    yt = 5 * (0.2969 * np.sqrt(xi) - 0.1260 * xi - 0.3516 * xi ** 2 + 0.2843 * xi ** 3 - 0.1036 * xi ** 4)
    tile = TILE[M["carbon"].name]
    bm, uvl = new_bm()
    rings, uvr = [], []
    for k in range(nrad):
        r = rn[k] * PROP_R
        yle = 0.36 * c[k] - 0.004 * rn[k] ** 2
        cam = 0.045 * c[k] * 4 * xi * (1 - xi)
        up = [(yle - x * c[k], cam[i] + th[k] * yt[i]) for i, x in enumerate(xi)]
        lo = [(yle - x * c[k], cam[i] - 0.55 * th[k] * yt[i]) for i, x in enumerate(xi)][-2:0:-1]
        sec = up + lo
        ring, uvs = [], []
        acc = 0.0
        prev = None
        for (yc, zc_) in sec:
            yb = yc * math.cos(beta[k]) - zc_ * math.sin(beta[k])
            zb = yc * math.sin(beta[k]) + zc_ * math.cos(beta[k])
            ring.append(bm.verts.new((r, sign * yb, z0 + zb)))
            if prev is not None:
                acc += math.hypot(yc - prev[0], zc_ - prev[1])
            prev = (yc, zc_)
            uvs.append((acc / tile, r / tile))
        rings.append(ring)
        uvr.append(uvs)
    m = len(rings[0])
    for k in range(nrad - 1):
        tipz = rn[k] > 0.88
        for j in range(m):
            j2 = (j + 1) % m
            vs = (rings[k][j], rings[k][j2], rings[k + 1][j2], rings[k + 1][j])
            f = bm.faces.new(vs if sign > 0 else vs[::-1])
            f.material_index = 1 if tipz else 0
            u2 = uvr[k][j2][0] if j2 else uvr[k][-1][0] + 0.001
            uvs = ((uvr[k][j][0], uvr[k][j][1]), (u2, uvr[k][j][1]), (u2, uvr[k + 1][j][1]),
                   (uvr[k + 1][j][0], uvr[k + 1][j][1]))
            for lp, uv in zip(f.loops, uvs if sign > 0 else uvs[::-1]):
                lp[uvl].uv = uv
    for ring, mi in ((rings[0], 0), (rings[-1], 1)):
        f = bm.faces.new(ring)
        f.material_index = mi
    o = C._obj_from_bm("blade", bm, M["carbon"], ctx=ctx)
    o.data.materials.append(M["tip"])
    return o


def build_rotor(M, spin):
    sign = 1 if spin == "ccw" else -1
    ctx = C.Ctx("wyvern_rotor_" + spin)
    parts = []
    parts.append(C.lathe(ctx, "bell", [(0, -0.0262), (0.0205, -0.0262), (0.0216, -0.0252), (0.0219, -0.0095),
                                       (0.0207, -0.0068), (0.0130, -0.0062), (0, -0.0062)], (0, 0, 0), M["anod"],
                         seg=48))
    parts.append(C.cyl(ctx, "bell_band", 0.0222, 0.0042, (0, 0, -0.0195), M["gun"], seg=48))
    for k in range(12):
        a = 2 * math.pi * k / 12
        parts.append(C.box(ctx, "bell_vent", (0.0030, 0.0016, 0.0046), (0.0221 * math.cos(a), 0.0221 * math.sin(a),
                                                                        -0.0195), M["dark"], bev=0.0, rot=(0, 0, a)))
    for k in range(3):
        a = 2 * math.pi * k / 3 + 0.4
        parts.append(C.cyl(ctx, "bell_mark", 0.0022, 0.0006, (0.0165 * math.cos(a), 0.0165 * math.sin(a), -0.0060),
                           M["anodm"], seg=12))
    parts.append(C.lathe(ctx, "hub", [(0, -0.0062), (0.0108, -0.0062), (0.0112, -0.0050), (0.0112, 0.0022),
                                      (0.0100, 0.0034), (0, 0.0034)], (0, 0, 0), M["dark"], seg=32))
    for k in range(2):
        b = blade_mesh(ctx, M, sign)
        b.rotation_euler = (0, 0, math.pi * k)
        parts.append(b)
    parts.append(C.cyl(ctx, "nut_hex", 0.0062, 0.0042, (0, 0, 0.0054), M["anod"] if sign > 0 else M["anodm"], seg=6,
                       bev=0.0004))
    parts.append(C.lathe(ctx, "nut_dome", [(0, 0.0074), (0.0046, 0.0074), (0.0042, 0.0094), (0.0026, 0.0108),
                                           (0, 0.0112)], (0, 0, 0), M["anod"] if sign > 0 else M["anodm"], seg=24))
    for o in parts:
        ensure_uv(o)
    return C.join(ctx, parts, "wyvern_rotor_" + spin, smooth_deg=40)


# ================================================================= export
def export_glb(obj, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not any(m.type == "TRIANGULATE" for m in obj.modifiers):
        obj.modifiers.new("tri", "TRIANGULATE").keep_custom_normals = True
    C._select_only([obj])
    kw = dict(filepath=str(path), export_format="GLB", use_selection=True, export_yup=False, export_apply=True,
              export_texcoords=True, export_normals=True, export_tangents=True, export_materials="EXPORT",
              export_image_format="AUTO", export_cameras=False, export_lights=False, export_animations=False,
              export_extras=False)
    try:
        bpy.ops.export_scene.gltf(**kw)
    except TypeError as e:
        print("gltf kwargs fallback:", e)
        bpy.ops.export_scene.gltf(filepath=str(path), export_format="GLB", use_selection=True, export_yup=False)
    print("wrote %s (%.0f kB, %d tris)" % (path, path.stat().st_size / 1024, C.tri_count(obj)))


def bbox(obj):
    bpy.context.view_layer.update()
    vs = np.array([obj.matrix_world @ v.co for v in obj.data.vertices])
    return vs.min(0), vs.max(0)


# ================================================================= renders
def world_bg(color, strength, fog=0.0):
    w = bpy.data.worlds.new("wy_world")
    w.use_nodes = True
    nt = w.node_tree
    nt.nodes["Background"].inputs["Color"].default_value = list(C._rgb(color)) + [1]
    nt.nodes["Background"].inputs["Strength"].default_value = strength
    if fog > 0:
        vs = nt.nodes.new("ShaderNodeVolumeScatter")
        vs.inputs["Density"].default_value = fog
        vs.inputs["Anisotropy"].default_value = 0.45
        vs.inputs["Color"].default_value = list(C._rgb("#c8c2e8")) + [1]
        out = [n for n in nt.nodes if n.type == "OUTPUT_WORLD"][0]
        nt.links.new(vs.outputs["Volume"], out.inputs["Volume"])
    bpy.context.scene.world = w


def light(name, kind, loc, color, energy, size, target, spot=None):
    ld = bpy.data.lights.new(name, kind)
    ld.color = C._rgb(color)
    ld.energy = energy
    if kind == "AREA":
        ld.shape = "DISK"
        ld.size = size
    elif kind == "SPOT":
        ld.spot_size = math.radians(spot or 40)
        ld.spot_blend = 0.6
        ld.shadow_soft_size = size
    else:
        ld.shadow_soft_size = size
    ob = bpy.data.objects.new(name, ld)
    ob.location = loc
    bpy.context.scene.collection.objects.link(ob)
    if target is not None:
        c = ob.constraints.new("TRACK_TO")
        c.target = target
        c.track_axis = "TRACK_NEGATIVE_Z"
        c.up_axis = "UP_Y"
    return ob


def eevee(samples, raytrace=True):
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_EEVEE"
    ev = sc.eevee
    ev.taa_render_samples = samples
    for attr, val in (("use_raytracing", raytrace), ("use_shadows", True), ("use_gtao", True),
                      ("use_volumetric_shadows", True), ("volumetric_end", 14.0), ("volumetric_tile_size", "4"),
                      ("ray_tracing_method", "SCREEN"), ("use_fast_gi", True)):
        if hasattr(ev, attr):
            try:
                setattr(ev, attr, val)
            except Exception:
                pass
    sc.view_settings.view_transform = "AgX"
    try:
        sc.view_settings.look = "AgX - Medium High Contrast"
    except Exception:
        pass


def bloom(strength=0.5):
    sc = bpy.context.scene
    try:
        if hasattr(sc, "compositing_node_group"):
            ng = bpy.data.node_groups.new("wy_comp", "CompositorNodeTree")
            ng.interface.new_socket("Image", in_out="OUTPUT", socket_type="NodeSocketColor")
            rl = ng.nodes.new("CompositorNodeRLayers")
            gl = ng.nodes.new("CompositorNodeGlare")
            out = ng.nodes.new("NodeGroupOutput")
            sc.compositing_node_group = ng
        else:
            sc.use_nodes = True
            ng = sc.node_tree
            rl = ng.nodes.get("Render Layers") or ng.nodes.new("CompositorNodeRLayers")
            gl = ng.nodes.new("CompositorNodeGlare")
            out = ng.nodes.get("Composite") or ng.nodes.new("CompositorNodeComposite")
        for attr, val in (("glare_type", "BLOOM"), ("quality", "HIGH")):
            try:
                setattr(gl, attr, val)
            except Exception:
                pass
        for nm, val in (("Type", "Bloom"), ("Strength", strength), ("Threshold", 1.6), ("Size", 0.55),
                        ("Quality", "High")):
            if nm in gl.inputs:
                try:
                    gl.inputs[nm].default_value = val
                except Exception:
                    pass
        ng.links.new(rl.outputs["Image"], gl.inputs["Image"])
        ng.links.new(gl.outputs["Image"], out.inputs[0])
        print("bloom on")
    except Exception as e:
        print("bloom skipped:", e)


def rig(body, rotors, loc, rpy, ctx_name="hero_rig"):
    """Body + 4 rotor instances under an empty, posed for a render."""
    empty = bpy.data.objects.new(ctx_name, None)
    bpy.context.scene.collection.objects.link(empty)
    body.parent = empty
    spin0 = [0.35, 1.25, 2.2, 0.9]
    for i, ((rx, ry), spin) in enumerate(ROTORS):
        o = bpy.data.objects.new("rotor_%d" % i, rotors[spin].data)
        bpy.context.scene.collection.objects.link(o)
        o.location = (rx, ry, ROTOR_Z)
        o.rotation_euler = (0, 0, spin0[i])
        o.parent = empty
    empty.location = loc
    empty.rotation_euler = rpy
    bpy.context.view_layer.update()
    return empty


def render_hero(body, rotors, path, res, samples):
    sc = bpy.context.scene
    center = Vector((0.0, 0.0, 0.62))
    rig(body, rotors, center, (math.radians(7), math.radians(6), math.radians(-8)))
    tgt = bpy.data.objects.new("hero_target", None)
    tgt.location = center + Vector((0.02, 0, -0.03))
    sc.collection.objects.link(tgt)
    world_bg("#040208", 0.28, fog=0.0035)
    # wet dark floor
    fm = pbr("hero_floor", "#07050b", 0.0, 0.16)
    nt = fm.node_tree
    nz = nt.nodes.new("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = 3.5
    nz.inputs["Detail"].default_value = 6
    ramp = nt.nodes.new("ShaderNodeMapRange")
    ramp.inputs["From Min"].default_value = 0.45
    ramp.inputs["From Max"].default_value = 0.62
    ramp.inputs["To Min"].default_value = 0.05
    ramp.inputs["To Max"].default_value = 0.42
    nt.links.new(nz.outputs["Fac"], ramp.inputs["Value"])
    nt.links.new(ramp.outputs["Result"], nt.nodes["Principled BSDF"].inputs["Roughness"])
    floor = C.box(None, "hero_floor", (40, 40, 0.02), (0, 0, -0.01), fm, bev=0.0)
    floor.data.materials[0] = fm
    # background bokeh lights + a few ruined pillars in silhouette
    rng = np.random.default_rng(4)
    for k in range(26):
        a = rng.uniform(-1.2, 1.2) + math.radians(180 + 32)
        dist = rng.uniform(4.5, 12.0)
        p = (dist * math.cos(a), -dist * math.sin(a) * 0.9, rng.uniform(0.12, 1.5))
        colr = ["#ff2fb4", "#9a4dff", "#c58cff", "#ff5ad0"][k % 4]
        m = pbr("bokeh_%d" % k, colr, 0.0, 0.5, emit=colr, estr=float(rng.uniform(8, 30)))
        C.sphere(None, "bokeh", float(rng.uniform(0.012, 0.035)), p, m, seg=12, rings=6)
    pm = pbr("hero_pillar", "#0d0a12", 0.0, 0.7)
    for p in ((-3.2, 2.6), (-4.6, -0.6), (-2.6, -3.4), (-6.0, 3.2)):
        C.cyl(None, "pillar", 0.28, 6.0, (p[0], p[1], 3.0), pm, seg=24)
    # neutral-cool key + broad top softbox so the carbon reads dark and glossy, violet/magenta only on the rims
    light("key", "AREA", (1.5, -1.5, 2.4), "#eef2ff", 260, 1.2, tgt)
    light("top", "AREA", (0.2, 0.3, 2.8), "#f2f2ff", 90, 2.2, tgt)
    light("rim_magenta", "AREA", (-1.7, 1.2, 1.4), "#ff2fa8", 170, 0.9, tgt)
    light("rim_violet", "AREA", (-1.3, -1.7, 1.0), "#7b3cff", 150, 0.9, tgt)
    for k, (px_, py_) in enumerate(((-3.2, 2.6), (-4.6, -0.6), (-6.0, 3.2))):
        pt = bpy.data.objects.new("pillar_t%d" % k, None)
        pt.location = (px_ + 0.25, py_ - 0.2, 0.4)
        sc.collection.objects.link(pt)
        light("pillar_spot%d" % k, "SPOT", (px_ + 0.9, py_ - 0.7, 4.5), ("#7a2cff", "#ff2fa8", "#7a2cff")[k], 600, 0.1,
              pt, spot=14)
    light("shaft", "SPOT", (-0.5, 0.4, 3.6), "#b04dff", 50, 0.15, tgt, spot=20)
    light("belly_fill", "AREA", (0.9, -0.3, 0.05), "#4a1a8a", 30, 1.0, tgt)
    cd = bpy.data.cameras.new("hero_cam")
    cd.lens = 70
    cd.clip_start = 0.02
    cd.clip_end = 60
    cd.dof.use_dof = True
    cd.dof.focus_object = tgt
    cd.dof.aperture_fstop = 2.4
    cam = bpy.data.objects.new("hero_cam", cd)
    az, el, dist = math.radians(38), math.radians(11.0), 2.1
    cam.location = (center.x + dist * math.cos(el) * math.cos(az), center.y - dist * math.cos(el) * math.sin(az),
                    center.z + dist * math.sin(el))
    sc.collection.objects.link(cam)
    c = cam.constraints.new("TRACK_TO")
    c.target = tgt
    c.track_axis = "TRACK_NEGATIVE_Z"
    c.up_axis = "UP_Y"
    sc.camera = cam
    eevee(samples)
    bloom(0.45)
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = 100
    sc.render.image_settings.file_format = "PNG"
    sc.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)
    print("hero ->", path)


def render_review(body, rotors, outdir, res=(1100, 800)):
    """Flat-lit orthographic top/side/front views with the x500 collision boxes and prop discs as wireframe."""
    sc = bpy.context.scene
    outdir.mkdir(parents=True, exist_ok=True)
    rig(body, rotors, (0, 0, 0), (0, 0, 0), "review_rig")
    cm = pbr("col_wire", "#00e5ff", 0.0, 0.5, emit="#00e5ff", estr=3.0)
    cols = [((0, 0, 0.007), (0.3536, 0.3536, 0.05), (0, 0, 0)), ((0, -0.098, -0.123), (0.015, 0.015, 0.21), (-0.35, 0, 0)),
            ((0, 0.098, -0.123), (0.015, 0.015, 0.21), (0.35, 0, 0)), ((0, -0.132, -0.2195), (0.25, 0.015, 0.015), (0, 0, 0)),
            ((0, 0.132, -0.2195), (0.25, 0.015, 0.015), (0, 0, 0))]
    for loc, size, rot in cols:
        b = C.box(None, "col", size, loc, cm, bev=0.0, rot=rot)
        w = b.modifiers.new("wire", "WIREFRAME")
        w.thickness = 0.0012
    for (rx, ry), _ in ROTORS:
        C.torus(None, "disc", PROP_R, 0.0012, (rx, ry, ROTOR_Z), cm, major=96, minor=6)
    world_bg("#9a96a6", 0.9)
    tgt = bpy.data.objects.new("rv_target", None)
    sc.collection.objects.link(tgt)
    light("sun", "SUN", (1, -1, 3), "#ffffff", 3.5, 0.05, tgt)
    eevee(16, raytrace=False)
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.image_settings.file_format = "PNG"
    cd = bpy.data.cameras.new("rv_cam")
    cd.type = "ORTHO"
    cam = bpy.data.objects.new("rv_cam", cd)
    sc.collection.objects.link(cam)
    sc.camera = cam
    views = {"top": ((0, 0, 3), (0, 0, -math.pi / 2), 0.80), "side": ((0, -3, -0.03), (math.pi / 2, 0, 0), 0.80),
             "front": ((3, 0, -0.03), (math.pi / 2, 0, math.pi / 2), 0.62),
             "head": ((0.55, -0.35, 0.16), None, 0.20), "under": ((0.25, -0.5, -0.9), None, 0.75)}
    for nm, (loc, rot, scale) in views.items():
        cam.location = loc
        cd.ortho_scale = scale
        cam.constraints.clear()
        if rot is None:
            tg = bpy.data.objects.new("rv_t_" + nm, None)
            tg.location = (0.26, 0, 0.0) if nm == "head" else (0, 0, -0.05)
            sc.collection.objects.link(tg)
            c = cam.constraints.new("TRACK_TO")
            c.target = tg
            c.track_axis = "TRACK_NEGATIVE_Z"
            c.up_axis = "UP_Y"
        else:
            cam.rotation_euler = rot
        sc.render.filepath = str(outdir / ("review_%s.png" % nm))
        bpy.ops.render.render(write_still=True)
    print("review ->", outdir)


# ================================================================= main
def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--review", default="")
    ap.add_argument("--hero", default=str(IMG / "wyvern_hero.png"))
    ap.add_argument("--res", default="1600x900")
    ap.add_argument("--samples", type=int, default=96)
    a = ap.parse_args(argv)
    C.reset_scene()
    M = make_materials()
    body, S = build_body(M)
    rotors = {spin: build_rotor(M, spin) for spin in ("ccw", "cw")}
    export_glb(body, OUT / "wyvern_body.glb")
    for spin, o in rotors.items():
        export_glb(o, OUT / ("wyvern_rotor_%s.glb" % spin))
    lo, hi = bbox(body)
    print("body bbox min %s max %s" % (np.round(lo, 4), np.round(hi, 4)))
    lo, hi = bbox(rotors["ccw"])
    print("rotor bbox min %s max %s" % (np.round(lo, 4), np.round(hi, 4)))
    if a.review:
        render_review(body, rotors, Path(a.review))
        if not a.no_render:     # reload a clean scene for the hero
            for o in list(bpy.data.objects):
                if o.name not in (body.name, rotors["ccw"].name, rotors["cw"].name):
                    bpy.data.objects.remove(o)
            body.parent = None
            body.location = (0, 0, 0)
    if not a.no_render:
        IMG.mkdir(parents=True, exist_ok=True)
        w, h = (int(v) for v in a.res.split("x"))
        render_hero(body, rotors, Path(a.hero), (w, h), a.samples)


main()
