"""Shared bpy helpers for the robot builders (procedural geometry only, no external assets).

Convention: every robot is modelled in "ground" coordinates (z = 0 is the floor, +X forward,
+Y left, base_footprint under the drive axle centre).  Export shifts parts to their URDF link
frames: base_link sits at z = wheel_radius, wheels at their own centre.
"""
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import bmesh
import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / "src" / "fleet_description"
YAML = PKG / "config" / "robots.yaml"
MESH_DIR = PKG / "meshes"
BLEND_DIR = ROOT / "blender" / "blend"
PREVIEW_DIR = ROOT / "blender" / "previews"


# ----------------------------------------------------------------------------- config
def load_cfg():
    """robots.yaml -> dict. Blender's Python has no PyYAML, so the system python converts it."""
    try:
        import yaml  # noqa
        with open(YAML) as f:
            return yaml.safe_load(f)
    except ImportError:
        code = "import yaml,json,sys;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))"
        return json.loads(subprocess.check_output(["/usr/bin/python3", "-c", code, str(YAML)]))


# ----------------------------------------------------------------------------- scene
def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    for coll in (bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for b in list(coll):
            coll.remove(b)


def new_collection(name):
    c = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(c)
    return c


class Ctx:
    """Collects the objects of one robot into a collection."""

    def __init__(self, name):
        self.name = name
        self.coll = new_collection("robot_" + name)

    def link(self, obj):
        for c in list(obj.users_collection):
            c.objects.unlink(obj)
        self.coll.objects.link(obj)
        return obj


# ----------------------------------------------------------------------------- materials
# name: (base colour hex, metallic, roughness, emission hex or None, emission strength, alpha)
PALETTE = {
    "white":      ("#e9e9ef", 0.0, 0.35, None, 0, 1),
    "pearl":      ("#d6d3e0", 0.0, 0.30, None, 0, 1),
    "graphite":   ("#26262d", 0.0, 0.45, None, 0, 1),
    "charcoal":   ("#15151a", 0.0, 0.60, None, 0, 1),
    "plastic":    ("#2c2c34", 0.0, 0.70, None, 0, 1),
    "steel":      ("#9a9ca8", 0.9, 0.35, None, 0, 1),
    "darksteel":  ("#4b4c57", 0.9, 0.40, None, 0, 1),
    "gold":       ("#c9a24a", 1.0, 0.30, None, 0, 1),
    "copper":     ("#b06c4a", 0.9, 0.35, None, 0, 1),
    "rubber":     ("#101012", 0.0, 0.92, None, 0, 1),
    "violet":     ("#6a2bd9", 0.0, 0.30, None, 0, 1),
    "violetmetal": ("#5a2db0", 0.8, 0.28, None, 0, 1),
    "deeppurple": ("#2d1252", 0.4, 0.35, None, 0, 1),
    "scale":      ("#4a1f78", 0.7, 0.32, None, 0, 1),
    "scalehi":    ("#7a3cb8", 0.8, 0.25, None, 0, 1),
    "membrane":   ("#5a1478", 0.2, 0.55, None, 0, 1),
    "wyrm_horn":  ("#2b2233", 0.35, 0.24, None, 0, 1),
    "bone":       ("#cfc6b4", 0.0, 0.55, None, 0, 1),
    # legged robots
    "wyrm_scale": ("#4a1462", 0.65, 0.30, None, 0, 1),
    "wyrm_scale2": ("#70289a", 0.75, 0.26, None, 0, 1),
    "wyrm_skin":  ("#2e0e3e", 0.45, 0.42, None, 0, 1),
    "wyrm_claw":  ("#2a2230", 0.2, 0.25, None, 0, 1),
    "wyrm_iris":  ("#0c0014", 0.0, 0.30, "#a000d8", 0.6, 1),
    "wyrm_eyecore": ("#14001f", 0.0, 0.30, "#e83cff", 2.4, 1),
    "wyrm_crystal": ("#2a0660", 0.0, 0.30, "#b010ff", 0.8, 1),
    "wyrm_wing":  ("#2e0a40", 0.05, 0.55, "#5a0a6a", 0.05, 1),
    "wyrm_vein":  ("#5a1460", 0.4, 0.35, None, 0, 1),
    "wyrm_core":  ("#48208a", 0.0, 0.10, "#8a40ff", 0.9, 1),
    "hound_panel": ("#1c1b24", 0.35, 0.28, None, 0, 1),
    "hound_shell": ("#121118", 0.25, 0.22, None, 0, 1),
    "hound_joint": ("#3a3946", 0.85, 0.32, None, 0, 1),
    "visor":      ("#06050c", 0.2, 0.04, None, 0, 1),
    "carapace":   ("#100b18", 0.35, 0.10, None, 0, 1),
    "carapace_hi": ("#2a1644", 0.55, 0.14, None, 0, 1),
    "chitin":     ("#18121f", 0.30, 0.20, None, 0, 1),
    "spider_eye": ("#140626", 0.0, 0.04, "#a040ff", 1.1, 1),
    "spider_mark": ("#7a2ce0", 0.0, 0.25, "#8a3dff", 1.4, 1),
    "glass":      ("#0b0b14", 0.0, 0.05, None, 0, 1),
    "lens":       ("#050512", 0.3, 0.05, "#2040ff", 0.4, 1),
    "led_violet": ("#b06bff", 0.0, 0.3, "#9a4dff", 6.0, 1),
    "led_cyan":   ("#7fe8ff", 0.0, 0.3, "#40d8ff", 4.0, 1),
    "eye":        ("#e0a0ff", 0.0, 0.2, "#c050ff", 3.0, 1),
    "gem":        ("#a468ff", 0.0, 0.05, "#8a40ff", 2.5, 1),
    "solar":      ("#1a1f6e", 0.5, 0.20, None, 0, 1),
    "safety":     ("#e8a020", 0.0, 0.5, None, 0, 1),
}
_mat_cache = {}


def _rgb(h):
    h = h.lstrip("#")
    c = [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
    # hex given in sRGB; convert to linear
    return [((v + 0.055) / 1.055) ** 2.4 if v > 0.04045 else v / 12.92 for v in c]


def mat(name):
    if name in _mat_cache and _mat_cache[name].name in bpy.data.materials:
        return _mat_cache[name]
    base, metal, rough, emi, estr, alpha = PALETTE[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = _rgb(base) + [1]
    b.inputs["Metallic"].default_value = metal
    b.inputs["Roughness"].default_value = rough
    if emi:
        b.inputs["Emission Color"].default_value = _rgb(emi) + [1]
        b.inputs["Emission Strength"].default_value = estr
    b.inputs["Alpha"].default_value = alpha
    m.diffuse_color = _rgb(base) + [1]
    if name in FX:
        try:
            _add_fx(m, FX[name])
        except Exception as e:      # never let preview detail break a build
            print("FX skipped for", name, e)
    _mat_cache[name] = m
    return m


# Preview-only surface detail on top of the flat colours: procedural noise bump and a little colour
# variation (and subsurface for thin membranes). Gazebo reads only the MTL colours, which the OBJ
# exporter takes from the Base Color value (left set), so exports are unchanged.
FX = {
    "wyrm_skin":   dict(bump=0.30, scale=700.0, var=0.25),
    "wyrm_scale":  dict(bump=0.10, scale=1600.0, var=0.18),
    "wyrm_scale2": dict(bump=0.10, scale=1600.0, var=0.18),
    "wyrm_wing":   dict(bump=0.25, scale=420.0, var=0.35, sss=0.35),
    "wyrm_horn":   dict(bump=0.35, scale=260.0, var=0.30, stretch=6.0),
    "deeppurple":  dict(bump=0.15, scale=900.0, var=0.15),
    "carapace":    dict(bump=0.04, scale=300.0, var=0.20),
    "chitin":      dict(bump=0.20, scale=900.0, var=0.20),
    "hound_panel": dict(bump=0.03, scale=500.0, var=0.10),
    "wyrm_crystal": dict(bump=0.0, scale=100.0, var=0.0, spec=0.12),
    "wyrm_iris":   dict(bump=0.0, scale=100.0, var=0.0, spec=0.0),
    "wyrm_eyecore": dict(bump=0.0, scale=100.0, var=0.0, spec=0.0),
}


def _add_fx(m, fx):
    nt = m.node_tree
    b = nt.nodes["Principled BSDF"]
    tc = nt.nodes.new("ShaderNodeTexCoord")
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (1.0, 1.0, fx.get("stretch", 1.0))
    nt.links.new(tc.outputs["Object"], mp.inputs["Vector"])
    nz = nt.nodes.new("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = fx["scale"]
    nz.inputs["Detail"].default_value = 6.0
    nz.inputs["Roughness"].default_value = 0.6
    nt.links.new(mp.outputs["Vector"], nz.inputs["Vector"])
    bu = nt.nodes.new("ShaderNodeBump")
    bu.inputs["Strength"].default_value = fx["bump"]
    bu.inputs["Distance"].default_value = 0.0004
    nt.links.new(nz.outputs["Fac"], bu.inputs["Height"])
    nt.links.new(bu.outputs["Normal"], b.inputs["Normal"])
    col = list(b.inputs["Base Color"].default_value)
    if fx.get("var"):
        mx = nt.nodes.new("ShaderNodeMix")
        mx.data_type = "RGBA"
        ca, cb = [i for i in mx.inputs if i.type == "RGBA"][:2]
        fac = [i for i in mx.inputs if i.type == "VALUE" and i.name == "Factor"][0]
        out = [o for o in mx.outputs if o.type == "RGBA"][0]
        ca.default_value = [c * (1.0 - fx["var"]) for c in col[:3]] + [1]
        cb.default_value = [min(1.0, c * (1.0 + fx["var"])) for c in col[:3]] + [1]
        nz2 = nt.nodes.new("ShaderNodeTexNoise")
        nz2.inputs["Scale"].default_value = fx["scale"] * 0.08
        nt.links.new(mp.outputs["Vector"], nz2.inputs["Vector"])
        nt.links.new(nz2.outputs["Fac"], fac)
        nt.links.new(out, b.inputs["Base Color"])
    if "spec" in fx:
        b.inputs["Specular IOR Level"].default_value = fx["spec"]
    if fx.get("sss"):
        b.inputs["Subsurface Weight"].default_value = fx["sss"]
        b.inputs["Subsurface Radius"].default_value = (0.9, 0.2, 1.0)
        b.inputs["Subsurface Scale"].default_value = 0.004
    b.inputs["Base Color"].default_value = col


def reset_mat_cache():
    _mat_cache.clear()


# ----------------------------------------------------------------------------- geometry
AXES = {
    "z": Matrix.Identity(4),
    "y": Matrix.Rotation(math.radians(-90), 4, "X"),   # z -> y
    "x": Matrix.Rotation(math.radians(90), 4, "Y"),    # z -> x
}


def _obj_from_bm(name, bm, material, loc=(0, 0, 0), rot=(0, 0, 0), ctx=None):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    obj.location = loc
    obj.rotation_euler = rot
    if material:
        me.materials.append(mat(material) if isinstance(material, str) else material)
    if ctx:
        ctx.coll.objects.link(obj)
    else:
        bpy.context.scene.collection.objects.link(obj)
    return obj


def apply_mods(obj):
    dg = bpy.context.evaluated_depsgraph_get()
    ev = obj.evaluated_get(dg)
    new = bpy.data.meshes.new_from_object(ev)
    old = obj.data
    obj.modifiers.clear()
    obj.data = new
    if old.users == 0:
        bpy.data.meshes.remove(old)
    return obj


def bevel(obj, width, seg=2, angle=40):
    if width <= 0:
        return obj
    m = obj.modifiers.new("bevel", "BEVEL")
    m.width = width
    m.segments = seg
    m.limit_method = "ANGLE"
    m.angle_limit = math.radians(angle)
    return apply_mods(obj)


def box(ctx, name, size, loc, material, bev=0.003, rot=(0, 0, 0), seg=2):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Diagonal((size[0], size[1], size[2], 1)))
    return bevel(_obj_from_bm(name, bm, material, loc, rot, ctx), bev, seg)


def cyl(ctx, name, r, h, loc, material, axis="z", r2=None, seg=32, bev=0.0, rot=(0, 0, 0), cap=True):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=cap, cap_tris=False, segments=seg, radius1=r,
                          radius2=r if r2 is None else r2, depth=h, matrix=AXES[axis])
    return bevel(_obj_from_bm(name, bm, material, loc, rot, ctx), bev, 2)


def sphere(ctx, name, r, loc, material, scale=(1, 1, 1), seg=24, rings=12, rot=(0, 0, 0)):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=seg, v_segments=rings, radius=r,
                              matrix=Matrix.Diagonal((scale[0], scale[1], scale[2], 1)))
    return _obj_from_bm(name, bm, material, loc, rot, ctx)


def ico(ctx, name, r, loc, material, scale=(1, 1, 1), subdiv=2, rot=(0, 0, 0)):
    bm = bmesh.new()
    bmesh.ops.create_icosphere(bm, subdivisions=subdiv, radius=r,
                               matrix=Matrix.Diagonal((scale[0], scale[1], scale[2], 1)))
    return _obj_from_bm(name, bm, material, loc, rot, ctx)


def tube(ctx, name, p1, p2, r, material, seg=12, r2=None, bev=0.0):
    p1, p2 = Vector(p1), Vector(p2)
    d = p2 - p1
    q = Vector((0, 0, 1)).rotation_difference(d.normalized())
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=seg, radius1=r,
                          radius2=r if r2 is None else r2, depth=d.length)
    return bevel(_obj_from_bm(name, bm, material, (p1 + p2) / 2, q.to_euler(), ctx), bev, 1)


def lathe(ctx, name, profile, loc, material, seg=32, axis="z", scale=(1, 1, 1), rot=(0, 0, 0), bev=0.0,
          loop=False):
    """Revolve [(r, z), ...] around z. Points with r == 0 become single pole vertices
    (use them at both ends to get a closed solid)."""
    bm = bmesh.new()
    rings = []
    for r, z in profile:
        if r < 1e-9:
            rings.append([bm.verts.new((0, 0, z))])
        else:
            rings.append([bm.verts.new((r * math.cos(2 * math.pi * i / seg),
                                        r * math.sin(2 * math.pi * i / seg), z)) for i in range(seg)])
    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(seg):
            j = (i + 1) % seg
            if len(a) == 1 and len(b) == 1:
                continue
            if len(a) == 1:
                bm.faces.new((a[0], b[j], b[i]))
            elif len(b) == 1:
                bm.faces.new((a[i], a[j], b[0]))
            else:
                bm.faces.new((a[i], a[j], b[j], b[i]))
    if loop:
        a, b = rings[-1], rings[0]
        for i in range(seg):
            j = (i + 1) % seg
            bm.faces.new((a[i], a[j], b[j], b[i]))
    else:
        # open ends with r > 0: cap them
        for ring in (rings[0], rings[-1]):
            if len(ring) > 1:
                bm.faces.new(ring)
    bmesh.ops.transform(bm, matrix=AXES[axis] @ Matrix.Diagonal((scale[0], scale[1], scale[2], 1)),
                        verts=bm.verts)
    return bevel(_obj_from_bm(name, bm, material, loc, rot, ctx), bev, 1)


def torus(ctx, name, R, r, loc, material, major=48, minor=10, axis="z", scale=(1, 1, 1)):
    bm = bmesh.new()
    vs = []
    for i in range(major):
        a = 2 * math.pi * i / major
        row = []
        for j in range(minor):
            b = 2 * math.pi * j / minor
            rr = R + r * math.cos(b)
            row.append(bm.verts.new((rr * math.cos(a), rr * math.sin(a), r * math.sin(b))))
        vs.append(row)
    for i in range(major):
        for j in range(minor):
            bm.faces.new((vs[i][j], vs[(i + 1) % major][j], vs[(i + 1) % major][(j + 1) % minor],
                          vs[i][(j + 1) % minor]))
    bmesh.ops.transform(bm, matrix=AXES[axis] @ Matrix.Diagonal((scale[0], scale[1], scale[2], 1)),
                        verts=bm.verts)
    return _obj_from_bm(name, bm, material, loc, (0, 0, 0), ctx)


def mesh_from(ctx, name, verts, faces, material, loc=(0, 0, 0), rot=(0, 0, 0), bev=0.0):
    bm = bmesh.new()
    vs = [bm.verts.new(v) for v in verts]
    for f in faces:
        bm.faces.new([vs[i] for i in f])
    return bevel(_obj_from_bm(name, bm, material, loc, rot, ctx), bev, 1)


def cut(obj, cutter, keep_cutter=False):
    """Boolean difference (exact solver)."""
    m = obj.modifiers.new("cut", "BOOLEAN")
    m.operation = "DIFFERENCE"
    m.object = cutter
    m.solver = "EXACT"
    apply_mods(obj)
    if not keep_cutter:
        bpy.data.objects.remove(cutter)
    return obj


def union(obj, other):
    m = obj.modifiers.new("uni", "BOOLEAN")
    m.operation = "UNION"
    m.object = other
    m.solver = "EXACT"
    apply_mods(obj)
    bpy.data.objects.remove(other)
    return obj


def _select_only(objs):
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]


def join(ctx, objs, name, smooth_deg=35):
    objs = list(objs)
    bpy.context.view_layer.update()
    if os.environ.get("DBG"):
        for ob in objs:
            zs = [(ob.matrix_world @ Vector(c)).z for c in ob.bound_box]
            if min(zs) < -0.001:
                print("BELOW", ob.name, round(min(zs), 3), flush=True)
    for ob in objs:   # bake the object transform so joining never shifts anything
        ob.data.transform(ob.matrix_world)
        ob.matrix_world = Matrix.Identity(4)
    _select_only(objs)
    bpy.ops.object.join()
    o = bpy.context.view_layer.objects.active
    o.name = name
    o.data.name = name
    o.location = (0, 0, 0)
    o.rotation_euler = (0, 0, 0)
    # merge duplicate material slots
    bpy.ops.object.material_slot_remove_unused()
    _select_only([o])
    if smooth_deg:
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(smooth_deg))
    return o


def mirror_y(obj_list):
    for o in obj_list:
        o.location.y = -o.location.y


def tri_count(obj):
    dg = bpy.context.evaluated_depsgraph_get()
    me = obj.evaluated_get(dg).to_mesh()
    n = sum(len(p.vertices) - 2 for p in me.polygons)
    obj.evaluated_get(dg).to_mesh_clear()
    return n


# ----------------------------------------------------------------------------- export
def export_obj(obj, path, offset=(0, 0, 0)):
    """Export one object to OBJ, translated by `offset` (so the origin is the link frame).
    No axis conversion (forward Y, up Z), metres."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    old = obj.location.copy()
    obj.location = (old.x + offset[0], old.y + offset[1], old.z + offset[2])
    bpy.context.view_layer.update()
    _select_only([obj])
    bpy.ops.wm.obj_export(filepath=str(path), export_selected_objects=True, apply_modifiers=True,
                          export_materials=True, export_normals=True, export_uv=False,
                          export_triangulated_mesh=True, forward_axis="Y", up_axis="Z",
                          path_mode="STRIP", export_pbr_extensions=False)
    obj.location = old
    bpy.context.view_layer.update()


def make_wheel_instance(src, loc, name, ctx):
    o = bpy.data.objects.new(name, src.data)   # linked duplicate (shared mesh)
    o.location = loc
    ctx.coll.objects.link(o)
    return o
