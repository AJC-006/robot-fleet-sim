"""Studio lighting + EEVEE preview renders (headless)."""
import math

import bpy
from mathutils import Vector

from .common import _rgb, new_collection


def _light(name, kind, loc, color, energy, size, target, coll, spread=None):
    ld = bpy.data.lights.new(name, kind)
    ld.color = _rgb(color)
    ld.energy = energy
    if kind == "AREA":
        ld.shape = "DISK"
        ld.size = size
    elif kind == "SPOT":
        ld.spot_size = math.radians(spread or 60)
        ld.spot_blend = 0.8
        ld.shadow_soft_size = size
    else:
        ld.shadow_soft_size = size
    ob = bpy.data.objects.new(name, ld)
    ob.location = loc
    coll.objects.link(ob)
    c = ob.constraints.new("TRACK_TO")
    c.target = target
    c.track_axis = "TRACK_NEGATIVE_Z"
    c.up_axis = "UP_Y"
    return ob


def studio(center=(0, 0, 0.1), radius=0.4, floor_r=3.0):
    """Dark studio: glossy dark floor, warm key, violet rim, cool fill."""
    coll = new_collection("studio")
    scene = bpy.context.scene
    cx, cy, cz = center
    tgt = bpy.data.objects.new("studio_target", None)
    tgt.location = center
    coll.objects.link(tgt)
    world = bpy.data.worlds.new("studio_world")
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = _rgb("#0a0812") + [1]
    bg.inputs["Strength"].default_value = 0.08
    scene.world = world
    # floor
    from .common import cyl, mat
    fm = bpy.data.materials.new("studio_floor")
    fm.use_nodes = True
    b = fm.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = _rgb("#0d0b16") + [1]
    b.inputs["Roughness"].default_value = 0.35
    b.inputs["Metallic"].default_value = 0.0
    floor = cyl(None, "studio_floor", floor_r, 0.02, (cx, cy, -0.01), fm, seg=96)
    for c in list(floor.users_collection):
        c.objects.unlink(floor)
    coll.objects.link(floor)
    R = max(radius, 0.2)
    _light("key", "AREA", (cx + 2.6 * R, cy - 3.2 * R, cz + 3.0 * R), "#fff1e0", 70 * R * R * 9, 2.2 * R, tgt, coll)
    _light("rim_violet", "AREA", (cx - 3.4 * R, cy + 3.0 * R, cz + 1.8 * R), "#8a3dff", 270 * R * R * 9, 1.4 * R, tgt, coll)
    _light("rim2", "AREA", (cx - 3.0 * R, cy - 3.4 * R, cz + 1.0 * R), "#a45bff", 90 * R * R * 9, 1.2 * R, tgt, coll)
    _light("fill", "AREA", (cx + 3.6 * R, cy + 3.2 * R, cz + 0.8 * R), "#cdb8ff", 90 * R * R * 9, 3 * R, tgt, coll)
    return coll, tgt


def camera(loc, target, lens=55):
    cd = bpy.data.cameras.new("cam")
    cd.lens = lens
    cd.clip_start = 0.01
    cd.clip_end = 50
    cam = bpy.data.objects.new("cam", cd)
    cam.location = loc
    bpy.context.scene.collection.objects.link(cam)
    c = cam.constraints.new("TRACK_TO")
    c.target = target
    c.track_axis = "TRACK_NEGATIVE_Z"
    c.up_axis = "UP_Y"
    bpy.context.scene.camera = cam
    return cam


def render(path, res=(1280, 720), samples=48):
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    ev = scene.eevee
    ev.taa_render_samples = samples
    for attr, val in (("use_raytracing", False), ("use_shadows", True), ("use_gtao", True)):
        if hasattr(ev, attr):
            try:
                setattr(ev, attr, val)
            except Exception:
                pass
    scene.render.resolution_x, scene.render.resolution_y = res
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(path)
    scene.view_settings.view_transform = "AgX"
    try:
        scene.view_settings.look = "AgX - Medium High Contrast"
    except Exception:
        pass
    scene.view_settings.exposure = 0.0
    bpy.ops.render.render(write_still=True)


def hero(center, size, path, res=(1280, 720), az=38, el=17, fit=2.7):
    """3/4 view: camera front-right, slightly above. size = largest dimension of the robot."""
    coll, tgt = studio(center, size * 0.6)
    d = size * fit
    a, e = math.radians(az), math.radians(el)
    cam = camera((center[0] + d * math.cos(e) * math.cos(a), center[1] - d * math.cos(e) * math.sin(a),
                  center[2] + d * math.sin(e)), tgt, 50)
    render(path, res, samples=96)
