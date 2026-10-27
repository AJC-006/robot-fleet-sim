"""Build the robot-fleet-sim environments in Blender (headless).

    blender -b --factory-startup -P blender/build_worlds.py -- [--world keep|apartment|warehouse|lab] [--no-preview]

Per world this writes: src/fleet_worlds/models/<w>_env/{model.sdf,model.config,collision.json,meshes/<w>.glb},
worlds/<w>.sdf + <w>.yaml, maps/<w>.{pgm,yaml} (via tools/make_map.py), blender/blend/world_<w>.blend and
blender/previews/world_<w>.png.
"""
import importlib
import math
import os
import subprocess
import sys

import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "worlds"))
import lib  # noqa: E402
import sdfgen  # noqa: E402

PKG = os.path.join(ROOT, "src", "fleet_worlds")  # default output; --out DIR overrides
ALL = ["keep", "apartment", "warehouse", "lab"]
EXPORT_YUP = False  # Gazebo (assimp) does not convert glTF Y-up to Z-up: export raw Z-up (verified in the headless render)


def parse():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    global PKG
    worlds, preview = [], True
    i = 0
    while i < len(argv):
        if argv[i] == "--world":
            worlds.append(argv[i + 1])
            i += 1
        elif argv[i] == "--out":
            PKG = os.path.abspath(argv[i + 1])
            i += 1
        elif argv[i] == "--no-preview":
            preview = False
        i += 1
    return (worlds or ALL), preview


def build_world(name, do_preview):
    mod = importlib.import_module("w_" + name)
    importlib.reload(mod)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    lib._IMG.clear()
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    coll = scene.collection

    model_dir = os.path.join(PKG, "models", name + "_env")
    mesh_dir = os.path.join(model_dir, "meshes")
    chunked = bool(getattr(mod, "CHUNK", None))
    tex_dir = os.path.join(mesh_dir, "_srctex" if chunked else "textures")
    os.makedirs(mesh_dir, exist_ok=True)
    for sub in ("worlds", "maps"):
        os.makedirs(os.path.join(PKG, sub), exist_ok=True)

    used = lib.make_materials(name, mod.MATS, tex_dir)
    W = lib.WorldBuilder(name, mod.BOUNDS, mod.MATS)
    if chunked:
        W.chunk_size = mod.CHUNK
        W.chunk_origin = mod.CHUNK_ORIGIN
    mod.build(W)
    tiles = []
    if chunked:
        objs = {}
        import shutil
        for old in os.listdir(os.path.join(PKG, "models")):  # drop stale tiles of this world
            if old.startswith(name + "_t") or old in (name + "_lake", name + "_sky", name + "_water"):
                shutil.rmtree(os.path.join(PKG, "models", old), ignore_errors=True)
        fmt = getattr(mod, "EXPORT", "GLTF")
        for ck in W.chunk_keys():
            cname = "%s_%s" % (name, W.chunk_name(ck))
            cobjs = W.make_chunk(ck, coll)
            if not cobjs:
                continue
            bpy.ops.object.select_all(action="DESELECT")
            for o in cobjs:
                o.select_set(True)
            ext = "gltf" if fmt == "GLTF" else "glb"
            path = os.path.join(mesh_dir, cname + "." + ext)
            kw = dict(export_format="GLTF_SEPARATE", export_texture_dir="textures") if fmt == "GLTF" else dict(export_format="GLB")
            bpy.ops.export_scene.gltf(filepath=path, export_apply=True, export_yup=EXPORT_YUP, export_image_format="AUTO",
                                      use_selection=True, export_cameras=False, export_lights=False, **kw)
            sdfgen.write_tile_model(os.path.join(PKG, "models", cname), cname, "../%s_env/meshes/%s.%s" % (name, cname, ext))
            tiles.append(cname)
            for o in cobjs:
                me = o.data
                bpy.data.objects.remove(o)
                bpy.data.meshes.remove(me)
            print("  chunk %-14s %s" % (cname, os.path.getsize(path) // 1024 if ext == "glb" else ""))
        glb = os.path.join(mesh_dir, tiles[0] + ".gltf")
    else:
        objs = W.make_objects(coll)
        # ---- export (GLB embeds the textures; the loose copies are only needed while building)
        glb = os.path.join(mesh_dir, name + ".glb")
        glb_tmp = os.path.join(mesh_dir, "." + name + ".tmp.glb")
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.export_scene.gltf(filepath=glb_tmp, export_format="GLB", export_apply=True, export_yup=EXPORT_YUP,
                                  export_image_format="AUTO", use_selection=False, export_cameras=False,
                                  export_lights=False)
        os.replace(glb_tmp, glb)  # atomic: a running Gazebo never sees a half-written mesh
    # ---- sdf/yaml/json
    xmin, ymin, xmax, ymax = mod.OUTER
    sdfgen.write_collision_json(os.path.join(model_dir, "collision.json"), name, (xmin, ymin, xmax, ymax), W.prims,
                                {"spawn": mod.SPAWN, "goals": [list(g) for g in mod.GOALS]})
    sdfgen.write_model(model_dir, name, None if chunked else "meshes/%s.glb" % name, W.prims)
    sdfgen.write_world_sdf(os.path.join(PKG, "worlds", name + ".sdf"), name, mod.SCENE, W.lights, tiles)
    extra = {k: getattr(mod, v) for k, v in (("spawns", "SPAWNS"), ("auv", "AUV"), ("short_pairs", "SHORT_PAIRS")) if hasattr(mod, v)}
    sdfgen.write_world_yaml(os.path.join(PKG, "worlds", name + ".yaml"), name, mod.SPAWN, mod.GOALS, getattr(mod, "DRONE", None), extra)
    mm = ["/usr/bin/python3", os.path.join(ROOT, "tools", "make_map.py"), name, "--pkg", PKG]
    if getattr(mod, "MAP_RES", None):
        mm += ["--res", str(mod.MAP_RES)]
    if getattr(mod, "MAP_CROP", False):
        mm += ["--crop-seed=%s,%s" % (mod.SPAWN[0], mod.SPAWN[1])]
    subprocess.run(mm, check=True)

    # ---- save blend (textures packed so it is self-contained)
    if not chunked:
        bpy.ops.file.pack_all()
        blend = os.path.join(ROOT, "blender", "blend", "world_%s.blend" % name)
        os.makedirs(os.path.dirname(blend), exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=blend)

    tex_mb = sum(os.path.getsize(os.path.join(tex_dir, f)) for f in used) / 1e6
    glb_mb = os.path.getsize(glb) / 1e6 if os.path.exists(glb) else 0.0
    print("WORLD %-10s tris=%d prims=%d lights=%d textures=%.1fMB glb=%.1fMB" %
          (name, W.tri_count, len(W.prims), len(W.lights), tex_mb, glb_mb))

    if do_preview:
        if chunked:
            for ck in W.chunk_keys():
                for o in W.make_chunk(ck, coll):
                    objs[(ck, o.name)] = o
        preview(name, mod, W, objs)
    # textures are embedded in the GLB (and packed in the blend): drop the loose copies (the keep's tiles share
    # the textures/ folder next to the glTF files, so that one stays)
    import shutil
    shutil.rmtree(tex_dir, ignore_errors=True)


def preview(name, mod, W, objs):
    P = mod.PREVIEW
    scene = bpy.context.scene
    for tag, zmax in P.get("cut", []):
        for (mat, v0, v1) in W.tags.get(tag, []):
            me = objs[mat].data
            if W.mats[mat].get("bevel") and len(me.vertices) != len(W.acc[mat]["v"]):
                W.refill_plain(mat, me)
            for i in range(v0, v1):
                if me.vertices[i].co.z > zmax:
                    me.vertices[i].co.z = zmax
            me.update()
    for tag in P.get("hide", []):
        for (mat, v0, v1) in W.tags.get(tag, []):
            me = objs[mat].data
            if W.mats[mat].get("bevel") and len(me.vertices) != len(W.acc[mat]["v"]):
                W.refill_plain(mat, me)
            for i in range(v0, v1):
                me.vertices[i].co.z = -200.0
            me.update()
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x, scene.render.resolution_y = 1280, 720
    scene.render.film_transparent = False
    scene.eevee.taa_render_samples = 24
    scene.view_settings.view_transform = "AgX"
    wd = bpy.data.worlds.new("w")
    wd.use_nodes = True
    bg = wd.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (*P["world"], 1.0)
    bg.inputs["Strength"].default_value = P.get("world_strength", 1.0)
    scene.world = wd
    for L in W.lights:
        ld = bpy.data.lights.new(L["name"], "POINT")
        ld.energy = L["watts"] * P.get("light_gain", 1.0)
        ld.color = L["color"]
        ld.shadow_soft_size = 0.2
        ld.use_shadow = False
        ob = bpy.data.objects.new(L["name"], ld)
        scene.collection.objects.link(ob)
        ob.location = L["pos"]
    for i, (typ, pos, col, energy, size) in enumerate(P.get("extra_lights", [])):
        ld = bpy.data.lights.new("x%d" % i, typ)
        ld.energy = energy
        ld.color = col
        if typ == "AREA":
            ld.size = size
        elif typ == "POINT":
            ld.shadow_soft_size = size
        ob = bpy.data.objects.new("x%d" % i, ld)
        scene.collection.objects.link(ob)
        ob.location = pos
        if typ in ("AREA", "SUN", "SPOT"):
            ob.rotation_euler = P.get("light_rot", (0.0, 0.0, 0.0)) if typ == "SUN" else (0, 0, 0)
    cam = bpy.data.cameras.new("cam")
    cam.clip_end = 900
    co = bpy.data.objects.new("cam", cam)
    scene.collection.objects.link(co)
    scene.camera = co
    views = [("", P["cam"], P["target"], P.get("lens", 24))] + [("_" + v[0],) + tuple(v[1:]) for v in P.get("views", [])]
    only = [v for v in os.environ.get("PV", "").split(",") if v]
    if only:
        views = [v for v in views if v[0][1:] in only or (not v[0] and "all" in only)]
    hide_mats = set(P.get("hide_mats", []))
    show_in = P.get("show_in", {})        # view name -> materials that stay visible there (e.g. sky in ground views)
    for suf, pos, tgt, lens in views:
        if hide_mats:
            keep = set(show_in.get(suf[1:], []))
            for o in objs.values():
                if o.type == "MESH" and o.data.materials:
                    mn = o.data.materials[0].name[len(name) + 1:]
                    hide = mn in hide_mats and mn not in keep
                    if suf and mn == "roof":
                        hide = suf[1:] in P.get("roof_off", [])
                    o.hide_render = hide
        cam.lens = lens
        co.location = pos
        d = Vector(tgt) - Vector(pos)
        co.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
        out = os.path.join(ROOT, "blender", "previews", "world_%s%s.png" % (name, suf))
        os.makedirs(os.path.dirname(out), exist_ok=True)
        scene.render.filepath = out
        bpy.ops.render.render(write_still=True)
        print("PREVIEW", out)


if __name__ == "__main__":
    worlds, prev = parse()
    for w in worlds:
        build_world(w, prev)
