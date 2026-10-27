"""Build the seven ground robots procedurally in Blender (headless), export OBJ meshes, save .blend
and render previews.

    blender -b --factory-startup -P blender/build_robots.py -- [--robot NAME] [--no-render] [--lineup]

Without --robot all seven are built and the lineup is rendered. Wheeled robots export base.obj +
wheel.obj; legged robots (`kind: legged` in robots.yaml) export base.obj and one mesh per leg link
type (coxa, femur, tibia; <part>_l / <part>_r for body.mirrored_meshes), each in its URDF link frame.
"""
import argparse
import os
import importlib
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import bpy  # noqa: E402

from robots import common, preview  # noqa: E402

MODULES = {"round": "pebble", "box": "brick", "rover": "rover", "truck": "truck", "wyrm": "wyrm",
           "hound": "hound", "spider": "spider"}
ORDER = list(MODULES)


def module_name(name, cfg):
    """Builder module (blender/robots/<module>.py) of a robot: `builder:` in its robots.yaml entry, else the built-in table."""
    mod = cfg[name].get("builder") or MODULES.get(name)
    if not mod:
        sys.exit(f"robot {name}: set `builder: <module in blender/robots>` in its robots.yaml entry "
                 f"(existing: {sorted(set(MODULES.values()))}) or add a module, see docs/guides/add_a_robot.md")
    return mod
HERO_VIEW = {"rover": (55, 14), "wyrm": (62, 24), "hound": (36, 14), "spider": (34, 24)}   # (azimuth, elevation) deg


def build_one(name, cfg, render=True):
    if cfg[name].get("kind") == "legged":
        return build_legged(name, cfg, render)
    common.reset_scene()
    common.reset_mat_cache()
    rc = cfg[name]
    ctx = common.Ctx(name)
    mod = importlib.import_module("robots." + module_name(name, cfg))
    importlib.reload(mod)
    base, wheel = mod.build(rc, ctx)
    r = rc["wheel_radius"]
    sep = rc["wheel_separation"]
    # place wheel instances (ground coordinates: axle at z = wheel_radius)
    insts = []
    names = list(zip(rc["left_wheels"], rc["right_wheels"], rc["wheel_x"]))
    for ln, rn, x in names:
        insts.append((ln, wheel_inst(wheel, (x, sep / 2, r), ln, ctx)))
        insts.append((rn, wheel_inst(wheel, (x, -sep / 2, r), rn, ctx)))
    bpy.data.objects.remove(wheel)
    # export (origin at the URDF link frame)
    out = common.MESH_DIR / name
    common.export_obj(base, out / "base.obj", offset=(0, 0, -r))
    first = insts[0][1]
    common.export_obj(first, out / "wheel.obj", offset=(-first.location.x, -first.location.y, -first.location.z))
    tris = common.tri_count(base) + common.tri_count(first) * len(insts)
    print(f"[{name}] base tris {common.tri_count(base)}, wheel tris {common.tri_count(first)}, "
          f"total {tris}", flush=True)
    # preview
    import mathutils
    pts = [o.matrix_world @ mathutils.Vector(c) for o in ctx.coll.objects for c in o.bound_box]
    lo = [min(p[k] for p in pts) for k in range(3)]
    hi = [max(p[k] for p in pts) for k in range(3)]
    ext = [hi[k] - lo[k] for k in range(3)]
    center = [(lo[k] + hi[k]) / 2 for k in range(3)]
    size = max(ext[0], ext[1], ext[2] * 1.4)
    print(f"[{name}] bbox x {lo[0]:.3f}..{hi[0]:.3f} y {lo[1]:.3f}..{hi[1]:.3f} z {lo[2]:.3f}..{hi[2]:.3f}", flush=True)
    if render:
        v = HERO_VIEW.get(name, (38, 17))
        fit = 2.7
        if os.environ.get("HERO"):
            v0, v1, fit = (float(t) for t in os.environ["HERO"].split(","))
            v = (v0, v1)
        out_png = os.environ.get("HERO_OUT") or common.PREVIEW_DIR / f"robot_{name}.png"
        preview.hero((center[0], center[1], ext[2] * 0.4), size, out_png, az=v[0], el=v[1], fit=fit)
    common.BLEND_DIR.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(common.BLEND_DIR / f"robot_{name}.blend"))


def _bbox(objs):
    import mathutils
    pts = [o.matrix_world @ mathutils.Vector(c) for o in objs for c in o.bound_box]
    lo = [min(p[k] for p in pts) for k in range(3)]
    hi = [max(p[k] for p in pts) for k in range(3)]
    return lo, hi


def build_legged(name, cfg, render=True):
    """Legged robot: the module returns (base, parts) modelled in their link frames; export them, then
    pose linked copies with the forward kinematics (walking snapshot) for the preview and lineup."""
    from robots import legkit
    common.reset_scene()
    common.reset_mat_cache()
    rc = cfg[name]
    ctx = common.Ctx(name)
    mod = importlib.import_module("robots." + module_name(name, cfg))
    importlib.reload(mod)
    base, parts = mod.build(rc, ctx)
    n = legkit.export_links(name, base, parts)
    rb = legkit.robot(name, cfg)
    per_leg = {}
    for leg in rb.legs:
        side = "l" if leg.side > 0 else "r"
        for part in legkit.PARTS:
            key = f"{part}_{side}" if f"{part}_{side}" in n else part
            per_leg[key] = per_leg.get(key, 0) + 1
    total = n["base"] + sum(n[k] * c for k, c in per_leg.items())
    print(f"[{name}] tris " + ", ".join(f"{k} {v}" for k, v in n.items()) + f"; total posed {total}", flush=True)
    if total > 80000:
        print(f"[{name}] WARNING: over the 80k triangle budget", flush=True)
    # preview pose: base at standing height, legs from the FK of a walking snapshot
    bz = float(rc["standing_height"])
    base.location = (0, 0, bz)
    srcs = [o for p in parts.values() for o in (p.values() if isinstance(p, dict) else [p])]
    legkit.place_legs(ctx, rb, parts, legkit.pose_q(rb, os.environ.get("POSE", "walk")), bz)
    for o in srcs:
        ctx.coll.objects.unlink(o)
    bpy.context.view_layer.update()
    lo, hi = _bbox(list(ctx.coll.objects))
    ext = [hi[k] - lo[k] for k in range(3)]
    center = [(lo[k] + hi[k]) / 2 for k in range(3)]
    size = max(ext[0], ext[1], ext[2] * 1.4)
    print(f"[{name}] bbox x {lo[0]:.3f}..{hi[0]:.3f} y {lo[1]:.3f}..{hi[1]:.3f} z {lo[2]:.3f}..{hi[2]:.3f}", flush=True)
    if render:
        v = HERO_VIEW.get(name, (38, 17))
        fit = 2.5
        if os.environ.get("HERO"):
            v0, v1, fit = (float(t) for t in os.environ["HERO"].split(","))
            v = (v0, v1)
        out_png = os.environ.get("HERO_OUT") or common.PREVIEW_DIR / f"robot_{name}.png"
        focus = (center[0], center[1], ext[2] * 0.45)
        if os.environ.get("HERO_FOCUS"):          # close-up for inspection: "x,y,z,size"
            fx, fy, fz, size = (float(t) for t in os.environ["HERO_FOCUS"].split(","))
            focus = (fx, fy, fz)
        preview.hero(focus, size, out_png, az=v[0], el=v[1], fit=fit)
    common.BLEND_DIR.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(common.BLEND_DIR / f"robot_{name}.blend"))


def wheel_inst(src, loc, name, ctx):
    o = common.make_wheel_instance(src, loc, name, ctx)
    return o


def lineup(names):
    common.reset_scene()
    xs = []
    x = None
    prev_hi = 0.0
    for n in names:
        with bpy.data.libraries.load(str(common.BLEND_DIR / f"robot_{n}.blend"), link=False) as (src, dst):
            dst.collections = ["robot_" + n]
        coll = dst.collections[0]
        bpy.context.scene.collection.children.link(coll)
        bpy.context.view_layer.update()
        lo, hi = _bbox(list(coll.objects))
        # side by side along x with a constant gap between the bounding boxes
        shift = 0.0 if x is None else prev_hi + 0.12 - lo[0]
        for o in coll.objects:
            if o.parent is None:
                o.location.x += shift
        x = shift
        prev_hi = hi[0] + shift
        xs.append((lo[0] + shift, hi[0] + shift))
    mid = (xs[0][0] + xs[-1][1]) / 2
    span = xs[-1][1] - xs[0][0]
    coll, tgt = preview.studio((mid, 0, 0.15), 1.7 * span / 4.2, floor_r=12)
    tgt.location = (mid, 0, 0.12)
    # far enough back that both ends (round's skirt, spider's leg tips) stay in frame
    preview.camera((mid, -6.3 * span / 4.2, 1.35 * span / 4.2), tgt, 50)
    preview.render(common.PREVIEW_DIR / "robot_lineup.png", (1920, 700), 48)


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default=None, help="robots.yaml key (default: the built-in robots)")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--lineup", action="store_true")
    ap.add_argument("--lineup-only", action="store_true", help="only re-render robot_lineup.png from saved .blend files")
    a = ap.parse_args(argv)
    cfg = common.load_cfg()
    names = [] if a.lineup_only else ([a.robot] if a.robot else ORDER)
    for n in names:
        build_one(n, cfg, not a.no_render)
    if (a.lineup or a.lineup_only or not a.robot) and not a.no_render:
        lineup(ORDER)


main()
