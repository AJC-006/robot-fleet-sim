"""Per-part geometry statistics of the procedural robot meshes (input of tools/mass_properties.py).

    blender -b --factory-startup -P tools/export_part_stats.py -- [--robot NAME ...]

For every part the robot builder creates before it joins them into base.obj (shell, skirt, posts,
lidar puck, ...), and for the wheel, writes into blender/robots/part_stats/<robot>.json, in the
base_link frame (base mesh frame, z = world z - wheel_radius), SI units:
  name, materials, area, area_centroid, area_second_moment (3x3, about the base_link origin,
  integral of x_i x_j dA), volume (signed, closed meshes only), volume_centroid,
  volume_second_moment (integral of x_i x_j dV), watertight, bbox.
Second moments are exact sums over the triangles (flat triangle: A/12 (sum v_i v_j^T + (sum v)(sum v)^T)
and tetrahedron to the origin: V/20 (sum v_i v_j^T + (sum v)(sum v)^T)).
"""
import importlib
import json
import sys
from pathlib import Path

import bpy
import bmesh
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "blender"))
from robots import common  # noqa: E402

MODULES = {"round": "pebble", "box": "brick", "rover": "rover", "truck": "truck"}
OUT = REPO / "blender" / "robots" / "part_stats"


def stats(ob, dz):
    dg = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(dg)
    me = ev.to_mesh()
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    mw = np.array(ob.matrix_world)
    shift = np.array([0.0, 0.0, -dz])
    A = 0.0
    cA = np.zeros(3)
    SA = np.zeros((3, 3))
    V = 0.0
    cV = np.zeros(3)
    SV = np.zeros((3, 3))
    pts = []
    for f in bm.faces:
        v = [mw[:3, :3] @ np.array(l.vert.co) + mw[:3, 3] + shift for l in f.loops]
        pts += v
        a, b, c = v
        area = np.linalg.norm(np.cross(b - a, c - a)) / 2
        s = a + b + c
        A += area
        cA += area * s / 3
        SA += area / 12 * (np.outer(a, a) + np.outer(b, b) + np.outer(c, c) + np.outer(s, s))
        vol = np.dot(a, np.cross(b, c)) / 6
        V += vol
        cV += vol * s / 4
        SV += vol / 20 * (np.outer(a, a) + np.outer(b, b) + np.outer(c, c) + np.outer(s, s))
    watertight = all(e.is_manifold for e in bm.edges)
    mats = sorted({m.name for m in me.materials if m})
    bm.free()
    ev.to_mesh_clear()
    pts = np.array(pts) if pts else np.zeros((1, 3))
    return {"name": ob.name, "materials": mats, "area": A, "area_centroid": (cA / A).tolist() if A else [0, 0, 0],
            "area_second_moment": SA.tolist(), "volume": V,
            "volume_centroid": (cV / V).tolist() if abs(V) > 1e-12 else [0, 0, 0],
            "volume_second_moment": SV.tolist(), "watertight": bool(watertight),
            "bbox": [pts.min(0).tolist(), pts.max(0).tolist()]}


def run(name, cfg):
    common.reset_scene()
    common.reset_mat_cache()
    rc = cfg[name]
    r = rc["wheel_radius"]
    ctx = common.Ctx(name)
    mod = importlib.import_module("robots." + (cfg[name].get("builder") or MODULES[name]))
    importlib.reload(mod)
    rec = {}
    real_join = mod.join

    def hook(c, objs, nm, *a, **k):
        objs = list(objs)
        bpy.context.view_layer.update()
        if nm == "base":
            rec["parts"] = [stats(o, r) for o in objs]
        return real_join(c, objs, nm, *a, **k)

    mod.join = hook
    base, wheel = mod.build(rc, ctx)
    bpy.context.view_layer.update()
    wheel_stats = stats(wheel, 0.0)   # wheel mesh is built about its own axle (origin at the axle)
    out = {"robot": name, "frame": "base_link (z = world z - wheel_radius)", "parts": rec["parts"],
           "wheel": wheel_stats}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{name}.json").write_text(json.dumps(out, indent=1))
    print(f"[{name}] {len(rec['parts'])} parts, wheel area {wheel_stats['area']:.4f} volume {wheel_stats['volume']:.6f}")


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    names = [argv[i + 1] for i, a in enumerate(argv) if a == "--robot"] or list(MODULES)
    cfg = common.load_cfg()
    for n in names:
        run(n, cfg)
