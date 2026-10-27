#!/usr/bin/env python3
"""Find visual geometry the gpu_lidar sees (0.05-0.60 m band) that the occupancy map does not contain.

    python3 tools/audit_visual.py WORLD [--tol 0.15] [--min-cells 2] [--zmin 0.05 --zmax 0.6]

Parses the world's glTF/GLB files (models/<w>_env/meshes), clips every non-horizontal triangle to the z band,
rasterises its footprint at the map resolution and lists clusters of cells that are further than --tol from any
occupied map cell. Also lists occupied map cells with no visual within --tol (collision without a visual).
"""
import argparse, glob, json, math, os, struct, sys
import numpy as np, cv2, yaml
from scipy import ndimage
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_world import load_pgm, PKG

CT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
NC = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def load_gltf(path):
    if path.endswith(".glb"):
        b = open(path, "rb").read()
        jl, = struct.unpack("<I", b[12:16]); js = json.loads(b[20:20 + jl]); off = 20 + jl
        binb = b[off + 8:] if off < len(b) else b""
    else:
        js = json.load(open(path)); binb = None
    bufs = []
    for bf in js.get("buffers", []):
        bufs.append(open(os.path.join(os.path.dirname(path), bf["uri"]), "rb").read() if "uri" in bf else binb)
    return js, bufs


def acc(js, bufs, i):
    a = js["accessors"][i]; bv = js["bufferViews"][a["bufferView"]]
    dt = CT[a["componentType"]]; n = NC[a["type"]]
    stride = bv.get("byteStride", np.dtype(dt).itemsize * n)
    base = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
    buf = bufs[bv["buffer"]]
    if stride == np.dtype(dt).itemsize * n:
        return np.frombuffer(buf, dt, a["count"] * n, base).reshape(a["count"], n)
    return np.stack([np.frombuffer(buf, dt, n, base + k * stride) for k in range(a["count"])])


def node_mat(nd):
    if "matrix" in nd:
        return np.array(nd["matrix"]).reshape(4, 4).T
    M = np.eye(4)
    if "scale" in nd: M = np.diag(list(nd["scale"]) + [1]) @ M
    if "rotation" in nd:
        x, y, z, w = nd["rotation"]
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        T = np.eye(4); T[:3, :3] = R; M = T @ M
    if "translation" in nd:
        T = np.eye(4); T[:3, 3] = nd["translation"]; M = T @ M
    return M


def triangles(path):
    js, bufs = load_gltf(path)
    out = []
    def walk(i, P):
        nd = js["nodes"][i]; M = P @ node_mat(nd)
        if "mesh" in nd:
            for pr in js["meshes"][nd["mesh"]]["primitives"]:
                pos = acc(js, bufs, pr["attributes"]["POSITION"]).astype(float)
                idx = acc(js, bufs, pr["indices"]).reshape(-1).astype(int) if "indices" in pr else np.arange(len(pos))
                w = (np.c_[pos, np.ones(len(pos))] @ M.T)[:, :3]
                out.append(w[idx].reshape(-1, 3, 3))
        for c in nd.get("children", []): walk(c, M)
    for s in js["scenes"][js.get("scene", 0)]["nodes"]: walk(s, np.eye(4))
    return np.concatenate(out) if out else np.zeros((0, 3, 3))


def clip_z(poly, lo, hi):
    for axis_sign, lim in ((1, lo), (-1, -hi)):
        res = []
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            da, db = axis_sign * a[2] - lim, axis_sign * b[2] - lim
            if da >= 0: res.append(a)
            if (da >= 0) != (db >= 0):
                t = da / (da - db); res.append(a + t * (b - a))
        poly = res
        if not poly: return []
    return poly


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world"); ap.add_argument("--pkg", default=PKG)
    ap.add_argument("--tol", type=float, default=0.15); ap.add_argument("--min-cells", type=int, default=2)
    ap.add_argument("--zmin", type=float, default=0.05); ap.add_argument("--zmax", type=float, default=0.60)
    ap.add_argument("--top", type=int, default=60)
    ap.add_argument("--both", action="store_true")
    ap.add_argument("--skip", default="keep_lake,keep_sky,keep_water")
    a = ap.parse_args()
    wy = yaml.safe_load(open(os.path.join(a.pkg, "worlds", a.world + ".yaml")))
    my = yaml.safe_load(open(os.path.join(a.pkg, "maps", wy["map"])))
    img = load_pgm(os.path.join(a.pkg, "maps", my["image"])); res = my["resolution"]; ox, oy, _ = my["origin"]; h, w = img.shape
    occ = img < 250
    vis = np.zeros((h, w), np.uint8)
    files = sorted(glob.glob(os.path.join(a.pkg, "models", a.world + "_env", "meshes", "*.gl*")))
    skip = a.skip.split(",")
    for f in files:
        if any(os.path.basename(f).startswith(s) for s in skip): continue
        T = triangles(f)
        if len(T) == 0: continue
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); nl = np.linalg.norm(n, axis=1) + 1e-12
        keep = (np.abs(n[:, 2]) / nl < 0.9) & (T[:, :, 2].max(1) > a.zmin) & (T[:, :, 2].min(1) < a.zmax)
        for t in T[keep]:
            p = clip_z(list(t), a.zmin, a.zmax)
            if len(p) < 3: continue
            pts = np.array([[(q[0] - ox) / res, h - (q[1] - oy) / res] for q in p])
            if pts[:, 0].max() < 0 or pts[:, 1].max() < 0 or pts[:, 0].min() > w or pts[:, 1].min() > h: continue
            q = np.round(pts * 16).astype(np.int32)
            cv2.fillConvexPoly(vis, q, 1, shift=4)
            cv2.polylines(vis, [q], True, 1, 1, shift=4)
    vis = vis > 0
    print("vis cells", int(vis.sum()), "occ cells", int(occ.sum()), "files", len(files))
    d_occ = ndimage.distance_transform_edt(~occ) * res
    d_vis = ndimage.distance_transform_edt(~vis) * res
    checks = [("VISUAL-ONLY (lidar sees, map free)", vis & (d_occ > a.tol))]
    if a.both:
        checks.append(("COLLISION-ONLY (map occupied, no visual)", (img == 0) & (d_vis > a.tol * 2)))
    for name, bad in checks:
        lab, n = ndimage.label(bad, np.ones((3, 3)))
        print("%s: %d clusters" % (name, n))
        rows = []
        for i in range(1, n + 1):
            rr, cc = np.where(lab == i)
            if len(rr) < a.min_cells: continue
            rows.append((len(rr), d_occ[rr, cc].max(), ox + (cc.mean() + .5) * res, oy + (h - rr.mean() - .5) * res,
                         ox + cc.min() * res, ox + (cc.max() + 1) * res, oy + (h - rr.max() - 1) * res, oy + (h - rr.min()) * res))
        rows.sort(key=lambda r: -r[1] * min(r[0], 20))
        for r in rows[:a.top]:
            print("  %5d cells, %.2f m from map at (%.1f, %.1f)  x[%.1f,%.1f] y[%.1f,%.1f]" % r)
    return 0


if __name__ == "__main__":
    sys.exit(main())
