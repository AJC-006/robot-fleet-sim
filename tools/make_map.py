#!/usr/bin/env python3
"""Rasterise a world's collision primitives into a Nav2 occupancy map.

    python3 tools/make_map.py WORLD [--res 0.05] [--zmin 0.05] [--zmax 0.60]

Reads src/fleet_worlds/models/<WORLD>_env/collision.json and writes
src/fleet_worlds/maps/<WORLD>.{pgm,yaml}. Map frame == Gazebo world frame: the image's lower-left
corner is the world point `origin`, +x right, +y up (image row 0 is the top / max y).
Occupied = 0, free = 254, unknown = 205: the inside of thick obstacles (deeper than --hollow metres from
their surface), like a SLAM map. free_thresh 0.19 keeps 205 (p = 0.196) unknown in trinary mode.
Primitives whose z extent overlaps [zmin, zmax] are rasterised, so the map is exactly what a 2D lidar
mounted anywhere between 0.05 and 0.60 m sees.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "fleet_worlds")


def rasterise(data, res=0.05, zmin=0.05, zmax=0.60):
    x0, y0, x1, y1 = data["bounds"]
    w = int(math.ceil((x1 - x0) / res - 1e-9))
    h = int(math.ceil((y1 - y0) / res - 1e-9))
    ytop = y0 + h * res
    occ = np.zeros((h, w), bool)
    for p in data["primitives"]:
        if "rp" in p and any(abs(a) > 1e-6 for a in p["rp"]):
            continue  # tilted pieces are underwater (lake bed / ramp), never in the ground band
        cx, cy, cz, yaw = p["pose"]
        hz = (p["size"][2] if p["type"] == "box" else p["length"]) / 2
        if cz + hz <= zmin or cz - hz >= zmax:
            continue
        if p["type"] == "box":
            hx, hy = max(p["size"][0] / 2, res / 2 * 1.2), max(p["size"][1] / 2, res / 2 * 1.2)
            rad = math.hypot(hx, hy)
        else:
            rad = max(p["radius"], res * 0.6)
        # window of pixels around the primitive
        c0 = max(int((cx - rad - x0) / res) - 1, 0)
        c1 = min(int((cx + rad - x0) / res) + 2, w)
        r0 = max(int((ytop - (cy + rad)) / res) - 1, 0)
        r1 = min(int((ytop - (cy - rad)) / res) + 2, h)
        if c1 <= c0 or r1 <= r0:
            continue
        xs = x0 + (np.arange(c0, c1) + 0.5) * res
        ys = ytop - (np.arange(r0, r1) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)
        dx, dy = X - cx, Y - cy
        if p["type"] == "box":
            c, s = math.cos(yaw), math.sin(yaw)
            lx, ly = dx * c + dy * s, -dx * s + dy * c
            m = (np.abs(lx) <= hx) & (np.abs(ly) <= hy)
        else:
            m = dx * dx + dy * dy <= rad ** 2
        occ[r0:r1, c0:c1] |= m
    return occ, (x0, y0, w, h)


def crop_reachable(occ, x0, y0, res, seed, radius=0.31, margin=1.5):
    """Keep only what a robot of `radius` can reach from `seed` (x, y): unreachable free pixels (lake interior,
    land beyond the grounds, sealed rooms) become occupied, then crop to the reachable bounding box + margin."""
    from scipy import ndimage
    h, w = occ.shape
    dist = ndimage.distance_transform_edt(~occ) * res
    lab, _ = ndimage.label(dist >= radius)
    sx, sy = int((seed[0] - x0) / res), int(h - 1 - (seed[1] - y0) / res)
    comp = lab[sy, sx]
    assert comp != 0, "seed is not in free space"
    reach = lab == comp
    near = ndimage.distance_transform_edt(~reach) * res <= radius + 3 * res
    keep = near & ~occ
    occ = occ | ~keep
    rows, cols = np.where(keep)
    m = int(margin / res)
    r0, r1 = max(rows.min() - m, 0), min(rows.max() + m + 1, h)
    c0, c1 = max(cols.min() - m, 0), min(cols.max() + m + 1, w)
    ytop = y0 + h * res
    return occ[r0:r1, c0:c1], (x0 + c0 * res, ytop - r1 * res, c1 - c0, r1 - r0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--zmin", type=float, default=0.05)
    ap.add_argument("--zmax", type=float, default=0.60)
    ap.add_argument("--crop-seed", default=None, help="x,y: crop the map to what is reachable from this point")
    ap.add_argument("--hollow", type=float, default=0.30,
                    help="obstacle outline thickness in m; deeper obstacle interior is written as unknown (205) so an "
                         "AMCL particle sitting inside a thick rack/wall block scores badly instead of matching it (0 = off)")
    ap.add_argument("--pkg", default=PKG, help="package dir holding models/ (default: src/fleet_worlds)")
    ap.add_argument("--out", default=None, help="map output dir (default: <pkg>/maps)")
    a = ap.parse_args()
    a.out = a.out or os.path.join(a.pkg, "maps")
    src = os.path.join(a.pkg, "models", a.world + "_env", "collision.json")
    data = json.load(open(src))
    occ, (x0, y0, w, h) = rasterise(data, a.res, a.zmin, a.zmax)
    if a.crop_seed:
        sx, sy = map(float, a.crop_seed.split(","))
        occ, (x0, y0, w, h) = crop_reachable(occ, x0, y0, a.res, (sx, sy))
    img = np.where(occ, 0, 254).astype(np.uint8)
    if a.hollow > 0:
        from scipy import ndimage
        img[occ & (ndimage.distance_transform_edt(occ) * a.res > a.hollow)] = 205
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, a.world + ".pgm"), "wb") as fh:
        fh.write(b"P5\n# robot-fleet-sim %s, %.3f m/px\n%d %d\n255\n" % (a.world.encode(), a.res, w, h))
        fh.write(img.tobytes())
    with open(os.path.join(a.out, a.world + ".yaml"), "w") as fh:
        fh.write("image: %s.pgm\nmode: trinary\nresolution: %s\norigin: [%s, %s, 0.0]\nnegate: 0\n"
                 "occupied_thresh: 0.65\nfree_thresh: 0.19\n" % (a.world, a.res, round(x0, 4), round(y0, 4)))
    print("map %s: %dx%d px, origin (%.3f, %.3f), %.1f%% occupied" % (a.world, w, h, x0, y0, 100.0 * occ.mean()))


if __name__ == "__main__":
    sys.exit(main())
