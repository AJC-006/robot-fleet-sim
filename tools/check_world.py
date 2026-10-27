#!/usr/bin/env python3
"""Validate a world's map + yaml: spawn/goal clearance, reachability on the inflated map, PNG render.

    python3 tools/check_world.py WORLD [--inflate 0.28] [--clearance 0.6] [--png OUT.png]
"""
import argparse
import json
import math
import os
import sys
from collections import deque

import numpy as np
import yaml
from scipy import ndimage

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "fleet_worlds")


def load_pgm(path):
    with open(path, "rb") as fh:
        assert fh.readline().strip() == b"P5"
        line = fh.readline()
        while line.startswith(b"#"):
            line = fh.readline()
        w, h = map(int, line.split())
        fh.readline()
        return np.frombuffer(fh.read(), np.uint8).reshape(h, w)


def prepare(prims):
    out = []
    for p in prims:
        cx, cy, cz, yaw = p["pose"]
        r, pt = p.get("rp", (0.0, 0.0))
        cyw, syw, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pt), math.sin(pt), math.cos(r), math.sin(r)
        R = np.array([[cyw * cp, cyw * sp * sr - syw * cr, cyw * sp * cr + syw * sr],
                      [syw * cp, syw * sp * sr + cyw * cr, syw * sp * cr - cyw * sr],
                      [-sp, cp * sr, cp * cr]])
        if p["type"] == "box":
            half = np.array(p["size"]) / 2
        else:
            half = np.array([p["radius"], p["radius"], p["length"] / 2])
        out.append((np.array([cx, cy, cz]), R, half, p["type"] == "cylinder", p.get("rp") is not None and any(abs(v) > 1e-6 for v in p["rp"])))
    return out


def nearest(P, x, y, z):
    """(distance to the nearest primitive, lowest underside above (x, y, z) if the point is under one)."""
    q = np.array([x, y, z])
    dmin, ceil = 1e9, 1e9
    for c, R, half, cyl, tilted in P:
        l = R.T @ (q - c)
        if cyl:
            e = np.array([max(math.hypot(l[0], l[1]) - half[0], 0.0), 0.0, max(abs(l[2]) - half[2], 0.0)])
            inside = e[0] == 0
        else:
            e = np.maximum(np.abs(l) - half, 0.0)
            inside = e[0] == 0 and e[1] == 0
        d = float(np.linalg.norm(e))
        dmin = min(dmin, d)
        if inside and not tilted and c[2] - half[2] > z:
            ceil = min(ceil, c[2] - half[2])
    return dmin, ceil


def catalog_lidars(names=None):
    """{catalog stem: (range_max m, samples per revolution)} for the lidars the robots in robots.yaml use."""
    comps = os.path.join(ROOT, "src", "fleet_components", "components", "lidar")
    if names is None:
        robots = yaml.safe_load(open(os.path.join(ROOT, "src", "fleet_description", "config", "robots.yaml")))
        names = sorted({r["components"]["lidar"] for r in robots.values()
                        if isinstance(r, dict) and "lidar" in r.get("components", {})})
    out = {}
    for n in names:
        sim = yaml.safe_load(open(os.path.join(comps, n + ".yaml")))["sim"]
        out[n] = (float(sim["range_max"]), int(sim["samples_per_rev"]))
    return out


def lidar_check(a, occ, dist, res, ox, oy, h, w, lab, comp):
    ok = True
    for name, (rng_m, nb) in catalog_lidars(a.lidar_model).items():
        print("lidar %s (range %.0f m, %d beams)" % (name, rng_m, nb))
        ok &= lidar_check_one(a, occ, dist, res, ox, oy, h, w, lab, comp, rng_m, nb)
    return ok


def lidar_check_one(a, occ, dist, res, ox, oy, h, w, lab, comp, rng_m, nb):
    """Ray-cast nb beams (rng_m) from reachable sample points on the map; every point needs >= lidar-min hits."""
    step = max(int(a.lidar_step / res), 1)
    reach = (lab == comp)
    rr, cc = np.where(reach)
    sel = (rr % step == 0) & (cc % step == 0)
    rr, cc = rr[sel], cc[sel]
    ang = np.arange(nb) * 2 * math.pi / nb
    ks = np.arange(1, int(rng_m / res) + 1)
    dc = np.cos(ang)[:, None] * ks[None, :]
    dr = -np.sin(ang)[:, None] * ks[None, :]
    frac = np.zeros(len(rr))
    for i0 in range(0, len(rr), 64):
        r = rr[i0:i0 + 64, None, None] + dr[None]
        c = cc[i0:i0 + 64, None, None] + dc[None]
        ri, ci = np.rint(r).astype(int), np.rint(c).astype(int)
        inb = (ri >= 0) & (ri < h) & (ci >= 0) & (ci < w)
        hit = np.zeros(ri.shape, bool)
        hit[inb] = occ[ri[inb], ci[inb]]
        frac[i0:i0 + 64] = hit.any(axis=2).mean(axis=1)
    bad = frac < a.lidar_min
    print("lidar: %d sample points (%.1f m grid), hit fraction min %.2f, p5 %.2f, median %.2f; below %.0f%%: %d" %
          (len(frac), a.lidar_step, frac.min(), np.percentile(frac, 5), np.median(frac), a.lidar_min * 100, bad.sum()))
    if bad.any():
        order = np.argsort(frac)[:80]
        for k in order:
            if frac[k] < a.lidar_min:
                print("  LOW %.2f at (%.1f, %.1f)" % (frac[k], ox + (cc[k] + 0.5) * res, oy + (h - rr[k] - 0.5) * res))
    if a.png:
        np.save(a.png + ".lidar.npy", np.stack([ox + (cc + 0.5) * res, oy + (h - rr - 0.5) * res, frac], 1))
    return not bad.any()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world")
    ap.add_argument("--inflate", type=float, default=0.28)
    ap.add_argument("--clearance", type=float, default=0.6)
    ap.add_argument("--png")
    ap.add_argument("--lidar", action="store_true", help="ray-cast check: >= 30%% of the beams (catalog range) hit something from every reachable point")
    ap.add_argument("--lidar-step", type=float, default=1.0, help="sample spacing in m for --lidar")
    ap.add_argument("--lidar-min", type=float, default=0.30)
    ap.add_argument("--lidar-model", action="append",
                    help="catalog lidar stem to check (repeatable; default: every lidar a robot in robots.yaml uses, "
                         "with its catalog range and beam count)")
    ap.add_argument("--pkg", default=PKG, help="package dir (default: src/fleet_worlds)")
    a = ap.parse_args()
    PKGD = a.pkg
    wy = yaml.safe_load(open(os.path.join(PKGD, "worlds", a.world + ".yaml")))
    my = yaml.safe_load(open(os.path.join(PKGD, "maps", wy["map"])))
    img = load_pgm(os.path.join(PKGD, "maps", my["image"]))
    res, (ox, oy, _) = my["resolution"], my["origin"]
    h, w = img.shape
    occ = img < 250  # occupied (0) or unknown (205, inside thick obstacles): neither is drivable
    dist = ndimage.distance_transform_edt(~occ) * res  # metres to nearest occupied pixel

    def to_px(x, y):
        return int((x - ox) / res), int(h - 1 - (y - oy) / res)

    infl = dist < a.inflate
    free = ~infl
    pts = [("spawn", wy["spawn"]["x"], wy["spawn"]["y"], wy["spawn"]["yaw"])] + [(g["name"], g["x"], g["y"], g["yaw"]) for g in wy["goals"]]
    ok = True
    # flood fill from spawn on the inflated map
    sx, sy = to_px(pts[0][1], pts[0][2])
    lab, _ = ndimage.label(free)
    comp = lab[sy, sx]
    print("%s: map %dx%d, origin (%.2f, %.2f)" % (a.world, w, h, ox, oy))
    for (n, x, y, yaw) in pts:
        px, py = to_px(x, y)
        d = dist[py, px]
        reach = comp != 0 and lab[py, px] == comp
        flag = "OK " if (d >= a.clearance and reach) else "BAD"
        if flag == "BAD":
            ok = False
        print("  %s %-14s (%6.2f,%6.2f) clearance %.2f m  reachable=%s" % (flag, n, x, y, d, reach))
    print("free area reachable from spawn: %.1f m^2 of %.1f m^2 free" % ((lab == comp).sum() * res * res, (~occ).sum() * res * res))
    # loop check: cycles in the free space show as holes (enclosed obstacle components) inside the reachable area
    reach = lab == comp
    holes = ndimage.label(~reach & ~ndimage.binary_dilation(~reach, iterations=0) )[1]
    outer_bg = ndimage.label(~reach)[0]
    border = set(np.unique(np.concatenate([outer_bg[0], outer_bg[-1], outer_bg[:, 0], outer_bg[:, -1]])))
    n_islands = len([i for i in np.unique(outer_bg) if i != 0 and i not in border])
    print("obstacle islands the robot can circle (loops): %d" % n_islands)
    prims = json.load(open(os.path.join(PKGD, "models", a.world + "_env", "collision.json")))["primitives"]
    P = prepare(prims)
    drone = wy.get("drone")
    if drone:
        wps = [("takeoff", drone["takeoff"]["x"], drone["takeoff"]["y"], 0.5)] + [(w["name"], w["x"], w["y"], w["z"]) for w in drone["waypoints"]]
        print("drone: cruise %.1f m, %d waypoints" % (drone["cruise_altitude"], len(wps) - 1))
        for (n, x, y, z) in wps:
            dmin, ceil = nearest(P, x, y, z)
            ok_w = dmin >= (1.5 if n != "takeoff" else 0.5) and (ceil - z >= 1.5 or n == "takeoff")
            if not ok_w:
                ok = False
            print("  %s %-14s (%6.2f,%6.2f,%5.2f) nearest obstacle %.2f m, ceiling %s" % ("OK " if ok_w else "BAD", n, x, y, z, dmin, "none" if ceil > 1e8 else "%.2f m" % ceil))
    auv = wy.get("auv")
    if auv:
        wl = auv["water_level"]
        sp = auv["spawn"]
        pts_a = [("auv_spawn", sp["x"], sp["y"], sp["z"])] + [(w["name"], w["x"], w["y"], w["z"]) for w in auv["waypoints"]]
        print("auv: water level %.2f, %d waypoints" % (wl, len(pts_a) - 1))
        for (n, x, y, z) in pts_a:
            dmin, _ = nearest(P, x, y, z)
            ok_w = dmin >= 2.0 and z <= wl - 1.0
            if not ok_w:
                ok = False
            print("  %s %-14s (%6.2f,%6.2f,%6.2f) nearest obstacle/bed %.2f m, depth below surface %.2f m" % ("OK " if ok_w else "BAD", n, x, y, z, dmin, wl - z))
    if a.lidar:
        ok &= lidar_check(a, occ, dist, res, ox, oy, h, w, lab, comp)
    if a.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 11 * h / w))
        vis = np.where(occ, 0.0, np.where(infl, 0.55, 1.0))
        ax.imshow(vis, cmap="gray", vmin=0, vmax=1, extent=(ox, ox + w * res, oy, oy + h * res))
        for (n, x, y, yaw) in pts:
            c = "lime" if n == "spawn" else "red"
            ax.plot(x, y, "o", color=c, ms=7)
            ax.arrow(x, y, 0.5 * math.cos(yaw), 0.5 * math.sin(yaw), color=c, head_width=0.15)
            ax.annotate(n, (x, y), color="blue", fontsize=8, xytext=(4, 4), textcoords="offset points")
        ax.set_title("%s: black=occupied, grey=inflated %.2f m" % (a.world, a.inflate))
        fig.savefig(a.png, dpi=80, bbox_inches="tight")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
