"""wyvern_route_check: clearance of the flown route (every segment, not just the waypoints) against a world's
collision primitives (fleet_worlds models/<world>_env/collision.json).

    ros2 run fleet_drone wyvern_route_check lab warehouse keep apartment [--min 1.0]
"""
import argparse
import math
import sys

import numpy as np

from fleet_drone import route as R


def prepare(prims):
    out = []
    for p in prims:
        if p["type"] == "box" and p["pose"][2] + p["size"][2] / 2 <= 0.05:
            continue  # floor slabs
        cx, cy, cz, yaw = p["pose"]
        r, pt = p.get("rp", (0.0, 0.0))
        cyw, syw, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pt), math.sin(pt), math.cos(r), math.sin(r)
        Rm = np.array([[cyw * cp, cyw * sp * sr - syw * cr, cyw * sp * cr + syw * sr],
                       [syw * cp, syw * sp * sr + cyw * cr, syw * sp * cr - cyw * sr],
                       [-sp, cp * sr, cp * cr]])
        half = np.array(p["size"]) / 2 if p["type"] == "box" else np.array([p["radius"], p["radius"], p["length"] / 2])
        out.append((np.array([cx, cy, cz]), Rm, half, p["type"] == "cylinder"))
    return out


def clearance(P, q):
    d = 1e9
    for c, Rm, half, cyl in P:
        l = Rm.T @ (q - c)
        if cyl:
            e = np.array([max(math.hypot(l[0], l[1]) - half[0], 0.0), 0.0, max(abs(l[2]) - half[2], 0.0)])
        else:
            e = np.maximum(np.abs(l) - half, 0.0)
        d = min(d, float(np.linalg.norm(e)))
    return d


def check(world, mission_file="", step=0.25, verbose=True):
    m = R.load_mission(world, mission_file)
    allp = R.load_primitives(world)
    P = prepare(allp)
    g = R.ground_height(allp, m["takeoff"]["x"], m["takeoff"]["y"])
    pts = R.route_points(m, g)
    worst = 1e9
    rows = []
    for a, b in zip(pts[:-1], pts[1:]):
        A, B = np.array(a[1:4]), np.array(b[1:4])
        L = float(np.linalg.norm(B - A))
        n = max(int(L / step), 1)
        dm, at = 1e9, A
        for i in range(n + 1):
            q = A + (B - A) * i / n
            d = clearance(P, q)
            if d < dm:
                dm, at = d, q
        # the vertical takeoff/landing legs start at the floor: ignore the last 0.5 m above ground there
        rows.append((a[0], b[0], L, dm, at))
        if a[0] not in ("ground",) and b[0] not in ("land",):
            worst = min(worst, dm)
    if verbose:
        print("%s: %s, cruise %.1f m, %d waypoints, ground %.2f m" % (world, m["source"], m["cruise_altitude"], len(m["waypoints"]), g))
        for (an, bn, L, dm, at) in rows:
            print("  %-14s -> %-14s %6.1f m  min clearance %5.2f m at (%.1f, %.1f, %.1f)" % (an, bn, L, dm, *at))
    return worst, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("worlds", nargs="+")
    ap.add_argument("--min", type=float, default=1.0, help="required clearance in metres (drone radius 0.42 m)")
    a = ap.parse_args()
    ok = True
    for w in a.worlds:
        worst, _ = check(w)
        good = worst >= a.min
        ok &= good
        print("  -> %s worst en-route clearance %.2f m (required %.2f)" % ("OK " if good else "BAD", worst, a.min))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
