"""Reusable procedural parts: wheels, lidar puck, camera."""
import math

import bmesh
import bpy
from mathutils import Matrix

from .common import (AXES, _obj_from_bm, bevel, box, cyl, join, lathe, mat, sphere, tube, torus)


def lug_ring(ctx, name, n, r, w, h, tang, material, pattern="straight", ang=0.5, axial=None, bev=0.0015):
    """Tread blocks around a wheel of radius r (axis Y after export). pattern: straight,
    chevron (two angled halves), stagger (two rows offset half a step)."""
    axial = axial or w * 0.92
    bm = bmesh.new()

    def add(a, zoff, length, rotx=0.0, tangent=tang):
        m = (Matrix.Rotation(a, 4, "Z") @ Matrix.Translation((r + h / 2 - 0.0008, 0, zoff))
             @ Matrix.Rotation(rotx, 4, "X") @ Matrix.Diagonal((h, tangent, length, 1)))
        bmesh.ops.create_cube(bm, size=1.0, matrix=m)

    for i in range(n):
        a = 2 * math.pi * i / n
        if pattern == "straight":
            add(a, 0, axial)
        elif pattern == "chevron":
            add(a, axial / 4, axial / 2 * 1.15, rotx=ang)
            add(a, -axial / 4, axial / 2 * 1.15, rotx=-ang)
        elif pattern == "stagger":
            add(a, axial / 4, axial / 2 * 0.95)
            add(a + math.pi / n, -axial / 4, axial / 2 * 0.95)
    bmesh.ops.transform(bm, matrix=AXES["y"], verts=bm.verts)
    o = _obj_from_bm(name, bm, material, (0, 0, 0), (0, 0, 0), ctx)
    return bevel(o, bev, 1, 50)


def tire_body(ctx, name, r, w, material, round_=0.006, bulge=0.0, seg=40):
    """Solid tyre disc (axis Y)."""
    k = round_
    prof = [(0, -w / 2), (r - k, -w / 2), (r - k * 0.3 + bulge, -w / 2 + k * 0.3), (r + bulge, -w / 2 + k),
            (r + bulge, w / 2 - k), (r - k * 0.3 + bulge, w / 2 - k * 0.3), (r - k, w / 2), (0, w / 2)]
    return lathe(ctx, name, prof, (0, 0, 0), material, seg=seg, axis="y")


def wheel(ctx, style, r, w, name="wheel"):
    """Wheel centred at the origin, axis along Y, symmetric left/right."""
    parts = []
    R, r = r, r - {"pebble": 0.003, "brick": 0.007, "rover": 0.005, "truck": 0.011}[style]
    if style == "pebble":
        parts.append(tire_body(ctx, "tire", r, w, "rubber", round_=0.006))
        parts.append(lug_ring(ctx, "lugs", 20, r, w, 0.003, 0.006, "rubber", "chevron", 0.5))
        for s in (-1, 1):
            parts.append(cyl(ctx, "hub", r * 0.55, 0.003, (0, s * (w / 2 + 0.0005), 0), "violetmetal", axis="y", seg=24))
            parts.append(cyl(ctx, "cap", r * 0.18, 0.004, (0, s * (w / 2 + 0.002), 0), "steel", axis="y", seg=16))
    elif style == "brick":
        parts.append(tire_body(ctx, "tire", r, w, "rubber", round_=0.007))
        parts.append(lug_ring(ctx, "lugs", 18, r, w, 0.007, 0.012, "rubber", "stagger"))
        for s in (-1, 1):
            parts.append(cyl(ctx, "hub", r * 0.62, 0.004, (0, s * (w / 2 + 0.001), 0), "darksteel", axis="y", seg=24, bev=0.0008))
            parts.append(cyl(ctx, "ring", r * 0.62, 0.005, (0, s * (w / 2 + 0.003), 0), "violet", axis="y", seg=24, r2=r * 0.5))
            for k in range(6):
                a = 2 * math.pi * k / 6
                parts.append(cyl(ctx, "bolt", 0.0038, 0.005, (r * 0.38 * math.cos(a), s * (w / 2 + 0.005), r * 0.38 * math.sin(a)),
                                 "steel", axis="y", seg=8))
    elif style == "rover":
        ri = r - 0.005
        parts.append(lathe(ctx, "skin", [(ri, -w / 2), (r, -w / 2), (r, w / 2), (ri, w / 2)], (0, 0, 0),
                           "steel", seg=48, axis="y", loop=True, bev=0.0006))
        parts.append(lug_ring(ctx, "cleats", 24, r, w, 0.005, 0.005, "darksteel", "chevron", 0.45, bev=0.0006))
        for s in (-1, 1):
            parts.append(lathe(ctx, "rim", [(ri - 0.004, s * w / 2), (r, s * w / 2), (r, s * (w / 2 - 0.004)),
                                            (ri - 0.004, s * (w / 2 - 0.004))], (0, 0, 0), "darksteel", seg=48,
                               axis="y", loop=True))
        parts.append(cyl(ctx, "hubcore", r * 0.28, w * 0.9, (0, 0, 0), "darksteel", axis="y", seg=20, bev=0.001))
        parts.append(cyl(ctx, "cap", r * 0.16, w * 0.95, (0, 0, 0), "violetmetal", axis="y", seg=16))
        for k in range(8):
            a = 2 * math.pi * k / 8
            for yy in (-w * 0.28, w * 0.28):
                parts.append(tube(ctx, "spoke", (r * 0.25 * math.cos(a), yy, r * 0.25 * math.sin(a)),
                                  (ri * math.cos(a + 0.35), yy * 0.9, ri * math.sin(a + 0.35)), 0.0032, "steel", seg=6))
    elif style == "truck":
        parts.append(tire_body(ctx, "tire", r, w, "rubber", round_=0.012, bulge=0.0, seg=40))
        parts.append(lug_ring(ctx, "lugs", 16, r, w, 0.011, 0.016, "rubber", "stagger", bev=0.002))
        for s in (-1, 1):
            parts.append(cyl(ctx, "rimface", r * 0.62, 0.006, (0, s * (w / 2 - 0.001), 0), "violetmetal", axis="y", seg=32, bev=0.001))
            parts.append(torus(ctx, "lip", r * 0.6, 0.0035, (0, s * (w / 2 + 0.002), 0), "steel", major=32, minor=8, axis="y"))
            parts.append(cyl(ctx, "hubcap", r * 0.2, 0.008, (0, s * (w / 2 + 0.003), 0), "steel", axis="y", seg=16, bev=0.001))
            for k in range(5):
                a = 2 * math.pi * k / 5
                parts.append(cyl(ctx, "lug", 0.0035, 0.007, (r * 0.4 * math.cos(a), s * (w / 2 + 0.004), r * 0.4 * math.sin(a)),
                                 "steel", axis="y", seg=6))
    else:
        raise ValueError(style)
    return join(ctx, parts, name)


def lidar_puck(ctx, loc, r=0.036, h=0.04, accent="led_violet"):
    """Spinning-lidar look-alike; loc = the sensor plane centre (smoked window band centred on it).
    Returns the part objects (caller joins them into the base)."""
    x, y, z = loc
    return [
        cyl(ctx, "lidar_base", r, h * 0.28, (x, y, z - h * 0.36), "graphite", seg=32, bev=0.0015),
        cyl(ctx, "lidar_win", r * 0.93, h * 0.44, (x, y, z), "glass", seg=32),
        cyl(ctx, "lidar_cap", r * 0.97, h * 0.28, (x, y, z + h * 0.36), "graphite", seg=32, bev=0.0015),
        cyl(ctx, "lidar_dome", r * 0.55, h * 0.12, (x, y, z + h * 0.56), "plastic", seg=24, r2=r * 0.4, bev=0.001),
        torus(ctx, "lidar_led", r * 0.97, 0.0014, (x, y, z - h * 0.2), accent, major=32, minor=6),
    ]


def camera_box(ctx, loc, material="graphite"):
    x, y, z = loc
    return [box(ctx, "cam_body", (0.022, 0.05, 0.026), (x, y, z), material, bev=0.003),
            cyl(ctx, "cam_lens", 0.009, 0.008, (x + 0.014, y, z), "lens", axis="x", seg=16),
            cyl(ctx, "cam_ring", 0.0115, 0.004, (x + 0.011, y, z), "steel", axis="x", seg=16)]
