"""box: Brick - boxy rover with chunky wheels, one front caster and a lidar mast."""
import math

from .common import box, cut, cyl, join, sphere, torus, tube
from . import parts


def build(cfg, ctx):
    r_w, w_w, sep = cfg["wheel_radius"], cfg["wheel_width"], cfg["wheel_separation"]
    lid, cam = cfg["lidar"], cfg["camera"]
    P = []
    # chassis tub + hood
    P.append(box(ctx, "tub", (0.32, 0.24, 0.05), (0.02, 0, 0.075), "graphite", bev=0.004))
    P.append(box(ctx, "hood", (0.30, 0.232, 0.04), (0.02, 0, 0.12), "white", bev=0.009, seg=3))
    P.append(box(ctx, "stripe", (0.30, 0.04, 0.0015), (0.02, 0, 0.1408), "violet", bev=0))
    # bumpers
    P.append(box(ctx, "bump_f", (0.02, 0.25, 0.045), (0.185, 0, 0.088), "violet", bev=0.006))
    P.append(box(ctx, "bump_r", (0.02, 0.25, 0.045), (-0.145, 0, 0.088), "graphite", bev=0.006))
    # fenders over the wheels
    for s in (-1, 1):
        P.append(box(ctx, "fender", (0.14, 0.052, 0.008), (0.0, s * sep / 2, 2 * r_w + 0.012), "graphite", bev=0.003))
        P.append(box(ctx, "fender_edge", (0.14, 0.003, 0.016), (0.0, s * (sep / 2 + 0.026), 2 * r_w + 0.004), "violet", bev=0.001))
        P.append(box(ctx, "fender_in", (0.14, 0.012, 0.03), (0.0, s * (sep / 2 - 0.023), 2 * r_w + 0.000), "graphite", bev=0.002))
        # axle stubs
        P.append(cyl(ctx, "axle", 0.012, 0.03, (0, s * (sep / 2 - 0.03), r_w), "darksteel", axis="y", seg=16))
        # headlights
        P.append(box(ctx, "headlight", (0.006, 0.04, 0.016), (0.196, s * 0.075, 0.098), "led_cyan", bev=0.002))
        # tail lights
        P.append(box(ctx, "taillight", (0.006, 0.035, 0.012), (-0.156, s * 0.075, 0.098), "led_violet", bev=0.002))
        # underglow
        P.append(box(ctx, "glow", (0.20, 0.004, 0.004), (0.02, s * 0.1205, 0.058), "led_violet", bev=0.0))
    # top hardware: hatch with bolts, vent slats
    P.append(box(ctx, "hatch", (0.09, 0.11, 0.004), (0.085, 0, 0.142), "graphite", bev=0.0015))
    for sx in (-1, 1):
        for sy in (-1, 1):
            P.append(cyl(ctx, "bolt", 0.004, 0.003, (0.085 + sx * 0.037, sy * 0.047, 0.1445), "steel", seg=8))
    for k in range(5):
        P.append(box(ctx, "vent", (0.006, 0.07, 0.003), (-0.085 + k * 0.012, 0, 0.1418), "plastic", bev=0.0008))
    # sensor mast carrying the lidar
    mx, my = lid["x"], lid["y"]
    P.append(cyl(ctx, "mast_flange", 0.026, 0.008, (mx, my, 0.144), "darksteel", seg=24, bev=0.002))
    P.append(cyl(ctx, "mast", 0.011, lid["z"] - 0.144 - 0.015, (mx, my, 0.144 + (lid["z"] - 0.144 - 0.015) / 2), "steel", seg=16))
    for a in (0, 2.094, 4.188):
        P.append(tube(ctx, "gusset", (mx + 0.022 * math.cos(a), my + 0.022 * math.sin(a), 0.146),
                      (mx + 0.008 * math.cos(a), my + 0.008 * math.sin(a), 0.19), 0.003, "darksteel", seg=6))
    P += parts.lidar_puck(ctx, (mx, my, lid["z"]), r=0.037, h=0.04)
    # antenna (stays well below the lidar plane)
    P.append(tube(ctx, "ant", (-0.12, 0.09, 0.14), (-0.125, 0.093, 0.235), 0.0022, "darksteel", seg=6))
    P.append(sphere(ctx, "ant_tip", 0.005, (-0.125, 0.093, 0.238), "led_violet", seg=10, rings=6))
    # front camera
    P += parts.camera_box(ctx, (cam["x"] - 0.013, 0, cam["z"]))
    # front caster
    for c in cfg["casters"]:
        rc = c["radius"]
        P.append(sphere(ctx, "ball", rc, (c["x"], c["y"], rc), "steel", seg=20, rings=12))
        P.append(cyl(ctx, "seat", rc * 0.95, 0.012, (c["x"], c["y"], rc * 1.8), "darksteel", seg=20, bev=0.002))
    # rear skid
    P.append(box(ctx, "skid", (0.03, 0.12, 0.012), (-0.15, 0, 0.056), "graphite", bev=0.003))
    base = join(ctx, P, "base")
    wheel = parts.wheel(ctx, "brick", r_w, w_w)
    return base, wheel
