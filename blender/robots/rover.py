"""rover: Mars-rover look-alike (Curiosity/Perseverance flavoured). The rocker-bogie is rigid and
only part of the base mesh; the six wheels are separate meshes."""
import math

from .common import box, cyl, join, lathe, sphere, torus, tube
from . import parts


def build(cfg, ctx):
    r_w, w_w, sep = cfg["wheel_radius"], cfg["wheel_width"], cfg["wheel_separation"]
    lid, cam = cfg["lidar"], cfg["camera"]
    xs = cfg["wheel_x"]
    P = []
    # --- body: warm electronics box with deck
    P.append(box(ctx, "body", (0.30, 0.22, 0.075), (0.0, 0, 0.1575), "pearl", bev=0.008, seg=3))
    P.append(box(ctx, "belly", (0.26, 0.18, 0.012), (0.0, 0, 0.116), "darksteel", bev=0.003))
    P.append(box(ctx, "deck", (0.30, 0.22, 0.008), (0.0, 0, 0.2), "darksteel", bev=0.002))
    P.append(box(ctx, "web", (0.12, 0.17, 0.04), (0.04, 0, 0.222), "gold", bev=0.005, seg=3))   # gold foil box
    P.append(box(ctx, "stripe", (0.30, 0.012, 0.0015), (0.0, 0, 0.2011), "violet", bev=0))
    for s in (-1, 1):
        P.append(box(ctx, "sidepanel", (0.22, 0.003, 0.04), (0.0, s * 0.1105, 0.16), "graphite", bev=0.001))
        P.append(box(ctx, "sidestripe", (0.22, 0.0035, 0.004), (0.0, s * 0.1107, 0.185), "led_violet", bev=0))
    # --- RTG at the back
    P.append(cyl(ctx, "rtg", 0.028, 0.095, (-0.19, 0, 0.165), "graphite", axis="x", seg=24, bev=0.002))
    for k in range(5):
        P.append(cyl(ctx, "fin", 0.037, 0.004, (-0.155 - k * 0.017, 0, 0.165), "darksteel", axis="x", seg=24))
    P.append(box(ctx, "rtg_mount", (0.05, 0.08, 0.04), (-0.145, 0, 0.18), "darksteel", bev=0.002))
    # --- solar panel on four legs, tilted slightly
    tilt = 0.12
    pc = (-0.075, 0.0, 0.262)
    P.append(box(ctx, "frame", (0.15, 0.21, 0.008), pc, "darksteel", bev=0.002, rot=(0, tilt, 0)))
    P.append(box(ctx, "cells", (0.138, 0.198, 0.0025), (pc[0] + 0.0004, 0, pc[2] + 0.0052), "solar", bev=0.0005, rot=(0, tilt, 0)))
    for k in (-2, -1, 0, 1, 2):
        P.append(box(ctx, "gridy", (0.139, 0.0016, 0.0012), (pc[0] + 0.0004, k * 0.036, pc[2] + 0.0068), "steel", bev=0, rot=(0, tilt, 0)))
    for k in (-1, 0, 1):
        P.append(box(ctx, "gridx", (0.0016, 0.2, 0.0012), (pc[0] + 0.0004 + k * 0.04 * math.cos(tilt), 0, pc[2] + 0.0068 - k * 0.04 * math.sin(tilt)), "steel", bev=0, rot=(0, tilt, 0)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = pc[0] + sx * 0.06
            P.append(tube(ctx, "leg", (x, sy * 0.09, 0.204), (x, sy * 0.09, pc[2] - sx * 0.0 + 0.0 - sx * 0.06 * math.tan(tilt) * -1 - 0.0), 0.0035, "steel", seg=6))
    # --- mast with camera head and the lidar on top
    mx, my = lid["x"], lid["y"]
    P.append(cyl(ctx, "mast_base", 0.022, 0.012, (mx, my, 0.206 + 0.044), "darksteel", seg=20, bev=0.002))   # on the gold box
    mast_top = lid["z"] - 0.03
    P.append(cyl(ctx, "mast", 0.0075, mast_top - 0.25, (mx, my, 0.25 + (mast_top - 0.25) / 2), "steel", seg=16))
    hz = cam["z"]
    P.append(box(ctx, "head", (0.048, 0.105, 0.045), (mx + 0.006, my, hz), "graphite", bev=0.006, seg=3))
    P.append(box(ctx, "head_top", (0.05, 0.108, 0.006), (mx + 0.006, my, hz + 0.025), "violetmetal", bev=0.002))
    for s in (-1, 1):
        P.append(cyl(ctx, "eye_ring", 0.0115, 0.01, (mx + 0.03, my + s * 0.034, hz + 0.002), "steel", axis="x", seg=20))
        P.append(cyl(ctx, "eye_lens", 0.0085, 0.01, (mx + 0.0335, my + s * 0.034, hz + 0.002), "lens", axis="x", seg=16))
        P.append(cyl(ctx, "side_cam", 0.006, 0.01, (mx + 0.006, my + s * 0.057, hz), "lens", axis="y", seg=12))
    P.append(box(ctx, "head_light", (0.008, 0.02, 0.01), (mx + 0.031, my, hz - 0.012), "led_cyan", bev=0.001))
    P += parts.lidar_puck(ctx, (mx, my, lid["z"]), r=0.034, h=0.036, accent="led_violet")
    # --- high gain antenna dish
    dish = lathe(ctx, "dish", [(0, 0.0), (0.014, 0.0015), (0.028, 0.006), (0.035, 0.0115), (0.0345, 0.0125),
                               (0.027, 0.008), (0.012, 0.0035), (0, 0.002)], (-0.03, 0.085, 0.292), "white",
                 seg=24, rot=(0.0, -0.35, 0.0))
    P.append(dish)
    P.append(tube(ctx, "dish_stem", (-0.03, 0.085, 0.208), (-0.03, 0.085, 0.289), 0.0035, "steel", seg=6))
    P.append(cyl(ctx, "feed", 0.003, 0.02, (-0.03 - 0.002, 0.085, 0.304), "steel", seg=6))
    # --- rocker-bogie (rigid): struts, rocker, bogie
    for s in (-1, 1):
        yin = s * (sep / 2 - w_w / 2 - 0.006)
        top = 0.12
        for x in xs:
            P.append(cyl(ctx, "axle", 0.0095, 0.018, (x, s * (sep / 2 - w_w / 2 - 0.011), r_w), "darksteel", axis="y", seg=14))
            P.append(tube(ctx, "strut", (x, yin, r_w), (x, yin, top), 0.0065, "darksteel", seg=10))
            P.append(sphere(ctx, "knuckle", 0.0095, (x, yin, top), "steel", seg=12, rings=8))
        kn = (0.04, s * 0.118, 0.17)
        bp = (-0.10, s * 0.132, 0.135)
        link = [((xs[0], yin, top), kn), (kn, bp), ((xs[1], yin, top), bp), (bp, (xs[2], yin, top))]
        for a, b in link:
            P.append(tube(ctx, "arm", a, b, 0.0075, "steel", seg=12))
        for p in (kn, bp):
            P.append(sphere(ctx, "pivot", 0.0125, p, "darksteel", seg=14, rings=8))
        P.append(tube(ctx, "diffbar", (0.04, s * 0.108, 0.17), (0.04, s * 0.05, 0.17), 0.006, "darksteel", seg=8))
    base = join(ctx, P, "base")
    wheel = parts.wheel(ctx, "rover", r_w, w_w)
    return base, wheel
