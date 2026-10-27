"""round: Pebble - compact disc robot, Roomba/TurtleBot-like."""
import math

from .common import box, cut, cyl, join, lathe, sphere, torus
from . import parts


def build(cfg, ctx):
    r_w, w_w, sep = cfg["wheel_radius"], cfg["wheel_width"], cfg["wheel_separation"]
    lid = cfg["lidar"]
    P = []
    # --- shell: lower body + deck
    shell = lathe(ctx, "shell", [(0, 0.028), (0.126, 0.028), (0.139, 0.040), (0.140, 0.070), (0.131, 0.088),
                                 (0.105, 0.098), (0, 0.101)], (0, 0, 0), "pearl", seg=64, bev=0.0008)
    for s in (-1, 1):
        cutter = cyl(ctx, "well", r_w + 0.006, 0.052, (0, s * (sep / 2 + 0.01), r_w), "pearl", axis="y", seg=32)
        cut(shell, cutter)
    P.append(shell)
    # dark underside skirt + front bumper band
    P.append(lathe(ctx, "skirt", [(0, 0.022), (0.118, 0.022), (0.126, 0.028), (0.119, 0.032), (0, 0.032)],
                   (0, 0, 0), "graphite", seg=64))
    for s in (-1, 1):
        P[-1] = P[-1]  # keep
    # violet light ring around the waist
    P.append(torus(ctx, "ring", 0.1405, 0.0022, (0, 0, 0.056), "led_violet", major=96, minor=8))
    # top deck inlay
    P.append(lathe(ctx, "deck", [(0, 0.1005), (0.092, 0.1005), (0.094, 0.1018), (0.092, 0.1026), (0, 0.1026)],
                   (0, 0, 0), "graphite", seg=64))
    P.append(torus(ctx, "deck_ring", 0.075, 0.0016, (0, 0, 0.1026), "violetmetal", major=64, minor=6))
    # buttons / status LEDs
    P.append(cyl(ctx, "btn", 0.008, 0.003, (-0.07, 0.045, 0.103), "led_violet", seg=16))
    P.append(cyl(ctx, "btn2", 0.006, 0.003, (-0.075, 0.015, 0.103), "led_cyan", seg=16))
    # lidar on four posts
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        P.append(cyl(ctx, "post", 0.0045, 0.052, (0.032 * math.cos(a), 0.032 * math.sin(a), 0.126), "steel", seg=10))
    P += parts.lidar_puck(ctx, (lid["x"], lid["y"], lid["z"]), r=0.037, h=0.042)
    # front sensor eye (camera mount position)
    P += parts.camera_box(ctx, (cfg["camera"]["x"] - 0.013, 0, cfg["camera"]["z"]))
    # casters
    for c in cfg["casters"]:
        P.append(sphere(ctx, "ball", c["radius"], (c["x"], c["y"], c["radius"]), "steel", seg=16, rings=10))
        P.append(cyl(ctx, "ball_seat", c["radius"] * 0.9, 0.006, (c["x"], c["y"], c["radius"] * 1.7), "graphite", seg=16))
    base = join(ctx, P, "base")
    wheel = parts.wheel(ctx, "pebble", r_w, w_w)
    return base, wheel
