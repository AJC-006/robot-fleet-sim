"""truck: small pickup / monster truck - cab + cargo bed, four chunky treaded tyres (skid steer)."""
import math

from .common import box, cut, cyl, join, lathe, sphere, torus, tube
from . import parts


def fender(ctx, x, y, r, width, material):
    """Half-round wheel-arch flare over the top of a wheel centred at (x, y, r)."""
    ring = lathe(ctx, "flare", [(r + 0.010, -width / 2), (r + 0.017, -width / 2), (r + 0.017, width / 2),
                                (r + 0.010, width / 2)], (x, y, r), material, seg=40, axis="y", loop=True)
    cutter = box(ctx, "flare_cut", (0.4, 0.4, 0.3), (x, y, r - 0.15 - 0.002), material, bev=0)
    return cut(ring, cutter)


def build(cfg, ctx):
    r_w, w_w, sep = cfg["wheel_radius"], cfg["wheel_width"], cfg["wheel_separation"]
    lid, cam = cfg["lidar"], cfg["camera"]
    xs = cfg["wheel_x"]
    P = []
    # --- chassis: frame rails, axles, springs
    P.append(box(ctx, "frame", (0.44, 0.19, 0.03), (0.0, 0, 0.115), "darksteel", bev=0.003))
    for x in xs:
        P.append(cyl(ctx, "axle", 0.011, sep - w_w - 0.01, (x, 0, r_w), "darksteel", axis="y", seg=16))
        P.append(sphere(ctx, "diff", 0.026, (x, 0, r_w), "graphite", scale=(1, 1, 0.9), seg=16, rings=10))
        for s in (-1, 1):
            P.append(cyl(ctx, "spring", 0.013, 0.05, (x + 0.0, s * 0.105, 0.105), "violetmetal", seg=14))
            P.append(tube(ctx, "shock", (x + 0.025, s * 0.105, r_w + 0.01), (x + 0.025, s * 0.115, 0.13), 0.005, "steel", seg=8))
    # --- body tub (lower, graphite) with violet rocker stripe
    P.append(box(ctx, "tub", (0.46, 0.26, 0.07), (0.0, 0, 0.155), "graphite", bev=0.006, seg=3))
    for s in (-1, 1):
        P.append(box(ctx, "rocker", (0.30, 0.003, 0.012), (0.0, s * 0.1315, 0.133), "violet", bev=0.001))
        P.append(box(ctx, "step", (0.12, 0.02, 0.008), (0.025, s * 0.14, 0.123), "darksteel", bev=0.002))
    # wheel flares (arches) in body colour
    for x in xs:
        for s in (-1, 1):
            P.append(fender(ctx, x, s * sep / 2, r_w, w_w + 0.012, "deeppurple"))
    # --- hood + grille + headlights
    P.append(box(ctx, "hood", (0.14, 0.25, 0.026), (0.165, 0, 0.203), "deeppurple", bev=0.007, seg=3))
    P.append(box(ctx, "hood_stripe", (0.14, 0.035, 0.0015), (0.165, 0, 0.2165), "violet", bev=0))
    P.append(box(ctx, "hood_scoop", (0.05, 0.07, 0.014), (0.15, 0, 0.222), "graphite", bev=0.004))
    P.append(box(ctx, "grille", (0.004, 0.17, 0.04), (0.2355, 0, 0.17), "charcoal", bev=0.001))
    for k in range(4):
        P.append(box(ctx, "grille_bar", (0.006, 0.17, 0.004), (0.2375, 0, 0.158 + k * 0.011), "steel", bev=0.0008))
    for s in (-1, 1):
        P.append(box(ctx, "headlight", (0.008, 0.04, 0.026), (0.2375, s * 0.1, 0.18), "led_cyan", bev=0.004))
        P.append(box(ctx, "taillight", (0.008, 0.035, 0.02), (-0.2375, s * 0.1, 0.17), "led_violet", bev=0.003))
    # bumpers
    P.append(box(ctx, "bump_f", (0.025, 0.27, 0.03), (0.249, 0, 0.13), "steel", bev=0.005))
    P.append(box(ctx, "bump_r", (0.02, 0.27, 0.026), (-0.246, 0, 0.13), "darksteel", bev=0.005))
    # --- cab
    P.append(box(ctx, "cab", (0.15, 0.25, 0.07), (0.025, 0, 0.225), "deeppurple", bev=0.006, seg=3))
    P.append(box(ctx, "roof", (0.13, 0.255, 0.01), (0.022, 0, 0.267), "deeppurple", bev=0.004))
    P.append(box(ctx, "roof_stripe", (0.13, 0.035, 0.0015), (0.022, 0, 0.2725), "violet", bev=0))
    for s in (-1, 1):
        P.append(box(ctx, "win_side", (0.095, 0.2545, 0.034), (0.03, 0, 0.231), "glass", bev=0.0))
    P.append(box(ctx, "win_front", (0.0045, 0.21, 0.036), (0.1005, 0, 0.231), "glass", bev=0.0))
    P.append(box(ctx, "win_rear", (0.0045, 0.21, 0.036), (-0.0505, 0, 0.231), "glass", bev=0.0))
    for x in (-0.0155, 0.0775):    # pillars
        P.append(box(ctx, "pillar", (0.012, 0.256, 0.04), (x, 0, 0.231), "deeppurple", bev=0.002))
    for s in (-1, 1):
        P.append(box(ctx, "mirror", (0.012, 0.03, 0.02), (0.085, s * 0.147, 0.235), "graphite", bev=0.003))
        P.append(tube(ctx, "mirror_arm", (0.085, s * 0.126, 0.232), (0.085, s * 0.141, 0.235), 0.0025, "darksteel", seg=6))
    # --- cargo bed
    P.append(box(ctx, "bed_floor", (0.19, 0.25, 0.01), (-0.145, 0, 0.19), "graphite", bev=0.002))
    for s in (-1, 1):
        P.append(box(ctx, "bed_wall", (0.19, 0.012, 0.045), (-0.145, s * 0.119, 0.2125), "deeppurple", bev=0.003))
        P.append(box(ctx, "bed_edge", (0.19, 0.016, 0.004), (-0.145, s * 0.119, 0.237), "darksteel", bev=0.001))
    P.append(box(ctx, "tailgate", (0.012, 0.25, 0.045), (-0.239, 0, 0.2125), "deeppurple", bev=0.003))
    P.append(box(ctx, "tail_edge", (0.016, 0.25, 0.004), (-0.239, 0, 0.237), "darksteel", bev=0.001))
    P.append(box(ctx, "headboard", (0.012, 0.25, 0.05), (-0.058, 0, 0.215), "deeppurple", bev=0.003))
    P.append(box(ctx, "bed_stripe", (0.0015, 0.04, 0.045), (-0.2455, 0, 0.2125), "violet", bev=0))
    # roll bar (top stays under the lidar plane)
    for s in (-1, 1):
        P.append(tube(ctx, "hoop", (-0.115, s * 0.105, 0.195), (-0.115, s * 0.105, 0.272), 0.0055, "steel", seg=8))
    P.append(tube(ctx, "hoop_top", (-0.115, -0.105, 0.272), (-0.115, 0.105, 0.272), 0.0055, "steel", seg=8))
    P.append(tube(ctx, "hoop_brace", (-0.115, 0.0, 0.272), (-0.058, 0.0, 0.25), 0.0045, "steel", seg=8))
    # --- roof light bar (below the lidar plane) and lidar mount
    P.append(box(ctx, "lightbar", (0.02, 0.2, 0.012), (0.075, 0, 0.278), "graphite", bev=0.003))
    for k in range(5):
        P.append(box(ctx, "lb_led", (0.003, 0.03, 0.007), (0.0855, (k - 2) * 0.038, 0.278), "led_cyan", bev=0.001))
    lx, ly = lid["x"], lid["y"]
    P.append(cyl(ctx, "lidar_post", 0.013, lid["z"] - 0.272 - 0.012, (lx, ly, 0.272 + (lid["z"] - 0.272 - 0.012) / 2), "steel", seg=16))
    P.append(cyl(ctx, "lidar_plate", 0.03, 0.005, (lx, ly, 0.2725), "darksteel", seg=20, bev=0.001))
    P += parts.lidar_puck(ctx, (lx, ly, lid["z"]), r=0.036, h=0.036)
    # camera in the windscreen
    P += parts.camera_box(ctx, (cam["x"] - 0.013, 0, cam["z"]))
    base = join(ctx, P, "base")
    wheel = parts.wheel(ctx, "truck", r_w, w_w)
    return base, wheel
