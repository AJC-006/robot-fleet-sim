"""hound: quadruped robot dog. Sleek dark shell with chamfered panels and violet light strips, a
sensor head with a black visor and stereo cameras, lidar on a short pedestal over the shoulders,
Spot/Go1-like legs (hip ab/ad drum, hip-pitch motor, blade thigh, slim shin, rubber foot)."""
import math

from mathutils import Vector

from . import legkit as LK
from . import parts
from .common import box, cyl, join, sphere, torus, tube

ZC = 0.012          # hull axis height in base_link


def hull_loft():
    xs = [-0.214, -0.19, -0.12, 0.0, 0.12, 0.19, 0.214]
    return LK.Loft([(x, 0, ZC) for x in xs],
                   ry=[0.056, 0.070, 0.077, 0.078, 0.077, 0.071, 0.058],
                   rz=[0.034, 0.044, 0.049, 0.050, 0.049, 0.045, 0.036],
                   sq=[3.2] * 7)


def torso(ctx, lid, cam):
    P = []
    H = hull_loft()
    P.append(H.mesh(ctx, "hull", "hound_shell", nu=36, na=48, cap_len=(0.018, 0.018)))
    # panels (gaps between them read as panel lines)
    P.append(H.patch(ctx, "top", "hound_panel", 0.05, 0.95, 0.70, math.pi - 0.70, 0.0022, 0.004, nu=24, na=14))
    for s in (1, -1):
        a0, a1 = (-0.42, 0.50) if s > 0 else (math.pi - 0.50, math.pi + 0.42)
        P.append(H.patch(ctx, "side", "hound_panel", 0.12, 0.88, a0, a1, 0.0022, 0.004, nu=20, na=10))
        # violet light strip in the groove between the top and side panels
        a = 0.60 if s > 0 else math.pi - 0.60
        P.append(H.patch(ctx, "led", "led_violet", 0.16, 0.84, a - 0.035, a + 0.035, 0.0016, 0.002, nu=20, na=1,
                         chamfer=0.3))
        # vents on the side panels
        for k in range(5):
            u = 0.40 + 0.045 * k
            P.append(H.patch(ctx, "vent", "charcoal", u, u + 0.022, a0 + 0.45 if s > 0 else a1 - 0.62,
                             a0 + 0.62 if s > 0 else a1 - 0.45, 0.0045, 0.003, nu=1, na=2, chamfer=0.2))
    P.append(H.patch(ctx, "belly", "graphite", 0.10, 0.90, math.pi + 0.55, 2 * math.pi - 0.55, 0.002, 0.004, nu=16, na=8))
    # dorsal spine trim + handle-like rear bumper
    P.append(H.patch(ctx, "spine", "hound_joint", 0.08, 0.40, math.pi / 2 - 0.05, math.pi / 2 + 0.05, 0.0050, 0.004, nu=10, na=1))
    P.append(H.patch(ctx, "spine2", "hound_joint", 0.60, 0.92, math.pi / 2 - 0.05, math.pi / 2 + 0.05, 0.0050, 0.004, nu=10, na=1))
    pts = [Vector((-0.222, s * 0.035, ZC + 0.006)) for s in (1, -1)]
    P.append(tube(ctx, "bumper", pts[0], pts[1], 0.009, "rubber", seg=12))
    for p in pts:
        P.append(sphere(ctx, "bumper_end", 0.009, p, "rubber", seg=12, rings=8))
    # rear light bar
    P.append(box(ctx, "taillight", (0.004, 0.05, 0.008), (-0.229, 0, ZC + 0.024), "led_violet", bev=0.0015))
    # lidar pedestal + puck (sensor plane at lidar z)
    lz = lid["z"] - lid["_bz"]          # sensor plane in base_link
    top = ZC + 0.050 + 0.004
    # streamlined fairing from the back up to the puck
    F = LK.Loft([(lid["x"], 0, top - 0.006), (lid["x"] - 0.004, 0, top + 0.02), (lid["x"], 0, lz - 0.021)],
                ry=[0.034, 0.026, 0.024], rz=[0.060, 0.036, 0.026], up=(1, 0, 0), sq=[2.6, 2.4, 2.2])
    P.append(F.mesh(ctx, "fairing", "hound_panel", nu=10, na=32, caps=(False, True), cap_len=(None, 0.002)))
    P.append(torus(ctx, "ped_led", 0.0262, 0.0016, (lid["x"], 0, lz - 0.030), "led_violet", major=32, minor=6))
    P += parts.lidar_puck(ctx, (lid["x"], lid["y"], lz), r=0.034, h=0.042)
    return P


def head(ctx, cam):
    P = []
    cz = cam["z"] - cam["_bz"]
    L = LK.Loft([(0.196, 0, 0.030), (0.24, 0, 0.031), (0.285, 0, 0.032), (0.298, 0, 0.032)],
                ry=[0.050, 0.052, 0.050, 0.046], rz=[0.038, 0.040, 0.037, 0.033], sq=[3.0] * 4)
    P.append(L.mesh(ctx, "head", "hound_panel", nu=16, na=40, caps=(True, True), cap_len=(0.01, 0.010)))
    # neck collar where the head meets the hull
    P.append(L.patch(ctx, "collar", "hound_joint", 0.0, 0.12, 0, 2 * math.pi, 0.002, 0.004, nu=3, na=40, chamfer=0.3))
    # black visor wrapping the face, light brow above it
    P.append(torus(ctx, "visor_led", 0.040, 0.0014, (0.300, 0, cz - 0.004), "led_violet", major=48, minor=6,
                   axis="x", scale=(0.68, 1.16, 1.0)))
    vis = sphere(ctx, "visor", 1.0, (0.293, 0, cz - 0.004), "visor", scale=(0.016, 0.044, 0.024), seg=32, rings=16)
    P.append(vis)
    for s in (1, -1):
        y = s * 0.020
        P.append(cyl(ctx, "lens_ring", 0.0085, 0.004, (0.306, y, cz), "hound_joint", axis="x", seg=20))
        P.append(cyl(ctx, "lens", 0.0062, 0.004, (0.308, y, cz), "lens", axis="x", seg=20))
        P.append(sphere(ctx, "ir", 0.003, (0.305, s * 0.034, cz - 0.012), "led_violet", seg=8, rings=6))
        # cheek plates
        P.append(L.patch(ctx, "cheek", "hound_shell", 0.25, 0.70, (-0.55 if s > 0 else math.pi - 0.15),
                         (0.15 if s > 0 else math.pi + 0.55), 0.002, 0.003, nu=6, na=5))
    P.append(cyl(ctx, "tof", 0.005, 0.004, (0.302, 0, cz - 0.022), "lens", axis="x", seg=12))
    # chin sensor
    P.append(sphere(ctx, "chin", 1.0, (0.262, 0, -0.006), "charcoal", scale=(0.022, 0.03, 0.008), seg=20, rings=8))
    return P


# ----------------------------------------------------------------------------- legs
def coxa(ctx, lc):
    """Left coxa (mirrored for the right): ab/ad drum on the x axis at the origin, bracket, and the
    hip-pitch motor on the y axis inboard of the femur joint at (0, lc, 0)."""
    P = []
    P.append(cyl(ctx, "abad", 0.031, 0.066, (0, 0, 0), "hound_joint", axis="x", seg=32, bev=0.003))
    for x in (0.034, -0.034):
        P.append(cyl(ctx, "abad_cap", 0.024, 0.004, (x, 0, 0), "hound_panel", axis="x", seg=28, bev=0.001))
    P.append(torus(ctx, "abad_led", 0.026, 0.0015, (0.0345, 0, 0), "led_violet", major=28, minor=6, axis="x"))
    P.append(box(ctx, "bracket", (0.040, lc * 0.75, 0.036), (0, lc * 0.42, -0.004), "hound_panel", bev=0.006, seg=3))
    yc = lc - 0.017
    P.append(cyl(ctx, "pitch", 0.038, 0.026, (0, yc, 0), "hound_joint", axis="y", seg=36, bev=0.003))
    P.append(cyl(ctx, "pitch_cap", 0.030, 0.004, (0, yc - 0.014, 0), "hound_panel", axis="y", seg=32, bev=0.001))
    return join(ctx, P, "coxa_l")


def femur(ctx, l1):
    """Left thigh: hub around the hip-pitch axis, blade to the knee, light strip on the outer face."""
    P = []
    yo = 0.008
    P.append(cyl(ctx, "hub", 0.034, 0.026, (0, yo, 0), "hound_panel", axis="y", seg=40, bev=0.003))
    P.append(torus(ctx, "hub_led", 0.025, 0.0018, (0, yo + 0.0135, 0), "led_violet", major=32, minor=6, axis="y"))
    P.append(cyl(ctx, "hub_cap", 0.016, 0.003, (0, yo + 0.0135, 0), "hound_joint", axis="y", seg=24))
    B = LK.Loft([(0.0, yo, 0), (0.05, yo, 0.002), (0.11, yo, 0.001), (l1, yo, 0)],
                ry=[0.012, 0.012, 0.011, 0.010], rz=[0.030, 0.028, 0.022, 0.018], sq=[3.0] * 4)
    P.append(B.mesh(ctx, "blade", "hound_shell", nu=16, na=24, caps=(False, False)))
    P.append(B.patch(ctx, "cover", "hound_panel", 0.10, 0.86, -0.85, 0.85, 0.0018, 0.003, nu=10, na=8))
    P.append(B.patch(ctx, "led", "led_violet", 0.18, 0.74, -0.06, 0.06, 0.0034, 0.002, nu=8, na=1, chamfer=0.3))
    P.append(cyl(ctx, "knee", 0.021, 0.028, (l1, yo, 0), "hound_joint", axis="y", seg=28, bev=0.002))
    P.append(cyl(ctx, "knee_cap", 0.014, 0.003, (l1, yo + 0.0145, 0), "hound_panel", axis="y", seg=20))
    return join(ctx, P, "femur_l")


def tibia(ctx, l2, rf):
    """Shin (symmetric): slim tapered link, ankle collar and rubber foot at x = l2."""
    P = []
    S = LK.Loft([(0.0, 0, 0), (0.05, 0, 0.003), (0.11, 0, 0.002), (l2 - rf * 0.6, 0, 0)],
                ry=[0.011, 0.010, 0.009, 0.008], rz=[0.017, 0.015, 0.011, 0.009], sq=[2.6] * 4)
    P.append(S.mesh(ctx, "shin", "hound_panel", nu=14, na=20, caps=(True, False), cap_len=(0.012, None)))
    P.append(S.patch(ctx, "shin_led", "led_violet", 0.15, 0.55, math.pi / 2 - 0.12, math.pi / 2 + 0.12, 0.0012, 0.0016,
                     nu=6, na=1, chamfer=0.3))
    P.append(cyl(ctx, "knee_in", 0.015, 0.022, (0, 0, 0), "hound_joint", axis="y", seg=24, bev=0.002))
    P.append(cyl(ctx, "ankle", 0.0105, 0.012, (l2 - rf * 0.95, 0, 0), "hound_joint", axis="x", seg=20, r2=0.013, bev=0.001))
    P.append(sphere(ctx, "foot", rf, (l2, 0, 0), "rubber", seg=24, rings=14))
    P.append(torus(ctx, "foot_ring", rf * 0.78, 0.0018, (l2 - rf * 0.55, 0, 0), "hound_joint", major=24, minor=6, axis="x"))
    return join(ctx, P, "tibia")


def build(cfg, ctx):
    bz = cfg["standing_height"]
    lid = dict(cfg["lidar"], _bz=bz)
    cam = dict(cfg["camera"], _bz=bz)
    lk = cfg["link"]
    P = torso(ctx, lid, cam) + head(ctx, cam)
    base = join(ctx, P, "base")
    cl = coxa(ctx, lk["coxa"])
    fl = femur(ctx, lk["femur"])
    parts_ = {"coxa": {"l": cl, "r": LK.mirror_obj(ctx, cl, "coxa_r")},
              "femur": {"l": fl, "r": LK.mirror_obj(ctx, fl, "femur_r")},
              "tibia": tibia(ctx, lk["tibia"], lk["foot_radius"])}
    return base, parts_
