"""lab: robotics lab, 28 x 20 m. Interior x[-14,14] y[-10,10], ceiling 5.8 m.

North band y[5,10]: server room x[-14,-8.5] | office x[-8.5,-3.5] | office x[-3.5,1.5] | storage + charging bay x[1.5,14].
Corridor y[3,5] (1.8 m wide) with four openings to the hall. Hall y[-10,3]:
  test arena x[-13.5,-5] y[-9.5,-2]   0.65 m rubber-floored walls with two openings, cones and boxes
  workshop   x[-3,2]     y[-9.8,-2.5] benches, tool wall, carts
  robot-arm cell x[-3,2] y[-0.8,2.6]  safety fence with two gates, 6-axis arm, workpiece table
  drone cage x[3.5,11.7] y[-8.2,0]    8.2 x 8.2 x 5 m net cage (collision walls + top), two gates, PX4 flights
Everything solid keeps a constant footprint between 0.05 and 0.6 m; decoration sits above 0.6 m or inside a footprint.
"""
import math

from common import hwall, vwall, zfloor

NAME = "lab"
BOUNDS = (-14.0, -10.0, 14.0, 10.0)
OUTER = (-14.25, -10.25, 14.25, 10.25)
SPAWN = (11.2, 7.4, 0.0)
GOALS = [
    ("arena", -9.2, -5.8, 0.0),
    ("arena_start", -12.2, -3.2, -math.pi / 2),
    ("workshop", -0.5, -5.0, math.pi / 2),
    ("robot_arm", -1.9, 0.4, 0.0),
    ("cage_gate", 2.5, -4.1, 0.0),
    ("cage_centre", 7.6, -4.1, 0.0),
    ("server_room", -11.2, 7.6, math.pi / 2),
    ("office_west", -6.0, 7.5, math.pi / 2),
    ("office_east", -0.5, 7.0, math.pi / 2),
    ("storage", 7.0, 8.0, math.pi / 2),
    ("corridor_west", -12.5, 4.0, math.pi),
    ("corridor_east", 12.5, 4.0, 0.0),
    ("south_east", 12.8, -8.8, -math.pi / 2),
]
DRONE = {
    "takeoff": (7.6, -4.1),
    "cruise": 2.8,
    "waypoints": [
        ("cage_sw", 5.7, -6.3, 1.8),
        ("cage_se", 9.5, -6.3, 2.6),
        ("cage_ne", 9.5, -1.9, 3.2),
        ("cage_nw", 5.7, -1.9, 2.2),
        ("cage_centre", 7.6, -4.1, 3.4),
        ("cage_low", 7.6, -6.4, 1.6),
    ],
}
SCENE = {
    "ambient": (0.42, 0.42, 0.45),
    "background": (0.55, 0.6, 0.65),
    "sun": {"color": (0.85, 0.85, 0.8), "dir": (0.35, 0.2, -1.0)},
}
PREVIEW = {
    "cam": (20.0, -27.0, 26.0), "target": (0.0, 0.0, 0.3), "lens": 34,
    "world": (0.6, 0.65, 0.72), "world_strength": 1.0,
    "hide": ["roof"],
    "cut": [("perim_s", 0.5), ("perim_e", 0.5), ("inner", 1.3), ("cage", 4.0)],
    "extra_lights": [("SUN", (0, 0, 10), (1.0, 0.96, 0.9), 3.5, 0)],
    "light_rot": (math.radians(35), math.radians(20), math.radians(30)),
    "light_gain": 0.25,
    "views": [
        ("cage", (3.0, 1.8, 3.0), (8.0, -5.0, 2.2), 22),
        ("arena", (-5.0, -1.5, 3.0), (-10.0, -6.0, 0.4), 22),
        ("workshop", (-3.6, -2.2, 2.8), (-0.5, -7.5, 0.8), 24),
        ("server", (-9.0, 5.4, 2.4), (-12.5, 8.5, 1.0), 24),
    ],
}

MATS = {
    "floor": dict(tex="concrete_floor_01", tile=2.0, tint=(1.35, 1.35, 1.4)),
    "floor_off": dict(tex="laminate_floor_02", tile=1.5),
    "floor_srv": dict(tex="concrete_floor_01", tile=2.0, tint=(0.8, 0.85, 1.0)),
    "wall": dict(tex="white_plaster_02", tile=2.0, tint=(1.35, 1.35, 1.4)),
    "wall_out": dict(tex="concrete_layers_02", tile=2.0, tint=(1.1, 1.1, 1.15)),
    "steel": dict(tex="metal_plate", tile=1.0, metal=0.3),
    "wood": dict(tex="oak_veneer_05", tile=1.0, bevel=0.008),
    "cab_blue": dict(color=(0.03, 0.12, 0.3), rough=0.45, bevel=0.008),
    "cab_grey": dict(color=(0.25, 0.27, 0.3), rough=0.5, metal=0.2, bevel=0.008),
    "top": dict(color=(0.05, 0.05, 0.055), rough=0.35),
    "red": dict(color=(0.55, 0.03, 0.02), rough=0.4, bevel=0.01),
    "orange": dict(color=(0.9, 0.3, 0.02), rough=0.4),
    "yellow": dict(color=(0.85, 0.6, 0.02), rough=0.5),
    "black": dict(color=(0.015, 0.015, 0.018), rough=0.5),
    "white": dict(color=(0.7, 0.7, 0.7), rough=0.5),
    "led_green": dict(color=(0.1, 0.9, 0.2), emit=((0.1, 1.0, 0.2), 4.0)),
    "led_blue": dict(color=(0.1, 0.4, 0.9), emit=((0.1, 0.4, 1.0), 4.0)),
    "led_red": dict(color=(0.9, 0.1, 0.05), emit=((1.0, 0.1, 0.05), 3.0)),
    "screen": dict(color=(0.05, 0.2, 0.4), emit=((0.2, 0.55, 1.0), 2.0)),
    "board": dict(color=(0.8, 0.8, 0.82), emit=((0.8, 0.8, 0.82), 0.4)),
    "rubber": dict(color=(0.03, 0.12, 0.07), rough=0.9),
    "d_yellow": dict(color=(0.8, 0.55, 0.02), rough=0.7),
    "d_white": dict(color=(0.65, 0.65, 0.65), rough=0.7),
    "d_black": dict(color=(0.01, 0.01, 0.012), rough=0.8),
    "d_green": dict(color=(0.05, 0.5, 0.1), rough=0.7),
    "peg": dict(color=(0.1, 0.1, 0.12), rough=0.8),
    "arena_wall": dict(color=(0.7, 0.35, 0.03), rough=0.5, bevel=0.01),
    "box_a": dict(color=(0.35, 0.22, 0.1), rough=0.8),
    "box_b": dict(color=(0.1, 0.25, 0.4), rough=0.7),
    "net": dict(color=(0.05, 0.05, 0.06), rough=0.8, alpha=0.18),
    "cage_post": dict(color=(0.7, 0.45, 0.02), rough=0.5, metal=0.4),
    "rack": dict(color=(0.04, 0.045, 0.055), rough=0.4, metal=0.5, bevel=0.01),
    "window": dict(color=(0.8, 0.9, 1.0), emit=((0.8, 0.9, 1.0), 2.0)),
    "frame": dict(color=(0.55, 0.57, 0.6), rough=0.4, metal=0.4),
    "desk": dict(tex="oak_veneer_05", tile=1.0, bevel=0.01),
    "chair": dict(color=(0.05, 0.08, 0.12), rough=0.8, bevel=0.015),
    "lampbox": dict(color=(1.0, 0.98, 0.92), emit=((1.0, 0.98, 0.92), 3.0)),
    "truss": dict(color=(0.1, 0.11, 0.13), rough=0.6, metal=0.4),
    "leaf": dict(color=(0.08, 0.34, 0.1), rough=0.6),
    "pot": dict(color=(0.45, 0.2, 0.1), rough=0.8, bevel=0.02),
    "door": dict(color=(0.35, 0.37, 0.4), rough=0.4, metal=0.4),
    "cone_w": dict(color=(0.75, 0.75, 0.75), rough=0.6),
    "carpet": dict(color=(0.12, 0.17, 0.25), rough=1.0),
}
H = 5.8


def bench(W, x0, y0, x1, y1, kind):
    if kind == "cab":
        W.boxp("cab_blue", x0, y0, x1, y1, 0.0, 0.85)
        W.boxp("top", x0, y0, x1, y1, 0.85, 0.05, solid=False)
        n = int(round((x1 - x0) / 0.55))
        for i in range(n):  # drawer fronts
            xc = x0 + (i + 0.5) * (x1 - x0) / n
            W.box("steel", xc, y0 - 0.005, 0.45, (x1 - x0) / n - 0.08, 0.01, 0.06, solid=False)
    else:
        W.boxp("wood", x0, y0, x1, y1, 0.84, 0.06, solid=False)
        for (lx, ly) in ((x0 + 0.05, y0 + 0.05), (x1 - 0.05, y0 + 0.05), (x0 + 0.05, y1 - 0.05), (x1 - 0.05, y1 - 0.05)):
            W.box("steel", lx, ly, 0.0, 0.07, 0.07, 0.84)
        W.boxp("steel", x0 + 0.03, y0 + 0.03, x1 - 0.03, y0 + 0.07, 0.62, 0.04, solid=False)  # rail above the band


def shelf_unit(W, x0, y0, x1, y1, seed=0):
    """Closed kick/bin base (solid, 0..0.65) with an open shelving frame above it."""
    W.boxp("cab_grey", x0, y0, x1, y1, 0.0, 0.65)
    for (px, py) in ((x0 + 0.02, y0 + 0.02), (x1 - 0.02, y0 + 0.02), (x0 + 0.02, y1 - 0.02), (x1 - 0.02, y1 - 0.02)):
        W.box("steel", px, py, 0.65, 0.04, 0.04, 1.5, solid=False)
    for k, z in enumerate((0.65, 1.1, 1.55, 2.0)):
        W.boxp("steel", x0, y0, x1, y1, z, 0.03, solid=False)
        if k < 3:
            n = 3
            for i in range(n):
                m = "box_a" if (i + k + seed) % 2 else "box_b"
                bw = (max(x1 - x0, y1 - y0) - 0.2) / n
                if (x1 - x0) > (y1 - y0):
                    W.box(m, x0 + 0.1 + (i + 0.5) * bw, (y0 + y1) / 2, z + 0.03, bw - 0.06, (y1 - y0) - 0.08, 0.28 + 0.04 * ((i * 7 + k) % 3), solid=False)
                else:
                    W.box(m, (x0 + x1) / 2, y0 + 0.1 + (i + 0.5) * bw, z + 0.03, (x1 - x0) - 0.08, bw - 0.06, 0.28 + 0.04 * ((i * 7 + k) % 3), solid=False)


def cart(W, x, y, yaw):
    W.box("red", x, y, 0.0, 0.8, 0.5, 0.8, yaw=yaw)
    W.box("black", x, y, 0.8, 0.8, 0.5, 0.04, yaw=yaw, solid=False)
    c, s = math.cos(yaw), math.sin(yaw)
    for sx in (-0.38, 0.38):  # handle uprights, inside the footprint edge
        W.box("steel", x + sx * c - 0.0 * s, y + sx * s, 0.84, 0.03, 0.03, 0.4, solid=False)
    W.box("steel", x - 0.38 * c, y - 0.38 * s, 1.2, 0.03, 0.46, 0.03, yaw=yaw, solid=False)
    W.box("steel", x + 0.38 * c, y + 0.38 * s, 1.2, 0.03, 0.46, 0.03, yaw=yaw, solid=False)
    W.box("steel", x, y, 1.2, 0.8, 0.03, 0.03, yaw=yaw, solid=False)


def arm(W, cx, cy):
    W.cyl("steel", cx, cy, 0.0, 0.35, 0.75, seg=24)
    W.cyl("black", cx, cy, 0.75, 0.3, 0.04, seg=24, solid=False)
    W.cyl("orange", cx, cy, 0.79, 0.2, 0.2, seg=20, solid=False)
    W.ellipsoid("orange", (cx, cy, 1.05), (0.17, 0.17, 0.17), seg=14, rings=8)
    W.frustum("orange", (cx, cy, 1.05), (cx + 0.1, cy, 1.65), 0.1, 0.085, seg=12)
    W.ellipsoid("black", (cx + 0.1, cy, 1.65), (0.13, 0.13, 0.13), seg=14, rings=8)
    W.frustum("orange", (cx + 0.1, cy, 1.65), (cx + 0.95, cy - 0.15, 1.9), 0.08, 0.065, seg=12)
    W.ellipsoid("black", (cx + 0.95, cy - 0.15, 1.9), (0.09, 0.09, 0.09), seg=12, rings=8)
    W.frustum("orange", (cx + 0.95, cy - 0.15, 1.9), (cx + 1.25, cy - 0.2, 1.35), 0.055, 0.05, seg=10)
    W.box("black", cx + 1.25, cy - 0.2, 1.22, 0.1, 0.16, 0.14, solid=False)
    W.box("steel", cx + 1.25, cy - 0.27, 1.08, 0.03, 0.03, 0.14, solid=False)
    W.box("steel", cx + 1.25, cy - 0.13, 1.08, 0.03, 0.03, 0.14, solid=False)
    W.ring("d_yellow", cx, cy, 1.9, 0.08, n=48)




def desk(W, x0, y0, x1, y1):
    W.boxp("desk", x0, y0, x1, y1, 0.72, 0.04, solid=False)
    for (lx, ly) in ((x0 + 0.05, y0 + 0.05), (x1 - 0.05, y0 + 0.05), (x0 + 0.05, y1 - 0.05), (x1 - 0.05, y1 - 0.05)):
        W.box("frame", lx, ly, 0.0, 0.06, 0.06, 0.72)
    cx = (x0 + x1) / 2
    W.box("black", cx, y1 - 0.15, 0.76, 0.5, 0.03, 0.4, solid=False)      # monitor
    W.box("screen", cx, y1 - 0.17, 0.78, 0.46, 0.01, 0.36, solid=False)
    W.box("black", cx, y0 + 0.4, 0.765, 0.4, 0.14, 0.015, solid=False)    # keyboard


def chair(W, x, y, yaw=0.0):
    W.cyl("chair", x, y, 0.0, 0.25, 0.5, seg=14)
    W.cyl("chair", x, y, 0.5, 0.25, 0.08, seg=14, solid=False)
    W.box("chair", x + 0.2 * math.sin(yaw), y - 0.2 * math.cos(yaw), 0.7, 0.4, 0.06, 0.45, yaw=yaw, solid=False)


def rack(W, x0, y0, x1, y1, face):
    """Server rack, solid footprint over the full height; face = '+x','-x','+y','-y' side with the LED strips."""
    W.boxp("rack", x0, y0, x1, y1, 0.0, 2.1)
    for k in range(10):
        z = 0.85 + k * 0.12
        m = ("led_green", "led_blue", "led_green", "led_red")[(k * 3 + int(x0 * 7 + y0 * 3)) % 4]
        if face == "-y":
            W.box(m, (x0 + x1) / 2, y0 - 0.008, z, (x1 - x0) - 0.2, 0.016, 0.03, solid=False)
        elif face == "+y":
            W.box(m, (x0 + x1) / 2, y1 + 0.008, z, (x1 - x0) - 0.2, 0.016, 0.03, solid=False)
        elif face == "+x":
            W.box(m, x1 + 0.008, (y0 + y1) / 2, z, 0.016, (y1 - y0) - 0.2, 0.03, solid=False)
        else:
            W.box(m, x0 - 0.008, (y0 + y1) / 2, z, 0.016, (y1 - y0) - 0.2, 0.03, solid=False)


def cone(W, x, y):
    W.cyl("orange", x, y, 0.0, 0.12, 0.45, seg=14)
    W.cyl("cone_w", x, y, 0.2, 0.125, 0.07, seg=14, solid=False)
    W.cyl("orange", x, y, 0.45, 0.06, 0.2, seg=10, solid=False)


def build(W):
    W.floor("floor", surface="concrete")
    W.perimeter("wall_out", 0.4, H)
    t = 0.2
    # ------------------------------------------------------------------ zone floors and markings
    zfloor(W, "rubber", -13.4, -9.4, -5.1, -2.1)
    zfloor(W, "floor_srv", -13.8, 5.1, -8.6, 9.8)
    zfloor(W, "floor_off", -8.4, 5.1, 1.4, 9.8)
    zfloor(W, "carpet", -8.3, 5.2, -3.6, 9.7, h=0.014)
    zfloor(W, "carpet", -3.4, 5.2, 1.3, 9.7, h=0.014)
    zfloor(W, "rubber", 3.5, -8.2, 11.7, 0.0)
    W.line("d_yellow", -13.8, 3.2, 13.8, 3.2, 0.08)
    W.line("d_yellow", -13.8, 4.8, 13.8, 4.8, 0.08)
    for x in range(-13, -5):
        W.line("d_white", x + 0.5, -9.4, x + 0.5, -2.1, 0.03, z=0.018)
    for y in range(-9, -2):
        W.line("d_white", -13.4, y + 0.5, -5.1, y + 0.5, 0.03, z=0.018)
    W.ring("d_yellow", 7.6, -4.1, 1.0, 0.1, n=48, z=0.02)
    W.line("d_white", 7.1, -4.1, 8.1, -4.1, 0.1, z=0.022)
    W.line("d_white", 7.1, -4.6, 7.1, -3.6, 0.1, z=0.022)
    W.line("d_white", 8.1, -4.6, 8.1, -3.6, 0.1, z=0.022)
    # ------------------------------------------------------------------ walls
    with W.tagged("inner"):
        hwall(W, 5.0, -14, 14, H, "wall", [(-11.2, 1.2), (-6.0, 1.2), (-1.5, 1.2), (4.0, 1.6), (11.0, 1.6)], t=t)
        hwall(W, 3.0, -14, 14, H, "wall", [(-9.0, 2.0), (-4.0, 2.0), (7.0, 2.0), (12.5, 1.6)], t=t)
        for x in (-8.5, -3.5, 1.5):
            vwall(W, x, 5, 10, H, "wall", [], t=t)
    # arena (0.65 m walls, two openings) and baffles
    with W.tagged("arena"):
        hwall(W, -2.0, -13.5, -5.0, 0.65, "arena_wall", [(-9.0, 1.6, 0.65)], t=t)
        hwall(W, -9.5, -13.5, -5.0, 0.65, "arena_wall", [], t=t)
        vwall(W, -13.5, -9.5, -2.0, 0.65, "arena_wall", [], t=t)
        vwall(W, -5.0, -9.5, -2.0, 0.65, "arena_wall", [(-6.0, 1.6, 0.65)], t=t)
        vwall(W, -11.0, -6.0, -2.0, 0.65, "arena_wall", [], t=t)
        vwall(W, -8.0, -9.5, -5.5, 0.65, "arena_wall", [], t=t)
    for (cx, cy) in ((-6.5, -4.0), (-6.5, -8.0), (-12.5, -8.4), (-9.8, -8.6), (-12.4, -5.2), (-6.4, -2.9)):
        cone(W, cx, cy)
    for (cx, cy, yw) in ((-6.3, -6.9, 0.3), (-12.6, -6.9, -0.2)):
        W.box("box_a", cx, cy, 0.0, 0.5, 0.5, 0.5, yaw=yw)
        W.box("black", cx, cy, 0.5, 0.52, 0.52, 0.02, yaw=yw, solid=False)
    # overhead gantry above the arena, decorative
    for x in (-13.2, -5.3):
        W.box("truss", x, -9.2, 0.0, 0.0, 0.0, 0.0, solid=False) if False else None
    for y in (-9.2, -2.3):
        W.boxp("truss", -13.5, y - 0.08, -5.0, y + 0.08, 3.6, 0.16, solid=False)
    for x in (-13.5, -5.0):
        W.boxp("truss", x - 0.08, -9.3, x + 0.08, -2.2, 3.6, 0.16, solid=False)
    # ------------------------------------------------------------------ server room
    for k in range(5):
        x0 = -13.7 + k * 0.72
        rack(W, x0, 8.9, x0 + 0.62, 9.75, "-y")
    for k in range(3):
        y0 = 5.7 + k * 0.95
        rack(W, -13.75, y0, -12.85, y0 + 0.65, "+x")
    W.boxp("steel", -13.75, 8.0, -9.0, 8.1, 2.3, 0.08, solid=False)  # cable tray
    W.boxp("steel", -13.75, 5.5, -13.65, 9.8, 2.3, 0.08, solid=False)
    W.boxp("frame", -9.9, 5.25, -8.7, 5.9, 0.0, 0.6, solid=True)       # UPS cabinet
    W.boxp("led_green", -9.88, 5.58, -8.72, 5.6, 0.45, 0.03, solid=False)
    # ------------------------------------------------------------------ offices
    for (ox, wx0) in ((-6.0, -8.3), (-1.5, -3.3)):
        desk(W, wx0 + 0.3, 8.8, wx0 + 2.1, 9.65)
        chair(W, wx0 + 1.2, 8.1, 0.0)
        W.boxp("cab_grey", wx0 + 3.6, 8.5, wx0 + 4.1, 9.6, 0.0, 1.3)                  # filing cabinet
        W.boxp("board", wx0 + 2.6, 9.77, wx0 + 3.5, 9.8, 1.0, 0.9, solid=False)       # whiteboard flush on the wall
        W.box("leaf", wx0 + 4.0 - 0.2, 5.7, 0.0, 0.0, 0.0, 0.0, solid=False) if False else None
        W.cyl("pot", wx0 + 4.4 - 0.2, 5.55, 0.0, 0.2, 0.45, seg=12)
        W.ellipsoid("leaf", (wx0 + 4.2, 5.55, 0.95), (0.3, 0.3, 0.4), seg=12, rings=8)
    desk(W, -6.2, 6.2, -4.4, 7.05) if False else None
    # ------------------------------------------------------------------ storage and charging bay
    for k in range(5):
        x0 = 2.0 + k * 2.4
        shelf_unit(W, x0, 9.2, x0 + 2.0, 9.8, seed=k)
    shelf_unit(W, 1.9, 6.6, 2.5, 8.8, seed=7)
    for k, y in enumerate((6.2, 7.5, 8.8)):
        W.boxp("cab_grey", 13.1, y - 0.25, 13.7, y + 0.25, 0.0, 0.8)                  # charger docks
        W.box("led_green", 13.095, y, 0.55, 0.01, 0.2, 0.05, solid=False)
        W.box("screen", 13.095, y, 0.7, 0.01, 0.2, 0.08, solid=False)
        W.decal("d_green", 12.3, y, 1.0, 0.5, z=0.016)
    cart(W, 5.0, 6.2, 0.0)
    cart(W, 9.0, 6.0, 0.4)
    W.box("box_a", 6.0, 8.6, 0.0, 0.6, 0.5, 0.5, yaw=0.2)
    W.box("box_b", 8.8, 8.7, 0.0, 0.5, 0.5, 0.45, yaw=-0.1)
    # ------------------------------------------------------------------ workshop
    bench(W, -2.8, -9.8, -0.6, -9.2, "cab")
    bench(W, -0.2, -9.8, 1.8, -9.2, "legs")
    bench(W, -2.2, -7.6, 0.6, -6.9, "legs")
    for k in range(8):                       # pegboard tool wall, flush above the benches
        W.boxp("peg", -2.8 + k * 0.6, -9.8, -2.3 + k * 0.6, -9.77, 1.1, 0.9, solid=False)
        W.box("steel", -2.55 + k * 0.6, -9.76, 1.5 + 0.1 * (k % 3), 0.05, 0.02, 0.3, solid=False)
    cart(W, 1.4, -3.5, 0.2)
    cart(W, -2.4, -3.2, -0.3)
    W.box("box_a", 1.5, -8.3, 0.0, 0.7, 0.6, 0.55, yaw=0.3)
    # ------------------------------------------------------------------ robot-arm cell
    with W.tagged("fence"):
        hwall(W, -0.8, -3.0, 2.0, 1.8, "net", [(-1.5, 1.4, 1.8)], t=0.06)
        vwall(W, -3.0, -0.8, 2.9, 1.8, "net", [(1.6, 1.4, 1.8)], t=0.06)
        vwall(W, 2.0, -0.8, 2.9, 1.8, "net", [], t=0.06)
    for (px, py) in ((-3.0, -0.8), (2.0, -0.8), (-3.0, 2.85), (2.0, 2.85)):
        W.box("cage_post", px, py, 0.0, 0.1, 0.1, 1.9)
    arm(W, 0.4, 1.2)
    bench(W, 0.9, -0.3, 1.7, 0.5, "legs")
    W.box("steel", 1.3, 0.1, 0.9, 0.3, 0.3, 0.12, solid=False)
    # ------------------------------------------------------------------ drone cage 8.2 x 8.2 x 5
    cx0, cy0, cx1, cy1, ch = 3.5, -8.2, 11.7, 0.0, 5.0
    with W.tagged("cage"):
        hwall(W, cy1, cx0, cx1, ch, "net", [(6.0, 1.6, ch)], t=0.08)
        hwall(W, cy0, cx0, cx1, ch, "net", [], t=0.08)
        vwall(W, cx0, cy0, cy1, ch, "net", [(-4.1, 1.6, ch)], t=0.08)
        vwall(W, cx1, cy0, cy1, ch, "net", [], t=0.08)
    for k in range(5):
        for (px, py) in ((cx0 + k * 2.05, cy0), (cx0 + k * 2.05, cy1)):
            W.box("cage_post", px, py, 0.0, 0.12, 0.12, ch + 0.1, solid=(k % 4 == 0))
    for k in range(1, 4):
        W.box("cage_post", cx0, cy0 + k * 2.05, 0.0, 0.12, 0.12, ch + 0.1, solid=False)
        W.box("cage_post", cx1, cy0 + k * 2.05, 0.0, 0.12, 0.12, ch + 0.1, solid=False)
    W.boxp("cage_post", cx0 - 0.06, cy0 - 0.06, cx1 + 0.06, cy0 + 0.06, ch, 0.1, solid=False)
    W.boxp("cage_post", cx0 - 0.06, cy1 - 0.06, cx1 + 0.06, cy1 + 0.06, ch, 0.1, solid=False)
    W.boxp("cage_post", cx0 - 0.06, cy0, cx0 + 0.06, cy1, ch, 0.1, solid=False)
    W.boxp("cage_post", cx1 - 0.06, cy0, cx1 + 0.06, cy1, ch, 0.1, solid=False)
    W.boxp("net", cx0, cy0, cx1, cy1, ch + 0.02, 0.02, solid=False)   # roof net: visual only
    W.boxp("steel", 5.0, -0.7, 6.0, -0.3, 0.0, 0.0, solid=False) if False else None
    W.box("cab_grey", 12.9, -3.0, 0.0, 0.7, 1.0, 0.8)                 # drone control desk outside the cage
    W.box("screen", 12.9, -3.0, 0.82, 0.5, 0.05, 0.4, solid=False)
    W.box("steel", 12.9, -4.5, 0.0, 0.3, 0.3, 1.0, solid=False) if False else None
    # ------------------------------------------------------------------ ceiling
    W.roof("steel", -14, -10, 14, 10, H, th=0.3, beam_mat="truss", beams=6, beam_axis="y", beam_w=0.3, beam_h=0.7)
    for x in (-12, -6, 0, 6, 12):
        for y in (-7.5, -2.5):
            W.box("lampbox", x, y, H - 0.1, 1.2, 0.3, 0.06, solid=False)
    for x in (-6.0, -1.5, 1.0):
        W.box("lampbox", x, 7.5, H - 0.1, 1.2, 0.3, 0.06, solid=False)
    for k in range(5):                                    # clerestory windows, flush on the outer walls
        W.boxp("window", -12 + k * 5.5, -9.78, -10.5 + k * 5.5, -9.76, 3.4, 1.0, solid=False)
        W.boxp("window", -12 + k * 5.5, 9.76, -10.5 + k * 5.5, 9.78, 3.4, 1.0, solid=False)
    # ------------------------------------------------------------------ lights
    L = W.light
    L("hall_a", -9.0, -6.0, 5.0, (1.0, 0.96, 0.9), 0.9, 12, 160)
    L("hall_b", -0.5, -6.0, 5.0, (1.0, 0.96, 0.9), 0.9, 12, 160)
    L("hall_c", -9.0, 0.0, 5.0, (1.0, 0.96, 0.9), 0.8, 12, 140)
    L("hall_d", -0.5, 0.5, 5.0, (1.0, 0.96, 0.9), 0.8, 12, 140)
    L("cage_a", 5.6, -6.0, 5.0, (0.95, 0.97, 1.0), 0.9, 12, 160)
    L("cage_b", 9.6, -6.0, 5.0, (0.95, 0.97, 1.0), 0.9, 12, 160)
    L("cage_c", 5.6, -2.0, 5.0, (0.95, 0.97, 1.0), 0.9, 12, 160)
    L("cage_d", 9.6, -2.0, 5.0, (0.95, 0.97, 1.0), 0.9, 12, 160)
    L("east", 12.5, -6.0, 5.0, (1.0, 0.96, 0.9), 0.7, 10, 120)
    L("corr_w", -9.0, 4.0, 4.5, (1.0, 0.98, 0.95), 0.6, 10, 100)
    L("corr_e", 6.0, 4.0, 4.5, (1.0, 0.98, 0.95), 0.6, 12, 100)
    L("server", -11.2, 7.4, 3.2, (0.45, 0.6, 1.0), 0.8, 7, 120)
    L("office_w", -6.0, 7.5, 3.5, (1.0, 0.95, 0.85), 0.8, 7, 110)
    L("office_e", -1.5, 7.5, 3.5, (1.0, 0.95, 0.85), 0.8, 7, 110)
    L("storage_a", 5.0, 7.5, 4.2, (1.0, 0.97, 0.92), 0.8, 9, 130)
    L("storage_b", 11.0, 7.5, 4.2, (1.0, 0.97, 0.92), 0.8, 9, 130)


# ---- zones, named spawns, short (< 60 m) pairs
_ZONES = {"arena":"arena","arena_start":"arena","workshop":"workshop","robot_arm":"workshop","cage_gate":"cage","cage_centre":"cage","server_room":"server","office_west":"offices","office_east":"offices","storage":"offices","corridor_west":"corridor","corridor_east":"corridor","south_east":"workshop"}
GOALS = [g + (_ZONES[g[0]],) for g in GOALS]
SPAWNS = {g[0]: (g[1], g[2], g[3]) for g in GOALS if g[0] in ['arena', 'workshop', 'cage_centre', 'server_room', 'office_east', 'corridor_east']}
SPAWNS["default"] = SPAWN
SHORT_PAIRS = []
for _i, _a in enumerate(GOALS):
    for _b in GOALS[_i + 1:]:
        _d = math.hypot(_a[1] - _b[1], _a[2] - _b[2])
        if _d < 60.0:
            SHORT_PAIRS.append([_a[0], _b[0], round(_d, 1)])
