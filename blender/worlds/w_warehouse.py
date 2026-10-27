"""warehouse: 48 x 32 m. Interior x[-24,24] y[-16,16].

Building x[-24,14] (roof at 10 m, clear airspace above the 5.5 m racks), outside yard x[14,24] (open sky).
  office block   x[-24,-15] y[2,16]   office + break room (enclosed, 3.2 m ceiling)
  charging area  x[-24,-15] y[-16,2]  AMR/forklift charging bays along the west wall
  rack field     x[-13,6]   y[-13.5,13.5]  four double-rack blocks (long axis y), 3 m aisles, 4 m cross aisle at y=0
  dock apron     x[6,14]    staged pallets, conveyor, forklifts; four dock doors in the x=14 wall (y -11,-4,4,11)
  yard           x[14,24]   containers (one is a walk-around island), a box trailer, guard hut, light poles
Racks carry one full-height collision column per run (constant footprint) so the drone can not fly through them.
"""
import math

from common import hwall, vwall, zfloor

NAME = "warehouse"
BOUNDS = (-24.0, -16.0, 24.0, 16.0)
OUTER = (-24.5, -16.5, 24.5, 16.5)
SPAWN = (6.9, 0.3, math.pi)
GOALS = [
    ("dock_north", 11.0, 11.0, 0.0),
    ("dock_south", 11.0, -11.0, 0.0),
    ("yard_gate", 19.5, -6.5, 0.0),
    ("yard_north", 19.5, 8.5, math.pi / 2),
    ("charging", -20.0, -9.0, math.pi),
    ("office", -17.5, 12.2, math.pi),
    ("break_room", -17.8, 5.5, math.pi),
    ("aisle_a", -8.4, -8.0, math.pi / 2),
    ("aisle_b", -3.2, 7.5, math.pi / 2),
    ("aisle_c", 2.0, -7.0, -math.pi / 2),
    ("cross_aisle", -9.0, 0.3, math.pi),
    ("staging", 11.7, -2.0, 0.0),
    ("south_lane", -5.0, -14.8, 0.0),
    ("north_lane", 0.0, 14.8, math.pi),
]
DRONE = {
    "takeoff": (-3.2, 0.3),
    "cruise": 7.5,
    "waypoints": [
        ("aisle_a_south", -8.4, -9.0, 7.2),
        ("aisle_a_north", -8.4, 9.0, 7.6),
        ("aisle_b_north", -3.2, 9.0, 8.2),
        ("aisle_b_south", -3.2, -9.0, 7.4),
        ("aisle_c_south", 2.0, -9.0, 7.8),
        ("aisle_c_north", 2.0, 9.0, 7.2),
        ("cross_aisle", -3.2, 0.3, 8.2),
        ("dock_apron", 10.5, 0.3, 7.5),
    ],
}
SCENE = {
    "ambient": (0.48, 0.48, 0.5),
    "background": (0.4, 0.42, 0.46),
    "sun": {"color": (0.85, 0.85, 0.8), "dir": (0.3, 0.2, -1.0)},
}
PREVIEW = {
    "cam": (30.0, -42.0, 34.0), "target": (0.0, 0.0, 0.4), "lens": 30,
    "world": (0.55, 0.58, 0.64), "world_strength": 1.0,
    "hide": ["roof"],
    "cut": [("perim_s", 0.5), ("perim_e", 0.5), ("inner", 1.3)],
    "extra_lights": [("SUN", (0, 0, 10), (1.0, 0.95, 0.88), 3.5, 0)],
    "light_rot": (math.radians(35), math.radians(18), math.radians(28)),
    "light_gain": 0.3,
    "views": [
        ("aisle", (-8.4, -15.4, 2.0), (-8.4, 0.0, 2.6), 20),
        ("docks", (6.5, -14.5, 3.0), (12.0, 0.0, 0.8), 22),
        ("yard", (14.8, -15.0, 3.0), (20.0, 2.0, 0.8), 22),
        ("office", (-14.3, 1.0, 2.6), (-20.0, 10.0, 0.8), 22),
        ("charging", (-14.3, -15.3, 3.0), (-21.0, -8.0, 0.6), 22),
    ],
}

MATS = {
    "floor": dict(tex="concrete_floor_worn_001", tile=3.0),
    "apron": dict(tex="concrete_layers_02", tile=3.0, tint=(1.1, 1.1, 1.1)),
    "asphalt": dict(tex="worn_asphalt", tile=3.0, tint=(0.75, 0.8, 0.95)),
    "wall": dict(tex="corrugated_iron_02", tile=2.0, tint=(0.75, 0.8, 0.85)),
    "fence": dict(tex="concrete_layers_02", tile=2.0, tint=(0.9, 0.9, 0.9)),
    "wall_in": dict(tex="concrete_floor_01", tile=2.0, tint=(1.2, 1.2, 1.2)),
    "plaster": dict(tex="painted_plaster_wall", tile=1.6, tint=(1.3, 1.25, 1.15)),
    "steel": dict(tex="metal_plate", tile=1.0, metal=0.3),
    "pallet": dict(tex="brown_planks_03", tile=1.0),
    "orange": dict(color=(0.8, 0.22, 0.01), rough=0.5),
    "blue": dict(color=(0.02, 0.1, 0.4), rough=0.5),
    "yellow": dict(color=(0.85, 0.6, 0.02), rough=0.5),
    "black": dict(color=(0.015, 0.015, 0.018), rough=0.6),
    "grey": dict(color=(0.2, 0.22, 0.25), rough=0.5, metal=0.2),
    "carton_a": dict(color=(0.32, 0.2, 0.09), rough=0.9),
    "carton_b": dict(color=(0.4, 0.27, 0.13), rough=0.9),
    "carton_c": dict(color=(0.25, 0.17, 0.09), rough=0.9),
    "shrink": dict(color=(0.55, 0.6, 0.65), rough=0.2),
    "door": dict(color=(0.18, 0.22, 0.28), rough=0.5, metal=0.3),
    "window": dict(color=(0.8, 0.9, 1.0), emit=((0.8, 0.9, 1.0), 2.5)),
    "d_yellow": dict(color=(0.8, 0.55, 0.02), rough=0.7),
    "d_white": dict(color=(0.7, 0.7, 0.7), rough=0.7),
    "d_black": dict(color=(0.01, 0.01, 0.012), rough=0.8),
    "d_green": dict(color=(0.04, 0.4, 0.1), rough=0.7),
    "belt": dict(color=(0.02, 0.02, 0.025), rough=0.9),
    "cont_red": dict(tex="corrugated_iron_02", tile=1.2, tint=(0.8, 0.22, 0.15)),
    "cont_blue": dict(tex="corrugated_iron_02", tile=1.2, tint=(0.15, 0.3, 0.6)),
    "cont_green": dict(tex="corrugated_iron_02", tile=1.2, tint=(0.2, 0.45, 0.25)),
    "cont_grey": dict(tex="corrugated_iron_02", tile=1.2, tint=(0.7, 0.72, 0.75)),
    "trailer": dict(tex="corrugated_iron_02", tile=1.2, tint=(0.9, 0.9, 0.92)),
    "desk": dict(tex="oak_veneer_05", tile=1.0, bevel=0.01),
    "chair": dict(color=(0.05, 0.08, 0.12), rough=0.8, bevel=0.015),
    "screen": dict(color=(0.03, 0.08, 0.15), emit=((0.15, 0.4, 0.8), 1.0)),
    "cabinet": dict(color=(0.45, 0.47, 0.5), rough=0.4, metal=0.5, bevel=0.01),
    "charger": dict(color=(0.75, 0.77, 0.8), rough=0.4, bevel=0.015),
    "led": dict(color=(0.1, 0.8, 0.25), emit=((0.15, 1.0, 0.3), 3.0)),
    "lampbox": dict(color=(1.0, 0.95, 0.85), emit=((1.0, 0.95, 0.85), 3.0)),
    "truss": dict(color=(0.1, 0.11, 0.13), rough=0.6, metal=0.4),
    "tire": dict(color=(0.01, 0.01, 0.012), rough=0.9),
}
CART = ("carton_a", "carton_b", "carton_c")
H = 10.0
RACK_H = 5.5


def rack(W, ax, u0, u1, v0, v1, bay=2.875, seed=0):
    """One rack run: u is the long axis, v the depth. ax = 'x' or 'y' gives the world axis of u."""
    def bp(mat, a0, b0, a1, b1, z, h, solid=False):
        if ax == "x":
            W.boxp(mat, a0, b0, a1, b1, z, h, solid=solid)
        else:
            W.boxp(mat, b0, a0, b1, a1, z, h, solid=solid)

    if ax == "x":
        W.col_box((u0 + u1) / 2, (v0 + v1) / 2, 0.0, u1 - u0, v1 - v0, RACK_H)
    else:
        W.col_box((v0 + v1) / 2, (u0 + u1) / 2, 0.0, v1 - v0, u1 - u0, RACK_H)
    n = max(1, int(round((u1 - u0) / bay)))
    bw = (u1 - u0) / n
    for i in range(n):
        bp(CART[(i + seed) % 3], u0 + i * bw + 0.01, v0 + 0.02, u0 + (i + 1) * bw - 0.01, v1 - 0.02, 0.0, 0.8)
    for i in range(n + 1):
        uu = min(max(u0 + i * bw, u0 + 0.05), u1 - 0.05)
        for vv in (v0 + 0.05, v1 - 0.05):
            if ax == "x":
                W.box("orange", uu, vv, 0.0, 0.1, 0.1, RACK_H - 0.1, solid=False)
            else:
                W.box("orange", vv, uu, 0.0, 0.1, 0.1, RACK_H - 0.1, solid=False)
    levels = [0.8, 1.75, 2.7, 3.65]
    for k, z in enumerate(levels):
        if k > 0:
            for vb in (v0 + 0.04, v1 - 0.04):
                bp("blue", u0, vb - 0.04, u1, vb + 0.04, z - 0.14, 0.14)
        for i in range(n):
            for j in range(2):
                h = (i * 5 + j * 3 + k * 7 + seed) % 5
                if h == 0:
                    continue
                m = CART[(i + j + k + seed) % 3]
                bx0 = u0 + i * bw + 0.15 + j * bw / 2
                bp(m, bx0, v0 + 0.12, bx0 + bw / 2 - 0.2, v1 - 0.12, z, 0.5 + 0.08 * h)
    for vb in (v0 + 0.04, v1 - 0.04):
        bp("blue", u0, vb - 0.04, u1, vb + 0.04, 4.6, 0.14)


def pallet_stack(W, x, y, yaw, nx=3, ny=2, nz=3, seed=0, wrap=False):
    W.box("pallet", x, y, 0.0, 1.2, 0.8, 0.15, yaw=yaw, solid=False)
    ch = 0.3
    c, s = math.cos(yaw), math.sin(yaw)
    for k in range(nz):
        for i in range(nx):
            for j in range(ny):
                lx = -0.6 + (i + 0.5) * 1.2 / nx
                ly = -0.4 + (j + 0.5) * 0.8 / ny
                W.box("shrink" if wrap else CART[(i + j + k + seed) % 3], x + lx * c - ly * s, y + lx * s + ly * c, 0.15 + k * ch,
                      1.2 / nx - 0.02, 0.8 / ny - 0.02, ch - 0.01, yaw=yaw, solid=False)
    W.col_box(x, y, 0.0, 1.2, 0.8, 0.15 + nz * ch, yaw)


def pallet_pile(W, x, y, yaw, n=5):
    for k in range(n):
        W.box("pallet", x, y, k * 0.15, 1.2, 0.8, 0.14, yaw=yaw, solid=False)
    W.col_box(x, y, 0.0, 1.2, 0.8, n * 0.15, yaw)


def forklift(W, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)

    def P(lx, ly):
        return (x + lx * c - ly * s, y + lx * s + ly * c)

    def B(m, lx, ly, z0, sx, sy, h, solid=False):
        px, py = P(lx, ly)
        W.box(m, px, py, z0, sx, sy, h, yaw=yaw, solid=solid)

    B("yellow", -0.075, 0.0, 0.0, 1.65, 1.1, 0.95, solid=True)
    B("black", -0.9, 0.0, 0.25, 0.04, 0.9, 0.5)
    for lx in (0.35, -0.55):
        for sy in (-1, 1):
            px, py = P(lx, sy * 0.5)
            qx, qy = P(lx, sy * 0.56)
            W.frustum("black", (px, py, 0.3), (qx, qy, 0.3), 0.3, 0.3, seg=16)
    B("black", -0.35, 0.0, 0.95, 0.4, 0.4, 0.3)
    B("black", -0.55, 0.0, 1.1, 0.06, 0.4, 0.4)
    for (lx, ly) in ((-0.8, -0.5), (-0.8, 0.5), (0.2, -0.5), (0.2, 0.5)):
        B("black", lx, ly, 0.95, 0.06, 0.06, 1.15)
    B("black", -0.3, 0.0, 2.1, 1.2, 1.1, 0.05)
    for ly in (-0.3, 0.3):
        B("steel", 0.82, ly, 0.0, 0.1, 0.1, 2.2, solid=True)
    B("steel", 0.82, 0.0, 1.6, 0.1, 0.7, 0.08)
    B("black", 0.92, 0.0, 0.7, 0.05, 0.8, 0.5)
    for ly in (-0.3, 0.3):
        B("steel", 1.55, ly, 0.7, 1.2, 0.1, 0.04)
    B("orange", -0.6, 0.0, 2.15, 0.3, 0.2, 0.08)


def conveyor(W, x0, x1, y0, y1):
    W.boxp("blue", x0, y0, x1, y1, 0.0, 0.8)
    W.boxp("belt", x0 + 0.05, y0 + 0.07, x1 - 0.05, y1 - 0.07, 0.8, 0.03, solid=False)
    n = int((x1 - x0) / 0.12)
    for i in range(n):
        W.box("steel", x0 + 0.06 + i * 0.12, (y0 + y1) / 2, 0.83, 0.03, y1 - y0 - 0.14, 0.015, solid=False)
    W.boxp("steel", x0, y0, x1, y0 + 0.04, 0.8, 0.12, solid=False)
    W.boxp("steel", x0, y1 - 0.04, x1, y1, 0.8, 0.12, solid=False)


def pillar(W, x, y):
    W.box("grey", x, y, 0.0, 0.5, 0.5, H)
    W.box("yellow", x, y, 0.0, 0.505, 0.505, 1.0, solid=False)
    W.box("steel", x, y, 0.0, 0.7, 0.7, 0.04, solid=False)


def container(W, x, y, along, mat, stack=1):
    """Shipping container 6.1 x 2.4 x 2.6 (solid, full height of the stack), ribbed doors at one end."""
    L, Wd = 6.1, 2.4
    sx, sy = (L, Wd) if along == "x" else (Wd, L)
    for k in range(stack):
        W.box(mat, x, y, k * 2.6, sx, sy, 2.6, solid=(k == 0))
        # corner posts + roof ribs
        for ex in (-1, 1):
            for ey in (-1, 1):
                W.box("steel", x + ex * (sx / 2 - 0.03), y + ey * (sy / 2 - 0.03), k * 2.6, 0.12, 0.12, 2.6, solid=False)
        W.box("steel", x, y, k * 2.6 + 2.56, sx + 0.04, sy + 0.04, 0.06, solid=False)
        W.box("steel", x, y, k * 2.6, sx + 0.04, sy + 0.04, 0.1, solid=False)
    if stack > 1:
        W.col_box(x, y, 0.0, sx, sy, 2.6 * stack)
    # doors end
    if along == "x":
        W.box("steel", x + L / 2 + 0.03, y, 0.1, 0.05, Wd - 0.1, 2.4, solid=False)
        W.box("black", x + L / 2 + 0.065, y, 1.0, 0.03, 0.04, 1.4, solid=False)
    else:
        W.box("steel", x, y + L / 2 + 0.03, 0.1, Wd - 0.1, 0.05, 2.4, solid=False)
        W.box("black", x, y + L / 2 + 0.065, 1.0, 0.04, 0.03, 1.4, solid=False)


def desk(W, x0, y0, x1, y1, mon_side=1):
    top = 0.75
    W.boxp("desk", x0, y0, x1, y1, top, 0.04, solid=False)
    for (lx, ly) in ((x0 + 0.05, y0 + 0.05), (x1 - 0.05, y0 + 0.05), (x0 + 0.05, y1 - 0.05), (x1 - 0.05, y1 - 0.05)):
        W.box("steel", lx, ly, 0.0, 0.06, 0.06, top)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    W.box("black", cx, cy + mon_side * (y1 - y0) / 2 * 0.5, top + 0.04, 0.5, 0.04, 0.3, solid=False)
    W.box("screen", cx, cy + mon_side * ((y1 - y0) / 2 * 0.5 - 0.025), top + 0.07, 0.46, 0.012, 0.26, solid=False)
    W.box("black", cx, cy - mon_side * 0.05, top + 0.04, 0.4, 0.14, 0.015, solid=False)


def office_chair(W, x, y):
    W.cyl("chair", x, y, 0.0, 0.25, 0.46)
    W.box("chair", x, y, 0.46, 0.45, 0.45, 0.08, solid=False)
    W.box("chair", x, y - 0.2, 0.54, 0.45, 0.06, 0.45, solid=False)


def build(W):
    W.floor("floor", surface="concrete")
    t = 0.4
    # ------------------------------------------------------------------ perimeter: 10 m building walls, 3.5 m yard fence
    W.wall((-24 - t, -16 - t / 2), (14.4, -16 - t / 2), t, H, "wall", tag="perim_s", ext=False)
    W.wall((14.4, -16 - t / 2), (24 + t, -16 - t / 2), t, 3.5, "fence", tag="perim_s", ext=False)
    W.wall((-24 - t, 16 + t / 2), (14.4, 16 + t / 2), t, H, "wall", tag="perim_n", ext=False)
    W.wall((14.4, 16 + t / 2), (24 + t, 16 + t / 2), t, 3.5, "fence", tag="perim_n", ext=False)
    W.wall((-24 - t / 2, -16), (-24 - t / 2, 16), t, H, "wall", tag="perim_w", ext=False)
    W.wall((24 + t / 2, -16), (24 + t / 2, 16), t, 3.5, "fence", tag="perim_e", ext=False)
    # yard + apron surfaces
    zfloor(W, "asphalt", 14.2, -16, 24, 16, h=0.02)
    zfloor(W, "apron", 6.0, -16, 14.2, 16, h=0.01)
    # ------------------------------------------------------------------ building east wall with four dock doors
    with W.tagged("inner"):
        vwall(W, 14.0, -16, 16, H, "wall_in", [(y, 2.7, 3.4) for y in (-11, -4, 4, 11)], t)
        # office block (3.2 m ceiling): east wall with two doors, south wall with one, partition with one
        vwall(W, -15.0, 2, 16, 3.2, "plaster", [(12.0, 1.3, 2.15), (5.5, 1.3, 2.15)], 0.2)
        hwall(W, 2.0, -24, -15, 3.2, "plaster", [(-19.5, 1.4, 2.15)], 0.2)
        hwall(W, 9.0, -24, -15, 3.2, "plaster", [(-20.0, 1.2, 2.15)], 0.2)
    W.boxp("plaster", -24, 2.0, -14.9, 16, 3.2, 0.15, solid=True, tag="roof")      # office ceiling slab
    # ------------------------------------------------------------------ roof over the building (hidden in the preview)
    W.roof("steel", -24, -16, 14.2, 16, H, th=0.3, beam_mat="truss", beams=9, beam_axis="x", beam_w=0.4, beam_h=0.9)
    for x in range(-20, 14, 8):
        for y in (-12, -4, 4, 12):
            W.box("lampbox", x, y, H - 1.3, 1.6, 0.35, 0.08, solid=False, tag="roof")
            W.box("black", x, y, H - 1.22, 0.05, 0.05, 1.2, solid=False, tag="roof")
    # clerestory windows on the long walls
    for x in range(-20, 12, 6):
        W.boxp("window", x - 1.4, 15.985, x + 1.4, 15.99, 6.6, 1.5, solid=False, tag="perim_n")
        W.boxp("window", x - 1.4, -15.99, x + 1.4, -15.985, 6.6, 1.5, solid=False, tag="perim_s")
    # ------------------------------------------------------------------ dock doors: roller shutters, bumpers, hazard stripes
    for y in (-11, -4, 4, 11):
        W.boxp("door", 13.92, y - 1.35, 13.99, y + 1.35, 3.4, 0.5, solid=False, tag="inner")    # shutter box
        for k in range(4):
            W.boxp("black", 13.9, y - 1.35, 13.93, y + 1.35, 3.0 - k * 0.12, 0.03, solid=False, tag="inner")
        W.stripes("d_yellow", "d_black", 12.4, y - 1.45, 12.9, y + 1.45, step=0.25, along="y")
        for sy in (-1.65, 1.65):
            W.box("black", 14.45, y + sy, 0.0, 0.3, 0.3, 0.9)
            W.box("yellow", 14.45, y + sy, 0.9, 0.3, 0.3, 0.08, solid=False)
    # ------------------------------------------------------------------ rack field
    for bi, cx in enumerate((-11.0, -5.8, -0.6, 4.6)):
        for ri, (ya, yb) in enumerate(((-13.5, -2.0), (2.0, 13.5))):
            rack(W, "y", ya, yb, cx - 1.1, cx, seed=bi + ri)
            rack(W, "y", ya, yb, cx, cx + 1.1, seed=bi + ri + 1)
    for (px, py) in ((8.0, -12.0), (8.0, -8.0), (8.0, 8.0), (8.0, 12.0)):
        pillar(W, px, py)
    # aisle markings
    for cx in (-8.4, -3.2, 2.0):
        for ya, yb in ((-13.5, -2.0), (2.0, 13.5)):
            W.line("d_yellow", cx - 1.35, ya, cx - 1.35, yb, 0.08)
            W.line("d_yellow", cx + 1.35, ya, cx + 1.35, yb, 0.08)
    W.line("d_white", -13.5, 0.3, 6.0, 0.3, 0.12)
    for y in (-14.75, 14.75):
        W.line("d_white", -14.6, y, 13.6, y, 0.08)
    # ------------------------------------------------------------------ dock apron: staged pallets, conveyor, forklifts
    conveyor(W, 9.0, 11.0, -0.4, 0.5)
    conveyor(W, 12.4, 13.7, -0.4, 0.5)
    stage = [(8.9, -7.4, 0.0), (10.3, -7.4, 0.05), (11.8, -7.5, -0.05), (9.2, -14.2, 0.0), (10.9, -14.2, 0.1), (12.6, -14.2, 0.0),
             (8.9, 7.4, 0.0), (10.3, 7.5, 0.05), (11.8, 7.4, -0.05), (9.2, 14.2, 0.0), (10.9, 14.2, -0.05), (12.6, 14.2, 0.05),
             (9.5, -2.2, 0.0), (11.4, 2.3, 0.05)]
    for i, (x, y, yaw) in enumerate(stage):
        if i == 5:
            pallet_pile(W, x, y, yaw)
        else:
            pallet_stack(W, x, y, yaw, nx=3, ny=2, nz=2 + (i % 3), seed=i, wrap=(i % 4 == 1))
    forklift(W, 12.6, -9.2, 1.9)
    forklift(W, 12.4, 9.4, -2.2)
    # ------------------------------------------------------------------ charging area (SW)
    for i, y in enumerate([-14.4 + 1.9 * k for k in range(7)]):
        W.box("charger", -23.6, y, 0.0, 0.4, 0.45, 1.3)
        W.box("led", -23.38, y, 1.0, 0.02, 0.2, 0.06, solid=False)
        W.box("black", -23.38, y, 0.75, 0.02, 0.2, 0.2, solid=False)
        # bay outline
        W.line("d_yellow", -23.2, y - 0.75, -20.6, y - 0.75, 0.06)
        W.line("d_yellow", -23.2, y + 0.75, -20.6, y + 0.75, 0.06)
        W.line("d_yellow", -20.6, y - 0.75, -20.6, y + 0.75, 0.06)
        if i in (1, 3, 5):                               # parked carts on charge
            W.box("blue", -22.0, y, 0.0, 1.0, 0.7, 0.45)
            W.box("steel", -22.0, y, 0.45, 1.02, 0.72, 0.03, solid=False)
            W.box("orange", -22.0, y, 0.48, 0.2, 0.2, 0.06, solid=False)
    for y in (-15.0, -12.0, -9.0, -6.0, -3.0, 0.0):
        W.cyl("yellow", -17.2, y, 0.0, 0.1, 0.9, seg=12)
    W.stripes("d_yellow", "d_black", -17.6, -16.0, -16.9, 1.8, step=0.35, along="y")
    forklift(W, -19.0, -4.8, 0.2)
    # ------------------------------------------------------------------ office block
    zfloor(W, "apron", -24, 2, -15, 16, h=0.012)
    W.decal("d_green", -19.5, 12.5, 5.0, 3.0, z=0.016)
    # window band on the hall side (visual)
    for y in (3.5, 8.0, 14.4):
        W.boxp("window", -14.94, y - 0.8, -14.92, y + 0.8, 1.2, 1.3, solid=False)
        W.boxp("black", -14.95, y - 0.85, -14.9, y + 0.85, 1.15, 0.05, solid=False)
        W.boxp("black", -14.95, y - 0.85, -14.9, y + 0.85, 2.5, 0.05, solid=False)
    # office (y 9..16)
    for k, (x0, y0) in enumerate(((-23.2, 14.6), (-21.2, 14.6), (-23.2, 11.2), (-21.2, 11.2))):
        desk(W, x0, y0 - 0.0, x0 + 1.5, y0 + 0.75, mon_side=1)
        office_chair(W, x0 + 0.75, y0 - 0.5)
    desk(W, -18.2, 15.0, -16.2, 15.8, mon_side=1)
    office_chair(W, -17.2, 14.45)
    for k in range(3):
        W.box("cabinet", -23.6 + 0.62 * k, 9.45, 0.0, 0.55, 0.6, 1.4)
    W.box("cabinet", -15.45, 15.3, 0.0, 0.6, 0.6, 1.4)
    # break room (y 2..9)
    W.boxp("desk", -22.2, 5.2, -19.8, 6.2, 0.75, 0.04, solid=False)
    for (lx, ly) in ((-22.15, 5.25), (-19.85, 5.25), (-22.15, 6.15), (-19.85, 6.15)):
        W.box("steel", lx, ly, 0.0, 0.06, 0.06, 0.75)
    for (cx, cy, bx, by) in ((-21.4, 4.65, 0, -1), (-20.6, 4.65, 0, -1), (-21.4, 6.75, 0, 1), (-20.6, 6.75, 0, 1)):
        W.box("chair", cx, cy, 0.0, 0.42, 0.42, 0.46)
    W.boxp("steel", -24, 7.8, -21.0, 8.4, 0.0, 0.9)            # kitchenette counter
    W.boxp("black", -24, 7.78, -21.0, 8.42, 0.9, 0.04, solid=False)
    W.box("steel", -15.55, 8.4, 0.0, 0.7, 0.7, 1.85)           # fridge
    W.box("cabinet", -23.65, 3.2, 0.0, 0.5, 0.5, 1.2)          # water cooler
    W.cyl("window", -23.65, 3.2, 1.2, 0.17, 0.4, seg=12, solid=False)
    # ------------------------------------------------------------------ yard: containers, trailer, hut, light poles
    container(W, 18.4, 0.0, "y", "cont_red")                       # walk-around island x[17.2,19.6] y[-3.05,3.05]
    container(W, 19.0, 14.6, "x", "cont_blue", stack=2)
    container(W, 19.0, -14.4, "x", "cont_green", stack=2)
    container(W, 22.5, 6.6, "y", "cont_grey")
    # box trailer parked at dock door y = -11
    W.box("trailer", 19.2, -11.0, 1.0, 8.4, 2.5, 2.5, solid=False)
    W.col_box(19.2, -11.0, 0.0, 8.4, 2.5, 3.5)
    W.box("steel", 19.2, -11.0, 0.9, 8.4, 2.4, 0.15, solid=False)
    W.box("trailer", 15.3, -11.0, 0.0, 0.1, 2.5, 3.5, solid=False)
    for lx in (1.6, 2.5, 3.4):
        for sy in (-1, 1):
            W.frustum("tire", (19.2 + lx, -11.0 + sy * 1.05, 0.5), (19.2 + lx, -11.0 + sy * 1.26, 0.5), 0.5, 0.5, seg=16)
    # guard hut
    W.box("plaster", 22.4, -7.2, 0.0, 2.6, 2.4, 2.6)
    W.box("steel", 22.4, -7.2, 2.6, 3.0, 2.8, 0.15, solid=False)
    W.boxp("window", 21.12, -7.9, 21.14, -6.5, 1.1, 1.0, solid=False)
    # fence gate (visual) and bollards by the dock doors
    for dy in (-2.5, 2.5):
        W.boxp("steel", 23.9, 9.0 + dy - 0.15, 24.0, 9.0 + dy + 0.15, 0.0, 3.3, solid=False)
    W.boxp("steel", 23.92, 6.5, 23.97, 11.5, 0.2, 0.08, solid=False)
    W.boxp("steel", 23.92, 6.5, 23.97, 11.5, 2.8, 0.08, solid=False)
    for k in range(10):
        W.boxp("steel", 23.92, 6.55 + k * 0.5, 23.96, 6.6 + k * 0.5, 0.2, 2.7, solid=False)
    for (px, py) in ((16.0, -6.0), (16.0, 6.0), (22.0, -1.5), (22.0, 12.0)):
        W.cyl("grey", px, py, 0.0, 0.12, 6.0, seg=10)
        W.box("lampbox", px, py, 6.0, 0.8, 0.3, 0.1, solid=False)
    # yard markings
    W.line("d_white", 14.5, -8.0, 24.0, -8.0, 0.15)
    W.line("d_white", 14.5, 7.0, 24.0, 7.0, 0.15)

    # ------------------------------------------------------------------ lights (point lights, no shadows)
    L = W.light
    cool = (1.0, 0.97, 0.9)
    for i, x in enumerate((-18, -10, -2, 6)):
        for j, y in enumerate((-10, 0, 10)):
            L("hall_%d_%d" % (i, j), x, y, 8.8, cool, 1.2, 16, 400)
    L("docks_s", 10.5, -8.0, 6.0, cool, 1.0, 12, 300)
    L("docks_n", 10.5, 8.0, 6.0, cool, 1.0, 12, 300)
    L("charging", -20.0, -8.0, 5.0, (0.8, 0.95, 1.0), 1.0, 11, 300)
    L("office", -19.5, 12.5, 2.9, (1.0, 0.95, 0.85), 0.9, 8, 150)
    L("break", -19.5, 5.5, 2.9, (1.0, 0.95, 0.85), 0.8, 8, 150)
    L("yard_a", 18.0, -7.0, 6.2, (1.0, 0.85, 0.6), 1.0, 14, 350)
    L("yard_b", 18.0, 8.0, 6.2, (1.0, 0.85, 0.6), 1.0, 14, 350)


# ---- zones, named spawns, short (< 60 m) pairs
_ZONES = {"dock_north":"docks","dock_south":"docks","yard_gate":"yard","yard_north":"yard","charging":"charging","office":"office","break_room":"office","aisle_a":"aisles","aisle_b":"aisles","aisle_c":"aisles","cross_aisle":"aisles","staging":"docks","south_lane":"lanes","north_lane":"lanes"}
GOALS = [g + (_ZONES[g[0]],) for g in GOALS]
SPAWNS = {g[0]: (g[1], g[2], g[3]) for g in GOALS if g[0] in ['charging', 'office', 'aisle_a', 'aisle_c', 'dock_north', 'yard_gate', 'cross_aisle']}
SPAWNS["default"] = SPAWN
SHORT_PAIRS = []
for _i, _a in enumerate(GOALS):
    for _b in GOALS[_i + 1:]:
        _d = math.hypot(_a[1] - _b[1], _a[2] - _b[2])
        if _d < 60.0:
            SHORT_PAIRS.append([_a[0], _b[0], round(_d, 1)])
