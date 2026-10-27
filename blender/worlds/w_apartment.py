"""apartment: 22 x 16 m flat. Interior x[-11,11] y[-8,8], ceiling 2.7 m.

South band (y -8..-1): entry hall x[-11,-7] | living + dining x[-7,4.5] (balcony strip y[-8,-6.2]) | kitchen x[4.5,11].
Corridor y[-1,1] runs the full 22 m. North band (y 1..8): master bedroom x[-11,-4.5] (ensuite x[-4.5,-1] y[4.5,8]),
kid's room x[-4.5,-1] y[1,4.5], guest bedroom x[-1,4.5], study x[4.5,11] y[1,5], family bathroom x[4.5,11] y[5,8].
Loops: entry-living-corridor, living-kitchen-corridor, guest bedroom-study-corridor.
All low furniture is solid with a constant footprint; decoration is above 0.6 m or inside a footprint.
"""
import math

from apt_props import books, curtain, curtain_rod, picture, plant, stool, table, table_lamp, wbox
from common import hwall, vwall, zfloor

NAME = "apartment"
BOUNDS = (-11.0, -8.0, 11.0, 8.0)
OUTER = (-11.2, -8.2, 11.2, 8.2)
SPAWN = (-9.0, -4.5, 0.0)
GOALS = [
    ("living_room", -2.6, -2.9, 0.0),
    ("balcony", -0.5, -7.1, 0.0),
    ("dining", -0.4, -3.4, 0.0),
    ("kitchen", 8.2, -2.8, 0.0),
    ("corridor_east", 9.6, 0.0, 0.0),
    ("master_bedroom", -7.3, 3.1, 0.0),
    ("ensuite", -3.1, 5.5, 0.0),
    ("kids_room", -2.55, 2.2, 0.0),
    ("guest_bedroom", 1.8, 3.4, math.pi / 2),
    ("study", 7.6, 3.0, 0.0),
    ("family_bath", 7.6, 6.3, 0.0),
    ("entry", -9.0, -2.3, 0.0),
]
DRONE = None
SCENE = {
    "ambient": (0.5, 0.5, 0.52),
    "background": (0.7, 0.78, 0.9),
    "sun": {"color": (0.9, 0.85, 0.75), "dir": (0.4, 0.25, -1.0)},
}
PREVIEW = {
    "cam": (12.0, -24.0, 22.0), "target": (0.0, 0.0, 0.3), "lens": 36,
    "world": (0.7, 0.75, 0.85), "world_strength": 1.0,
    "hide": ["roof"],
    "cut": [("perim_s", 0.6), ("perim_e", 0.6), ("inner", 1.15)],
    "extra_lights": [("SUN", (0, 0, 10), (1.0, 0.95, 0.88), 3.2, 0)],
    "light_rot": (math.radians(35), math.radians(15), math.radians(25)),
    "light_gain": 0.1,
    "views": [
        ("living", (-6.5, -1.4, 2.0), (0.0, -5.0, 0.6), 24),
        ("kitchen", (5.5, -1.5, 2.2), (9.0, -6.0, 0.6), 24),
        ("master", (-4.7, 1.4, 2.0), (-9.0, 5.0, 0.5), 24),
        ("study", (5.0, 1.4, 2.0), (9.5, 3.5, 0.8), 24),
    ],
}

MATS = {
    "floor_wood": dict(tex="laminate_floor_02", tile=1.5),
    "floor_parquet": dict(tex="herringbone_parquet", tile=1.2),
    "floor_tile": dict(tex="stone_tiles_02", tile=1.6, tint=(1.5, 1.45, 1.4)),
    "deck": dict(tex="brown_planks_03", tile=1.2, tint=(0.8, 0.75, 0.7)),
    "wall": dict(tex="painted_plaster_wall", tile=1.6, tint=(1.35, 1.25, 1.1)),
    "wall_out": dict(tex="painted_plaster_wall", tile=1.6, tint=(1.15, 1.08, 1.0)),
    "wall_teal": dict(tex="white_plaster_02", tile=1.6, tint=(0.28, 0.5, 0.52)),
    "wall_plum": dict(tex="white_plaster_02", tile=1.6, tint=(0.45, 0.25, 0.4)),
    "wall_tile": dict(color=(0.8, 0.85, 0.86), rough=0.15),
    "oak": dict(tex="oak_veneer_05", tile=1.0, bevel=0.012),
    "walnut": dict(tex="oak_veneer_05", tile=1.0, tint=(0.45, 0.3, 0.2), bevel=0.012),
    "white": dict(color=(0.75, 0.75, 0.72), rough=0.5, bevel=0.01),
    "ceramic": dict(color=(0.85, 0.87, 0.88), rough=0.15, bevel=0.015),
    "counter": dict(color=(0.04, 0.04, 0.045), rough=0.3, bevel=0.006),
    "cab_sage": dict(color=(0.14, 0.24, 0.17), rough=0.5, bevel=0.01),
    "steel": dict(color=(0.5, 0.52, 0.55), rough=0.25, metal=0.9, bevel=0.008),
    "sofa": dict(color=(0.12, 0.17, 0.25), rough=0.95, bevel=0.035),
    "armchair": dict(color=(0.45, 0.2, 0.12), rough=0.95, bevel=0.035),
    "cushion": dict(color=(0.55, 0.3, 0.1), rough=0.95, bevel=0.03),
    "cushion2": dict(color=(0.5, 0.58, 0.45), rough=0.95, bevel=0.03),
    "linen": dict(color=(0.7, 0.68, 0.64), rough=0.95, bevel=0.025),
    "blanket": dict(color=(0.12, 0.2, 0.3), rough=0.95, bevel=0.02),
    "blanket2": dict(color=(0.45, 0.14, 0.18), rough=0.95, bevel=0.02),
    "blanket3": dict(color=(0.2, 0.4, 0.35), rough=0.95, bevel=0.02),
    "rug_a": dict(color=(0.42, 0.14, 0.11), rough=1.0),
    "rug_b": dict(color=(0.17, 0.3, 0.36), rough=1.0),
    "rug_c": dict(color=(0.55, 0.42, 0.26), rough=1.0),
    "black": dict(color=(0.012, 0.012, 0.015), rough=0.4),
    "screen": dict(color=(0.03, 0.08, 0.15), emit=((0.15, 0.4, 0.8), 1.0)),
    "plant": dict(color=(0.06, 0.32, 0.08), rough=0.7),
    "pot": dict(color=(0.45, 0.2, 0.1), rough=0.8, bevel=0.02),
    "window": dict(color=(0.7, 0.85, 1.0), emit=((0.75, 0.88, 1.0), 2.2)),
    "frame": dict(color=(0.7, 0.7, 0.68), rough=0.5, bevel=0.004),
    "door": dict(color=(0.22, 0.1, 0.04), rough=0.6),
    "frame_dk": dict(color=(0.07, 0.045, 0.03), rough=0.5, bevel=0.004),
    "art_a": dict(color=(0.1, 0.28, 0.32), rough=0.8),
    "art_b": dict(color=(0.75, 0.5, 0.18), rough=0.8),
    "art_c": dict(color=(0.55, 0.2, 0.12), rough=0.8),
    "art_d": dict(color=(0.85, 0.82, 0.74), rough=0.9),
    "curtain": dict(color=(0.5, 0.42, 0.32), rough=1.0),
    "curtain2": dict(color=(0.58, 0.5, 0.4), rough=1.0),
    "leaf": dict(color=(0.08, 0.34, 0.1), rough=0.6),
    "leaf2": dict(color=(0.16, 0.42, 0.14), rough=0.6),
    "shade": dict(color=(0.9, 0.78, 0.55), emit=((1.0, 0.78, 0.45), 0.8), rough=0.9),
    "book_a": dict(color=(0.5, 0.05, 0.05), rough=0.8),
    "book_b": dict(color=(0.05, 0.2, 0.4), rough=0.8),
    "book_c": dict(color=(0.7, 0.55, 0.1), rough=0.8),
    "lamp": dict(color=(1.0, 0.85, 0.55), emit=((1.0, 0.8, 0.5), 1.5)),
    "water": dict(color=(0.5, 0.7, 0.8), rough=0.05),
    "chair": dict(color=(0.3, 0.2, 0.12), rough=0.7, bevel=0.01),
    "mirror": dict(color=(0.7, 0.78, 0.82), metal=1.0, rough=0.05),
    "planter": dict(color=(0.3, 0.3, 0.32), rough=0.8, bevel=0.02),
}

H = 2.7
T = 0.2


def casing(W, horiz, pos, c, w, h=2.15):
    """Door casing strips either side of an opening (proud of the wall by 3 cm)."""
    for s in (-1, 1):
        o = c + s * (w / 2 + 0.035)
        if horiz:
            W.box("frame", o, pos, 0.0, 0.07, T + 0.06, h, solid=False)
        else:
            W.box("frame", pos, o, 0.0, T + 0.06, 0.07, h, solid=False)
    if horiz:
        W.box("frame", c, pos, h, w + 0.14, T + 0.06, 0.07, solid=False)
    else:
        W.box("frame", pos, c, h, T + 0.06, w + 0.14, 0.07, solid=False)


def hw(W, y, xa, xb, doors):
    hwall(W, y, xa, xb, H, "wall", [(d[0], d[1], 2.15) for d in doors], T)
    for d in doors:
        casing(W, True, y, d[0], d[1])


def vw(W, x, ya, yb, doors):
    vwall(W, x, ya, yb, H, "wall", [(d[0], d[1], d[2] if len(d) > 2 else 2.15) for d in doors], T)
    for d in doors:
        casing(W, False, x, d[0], d[1], d[2] if len(d) > 2 else 2.15)


def window(W, x, y, nx, ny, w, z0=0.95, z1=2.15):
    """Glowing window pane with frame, flush on the wall face (visual only)."""
    wbox(W, "frame", x, y, z0 - 0.05, w + 0.1, z1 - z0 + 0.1, 0.05, nx, ny)
    wbox(W, "window", x, y, z0, w, z1 - z0, 0.06, nx, ny)
    wbox(W, "frame", x, y, z0, 0.04, z1 - z0, 0.07, nx, ny)
    wbox(W, "frame", x, y, (z0 + z1) / 2, w, 0.04, 0.07, nx, ny)
    wbox(W, "frame", x, y, z0 - 0.08, w + 0.18, 0.04, 0.12, nx, ny)


def furn(W, mat, x0, y0, x1, y1, h):
    W.boxp(mat, x0, y0, x1, y1, 0.0, h)


def chair(W, x, y, bx, by):
    W.box("chair", x, y, 0.0, 0.42, 0.42, 0.46)
    W.box("cushion2", x, y, 0.46, 0.36, 0.36, 0.04, solid=False)
    sx, sy = (0.04, 0.42) if bx else (0.42, 0.04)
    W.box("chair", x + bx * 0.19, y + by * 0.19, 0.5, sx, sy, 0.5, solid=False)


def armchair(W, x, y, bx, by, mat="armchair"):
    W.box(mat, x, y, 0.0, 0.85, 0.85, 0.5)
    sx, sy = (0.2, 0.85) if bx else (0.85, 0.2)
    W.box(mat, x + bx * 0.32, y + by * 0.32, 0.5, sx, sy, 0.4, solid=False)
    W.box("cushion", x - bx * 0.08, y - by * 0.08, 0.5, 0.6, 0.6, 0.08, solid=False)


def sofa(W, x0, y0, x1, y1, back):
    """Solid body 0.5 m; back/arms/cushions sit inside the footprint. back: 'N','S','E','W'."""
    W.boxp("sofa", x0, y0, x1, y1, 0.0, 0.5)
    t = 0.24
    if back == "N":
        W.boxp("sofa", x0, y1 - t, x1, y1, 0.5, 0.42, solid=False)
        W.boxp("sofa", x0, y0, x0 + 0.2, y1 - t, 0.5, 0.2, solid=False)
        W.boxp("sofa", x1 - 0.2, y0, x1, y1 - t, 0.5, 0.2, solid=False)
        n = int((x1 - x0 - 0.4) / 0.8)
        for i in range(n):
            cx0 = x0 + 0.2 + i * (x1 - x0 - 0.4) / n
            W.boxp("cushion2" if i % 2 else "sofa", cx0 + 0.02, y0 + 0.02, cx0 + (x1 - x0 - 0.4) / n - 0.02, y1 - t, 0.5, 0.1, solid=False)
        W.ellipsoid("cushion", (x0 + 0.5, y1 - 0.4, 0.78), (0.24, 0.07, 0.2), seg=10, rings=6, pitch=0.3)
        W.ellipsoid("cushion2", (x1 - 0.5, y1 - 0.4, 0.76), (0.22, 0.07, 0.18), seg=10, rings=6, pitch=0.3)
    else:  # back on the south side
        W.boxp("sofa", x0, y0, x1, y0 + t, 0.5, 0.42, solid=False)
        W.boxp("sofa", x0, y0 + t, x0 + 0.2, y1, 0.5, 0.2, solid=False)
        W.boxp("sofa", x1 - 0.2, y0 + t, x1, y1, 0.5, 0.2, solid=False)
        n = int((x1 - x0 - 0.4) / 0.8)
        for i in range(n):
            cx0 = x0 + 0.2 + i * (x1 - x0 - 0.4) / n
            W.boxp("cushion2" if i % 2 else "sofa", cx0 + 0.02, y0 + t, cx0 + (x1 - x0 - 0.4) / n - 0.02, y1 - 0.02, 0.5, 0.1, solid=False)
        W.ellipsoid("cushion", (x0 + 0.5, y0 + 0.4, 0.78), (0.24, 0.07, 0.2), seg=10, rings=6, pitch=-0.3)
        W.ellipsoid("cushion2", (x1 - 0.5, y0 + 0.4, 0.76), (0.22, 0.07, 0.18), seg=10, rings=6, pitch=-0.3)


def bed(W, x0, y0, x1, y1, head, blanket="blanket"):
    """Solid frame 0.5 m; mattress, blanket, pillows and headboard sit inside the footprint."""
    W.boxp("oak", x0, y0, x1, y1, 0.0, 0.5)
    W.boxp("linen", x0 + 0.03, y0 + 0.03, x1 - 0.03, y1 - 0.03, 0.5, 0.11, solid=False)
    hb = 0.08
    if head == "W":
        W.boxp("walnut", x0, y0, x0 + hb, y1, 0.5, 0.75, solid=False)
        W.boxp(blanket, x0 + 0.7, y0 + 0.02, x1 - 0.02, y1 - 0.02, 0.58, 0.05, solid=False)
        ps = [(x0 + 0.32, y0 + (y1 - y0) * k) for k in (0.27, 0.73)]
    elif head == "E":
        W.boxp("walnut", x1 - hb, y0, x1, y1, 0.5, 0.75, solid=False)
        W.boxp(blanket, x0 + 0.02, y0 + 0.02, x1 - 0.7, y1 - 0.02, 0.58, 0.05, solid=False)
        ps = [(x1 - 0.32, y0 + (y1 - y0) * k) for k in (0.27, 0.73)]
    elif head == "N":
        W.boxp("walnut", x0, y1 - hb, x1, y1, 0.5, 0.75, solid=False)
        W.boxp(blanket, x0 + 0.02, y0 + 0.02, x1 - 0.02, y1 - 0.7, 0.58, 0.05, solid=False)
        ps = [(x0 + (x1 - x0) * k, y1 - 0.32) for k in (0.27, 0.73)]
    else:
        W.boxp("walnut", x0, y0, x1, y0 + hb, 0.5, 0.75, solid=False)
        W.boxp(blanket, x0 + 0.02, y0 + 0.7, x1 - 0.02, y1 - 0.02, 0.58, 0.05, solid=False)
        ps = [(x0 + (x1 - x0) * k, y0 + 0.32) for k in (0.27, 0.73)]
    wide = (x1 - x0) > (y1 - y0) if head in "WE" else (y1 - y0) > (x1 - x0)
    for (px, py) in ps:
        W.ellipsoid("linen", (px, py, 0.66), (0.28, 0.2, 0.08) if head in "NS" else (0.2, 0.28, 0.08), seg=10, rings=6)


def nightstand(W, cx, cy, lamp=True):
    W.box("walnut", cx, cy, 0.0, 0.45, 0.45, 0.55)
    W.box("walnut", cx, cy, 0.55, 0.47, 0.47, 0.03, solid=False)
    if lamp:
        table_lamp(W, cx, cy, 0.58)


def tv(W, x, y, nx, ny, w=1.3):
    wbox(W, "black", x, y, 0.95, w, 0.75, 0.05, nx, ny, off=0.02)
    wbox(W, "screen", x, y, 0.98, w - 0.06, 0.69, 0.052, nx, ny, off=0.02)


def big_plant(W, x, y, s=2.0):
    W.cyl("pot", x, y, 0.0, 0.2, 0.62, seg=14)
    plant(W, x, y, 0.62, s=s, n=14, pot=False, seed=x)


def build(W):
    W.floor("floor_wood", surface="laminate")
    W.perimeter("wall_out", 0.4, H)
    # ------------------------------------------------------------------ zone floors
    zfloor(W, "floor_tile", -11, -8, -7.1, -1.1)
    zfloor(W, "floor_tile", 4.6, -8, 11, -1.1)
    zfloor(W, "floor_tile", -4.4, 4.6, -1.1, 8)
    zfloor(W, "floor_tile", 4.6, 5.1, 11, 8)
    zfloor(W, "floor_parquet", -11, -0.9, 11, 0.9)
    zfloor(W, "deck", -7.1, -8, 4.6, -6.2)
    # ------------------------------------------------------------------ walls
    with W.tagged("inner"):
        hw(W, -1, -11, 11, [(-9, 1.2), (-3, 1.4), (1.0, 1.4), (8, 1.2)])
        hw(W, 1, -11, 11, [(-6, 1.2), (-2.75, 1.2), (1.8, 1.2), (7.5, 1.2)])
        vw(W, -7, -8, -1, [(-4.0, 1.6)])
        vw(W, 4.5, -8, -1, [(-4.8, 2.2, 2.3)])
        vw(W, -4.5, 1, 8, [(6.2, 1.2)])
        vw(W, -1, 1, 8, [])
        hw(W, 4.5, -4.5, -1, [])
        vw(W, 4.5, 1, 8, [(3.0, 1.2), (6.5, 1.2)])
        hw(W, 5, 4.5, 11, [])
        # glazed balcony screen: solid sill below 0.9 m, glass above, three gaps (two doors, one wall panel)
        for (a, b) in ((-7.1, -3.6), (-1.4, 0.3), (1.7, 4.6)):
            W.boxp("wall", a, -6.2, b, -6.0, 0.0, 0.9, solid=True)
            W.boxp("window", a + 0.04, -6.15, b - 0.04, -6.05, 0.9, 1.6, solid=False)
            for mx in [a + 0.05 + k * 0.9 for k in range(int((b - a) / 0.9) + 1)]:
                if mx < b:
                    W.boxp("frame", mx - 0.025, -6.2, mx + 0.025, -6.0, 0.9, 1.6, solid=False)
            W.boxp("frame", a, -6.2, b, -6.0, 2.5, 0.2, solid=False)
            W.boxp("frame", a, -6.22, b, -5.98, 0.9, 0.04, solid=False)
        for (a, b) in ((-3.6, -1.4), (0.3, 1.7)):
            W.boxp("frame", a, -6.2, b, -6.0, 2.3, 0.4, solid=False)
            W.boxp("frame", a, -6.2, a + 0.05, -6.0, 0.0, 2.3, solid=False)
            W.boxp("frame", b - 0.05, -6.2, b, -6.0, 0.0, 2.3, solid=False)
    # ceiling slabs (hidden in the preview); the balcony strip stays open to the sky
    W.roof("wall_out", -11.0, -8.0, -7.1, 8.0, H, th=0.2)
    W.roof("wall_out", 4.6, -8.0, 11.0, 8.0, H, th=0.2)
    W.roof("wall_out", -7.1, -6.0, 4.6, 8.0, H, th=0.2)
    # skirting boards (3 cm proud, 9 cm high would be a wall detail; kept under the solid wall footprint) are skipped on purpose

    # ------------------------------------------------------------------ entry hall
    W.decal("rug_c", -9.0, -4.5, 2.2, 3.4, z=0.006)
    furn(W, "walnut", -11.0, -6.5, -10.55, -5.1, 0.9)               # shoe cabinet
    W.boxp("walnut", -11.0, -6.55, -10.5, -5.05, 0.9, 0.04, solid=False)
    table_lamp(W, -10.75, -5.8, 0.94)
    wbox(W, "mirror", -11.0, -5.8, 1.15, 1.0, 0.9, 0.025, 1, 0)
    wbox(W, "frame_dk", -11.0, -5.8, 1.1, 1.1, 1.0, 0.02, 1, 0)
    for k in range(4):                                             # coat hooks and coats
        wbox(W, "black", -11.0, -3.4 - k * 0.22, 1.6, 0.04, 0.04, 0.08, 1, 0)
    for k, m in enumerate(("blanket", "armchair", "cab_sage")):
        wbox(W, m, -11.0, -3.4 - k * 0.22, 0.75, 0.16, 0.85, 0.14, 1, 0, off=0.06)
    # front door on the south wall
    wbox(W, "door", -8.8, -8.0, 0.0, 1.05, 2.15, 0.05, 0, 1)
    wbox(W, "frame_dk", -8.8, -8.0, 2.12, 1.2, 0.08, 0.07, 0, 1)
    wbox(W, "steel", -8.35, -8.0, 1.0, 0.04, 0.2, 0.09, 0, 1, off=0.05)
    picture(W, -9.8, -1.1, 1.3, 0.8, 0.6, 0, 1, 2)
    big_plant(W, -10.45, -7.4, 2.2)

    # ------------------------------------------------------------------ living + dining
    W.decal("rug_a", -3.4, -3.7, 4.6, 2.9, z=0.006)
    W.decal("rug_b", -3.4, -3.7, 4.2, 2.5, z=0.01)
    sofa(W, -6.0, -5.95, -3.3, -5.0, "S")                            # back to the balcony screen
    furn(W, "walnut", -4.95, -4.0, -3.85, -3.4, 0.42)               # coffee table (solid)
    W.boxp("walnut", -5.0, -4.05, -3.8, -3.35, 0.42, 0.03, solid=False)
    W.boxp("blanket3", -4.6, -3.95, -4.1, -3.55, 0.45, 0.02, solid=False)
    books(W, -4.45, -3.7, 0.47, 3)
    armchair(W, -1.7, -5.45, 0, 1)
    furn(W, "walnut", -6.1, -1.55, -4.2, -1.1, 0.75)                 # TV console
    W.boxp("oak", -6.12, -1.57, -4.18, -1.1, 0.75, 0.03, solid=False)
    tv(W, -5.15, -1.1, 0, -1, 1.35)
    table_lamp(W, -4.4, -1.35, 0.78)
    furn(W, "walnut", -6.9, -6.0, -6.5, -4.9, 2.0)                  # bookcase on the west wall
    wbox(W, "frame_dk", -6.9, -5.45, 2.0, 0.3, 0.04, 1.1, 1, 0)
    plant(W, -6.7, -5.45, 2.04, s=1.1, n=8, pot=True, seed=3)
    big_plant(W, -1.0, -5.65, 2.3)
    # dining
    W.decal("rug_c", 2.2, -3.4, 3.3, 2.7, z=0.006)
    table(W, 1.3, -3.9, 3.1, -2.9, 0.75)
    W.boxp("ceramic", 1.9, -3.55, 2.5, -3.25, 0.8, 0.02, solid=False)
    W.cyl("water", 2.2, -3.4, 0.8, 0.05, 0.2, seg=10, solid=False)
    for px in (1.7, 2.7):
        chair(W, px, -4.35, 0, -1)
        chair(W, px, -2.45, 0, 1)
    chair(W, 0.75, -3.4, -1, 0)
    chair(W, 3.65, -3.4, 1, 0)
    furn(W, "walnut", 2.4, -1.55, 3.9, -1.1, 0.85)                   # sideboard
    W.boxp("oak", 2.38, -1.57, 3.92, -1.1, 0.85, 0.03, solid=False)
    plant(W, 3.4, -1.35, 0.88, s=0.9, n=8, pot=True, seed=7)
    picture(W, 3.15, -1.1, 1.35, 1.0, 0.7, 0, -1, 0)
    picture(W, 0.0, -1.1, 1.35, 0.6, 0.8, 0, -1, 3)
    W.cyl("steel", 2.2, -3.4, 1.85, 0.015, 0.85, seg=6, solid=False)   # pendant over the table
    W.frustum("shade", (2.2, -3.4, 1.55), (2.2, -3.4, 1.85), 0.3, 0.1, seg=16)
    W.frustum("shade", (-3.4, -3.6, 1.55), (-3.4, -3.6, 1.85), 0.26, 0.09, seg=16)
    W.cyl("steel", -3.4, -3.6, 1.85, 0.015, 0.85, seg=6, solid=False)
    # living room curtains + wall art
    curtain(W, -5.3, -6.0, 0, 1, 0.95, 2.55, 0.9, mat="curtain")
    curtain(W, 0.9, -6.0, 0, 1, 0.95, 2.55, 0.5, mat="curtain")
    picture(W, -6.9, -2.9, 1.4, 0.9, 0.65, 1, 0, 1)
    picture(W, 4.4, -2.0, 1.4, 0.7, 0.9, -1, 0, 2)

    # ------------------------------------------------------------------ balcony (open sky)
    for (a, b) in ((-6.8, -5.6), (-5.6, -4.4), (3.3, 4.4)):
        W.box("planter", (a + b) / 2, -7.82, 0.0, b - a - 0.05, 0.3, 0.62)
        plant(W, (a + b) / 2, -7.82, 0.62, s=1.3, n=11, pot=False, seed=a)
    W.cyl("steel", 0.8, -7.5, 0.0, 0.05, 0.72, seg=8)
    W.cyl("steel", 0.8, -7.5, 0.72, 0.38, 0.03, seg=20, solid=False)
    W.cyl("steel", 0.8, -7.5, 0.0, 0.2, 0.04, seg=16, solid=False)

    # ------------------------------------------------------------------ kitchen
    furn(W, "cab_sage", 4.7, -8.0, 10.95, -7.38, 0.9)
    furn(W, "cab_sage", 10.38, -7.4, 11.0, -3.9, 0.9)
    W.boxp("counter", 4.7, -8.0, 10.95, -7.36, 0.9, 0.04, solid=False)
    W.boxp("counter", 10.36, -7.4, 11.0, -3.9, 0.9, 0.04, solid=False)
    wbox(W, "wall_tile", 8.0, -8.0, 0.94, 6.2, 0.55, 0.02, 0, 1)
    wbox(W, "wall_tile", 11.0, -5.6, 0.94, 3.4, 0.55, 0.02, -1, 0)
    for (a, b) in ((4.8, 6.9), (9.4, 10.9)):
        wbox(W, "cab_sage", (a + b) / 2, -8.0, 1.5, b - a, 0.7, 0.34, 0, 1)
    wbox(W, "steel", 8.2, -8.0, 1.5, 1.0, 0.1, 0.4, 0, 1)             # extractor hood
    wbox(W, "steel", 8.2, -8.0, 1.6, 0.28, 0.9, 0.3, 0, 1)
    W.boxp("steel", 7.6, -7.9, 8.4, -7.5, 0.93, 0.01, solid=False)    # hob
    W.cyl("black", 7.8, -7.7, 0.94, 0.09, 0.012, seg=14, solid=False)
    W.cyl("black", 8.2, -7.7, 0.94, 0.11, 0.012, seg=14, solid=False)
    W.cyl("steel", 8.9, -7.7, 0.94, 0.12, 0.1, seg=12, solid=False)   # pot on the hob
    furn(W, "steel", 10.1, -1.85, 10.9, -1.1, 1.85)                   # fridge
    W.box("black", 9.7, -1.86, 1.0, 0.03, 0.03, 0.5, solid=False)
    furn(W, "oak", 6.4, -5.05, 8.6, -4.15, 0.92)                      # island
    W.boxp("counter", 6.35, -5.1, 8.65, -4.1, 0.92, 0.04, solid=False)
    W.cyl("ceramic", 7.0, -4.6, 0.96, 0.15, 0.06, seg=14, solid=False)
    W.cyl("plant", 7.0, -4.6, 1.0, 0.1, 0.02, seg=10, solid=False)
    for sx in (6.8, 7.5, 8.2):
        stool(W, sx, -3.8)
    window(W, 8.1, -8.0, 0, 1, 2.0, 1.2, 2.2)
    window(W, 11.0, -2.9, -1, 0, 1.4)
    curtain(W, 11.0, -2.9, -1, 0, 1.0, 2.3, 1.6, mat="curtain")
    for px in (7.2, 8.0):
        W.cyl("steel", px, -4.6, 1.9, 0.01, 0.55, seg=6, solid=False)
    W.frustum("shade", (7.2, -4.6, 1.5), (7.2, -4.6, 1.9), 0.18, 0.08, seg=14)
    W.frustum("shade", (8.0, -4.6, 1.5), (8.0, -4.6, 1.9), 0.18, 0.08, seg=14)
    picture(W, 4.6, -7.4, 1.4, 0.6, 0.45, 1, 0, 0)
    big_plant(W, 5.1, -7.5, 1.8)

    # ------------------------------------------------------------------ corridor
    W.decal("rug_b", 0.0, 0.0, 14.0, 0.8, z=0.008)
    W.decal("rug_a", 0.0, 0.0, 13.6, 0.5, z=0.012)
    for i, x in enumerate((-7.6, -4.6, -1.1, 3.8, 5.5, 9.8)):
        picture(W, x, 0.9, 1.35, 0.5, 0.7, 0, -1, i)
    for i, x in enumerate((-10.0, -5.4, -0.2, 4.0, 6.2)):
        picture(W, x, -0.9, 1.35, 0.6, 0.45, 0, 1, i + 1)
    window(W, 11.0, 0.0, -1, 0, 1.0, 0.95, 2.1)
    for x in (-5, 5):
        wbox(W, "lamp", x, -0.9, 1.9, 0.12, 0.2, 0.08, 0, 1)
        wbox(W, "black", x, -0.9, 1.85, 0.14, 0.04, 0.08, 0, 1)

    # ------------------------------------------------------------------ master bedroom (x -11..-4.5, y 1..8)
    W.decal("rug_b", -8.6, 4.5, 4.2, 3.4, z=0.006)
    bed(W, -11.0, 3.6, -8.9, 5.4, "W", "blanket2")
    nightstand(W, -10.75, 3.2)
    nightstand(W, -10.75, 5.8)
    wbox(W, "wall_plum", -11.0, 4.5, 0.05, 3.4, 2.4, 0.02, 1, 0)
    picture(W, -11.0, 4.5, 1.6, 1.3, 0.8, 1, 0, 3)
    furn(W, "walnut", -10.9, 7.4, -8.2, 8.0, 2.3)                    # wardrobe
    for k in range(4):
        W.boxp("walnut", -10.9 + 0.675 * k + 0.03, 7.36, -10.9 + 0.675 * (k + 1) - 0.03, 7.4, 0.1, 2.1, solid=False)
        W.box("steel", -10.9 + 0.675 * k + 0.55, 7.35, 1.0, 0.02, 0.03, 0.3, solid=False)
    furn(W, "walnut", -9.6, 1.1, -8.1, 1.65, 0.8)                    # dresser
    W.boxp("oak", -9.62, 1.1, -8.08, 1.67, 0.8, 0.03, solid=False)
    wbox(W, "mirror", -8.85, 1.1, 1.2, 0.9, 1.0, 0.025, 0, 1)
    wbox(W, "frame_dk", -8.85, 1.1, 1.15, 1.0, 1.1, 0.02, 0, 1)
    table_lamp(W, -9.0, 1.4, 0.83)
    window(W, -6.6, 8.0, 0, -1, 1.8)
    curtain(W, -6.6, 8.0, 0, -1, 0.95, 2.45, 2.4)
    armchair(W, -5.5, 7.25, 0, 1, "armchair")
    big_plant(W, -10.5, 1.55, 1.8)

    # ------------------------------------------------------------------ ensuite (x -4.5..-1, y 4.5..8)
    W.box("ceramic", -3.5, 7.62, 0.0, 1.7, 0.75, 0.55)                # bathtub
    W.boxp("water", -4.3, 7.3, -2.7, 7.95, 0.55, 0.02, solid=False)
    W.box("steel", -4.2, 7.9, 0.55, 0.05, 0.05, 0.3, solid=False)
    furn(W, "ceramic", -1.65, 7.1, -1.1, 7.5, 0.42)                  # toilet bowl
    W.boxp("ceramic", -1.5, 7.1, -1.12, 7.5, 0.42, 0.4, solid=False)
    furn(W, "oak", -1.6, 5.2, -1.12, 6.2, 0.85)                      # vanity
    W.boxp("ceramic", -1.58, 5.35, -1.2, 6.05, 0.85, 0.02, solid=False)
    wbox(W, "mirror", -1.1, 5.7, 1.2, 0.9, 0.8, 0.025, -1, 0)
    wbox(W, "wall_tile", -3.5, 8.0, 0.95, 1.8, 1.0, 0.02, 0, -1)
    window(W, -3.5, 8.0, 0, -1, 0.8, 1.7, 2.3)
    W.decal("rug_c", -3.1, 5.9, 1.0, 0.7, z=0.016)

    # ------------------------------------------------------------------ kid's room (x -4.5..-1, y 1..4.5)
    W.decal("rug_a", -2.7, 2.7, 1.8, 1.5, z=0.006)
    bed(W, -4.4, 2.35, -3.5, 4.4, "N", "blanket3")
    table(W, -3.15, 3.85, -1.4, 4.4, 0.74)
    chair(W, -2.3, 3.4, 0, 1)
    books(W, -2.1, 4.1, 0.77, 3)
    table_lamp(W, -1.65, 4.15, 0.79)
    furn(W, "oak", -1.5, 1.15, -1.1, 2.2, 1.5)                       # shelf unit
    for k, z in enumerate((0.9, 1.15, 1.4)):
        books(W, -1.3, 1.5 + 0.35 * k, z + 0.04, 2, yaw=1.57)
    picture(W, -4.4, 1.7, 1.4, 0.6, 0.5, 1, 0, 3)

    # ------------------------------------------------------------------ guest bedroom (x -1..4.5, y 1..8)
    W.decal("rug_c", 2.0, 5.5, 3.0, 2.6, z=0.006)
    bed(W, 1.2, 5.9, 2.9, 8.0, "N", "blanket")
    nightstand(W, 0.8, 7.6)
    nightstand(W, 3.3, 7.6)
    furn(W, "walnut", -0.9, 4.0, -0.28, 6.8, 2.2)                    # wardrobe on the west wall
    W.boxp("walnut", -0.28, 4.05, -0.24, 6.75, 0.1, 2.1, solid=False)
    window(W, 4.0, 8.0, 0, -1, 0.9)
    curtain(W, 4.0, 8.0, 0, -1, 0.95, 2.4, 1.5)
    big_plant(W, 3.95, 1.55, 1.8)
    picture(W, 2.0, 8.0, 1.5, 1.0, 0.7, 0, -1, 0)

    # ------------------------------------------------------------------ study (x 4.5..11, y 1..5)
    W.decal("rug_b", 8.0, 3.0, 3.2, 2.6, z=0.006)
    table(W, 10.1, 1.9, 10.9, 4.1, 0.75)                             # desk along the east wall
    W.boxp("black", 10.2, 2.7, 10.6, 3.3, 0.78, 0.02, solid=False)    # laptop
    W.box("screen", 10.82, 3.0, 0.8, 0.02, 0.5, 0.35, solid=False)
    table_lamp(W, 10.55, 2.1, 0.78)
    books(W, 10.5, 3.8, 0.78, 3)
    chair(W, 9.55, 3.0, -1, 0)
    window(W, 11.0, 3.0, -1, 0, 1.8, 1.15, 2.2)
    curtain(W, 11.0, 3.0, -1, 0, 1.0, 2.35, 2.4)
    furn(W, "walnut", 4.62, 3.75, 4.98, 4.85, 2.1)                    # bookcase beside the west doorway
    for k, z in enumerate((0.7, 1.1, 1.5, 1.9)):
        books(W, 4.8, 4.0 + 0.3 * (k % 2), z, 3, yaw=1.57)
    furn(W, "walnut", 8.4, 1.1, 10.8, 1.5, 1.9)                       # shelves on the south wall
    for k, z in enumerate((0.7, 1.1, 1.5)):
        books(W, 8.8 + 0.5 * k, 1.35, z, 3)
    armchair(W, 5.5, 1.75, 0, 1, "sofa")
    big_plant(W, 7.0, 4.55, 1.9)
    wbox(W, "art_c", 8.0, 4.9, 1.4, 1.2, 0.8, 0.04, 0, -1)

    # ------------------------------------------------------------------ family bathroom (x 4.5..11, y 5..8)
    W.box("ceramic", 5.9, 7.62, 0.0, 1.7, 0.75, 0.55)                 # tub
    W.boxp("water", 5.1, 7.3, 6.7, 7.95, 0.55, 0.02, solid=False)
    furn(W, "oak", 8.0, 7.45, 9.6, 8.0, 0.85)                         # vanity
    W.boxp("ceramic", 8.1, 7.55, 9.5, 7.95, 0.85, 0.02, solid=False)
    wbox(W, "mirror", 8.8, 8.0, 1.2, 1.3, 0.9, 0.025, 0, -1)
    furn(W, "ceramic", 10.35, 7.0, 11.0, 7.4, 0.42)                    # toilet
    W.boxp("ceramic", 10.6, 7.0, 10.98, 7.4, 0.42, 0.4, solid=False)
    W.box("wall_tile", 10.45, 5.55, 0.0, 1.0, 0.9, 0.012, solid=False)  # shower tray decal
    W.boxp("window", 9.93, 5.1, 9.97, 6.1, 0.1, 2.0, solid=False)
    W.boxp("window", 9.93, 6.07, 11.0, 6.1, 0.1, 2.0, solid=False)
    W.boxp("frame", 9.9, 5.1, 9.96, 6.13, 2.1, 0.04, solid=False)
    wbox(W, "wall_tile", 6.0, 8.0, 0.95, 2.0, 1.1, 0.02, 0, -1)
    window(W, 6.0, 8.0, 0, -1, 0.9, 1.8, 2.4)
    wbox(W, "steel", 7.0, 5.1, 1.2, 0.7, 0.03, 0.05, 0, 1)             # towel rail
    wbox(W, "blanket2", 7.0, 5.1, 0.75, 0.55, 0.45, 0.04, 0, 1, off=0.05)

    # ------------------------------------------------------------------ lights (point lights, no shadows)
    L = W.light
    warm = (1.0, 0.82, 0.58)
    L("entry", -9.0, -4.5, 2.4, warm, 0.7, 7, 120)
    L("living_w", -4.0, -3.6, 2.4, warm, 0.8, 8, 150)
    L("dining", 2.2, -3.4, 2.4, warm, 0.8, 8, 150)
    L("balcony", -1.0, -7.2, 2.6, (0.8, 0.9, 1.0), 0.6, 6, 100)
    L("kitchen", 8.0, -4.6, 2.4, (1.0, 0.95, 0.85), 0.9, 8, 150)
    L("corridor_w", -6.0, 0.0, 2.4, warm, 0.6, 7, 100)
    L("corridor_m", 0.0, 0.0, 2.4, warm, 0.6, 7, 100)
    L("corridor_e", 6.0, 0.0, 2.4, warm, 0.6, 7, 100)
    L("master", -8.0, 4.5, 2.4, (1.0, 0.75, 0.5), 0.8, 8, 120)
    L("ensuite", -3.0, 6.0, 2.4, (1.0, 0.95, 0.9), 0.7, 5, 100)
    L("kids", -2.7, 2.7, 2.4, (1.0, 0.9, 0.7), 0.7, 5, 100)
    L("guest", 1.8, 4.6, 2.4, warm, 0.7, 7, 120)
    L("study", 8.0, 3.0, 2.4, (1.0, 0.95, 0.85), 0.8, 7, 120)
    L("bath", 8.0, 6.5, 2.4, (1.0, 0.95, 0.9), 0.7, 6, 100)


# ---- zones, named spawns, short (< 60 m) pairs
_ZONES = {"living_room":"living","dining":"living","balcony":"living","kitchen":"kitchen","corridor_east":"corridor","master_bedroom":"bedrooms","ensuite":"bedrooms","kids_room":"bedrooms","guest_bedroom":"bedrooms","study":"study","family_bath":"study","entry":"entry"}
GOALS = [g + (_ZONES[g[0]],) for g in GOALS]
SPAWNS = {g[0]: (g[1], g[2], g[3]) for g in GOALS if g[0] in ['entry', 'kitchen', 'master_bedroom', 'study', 'living_room', 'balcony']}
SPAWNS["default"] = SPAWN
SHORT_PAIRS = []
for _i, _a in enumerate(GOALS):
    for _b in GOALS[_i + 1:]:
        _d = math.hypot(_a[1] - _b[1], _a[2] - _b[2])
        if _d < 60.0:
            SHORT_PAIRS.append([_a[0], _b[0], round(_d, 1)])
