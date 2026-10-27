"""Props for the keep (reused from the first 40 x 30 m version) plus giant-scale pieces."""
import math, random
from common import banner, barrel, brazier, chain, crate, hwall, torch, vwall, wheel, zfloor
rng = random.Random(7)
def sarcophagus(W, x, y, yaw=0.0):
    W.box("stone", x, y, 0.0, 2.1, 0.95, 0.85, yaw=yaw)
    W.box("stone", x, y, 0.85, 2.2, 1.05, 0.08, yaw=yaw, solid=False)
    W.box("stone", x, y, 0.93, 1.8, 0.7, 0.12, yaw=yaw, solid=False)
    c, s = math.cos(yaw), math.sin(yaw)
    W.box("trim", x + 0.55 * c, y + 0.55 * s, 1.04, 0.5, 0.06, 0.02, yaw=yaw + math.pi / 2, solid=False)
    W.ellipsoid("bone", (x - 0.7 * c, y - 0.7 * s, 1.1), (0.14, 0.12, 0.1), seg=8, rings=5)


def gold_pile(W, x, y, r, h):
    """Solid footprint (r, 0.7 m), gold dome above."""
    W.cyl("gold_old", x, y, 0.0, r, 0.7, seg=18, r_top=r * 0.97)
    W.ellipsoid("gold", (x, y, 0.69), (r * 0.97, r * 0.97, h * 0.55), seg=28, rings=12)
    n = int(18 + 14 * r)
    for k in range(n):                      # lumpy coins and goblets heaped over the dome
        a = k * 2.399 + x * 3.1
        f = math.sqrt((k + 0.5) / n)        # fibonacci spiral fills the disc
        rr = r * 0.9 * f
        zz = 0.69 + h * 0.55 * math.sqrt(max(0.0, 1 - (rr / (r * 0.97)) ** 2)) + 0.02 * ((k * 7) % 5)
        sz = 0.11 + 0.025 * (k % 4)
        W.ellipsoid("gold_old" if k % 3 == 0 else "gold", (x + rr * math.cos(a), y + rr * math.sin(a), zz), (sz * 1.3, sz, sz * 0.55), seg=8, rings=5)
    W.ellipsoid("gold", (x, y, 0.69 + h * 0.55 + 0.03), (r * 0.3, r * 0.26, h * 0.3), seg=10, rings=6)
    for i in range(int(8 + r * 6)):
        a = rng.uniform(0, 2 * math.pi)
        rr = rng.uniform(0.1, r * 0.85)
        z = 0.69 + h * (1 - rr / (r * 0.97)) * 0.9
        W.ellipsoid("gold" if i % 4 else "gold_old", (x + rr * math.cos(a), y + rr * math.sin(a), z + 0.04), (0.13, 0.11, 0.07), seg=6, rings=4)
    for i in range(3):
        a = rng.uniform(0, 2 * math.pi)
        rr = rng.uniform(0.2, r * 0.7)
        W.ellipsoid("gem", (x + rr * math.cos(a), y + rr * math.sin(a), 0.69 + h * (1 - rr / r) * 0.9 + 0.1), (0.09, 0.09, 0.12), seg=6, rings=4)


def chest(W, x, y, yaw=0.0, open_=True):
    W.box("wood", x, y, 0.0, 0.95, 0.6, 0.62, yaw=yaw)
    W.box("trim", x, y, 0.0, 0.97, 0.62, 0.05, yaw=yaw, solid=False)
    W.box("trim", x, y, 0.57, 0.97, 0.62, 0.05, yaw=yaw, solid=False)
    W.ellipsoid("gold", (x, y, 0.72), (0.38, 0.24, 0.14), seg=10, rings=6)
    W.ellipsoid("gold_old", (x + 0.15, y + 0.05, 0.8), (0.18, 0.14, 0.1), seg=8, rings=5)


def portcullis(W, x, ya, yb, z0, z1, nx):
    n = int((yb - ya) / 0.25)
    for i in range(n + 1):
        W.box("iron", x, ya + i * (yb - ya) / n, z0, 0.06, 0.06, z1 - z0, solid=False)
    for z in [z0 + k * 0.5 for k in range(int((z1 - z0) / 0.5) + 1)]:
        W.box("iron", x, (ya + yb) / 2, z, 0.05, yb - ya, 0.05, solid=False)
    for i in range(n + 1):
        W.frustum("iron", (x, ya + i * (yb - ya) / n, z0), (x, ya + i * (yb - ya) / n, z0 - 0.0), 0.03, 0.0, seg=4)


def rib_arch(W, x, y_a, y_b, top, mat="bone"):
    """Dragon rib arc over the y-range: solid bone post at each foot, curved ribs above."""
    for y in (y_a, y_b):
        W.cyl(mat, x, y, 0.0, 0.15, 0.7, seg=8, r_top=0.12)
    n = 10
    pts = []
    for i in range(n + 1):
        a = math.pi * i / n
        yy = (y_a + y_b) / 2 + (y_b - y_a) / 2 * -math.cos(a)
        zz = 0.6 + (top - 0.6) * math.sin(a)
        pts.append((x + 0.12 * math.sin(a), yy, zz))
    for i in range(n):
        r0 = 0.11 * (1 - 0.35 * abs(2 * i / n - 1))
        W.frustum(mat, pts[i], pts[i + 1], r0, r0 * 0.95, seg=6)




# ---------------------------------------------------------------------- giant-scale pieces
def pillar_big(W, x, y, h, r=1.5, mat="pillar", base="stone"):
    """Solid column r, fluted look from a ring of thin half-round ribs above 0.6 m (inside r + 0.12)."""
    W.cyl(mat, x, y, 0.0, r, h, seg=24)
    W.cyl(base, x, y, 0.0, r + 0.0, 1.4, seg=24, solid=False)           # plinth ring flush in the footprint
    W.cyl(base, x, y, 1.4, r + 0.18, 0.4, seg=24, solid=False)
    W.cyl(base, x, y, 1.8, r + 0.08, 0.3, seg=24, solid=False)
    for k in range(10):                                                  # flutes
        a = 2 * math.pi * k / 10
        W.frustum("stone", (x + (r + 0.02) * math.cos(a), y + (r + 0.02) * math.sin(a), 2.4),
                  (x + (r + 0.02) * math.cos(a), y + (r + 0.02) * math.sin(a), h - 4.0), 0.1, 0.1, seg=5)
    W.cyl(base, x, y, h - 4.0, r + 0.14, 0.5, seg=24, solid=False)
    W.cyl(base, x, y, h - 3.4, r + 0.1, 0.3, seg=24, solid=False)
    W.cyl(base, x, y, h - 3.1, r + 0.2, 3.1, seg=24, solid=False, r_top=r + 1.6)    # capital flaring out
    W.cyl("trim", x, y, h - 3.15, r + 0.22, 0.12, seg=24, solid=False)


def merlons(W, x0, y0, x1, y1, z, t, mat="wallg", step=4.0, w=2.2, h=2.4):
    """Battlement teeth along a wall top (non-solid, inside the wall footprint)."""
    L = math.hypot(x1 - x0, y1 - y0)
    n = int(L / step)
    ux, uy = (x1 - x0) / L, (y1 - y0) / L
    yaw = math.atan2(uy, ux)
    for i in range(n):
        s = (i + 0.5) * L / n
        for side in (-1, 1):
            cx = x0 + ux * s - uy * side * (t / 2 - 0.55)
            cy = y0 + uy * s + ux * side * (t / 2 - 0.55)
            if abs(math.sin(yaw)) < 0.01:
                W.box(mat, cx, cy, z, w, 1.1, h, solid=False)
            else:
                W.box(mat, cx, cy, z, 1.1, w, h, solid=False)


def tower(W, x, y, r, h, roof=True, mat="wallg_out"):
    W.cyl(mat, x, y, 0.0, r, h, seg=28)
    W.cyl("stone", x, y, h * 0.5, r + 0.35, 0.8, seg=28, solid=False)
    W.cyl("stone", x, y, h - 1.6, r + 0.9, 1.6, seg=28, solid=False)
    W.cyl("stone", x, y, h, r + 0.9, 0.6, seg=28, solid=False)
    for k in range(3):                                                    # arrow slits / lit windows
        a = 0.9 + k * 2.1
        W.box("glass", x + (r + 0.02) * math.cos(a), y + (r + 0.02) * math.sin(a), h * 0.62, 0.5, 0.5, 3.0, yaw=a, solid=False)
    if roof:
        W.cyl("banner_p", x, y, h + 0.6, r + 1.2, r * 1.6, seg=28, solid=False, r_top=0.15)
        W.frustum("iron", (x, y, h + 0.6 + r * 1.6), (x, y, h + 0.6 + r * 1.6 + 5), 0.12, 0.06, seg=5)


def parterre(W, x0, y0, x1, y1, n=None):
    """Raised planter bed: solid block 0.62 m, soil cap and shrubs above."""
    W.boxp("planter", x0, y0, x1, y1, 0.0, 0.62)
    W.boxp("soil", x0 + 0.15, y0 + 0.15, x1 - 0.15, y1 - 0.15, 0.62, 0.03, solid=False)
    L = max(x1 - x0, y1 - y0)
    n = n or max(2, int(L / 1.7))
    for i in range(n):
        f = (i + 0.5) / n
        cx = x0 + (x1 - x0) * (f if (x1 - x0) >= (y1 - y0) else 0.5)
        cy = y0 + (y1 - y0) * (f if (y1 - y0) > (x1 - x0) else 0.5)
        s = rng.uniform(0.55, 0.85)
        W.ellipsoid("hedge", (cx, cy, 0.62 + s * 0.45), (s * 1.0, min(s, (min(x1 - x0, y1 - y0)) * 0.42), s * 0.7), seg=8, rings=5)
        if i % 3 == 1:
            W.ellipsoid("bloom", (cx + 0.1, cy, 0.62 + s * 1.05), (0.22, 0.22, 0.18), seg=6, rings=4)


def tree(W, x, y, h=7.0):
    W.cyl("wood", x, y, 0.0, 0.42, h * 0.55, seg=10, r_top=0.28)
    for (dx, dy, dz, s) in [(0, 0, h * 0.8, 2.3), (1.2, 0.5, h * 0.62, 1.7), (-1.1, -0.6, h * 0.66, 1.8), (0.2, -1.2, h * 0.72, 1.6)]:
        W.ellipsoid("leaf", (x + dx, y + dy, dz), (s, s, s * 0.75), seg=9, rings=6)


def lamp_post(W, x, y, h=5.0, glow="lamp_g"):
    W.cyl("iron", x, y, 0.0, 0.14, h, seg=8)
    W.cyl("iron", x, y, 0.0, 0.14, 0.01, seg=8, solid=False)
    W.cyl("iron", x, y, 0.5, 0.26, 0.5, seg=8, solid=False, r_top=0.14)
    W.box(glow, x, y, h, 0.4, 0.4, 0.6, solid=False)
    W.frustum("iron", (x, y, h + 0.6), (x, y, h + 0.95), 0.3, 0.0, seg=4)


def bench(W, x, y, yaw=0.0, l=2.2):
    W.box("stone", x, y, 0.0, l, 0.6, 0.62, yaw=yaw)
    W.box("wood", x, y, 0.62, l + 0.1, 0.62, 0.08, yaw=yaw, solid=False)
    c, s = math.cos(yaw), math.sin(yaw)
    W.box("wood", x - 0.22 * s, y + 0.22 * c, 0.7, l, 0.08, 0.7, yaw=yaw, solid=False)


def brazier_big(W, x, y, h=2.6, flame="flame"):
    """Stone stand (solid, constant footprint r 0.55) with a wide bowl and a tall flame above."""
    W.cyl("stone", x, y, 0.0, 0.55, h, seg=14)
    W.cyl("trim", x, y, 0.0, 0.58, 0.14, seg=14, solid=False)
    W.cyl("iron", x, y, h, 0.55, 0.45, seg=14, solid=False, r_top=1.0)
    W.cyl("iron", x, y, h + 0.45, 1.02, 0.1, seg=14, solid=False)
    W.frustum(flame, (x, y, h + 0.5), (x, y, h + 2.4), 0.8, 0.0, seg=10)
    W.frustum(flame, (x + 0.2, y, h + 0.5), (x + 0.3, y, h + 1.8), 0.5, 0.0, seg=8)


def fountain(W, x, y, r=4.5):
    W.cyl("stone", x, y, 0.0, r, 0.9, seg=36)
    W.cyl("trim", x, y, 0.88, r + 0.12, 0.1, seg=36, solid=False)
    W.cyl("water_p", x, y, 0.88, r - 0.5, 0.05, seg=36, solid=False)
    W.cyl("stone", x, y, 0.9, 0.8, 3.6, seg=14, solid=False, r_top=0.45)
    W.cyl("stone", x, y, 4.4, 2.0, 0.35, seg=24, solid=False, r_top=2.4)
    W.cyl("water_p", x, y, 4.75, 1.9, 0.05, seg=24, solid=False)
    W.cyl("stone", x, y, 4.8, 0.3, 1.8, seg=10, solid=False, r_top=0.15)
    W.ellipsoid("rune", (x, y, 6.9), (0.35, 0.35, 0.5), seg=10, rings=6)
    for k in range(8):
        a = 2 * math.pi * k / 8
        W.ellipsoid("marble", (x + (r - 0.25) * math.cos(a), y + (r - 0.25) * math.sin(a), 1.35), (0.28, 0.28, 0.4), seg=8, rings=5)


def knight_statue(W, x, y, yaw=0.0, s=1.0):
    """Plinth (solid 2.2 x 2.2 x 1.1) with a marble knight above."""
    W.box("stone", x, y, 0.0, 2.2, 2.2, 1.1, yaw=yaw)
    W.box("trim", x, y, 1.1, 2.4, 2.4, 0.1, yaw=yaw, solid=False)
    z = 1.2
    W.frustum("marble", (x, y, z), (x, y, z + 1.0 * s), 0.38 * s, 0.34 * s, seg=8)
    W.frustum("marble", (x, y, z + 1.0 * s), (x, y, z + 2.0 * s), 0.34 * s, 0.4 * s, seg=8)
    W.ellipsoid("marble", (x, y, z + 2.35 * s), (0.22 * s, 0.22 * s, 0.26 * s), seg=8, rings=6)
    c, sn = math.cos(yaw), math.sin(yaw)
    W.frustum("steel", (x + 0.5 * s * c, y + 0.5 * s * sn, z + 1.0 * s), (x + 0.5 * s * c, y + 0.5 * s * sn, z + 3.6 * s), 0.06 * s, 0.0, seg=5)
    W.ellipsoid("marble", (x - 0.05 * sn - 0.4 * s * c, y + 0.05 * c - 0.4 * s * sn, z + 1.6 * s), (0.08 * s, 0.5 * s, 0.6 * s), seg=6, rings=5, yaw=yaw)


def chandelier(W, x, y, z, r=3.0, n=12):
    W.frustum("iron", (x, y, z + 8.0), (x, y, z + 0.4), 0.06, 0.06, seg=5)
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        W.frustum("trim", (x + r * math.cos(a0), y + r * math.sin(a0), z), (x + r * math.cos(a1), y + r * math.sin(a1), z), 0.1, 0.1, seg=5)
        if k % 2 == 0:
            p = (x + r * math.cos(a0), y + r * math.sin(a0))
            W.frustum("wax" if "wax" in W.mats else "bone", (p[0], p[1], z), (p[0], p[1], z + 0.5), 0.07, 0.07, seg=5)
            W.frustum("flame", (p[0], p[1], z + 0.5), (p[0], p[1], z + 0.85), 0.09, 0.0, seg=5)
    for k in range(4):
        a = math.pi / 2 * k
        W.frustum("iron", (x + r * math.cos(a), y + r * math.sin(a), z), (x, y, z + 1.4), 0.04, 0.04, seg=4)


def stalagmite(W, x, y, r, h):
    W.cyl("rock", x, y, 0.0, r, h, seg=10, r_top=r * 0.18)


def long_table(W, x, y, l, w=0.9, yaw=0.0, h=0.78):
    W.box("wood", x, y, 0.0, l, w, h, yaw=yaw)
    W.box("wood", x, y, h, l + 0.1, w + 0.1, 0.06, yaw=yaw, solid=False)


def rack(W, x, y, yaw=0.0, l=2.4, items=4):
    """Weapon rack: solid base 0.62 tall, spears/swords above."""
    W.box("wood", x, y, 0.0, l, 0.5, 0.62, yaw=yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    for i in range(items * 2):
        u = -l / 2 + 0.2 + i * (l - 0.4) / (items * 2 - 1)
        px, py = x + u * c, y + u * s
        W.frustum("steel" if i % 2 else "wood", (px, py, 0.7), (px, py, 2.1 + 0.15 * (i % 3)), 0.025, 0.012, seg=4)
        if i % 2:
            W.frustum("steel", (px, py, 2.1 + 0.15 * (i % 3)), (px, py, 2.4 + 0.15 * (i % 3)), 0.05, 0.0, seg=4)
    W.box("wood", x, y, 1.5, l, 0.1, 0.07, yaw=yaw, solid=False)


def anvil(W, x, y, yaw=0.0):
    W.box("wood", x, y, 0.0, 0.6, 0.6, 0.62, yaw=yaw)
    W.box("iron", x, y, 0.62, 0.9, 0.3, 0.2, yaw=yaw, solid=False)
    W.box("iron", x, y, 0.8, 0.5, 0.2, 0.2, yaw=yaw, solid=False)


def furnace(W, x, y, yaw=0.0):
    W.box("stone", x, y, 0.0, 2.0, 1.6, 1.6, yaw=yaw)
    W.box("stone", x, y, 1.6, 1.4, 1.1, 1.6, yaw=yaw, solid=False)
    W.cyl("stone", x, y, 3.2, 0.45, 1.6, seg=8, solid=False)
    c, s = math.cos(yaw), math.sin(yaw)
    W.box("flame", x + 0.82 * c, y + 0.82 * s, 0.65, 0.05, 0.9, 0.5, yaw=yaw, solid=False)


def pew(W, x, y, l=3.0, yaw=0.0):
    W.box("wood", x, y, 0.0, l, 0.5, 0.62, yaw=yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    W.box("wood", x - 0.18 * s, y + 0.18 * c, 0.62, l, 0.08, 0.6, yaw=yaw, solid=False)


def bunk(W, x, y, yaw=0.0):
    W.box("wood", x, y, 0.0, 2.0, 0.9, 0.62, yaw=yaw)
    W.box("cloth", x, y, 0.62, 1.9, 0.82, 0.12, yaw=yaw, solid=False)
    c, s = math.cos(yaw), math.sin(yaw)
    W.box("banner_c", x + 0.2 * c, y + 0.2 * s, 0.74, 1.2, 0.84, 0.05, yaw=yaw, solid=False)
    W.box("wood", x, y, 0.74, 2.0, 0.9, 0.0 + 0.04, yaw=yaw, solid=False)
