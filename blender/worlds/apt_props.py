"""Apartment decoration helpers (visual only; every piece sits above 0.60 m, on solid furniture or inside a footprint)."""
import math


def stool(W, x, y, h=0.65):
    W.cyl("oak", x, y, 0.0, 0.19, h, seg=16)
    W.cyl("cushion", x, y, h, 0.19, 0.06, seg=16, solid=False)


def table(W, x0, y0, x1, y1, top=0.72):
    W.boxp("oak", x0, y0, x1, y1, top, 0.05, solid=False)
    for (lx, ly) in ((x0 + 0.06, y0 + 0.06), (x1 - 0.06, y0 + 0.06), (x0 + 0.06, y1 - 0.06), (x1 - 0.06, y1 - 0.06)):
        W.box("oak", lx, ly, 0.0, 0.07, 0.07, top)


# ---------------------------------------------------------------- decoration helpers
# Everything here is visual only and sits above z = 0.60 m (or on top of solid furniture).
def wbox(W, mat, x, y, z0, w, h, d, nx, ny, off=0.0, tag=None):
    """Thin slab on a wall face. (x, y) = point on the wall (along-wall coordinate is x when ny != 0,
    else y); (nx, ny) = wall normal into the room; w along the wall, d out from the wall."""
    sx = w if ny else d
    sy = w if nx else d
    W.box(mat, x + nx * (off + d / 2), y + ny * (off + d / 2), z0, sx, sy, h, solid=False, tag=tag)


ART = [("art_a", "art_b", "art_c"), ("art_c", "art_d", "art_a"), ("art_b", "art_a", "art_d"), ("art_d", "art_c", "art_b")]


def picture(W, x, y, z0, w, h, nx, ny, kind=0, tag=None):
    bg, band, dot = ART[kind % len(ART)]
    wbox(W, "frame_dk", x, y, z0, w, h, 0.03, nx, ny, tag=tag)
    wbox(W, "art_d", x, y, z0 + 0.03, w - 0.06, h - 0.06, 0.032, nx, ny, tag=tag)
    wbox(W, bg, x, y, z0 + 0.06, w - 0.12, h - 0.12, 0.034, nx, ny, tag=tag)
    ax, ay = (x, y)
    # abstract accents: a band and a square, offset along the wall
    off = (w - 0.12) * 0.18
    wbox(W, band, x - off * ny if ny else x, y if ny else y + off * nx, z0 + h * 0.42, (w - 0.12) * 0.7, (h - 0.12) * 0.22, 0.036, nx, ny, tag=tag)
    wbox(W, dot, x + off * 1.3 * ny if ny else x, y if ny else y - off * 1.3 * nx, z0 + h * 0.2, (h - 0.12) * 0.34, (h - 0.12) * 0.34, 0.038, nx, ny, tag=tag)


def curtain(W, x, y, nx, ny, z0, z1, w, tag=None, mat="curtain"):
    n = max(3, int(w / 0.07))
    for i in range(n):
        c = (x if ny else y) - w / 2 + (i + 0.5) * w / n
        d = 0.045 + 0.03 * (i % 2)
        wbox(W, mat if i % 2 else mat + "2", c if ny else x, y if ny else c, z0, w / n, z1 - z0, d, nx, ny, off=0.035, tag=tag)


def curtain_rod(W, x, y, nx, ny, z, length, tag=None):
    wbox(W, "black", x, y, z, length, 0.025, 0.025, nx, ny, off=0.05, tag=tag)


def table_lamp(W, x, y, z):
    W.cyl("steel", x, y, z, 0.05, 0.02, seg=12, solid=False)
    W.cyl("steel", x, y, z + 0.02, 0.012, 0.2, seg=8, solid=False)
    W.frustum("shade", (x, y, z + 0.17), (x, y, z + 0.34), 0.115, 0.075, seg=14)


def plant(W, x, y, z, s=1.0, n=10, pot=True, seed=0):
    if pot:
        W.frustum("pot", (x, y, z), (x, y, z + 0.16 * s), 0.075 * s, 0.10 * s, seg=12)
        W.cyl("black", x, y, z + 0.155 * s, 0.09 * s, 0.01 * s, seg=12, solid=False)
    for k in range(n):
        a = k * 2.39996 + seed
        up = 0.22 * s + 0.07 * s * (k % 3)
        r = 0.05 * s + 0.03 * s * (k % 2)
        L = (0.13 + 0.03 * (k % 3)) * s
        ca, sa = math.cos(a), math.sin(a)
        c = (x + ca * (r + L * 0.55), y + sa * (r + L * 0.55), z + up + L * 0.35)
        W.ellipsoid("leaf" if k % 2 else "leaf2", c, (L, 0.045 * s, 0.008 * s), seg=8, rings=4, yaw=a, pitch=-0.55)


def books(W, x, y, z, n=3, yaw=0.0):
    for i in range(n):
        W.box(["book_a", "book_b", "book_c"][i % 3], x, y, z + i * 0.035, 0.24 - 0.02 * i, 0.17, 0.032, yaw=yaw + 0.15 * i, solid=False)

