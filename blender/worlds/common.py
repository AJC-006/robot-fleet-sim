"""Helpers shared by the world scripts: absolute-coordinate walls, zone floors, small props."""
import math


def hwall(W, y, xa, xb, h, mat, doors=(), t=0.4, tag="inner", arch_mat=None):
    """Wall along x at y from xa to xb. doors: (x_centre, width[, height[, arch]])."""
    W.wall((xa, y), (xb, y), t, h, mat, [_op(d, xa) for d in doors], tag=tag, arch_mat=arch_mat)


def vwall(W, x, ya, yb, h, mat, doors=(), t=0.4, tag="inner", arch_mat=None):
    W.wall((x, ya), (x, yb), t, h, mat, [_op(d, ya) for d in doors], tag=tag, arch_mat=arch_mat)


def _op(d, a):
    w = d[1]
    oh = d[2] if len(d) > 2 and d[2] else (2.1 if w < 1.6 else 2.3)
    arch = d[3] if len(d) > 3 else False
    return dict(c=abs(d[0] - a), w=w, h=oh, arch=arch)


def zfloor(W, mat, x0, y0, x1, y1, h=0.01, z0=0.0, tag=None):
    """A 1 cm floor patch laid on the base floor (different surface for a zone)."""
    W.boxp(mat, x0, y0, x1, y1, z0, h, solid=False, tag=tag)


def barrel(W, x, y, wood="wood", iron="iron", h=0.9, r=0.3):
    W.cyl(wood, x, y, 0.0, r, h, seg=14)
    for z in (0.12, h - 0.18):
        W.cyl(iron, x, y, z, r + 0.012, 0.04, seg=14, solid=False)


def crate(W, x, y, s=0.7, yaw=0.0, wood="wood", iron="iron", h=None):
    h = h or max(s, 0.65)
    W.box(wood, x, y, 0.0, s, s, h, yaw=yaw)
    W.box(iron, x, y, 0.0, s + 0.01, s + 0.01, 0.04, yaw=yaw, solid=False)
    W.box(iron, x, y, h - 0.04, s + 0.01, s + 0.01, 0.04, yaw=yaw, solid=False)


def torch(W, x, y, z, nx, ny, flame="flame", iron="iron"):
    ox, oy = x + nx * 0.12, y + ny * 0.12
    W.box(iron, x + nx * 0.05, y + ny * 0.05, z - 0.25, 0.08 if nx == 0 else 0.1, 0.08 if ny == 0 else 0.1, 0.3, solid=False)
    W.frustum(iron, (ox, oy, z - 0.05), (ox, oy, z + 0.12), 0.04, 0.11, seg=8)
    W.frustum(flame, (ox, oy, z + 0.1), (ox, oy, z + 0.5), 0.1, 0.0, seg=8)


def brazier(W, x, y, flame="flame", iron="iron", h=0.9):
    """Solid footprint r=0.3 up to h; bowl + flame above."""
    W.cyl(iron, x, y, 0.0, 0.3, h - 0.2, seg=12, r_top=0.3)
    W.cyl(iron, x, y, h - 0.2, 0.3, 0.2, seg=12, solid=False, r_top=0.42)
    W.frustum(flame, (x, y, h), (x, y, h + 0.6), 0.34, 0.0, seg=8)


def banner(W, x, y, z0, h, nx, ny, mat, w=0.9, trim="trim"):
    """Hanging banner flush on a wall face; (nx, ny) = wall normal into the room."""
    sx = w if ny != 0 else 0.04
    sy = w if nx != 0 else 0.04
    W.box(mat, x + nx * 0.03, y + ny * 0.03, z0, sx, sy, h, solid=False)
    W.box(trim, x + nx * 0.05, y + ny * 0.05, z0 + h - 0.1, sx + (0.06 if ny else 0), sy + (0.06 if nx else 0), 0.07, solid=False)
    W.box(trim, x + nx * 0.045, y + ny * 0.045, z0 + h * 0.55, 0.04 if ny == 0 else w * 0.5, 0.04 if nx == 0 else w * 0.5, h * 0.3, solid=False)
    W.box(trim, x + nx * 0.07, y + ny * 0.07, z0 + h, sx + (0.2 if ny else 0), sy + (0.2 if nx else 0), 0.05, solid=False)


def chain(W, x, y, z_top, z_bot, axis="x", mat="iron"):
    n = int((z_top - z_bot) / 0.1)
    for i in range(n):
        z = z_top - (i + 1) * 0.1
        if (i % 2 == 0) == (axis == "x"):
            W.box(mat, x, y, z, 0.05, 0.015, 0.11, solid=False)
        else:
            W.box(mat, x, y, z, 0.015, 0.05, 0.11, solid=False)


def wheel(W, c, r, axis, mat, rim, spokes=6, thick=0.06):
    """Spoked wheel; centre c (3d), axis 'x' or 'y' (the wheel axle direction)."""
    cx, cy, cz = c
    for s in range(spokes):
        a = math.pi * s / spokes
        if axis == "y":
            d = (math.cos(a) * r, 0, math.sin(a) * r)
        else:
            d = (0, math.cos(a) * r, math.sin(a) * r)
        W.frustum(mat, (cx - d[0], cy - d[1], cz - d[2]), (cx + d[0], cy + d[1], cz + d[2]), 0.02, 0.02, seg=4)
    ax = (0, thick / 2, 0) if axis == "y" else (thick / 2, 0, 0)
    W.frustum(rim, (cx - ax[0], cy - ax[1], cz - ax[2]), (cx + ax[0], cy + ax[1], cz + ax[2]), r, r, seg=18)
    W.frustum(mat, (cx - ax[0] * 1.6, cy - ax[1] * 1.6, cz - ax[2] * 1.6), (cx + ax[0] * 1.6, cy + ax[1] * 1.6, cz + ax[2] * 1.6), 0.07, 0.07, seg=8)


def tri_pairs(n):
    return [(i, (i + 1) % n) for i in range(n)]
