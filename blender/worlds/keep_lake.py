"""Keep: the lake (bed heightfield, quay, drowned dragon temple, wreck, weeds) and the sky."""
import math
import random

WX0, WX1, WY0, WY1 = 21.0, 163.0, -50.0, 50.0       # water extent
WATER_Z = -0.8
TEMPLE = (64.0, -26.0, 124.0, 26.0)                  # flattened platform x0 y0 x1 y1
TZ = -19.5
rng = random.Random(11)


def _ss(a, b, x):
    t = max(0.0, min(1.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


def bed(x, y):
    """Bed height: rocky shelf at -8 m by the quay falling to -20 m, flat temple platform, slipway ramp."""
    d = min(x - WX0, WX1 - x, y - WY0, WY1 - y)
    base = -(8.0 + 12.0 * _ss(0.0, 38.0, d))
    n = 0.9 * math.sin(x * 0.11 + 0.7) * math.cos(y * 0.15) + 0.5 * math.sin(x * 0.29 + y * 0.23) + 0.3 * math.sin(x * 0.7) * math.sin(y * 0.6)
    n *= _ss(0.0, 6.0, d)
    h = base + n
    tx0, ty0, tx1, ty1 = TEMPLE
    dx = max(tx0 - x, 0, x - tx1)
    dy = max(ty0 - y, 0, y - ty1)
    w = _ss(0.0, 14.0, math.hypot(dx, dy))
    h = TZ * (1 - w) + h * w
    if x < 62:                                       # slipway ramp
        ramp = -1.8 - (x - WX0) * 0.42
        wr = 1.0 - _ss(6.0, 11.0, abs(y))
        h = max(h, ramp * wr + h * (1 - wr)) if wr > 0 else h
    return max(-21.0, h)


def bed_mesh(W, step=2.0):
    nx = int(round((WX1 - WX0) / step))
    ny = int(round((WY1 - WY0) / step))
    sx, sy = (WX1 - WX0) / nx, (WY1 - WY0) / ny
    vs, uv = [], []
    for j in range(ny + 1):
        for i in range(nx + 1):
            x, y = WX0 + i * sx, WY0 + j * sy
            vs.append((x, y, bed(x, y)))
            uv.append((x / 4.0, y / 4.0))
    fs = []
    for j in range(ny):
        for i in range(nx):
            a = j * (nx + 1) + i
            fs.append((a, a + 1, a + nx + 2))
            fs.append((a, a + nx + 2, a + nx + 1))
    with W.chunk("lake"):
        W.add_mesh("mud", vs, fs, smooth=True, uv=uv)
        # darker rocky skirt where the bed is steep (second shell, tiny offset) gives a cheap two-tone look
    return nx, ny


def bed_collision(W, cell=10.0):
    nx = int(math.ceil((WX1 - WX0) / cell))
    ny = int(math.ceil((WY1 - WY0) / cell))
    sx, sy = (WX1 - WX0) / nx, (WY1 - WY0) / ny
    for j in range(ny):
        for i in range(nx):
            x0, y0 = WX0 + i * sx, WY0 + j * sy
            hs = [bed(x0 + a * sx / 4, y0 + b * sy / 4) for a in range(5) for b in range(5)]
            top = min(hs) - 0.15
            W.col_box(x0 + sx / 2, y0 + sy / 2, top - 1.2, sx + 0.02, sy + 0.02, 1.2)


def quay(W, gate_y=0.0):
    """Stone quay ring around the water, 2 m thick, top at +0.7 m (the parapet), wall down to -22 m."""
    ox0, oy0, ox1, oy1 = WX0 - 2, WY0 - 2, WX1 + 2, WY1 + 2
    t = 2.0
    segs = [((ox0 + ox1) / 2, oy0 + t / 2, ox1 - ox0, t), ((ox0 + ox1) / 2, oy1 - t / 2, ox1 - ox0, t),
            (ox0 + t / 2, (oy0 + oy1) / 2, t, oy1 - oy0 - 2 * t), (ox1 - t / 2, (oy0 + oy1) / 2, t, oy1 - oy0 - 2 * t)]
    for (cx, cy, sx, sy) in segs:
        W.box("quay", cx, cy, -22.0, sx, sy, 22.7, tile=3.0)
        W.box("trim", cx, cy, 0.7, sx, sy, 0.1, solid=False)
    # bollards + lamp posts along the top, and the water gate flanking the slipway
    n = 0
    for x in [ox0 + 12 + 14 * k for k in range(int((ox1 - ox0 - 24) / 14) + 1)]:
        for cy in (oy0 + t / 2, oy1 - t / 2):
            W.cyl("iron", x, cy, 0.7, 0.22, 0.0 + 0.001, seg=6, solid=False)
            W.cyl("stone", x, cy, 0.8, 0.24, 0.5, seg=8, solid=True, r_top=0.2)
            n += 1
    for y in [oy0 + 10 + 14 * k for k in range(int((oy1 - oy0 - 20) / 14) + 1)]:
        for cx in (ox0 + t / 2, ox1 - t / 2):
            W.cyl("stone", cx, y, 0.8, 0.24, 0.5, seg=8, solid=True, r_top=0.2)
    # water gate posts at the head of the slipway (west quay, y = gate_y): two solid piers + iron arch
    for s in (-1, 1):
        W.box("stone", ox0 + t / 2, gate_y + s * 5.2, 0.8, 1.6, 1.4, 4.5, solid=True)
        W.box("trim", ox0 + t / 2, gate_y + s * 5.2, 5.3, 1.9, 1.7, 0.2, solid=False)
    for k in range(9):
        a = math.pi * k / 8
        W.frustum("iron", (ox0 + t / 2, gate_y + 5.2 * math.cos(a), 5.4 + 0.0), (ox0 + t / 2, gate_y + 5.2 * math.cos(a + math.pi / 8), 5.4 + 2.4 * math.sin(a + math.pi / 8)), 0.07, 0.07, seg=4) if False else None
    for k in range(8):
        a0, a1 = math.pi * k / 8, math.pi * (k + 1) / 8
        W.frustum("iron", (ox0 + t / 2, gate_y - 5.2 * math.cos(a0), 5.5 + 2.6 * math.sin(a0)), (ox0 + t / 2, gate_y - 5.2 * math.cos(a1), 5.5 + 2.6 * math.sin(a1)), 0.12, 0.12, seg=5)
    W.ellipsoid("rune", (ox0 + t / 2, gate_y, 8.4), (0.4, 0.4, 0.6), seg=8, rings=5)


def water_surface(W):
    """The water surface is an SDF-native translucent box (see w_keep.WATER_SDF): Gazebo ignores the glTF alpha
    factor, so a mesh slab rendered fully invisible and the lake looked drained from below."""
    return


def rock(W, x, y, r, sink=0.35, col=True):
    z = bed(x, y)
    W.ellipsoid("rock", (x, y, z + r * (0.55 - sink)), (r, r * rng.uniform(0.8, 1.2), r * 0.75), seg=9, rings=6, yaw=rng.uniform(0, 3))
    if col and r >= 1.4:   # small rocks are visual only: every extra primitive costs the physics step
        W.col_cyl(x, y, z - 0.5, r * 0.85, r * 1.2)


def kelp(W, x, y, h):
    z = bed(x, y)
    n = max(3, int(h / 1.3))
    px, py, pz = x, y, z
    a = rng.uniform(0, 6.28)
    for i in range(n):
        nx_, ny_, nz = px + 0.25 * math.cos(a + i), py + 0.25 * math.sin(a + i), pz + h / n
        W.frustum("weed", (px, py, pz), (nx_, ny_, nz), 0.14 * (1 - i / (n + 1)) + 0.03, 0.14 * (1 - (i + 1) / (n + 1)) + 0.03, seg=4)
        px, py, pz = nx_, ny_, nz


def column(W, x, y, h, broken=False):
    z = bed(x, y)
    W.cyl("temple", x, y, z - 0.3, 1.1, h + 0.3, seg=14, solid=False, r_top=1.0 if not broken else 0.9)
    W.col_cyl(x, y, z - 0.3, 1.1, h + 0.3)
    W.cyl("temple", x, y, z - 0.3, 1.5, 1.0, seg=14, solid=False)
    if not broken:
        W.cyl("temple", x, y, z + h - 1.0, 1.4, 1.0, seg=14, solid=False, r_top=1.6)
        W.box("temple", x, y, z + h, 3.4, 3.4, 0.5, solid=False)
    else:
        for k in range(5):
            a = rng.uniform(0, 6.28)
            W.ellipsoid("temple", (x + 1.0 * math.cos(a), y + 1.0 * math.sin(a), z + h - 0.2 + 0.3 * k * 0), (0.7, 0.6, 0.35), seg=6, rings=4, yaw=a)


def fallen_drum(W, x, y, yaw, l=5.0, r=1.1):
    z = bed(x, y)
    c, s = math.cos(yaw), math.sin(yaw)
    W.frustum("temple", (x - l / 2 * c, y - l / 2 * s, z + r * 0.8), (x + l / 2 * c, y + l / 2 * s, z + r * 0.8), r, r, seg=12)
    W.box("temple", x, y, z - 0.5, l * 0.8 + 0.4, r * 1.7 + 0.2, r * 1.6 + 0.5, yaw=yaw, solid=False)
    W.col_box(x, y, z - 0.5, l, r * 2.0, r * 1.8 + 0.5, yaw=yaw)


def dragon_head(W, cx, cy, yaw):
    """Fallen giant dragon skull ~16 m long lying on its side, jaw agape, horns and teeth, sunk into the bed."""
    c, s = math.cos(yaw), math.sin(yaw)
    z0 = bed(cx, cy)

    def P(u, v, w):
        return (cx + u * c - v * s, cy + u * s + v * c, z0 + w)
    E = W.ellipsoid
    E("dragon_bone", P(0.0, 0.0, 2.6), (7.5, 3.1, 2.7), seg=18, rings=10, yaw=yaw)             # skull
    E("dragon_bone", P(6.5, 0.0, 2.0), (4.4, 2.0, 1.6), seg=14, rings=8, yaw=yaw)              # snout
    E("dragon_bone", P(4.5, 0.4, -0.2), (6.2, 2.4, 1.0), seg=14, rings=8, yaw=yaw)             # lower jaw, hinged open
    E("dragon_bone", P(-5.5, 0.0, 3.2), (4.5, 2.8, 2.4), seg=12, rings=7, yaw=yaw)             # neck stump / back of skull
    for sgn in (-1, 1):
        a = (P(-3.0, sgn * 1.9, 5.0), P(-9.5, sgn * 3.6, 9.5))
        W.frustum("dragon_bone", a[0], a[1], 0.8, 0.12, seg=8)                                  # horns
        E("black_socket", P(2.0, sgn * 2.4, 3.6), (1.5, 0.8, 1.2), seg=10, rings=6, yaw=yaw)
        E("rune", P(2.0, sgn * 2.6, 3.6), (0.45, 0.3, 0.5), seg=8, rings=5, yaw=yaw)
    for i in range(9):
        u = 3.0 + i * 0.95
        for sgn in (-1, 1):
            p0 = P(u, sgn * 1.6, 1.3)
            p1 = P(u + 0.1, sgn * 1.6, -0.4 + 0.2 * (i % 2))
            W.frustum("bone", p0, p1, 0.2, 0.0, seg=5)
            q0 = P(u + 0.2, sgn * 1.8, 0.45)
            q1 = P(u + 0.25, sgn * 1.8, 1.9 + 0.25 * (i % 2))
            W.frustum("bone", q0, q1, 0.18, 0.0, seg=5)
    # collision: skull + jaw + snout as yawed boxes
    W.col_box(cx + 0.0 * c, cy + 0.0 * s, z0 - 0.3, 15.0, 6.0, 6.2, yaw=yaw)
    W.col_box(cx + 7.0 * c, cy + 7.0 * s, z0 - 0.3, 6.0, 4.0, 4.0, yaw=yaw)


def wreck(W, cx, cy, yaw):
    z = bed(cx, cy)
    c, s = math.cos(yaw), math.sin(yaw)
    W.box("wreck", cx, cy, z - 0.2, 8.0, 2.8, 1.0, yaw=yaw, solid=False)
    for k in range(-3, 4):                                   # ribs
        for sgn in (-1, 1):
            W.box("wreck", cx + k * 1.0 * c - sgn * 1.25 * s, cy + k * 1.0 * s + sgn * 1.25 * c, z + 0.7, 0.12, 0.12, 1.6 - 0.1 * abs(k), yaw=yaw, solid=False)
    W.box("wreck", cx - 1.0 * c, cy - 1.0 * s, z + 0.7, 0.4, 0.4, 5.0, yaw=yaw, solid=False)    # broken mast
    W.box("cloth", cx - 1.0 * c, cy - 1.0 * s, z + 3.0, 0.05, 2.0, 2.5, yaw=yaw, solid=False)
    W.col_box(cx, cy, z - 0.3, 8.0, 2.8, 2.2, yaw=yaw)


def temple_ruins(W):
    tx0, ty0, tx1, ty1 = TEMPLE
    cxm = (tx0 + tx1) / 2
    with W.chunk("lake"):
        W.boxp("temple", tx0 + 2, ty0 + 2, tx1 - 2, ty1 - 2, TZ - 0.1, 0.5, solid=False)
        W.boxp("temple", tx0 + 6, ty0 + 6, tx1 - 6, ty1 - 6, TZ + 0.4, 0.35, solid=False)
        hs = [14.0, 14.0, 9.0, 14.0, 5.0, 14.0]
        for k in range(6):
            for sgn in (-1, 1):
                x = 76.0 + 9.0 * k
                h = hs[k] if sgn > 0 else hs[(k + 2) % 6]
                column(W, x, sgn * 9.0, h, broken=h < 12)
        for k in (0, 1, 3, 5):                                  # standing arches across the nave (lintel on two columns)
            x = 76.0 + 9.0 * k
            if hs[k] >= 14.0 and hs[(k + 2) % 6] >= 14.0:
                z = TZ + 14.5
                W.box("temple", x, 0.0, z, 3.0, 20.5, 1.6, solid=False)
                W.col_box(x, 0.0, z, 3.0, 20.5, 1.6)
                W.box("temple", x, 0.0, z + 1.6, 2.2, 8.0, 1.8, solid=False)
                W.col_box(x, 0.0, z + 1.6, 2.2, 8.0, 1.8)
        fallen_drum(W, 70.0, 14.0, 0.6)
        fallen_drum(W, 112.0, -17.0, 2.2, l=6.0)
        fallen_drum(W, 118.0, 12.0, 1.3, l=4.0)
        # altar / apse at the east end
        W.cyl("temple", 119.0, 0.0, TZ + 0.2, 4.0, 1.3, seg=18, solid=True, r_top=3.4)
        W.ellipsoid("rune", (119.0, 0.0, TZ + 2.0), (0.9, 0.9, 1.1), seg=10, rings=6)
        dragon_head(W, 99.0, -18.0, 0.35)
        wreck(W, 54.0, 24.0, 0.5)
        # rocks and weeds
        for _ in range(60):
            x, y = rng.uniform(WX0 + 3, WX1 - 3), rng.uniform(WY0 + 3, WY1 - 3)
            if tx0 - 2 < x < tx1 + 2 and ty0 - 2 < y < ty1 + 2:
                continue
            if x < 48 and abs(y) < 9:
                continue
            rock(W, x, y, rng.uniform(0.8, 3.2))
        for _ in range(14):
            rock(W, rng.uniform(tx0 + 3, tx1 - 3), rng.choice([-1, 1]) * rng.uniform(15, 24), rng.uniform(0.7, 1.6))
        for _ in range(300):
            x, y = rng.uniform(WX0 + 2, WX1 - 2), rng.uniform(WY0 + 2, WY1 - 2)
            if x < 48 and abs(y) < 9:
                continue
            kelp(W, x, y, rng.uniform(2.0, 7.0))
        # rubble among the columns
        for _ in range(40):
            x, y = rng.uniform(tx0 + 4, tx1 - 4), rng.uniform(ty0 + 4, ty1 - 4)
            if abs(y) < 3 or abs(abs(y) - 9) < 2.2:
                pass
            W.ellipsoid("temple", (x, y, bed(x, y) + 0.1), (rng.uniform(0.3, 0.9), rng.uniform(0.3, 0.8), 0.35), seg=6, rings=4, yaw=rng.uniform(0, 3))


def sky(W, R=330.0, cx=0.0, cy=-10.0):
    """Banded emissive dome (violet night, magenta glow at the horizon), stars, moon, dark ridge."""
    bands = [(0, 9, "sky_h"), (9, 22, "sky_l"), (22, 45, "sky_m"), (45, 90, "sky_t")]
    seg = 40
    with W.chunk("sky"):
        for (a0, a1, mat) in bands:
            vs, fs = [], []
            rings = max(2, int((a1 - a0) / 8))
            for r in range(rings + 1):
                el = math.radians(a0 + (a1 - a0) * r / rings)
                for k in range(seg):
                    az = 2 * math.pi * k / seg
                    vs.append((cx + R * math.cos(el) * math.cos(az), cy + R * math.cos(el) * math.sin(az), R * math.sin(el) - 8.0))
            for r in range(rings):
                for k in range(seg):
                    a = r * seg + k
                    b = r * seg + (k + 1) % seg
                    fs.append((a, b, b + seg, a + seg))
            # inward-facing: reverse winding
            fs = [tuple(reversed(f)) for f in fs]
            W.add_mesh(mat, vs, fs, smooth=False)
        sr = random.Random(5)
        for i in range(260):
            az = sr.uniform(0, 2 * math.pi)
            el = math.radians(sr.uniform(14, 85))
            d = R * 0.97
            p = (cx + d * math.cos(el) * math.cos(az), cy + d * math.cos(el) * math.sin(az), d * math.sin(el))
            s = sr.uniform(0.5, 1.4) * (1.0 if el < 1.0 else 1.6)
            W.ellipsoid("star", p, (s, s, s), seg=4, rings=3)
        mp = (cx - 120.0, cy - 260.0, 150.0)
        W.ellipsoid("moon", mp, (22.0, 22.0, 22.0), seg=18, rings=10)
        W.ellipsoid("moon_halo", (mp[0], mp[1] + 2.0, mp[2]), (30.0, 28.0, 30.0), seg=14, rings=8)
        # ridge: ring of dark jagged cones beyond the estate
        for i in range(70):
            az = 2 * math.pi * i / 70 + sr.uniform(-0.03, 0.03)
            d = sr.uniform(255, 285)
            h = sr.uniform(35, 90)
            r = sr.uniform(24, 46)
            x, y = cx + d * math.cos(az), cy + d * math.sin(az)
            W.cyl("ridge", x, y, -8.0, r, h + 8, seg=8, solid=False, r_top=r * 0.1)
