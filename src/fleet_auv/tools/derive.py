"""S1 derivations for knucker: envelope, decor volumes, mass budget + trim, inertia, strip-theory added mass
(2-D BEM per section), crossflow/axial damping, fin lift, T200 table fits. Writes derived.json next to this file.
usage: python3 -I derive.py <hull.obj> <knucker.yaml> <t200.json>"""
import sys, os, json, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import meshlib
import faulthandler; faulthandler.dump_traceback_later(120, repeat=False, file=sys.stderr)
OBJ, YML, T200 = sys.argv[1:4]
OUT = {}
rng = np.random.default_rng(1)

# ---------------- fluid (fresh lake water) ----------------
T_C = 15.0
a1, a2, a3, a4, a5 = -3.983035, 301.797, 522528.9, 69.34881, 999.974950   # Tanaka et al. 2001, Metrologia 38:301
RHO = a5 * (1 - (T_C + a1) ** 2 * (T_C + a2) / (a3 * (T_C + a4)))
MU = 1.1375e-3          # Pa s at 15 C (IAPWS 2008 / CRC Handbook table)
NU = MU / RHO
G = 9.80665
OUT['fluid'] = dict(T_C=T_C, rho=RHO, mu=MU, nu=NU, g=G, rho_4C=a5 * (1 - (4 + a1) ** 2 * (4 + a2) / (a3 * (4 + a4))),
                    rho_25C=a5 * (1 - (25 + a1) ** 2 * (25 + a2) / (a3 * (25 + a4))))

# ---------------- envelope (hull_r * 0.985, as build_auv.py lathes it) ----------------
import yaml
cfg = yaml.safe_load(open(YML))
H = cfg['hull']
L, R = H['length'], H['radius']
def hull_r(x):
    xn = L / 2; xn0 = xn - H['nose_len']; xt0 = -xn + H['tail_len']
    if x >= xn or x <= -xn: return 0.0
    if x > xn0:
        t = (x - xn0) / H['nose_len']; return R * (1 - t ** 2.2) ** (1 / 2.2) if t < 1 else 0.0
    if x >= xt0: return R
    t = min(1.0, max(0.0, (xt0 - x) / H['tail_len'])); sm = t * t * (3 - 2 * t)
    return R - (R - H['tail_radius']) * sm
SC = 0.985
xs = np.linspace(-L / 2, L / 2, 26001); dx = xs[1] - xs[0]
rs = np.array([SC * hull_r(x) for x in xs])
def bore_vol(xc, rb):
    """volume of the body of revolution inside a vertical cylinder of radius rb at x=xc (grid integration)."""
    g = np.linspace(-rb, rb, 801); d = g[1] - g[0]
    X, Y = np.meshgrid(xc + g, g, indexing='ij')
    inside = X ** 2 * 0 + ((X - xc) ** 2 + Y ** 2 <= rb ** 2)
    rr = np.interp(X, xs, rs)
    chord = 2 * np.sqrt(np.clip(rr ** 2 - Y ** 2, 0, None))
    v = (chord * inside).sum() * d * d
    return v
V_rev = np.sum(np.pi * rs ** 2) * dx
x_rev = np.sum(np.pi * rs ** 2 * xs) * dx / V_rev
TUN = [0.33, -0.30]   # vfore / vaft x from knucker.yaml
old_rb, new_rb, new_ri = 0.056, 0.058, 0.052    # old cutter r_out (build_auv.py); new: T200 OD 100 mm + 2 mm gap, 6 mm wall
Vb_old = [bore_vol(x, old_rb) for x in TUN]; Vb_new = [bore_vol(x, new_rb) for x in TUN]
V_env_old = V_rev - sum(Vb_old); V_env = V_rev - sum(Vb_new)
x_env = (V_rev * x_rev - sum(v * x for v, x in zip(Vb_new, TUN))) / V_env
# wetted area of revolution (outer skin) minus bore openings (approx 2 x pi rb^2 per tunnel, top+bottom)
drs = np.gradient(rs, dx)
S_rev = np.sum(2 * np.pi * rs * np.sqrt(1 + drs ** 2)) * dx
S_skin = S_rev - sum(2 * np.pi * new_rb ** 2 for _ in TUN)
chords_new = [2 * np.interp(x, xs, rs) for x in TUN]
V_tunwall = [np.pi * (new_rb ** 2 - new_ri ** 2) * c for c in chords_new]
V_env_r047 = V_rev - sum(bore_vol(x, 0.047) for x in TUN)
OUT['envelope'] = dict(V_env_r047=V_env_r047, V_rev=V_rev, x_rev=x_rev, V_env_oldbores=V_env_old, V_env=V_env, x_env=x_env, S_skin=S_skin,
                       tunnel_wall_vol=V_tunwall, tunnel_chord=chords_new, bore_new=Vb_new)

# ---------------- mesh: per-shell volumes, outside-envelope fraction ----------------
Vm, Fm, shells = meshlib.load(OBJ)
def inside_env(P):
    r = np.interp(P[:, 0], xs, rs)
    ins = (P[:, 1] ** 2 + P[:, 2] ** 2) < r ** 2
    for xc in TUN:
        ins &= ((P[:, 0] - xc) ** 2 + P[:, 1] ** 2) > new_rb ** 2
    return ins
def inside_shell(P, tri):
    """parity of +z ray crossings (xy-projected barycentric test)."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    v0 = b[:, :2] - a[:, :2]; v1 = c[:, :2] - a[:, :2]
    den = v0[:, 0] * v1[:, 1] - v0[:, 1] * v1[:, 0]
    ok = np.abs(den) > 1e-14
    a, b, c, v0, v1, den = a[ok], b[ok], c[ok], v0[ok], v1[ok], den[ok]
    cnt = np.zeros(len(P), int)
    for s in range(0, len(P), 2000):
        p = P[s:s + 2000]
        w = p[:, None, :2] - a[None, :, :2]
        l1 = (w[..., 0] * v1[None, :, 1] - w[..., 1] * v1[None, :, 0]) / den
        l2 = (v0[None, :, 0] * w[..., 1] - v0[None, :, 1] * w[..., 0]) / den
        l0 = 1 - l1 - l2
        hit = (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
        zi = l0 * a[None, :, 2] + l1 * b[None, :, 2] + l2 * c[None, :, 2]
        cnt[s:s + 2000] = (hit & (zi > p[:, None, 2])).sum(1)
    return cnt % 2 == 1
decor = []; replaced = []
for m, ids in shells:
    if m == 'k_hull': continue
    vol, c, I = meshlib.props(Vm, Fm, ids)
    if vol < 1e-7: continue
    tri = Vm[Fm[ids]]
    # horizontal duct assembly (|y|~0.15, x<-0.49) and the old tunnel tubes are replaced by the T200 + new tunnel walls
    if (abs(abs(c[1]) - 0.15) < 0.02 and c[0] < -0.49) or (m == 'k_duct'):
        replaced.append((m, vol, c.tolist())); continue
    lo, hi = tri.reshape(-1, 3).min(0), tri.reshape(-1, 3).max(0)
    N = 6000
    P = lo + (hi - lo) * rng.random((N, 3)) + 1e-9 * rng.standard_normal((N, 3))
    ins = inside_shell(P, tri)
    nin = ins.sum()
    if nin < 50:
        frac_out, c_out = 1.0, c
    else:
        out = ins & ~inside_env(P)
        frac_out = out.sum() / nin
        c_out = P[out].mean(0) if out.sum() else c
    decor.append(dict(mat=m, vol=vol, frac_out=frac_out, v_out=vol * frac_out, c=c_out.tolist(), I_unit=(I / vol).tolist()))
V_decor_out = sum(d['v_out'] for d in decor)
OUT['decor'] = dict(n=len(decor), V_total=sum(d['vol'] for d in decor), V_outside=V_decor_out,
                    by_mat={m: sum(d['v_out'] for d in decor if d['mat'] == m) for m in sorted(set(d['mat'] for d in decor))},
                    replaced_V=sum(r[1] for r in replaced))

# ---------------- components (mass kg, displaced m^3, position, inertia about own centroid) ----------------
def cyl_I(m, r_o, r_i, h, axis):
    a = 0.5 * m * (r_o ** 2 + r_i ** 2); t = m * (3 * (r_o ** 2 + r_i ** 2) + h ** 2) / 12
    d = {'x': [a, t, t], 'y': [t, a, t], 'z': [t, t, a]}[axis]; return np.diag(d)
def box_I(m, lx, ly, lz):
    return np.diag([m * (ly ** 2 + lz ** 2) / 12, m * (lx ** 2 + lz ** 2) / 12, m * (lx ** 2 + ly ** 2) / 12])
RHO_PETG = 1270.0   # Prusament PETG TDS: 1.27 g/cm3
WALL = 0.003
X_H = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0   # housing centre x (between tunnels: free span [-0.242, 0.272] -> X_H in [-0.074, 0.104])
TUBE_L, TUBE_OD, TUBE_ID = 0.300, 0.112, 0.103
FL_OD, FL_OUT = 0.1143, (0.027 - 0.0191) + 0.010     # flange beyond tube + 10 mm cap (BR-100665 table dims; verify with BR CAD)
comp = []
def add(name, m, V, xyz, I, inside_env_flag, src, Idisp=None):
    comp.append(dict(name=name, m=m, V=V, xyz=np.array(xyz, float), I=np.array(I, float), inenv=inside_env_flag, src=src,
                     Idisp=np.zeros((3, 3)) if Idisp is None else np.array(Idisp, float)))
# housing (BR 4in locking, aluminium)
m_tube = 1.066; m_fl = 2 * 0.107; m_cap = 2 * 0.234
V_h = np.pi * (TUBE_OD / 2) ** 2 * TUBE_L + 2 * np.pi * (FL_OD / 2) ** 2 * FL_OUT
add('housing_tube_4in_300mm_al', m_tube, np.pi * (TUBE_OD / 2) ** 2 * TUBE_L, [X_H, 0, 0], cyl_I(m_tube, TUBE_OD / 2, TUBE_ID / 2, TUBE_L, 'x'), True, 'BR-106279-300 1066 g, OD 112 ID 103',
    Idisp=cyl_I(RHO * np.pi * (TUBE_OD / 2) ** 2 * TUBE_L, TUBE_OD / 2, 0, TUBE_L, 'x'))
for s in (+1, -1):
    add('housing_flange+cap_%s' % ('fwd' if s > 0 else 'aft'), 0.107 + 0.234, np.pi * (FL_OD / 2) ** 2 * FL_OUT,
        [X_H + s * (TUBE_L / 2 + FL_OUT / 2), 0, 0], cyl_I(0.341, FL_OD / 2, 0, 0.03, 'x'), True, 'BR-100665 107 g + Al cap 234 g (blank = upper bound)')
# dry contents (mass only, V=0: inside the housing)
add('battery_BR_14.8V_18Ah', 1.152, 0.0, [X_H - 0.06, 0, -0.103 / 2 + 0.0742 / 2], cyl_I(1.152, 0.0371, 0, 0.146, 'x'), True, 'BR-100842 1152 g, D74.2x146')
add('esc_BR_basic_x4', 4 * 0.0163, 0.0, [X_H + 0.06, 0, -0.02], box_I(0.0652, 0.06, 0.06, 0.02), True, 'Basic ESC 16.3 g each')
add('ahrs_microstrain_cv7', 0.0083, 0.0, [X_H + 0.10, 0, 0.0], box_I(0.0083, 0.038, 0.024, 0.0086), True, '3DM-CV7 8.3 g')
add('electronics_tray_compute_wiring', 0.35, 0.0, [X_H + 0.06, 0, 0.02], box_I(0.35, 0.20, 0.08, 0.04), True, 'ESTIMATE (unpublished): Pi-class computer, power board, tray, wiring; weigh and replace')
# wet components
for nm, xyz, ax in (('t200_port', [-0.585, 0.15, 0], 'x'), ('t200_stbd', [-0.585, -0.15, 0], 'x'),
                    ('t200_vfore', [TUN[0], 0, 0], 'z'), ('t200_vaft', [TUN[1], 0, 0], 'z')):
    add(nm, 0.427, 0.427 / RHO - 0.239 / RHO, xyz, cyl_I(0.427, 0.05, 0, 0.113, ax), False, 'T200 427 g air / 239 g water (1 m cable)')
add('dvl_waterlinked_a50', 0.170, (0.170 - 0.105) / RHO, [-0.10, 0, -0.105 + 0.0125], cyl_I(0.17, 0.033, 0, 0.025, 'z'), True, 'A50 0.17 kg air / 0.105 kg water')
add('pressure_bar30', 0.014, 0.0, [X_H - TUBE_L / 2 - FL_OUT, 0, 0.03], np.zeros((3, 3)), True, 'Bar30 14 g (displacement unpublished, < 5 cm3)')
add('structure_clamps_saddles', 0.15, 0.15 / RHO_PETG, [X_H, 0, -0.06], box_I(0.15, 0.3, 0.1, 0.02), True, 'ESTIMATE: 2 enclosure clamps + printed saddles (PETG)')
# fairing skin (thin shell of revolution, PETG)
m_skin = S_skin * WALL * RHO_PETG
w = 2 * np.pi * rs * np.sqrt(1 + drs ** 2) * dx * WALL * RHO_PETG
Ixx_s = np.sum(w * rs ** 2); x_s = np.sum(w * xs) / w.sum()
Iyy_s = np.sum(w * (rs ** 2 / 2 + (xs - x_s) ** 2))
add('fairing_skin_petg_3mm', m_skin, S_skin * WALL, [x_s, 0, 0], np.diag([Ixx_s, Iyy_s, Iyy_s]) * m_skin / w.sum(), True, 'PETG 1270 kg/m3, 3 mm wall')
for k, x in enumerate(TUN):
    add('tunnel_wall_%d' % k, V_tunwall[k] * RHO_PETG, V_tunwall[k], [x, 0, 0], cyl_I(V_tunwall[k] * RHO_PETG, new_rb, new_ri, chords_new[k], 'z'), False, 'PETG tube r 52-58 mm')
for d in decor:
    m = d['v_out'] * RHO_PETG
    add('decor_' + d['mat'], m, d['v_out'], d['c'], np.array(d['I_unit']) * m, False, 'PETG 100% infill')

# ---------------- trim: foam (R-3318, 288 kg/m3) + ballast (BR 200 g SS: 191 g air, 165 g water) ----------------
RHO_FOAM = 288.0
def totals(cs):
    W = sum(c['m'] for c in cs)
    Vx = sum(c['V'] for c in cs if not c['inenv'])
    return W, Vx
# water inside the envelope = V_env - skin wall - solids inside the envelope
def budget(extra):
    cs = comp + extra
    V_in_solid = sum(c['V'] for c in cs if c['inenv'])
    V_water = V_env - V_in_solid
    V_disp = V_env + sum(c['V'] for c in cs if not c['inenv'])
    m_solid = sum(c['m'] for c in cs)
    return cs, V_water, V_disp, m_solid
NET = 0.010     # design: +1.0 % of weight positive at 15 C (still positive at 25 C: rho drops 0.2 %)
# foam saddle above housing first (max BG), then nose/tail blocks for pitch trim
r_in_mid = SC * R - WALL
sad_area = np.pi * (r_in_mid ** 2 - (FL_OD / 2 + 0.002) ** 2) / 3
sad_z = 2 / 3 * (r_in_mid ** 3 - (FL_OD / 2 + 0.002) ** 3) / (r_in_mid ** 2 - (FL_OD / 2 + 0.002) ** 2) * math.sin(math.pi / 3) / (math.pi / 3)
V_sad_max = sad_area * TUBE_L
X_NOSE, X_TAIL = 0.47, -0.45
Z_FT = 0.03    # nose/tail foam in the upper half of the bay: half-disc centroid 4 r / (3 pi) ~ 0.42 x 0.072 m
# nose / tail foam bays: fill from the top down (foam highest -> largest BG), skin wall and tunnel bores excluded
BAY = dict(nose=(X_H + TUBE_L / 2 + FL_OUT + 0.005, 0.60), tail=(-0.56, X_H - TUBE_L / 2 - FL_OUT - 0.005))
def bay_grid(nm):
    x0, x1 = BAY[nm]; mk = (xs >= x0) & (xs <= x1)
    for xt in TUN: mk &= np.abs(xs - xt) > new_rb + WALL
    return xs[mk], np.clip(rs[mk] - WALL, 0, None)
BAYG = {k: bay_grid(k) for k in BAY}
BAY_CAP = {k: float(np.sum(np.pi * g[1] ** 2) * dx) for k, g in BAYG.items()}
def _bay_table(nm, n=2001):
    """fill level d (foam occupies the part of each section above z = d) -> volume, centroid; tabulated once."""
    xb, rb = BAYG[nm]; rmax = rb.max()
    ds_ = np.linspace(rmax, -rmax, n); Vt = np.zeros(n); Cx = np.zeros(n); Cz = np.zeros(n)
    for i, d in enumerate(ds_):
        dd = np.clip(d, -rb, rb); ok = rb > 0
        A = np.where(ok, rb ** 2 * np.arccos(np.where(ok, dd / np.where(ok, rb, 1), 1)) - dd * np.sqrt(np.clip(rb ** 2 - dd ** 2, 0, None)), 0)
        zn = 2 / 3 * np.clip(rb ** 2 - dd ** 2, 0, None) ** 1.5
        Vt[i] = np.sum(A) * dx
        if Vt[i] > 0: Cx[i] = np.sum(A * xb) * dx / Vt[i]; Cz[i] = np.sum(zn) * dx / Vt[i]
        else: Cx[i] = np.mean(xb); Cz[i] = rmax
    return Vt, Cx, Cz
BAYT = {k: _bay_table(k) for k in BAY}
def bay_fill(nm, V):
    Vt, Cx, Cz = BAYT[nm]
    return np.array([np.interp(V, Vt, Cx), 0.0, np.interp(V, Vt, Cz)])
def foam_bay(nm, V):
    return foam('foam_' + nm, V, bay_fill(nm, max(V, 1e-7)))
def foam(nm, V, xyz):
    return dict(name=nm, m=V * RHO_FOAM, V=V, xyz=np.array(xyz, float), I=np.zeros((3, 3)), inenv=True, src='R-3318 288 kg/m3')
def ballast(n, xyz):
    return dict(name='ballast_BR_200g_x%d' % n, m=0.191 * n, V=n * (0.191 - 0.165) / RHO, xyz=np.array(xyz, float), I=np.zeros((3, 3)), inenv=True, src='BR 200 g SS')
def solve(nb):
    best = None
    # unknowns: V_sad (<= max), V_nose, V_tail ; equations: net buoyancy, x_G = x_B
    for fs in np.linspace(1.0, 0.0, 101):
        Vs = fs * V_sad_max
        base = [foam('foam_saddle', Vs, [X_H, 0, sad_z])] + ([ballast(nb, [X_H, 0, -0.072])] if nb else [])
        # linear in (Vn, Vt): w_i = (rho - rho_f) V lift; solve 2x2
        def resid(Vn, Vt):
            cs, Vw, Vd, ms = budget(base + [foam_bay('nose', Vn), foam_bay('tail', Vt)])
            W = (ms + RHO * Vw); B = RHO * Vd
            xg = (sum(c['m'] * c['xyz'][0] for c in cs) + RHO * (V_env * x_env - sum(c['V'] * c['xyz'][0] for c in cs if c['inenv']))) / W
            xb = (V_env * x_env + sum(c['V'] * c['xyz'][0] for c in cs if not c['inenv'])) / Vd
            return np.array([B - W * (1 + NET), W * (xg - xb)])
        Vn, Vt = 1e-3, 1e-3
        for _ in range(12):     # Newton: foam centroids move with the fill level
            r0 = resid(Vn, Vt); h = 1e-5
            J = np.column_stack([resid(Vn + h, Vt) - r0, resid(Vn, Vt + h) - r0]) / h
            dV = np.linalg.solve(J, -r0); Vn, Vt = Vn + dV[0], Vt + dV[1]
            if Vn < -1e-3 or Vt < -1e-3: break
        if Vn >= 0 and Vt >= 0 and np.max(np.abs(resid(Vn, Vt))) < 1e-6 and Vn <= BAY_CAP['nose'] and Vt <= BAY_CAP['tail']:
            return Vs, Vn, Vt, base
    return None
def cgcb(extra):
    cs, Vw, Vd, ms = budget(extra)
    cw = RHO * V_env * np.array([x_env, 0, 0]) - sum(RHO * c['V'] * c['xyz'] for c in cs if c['inenv'])
    Wt = ms + RHO * Vw
    return (sum(c['m'] * c['xyz'] for c in cs) + cw) / Wt, (V_env * np.array([x_env, 0, 0]) + sum(c['V'] * c['xyz'] for c in cs if not c['inenv'])) / Vd, Wt
# BG target: REMUS 100 (1.33 m, 30.5 kg) has z_G = 1.96 cm below CB (Prestero 2001, via Makam 2023 Table 2);
# knucker has no roll actuator, so BG is its only roll stiffness. Smallest ballast count reaching BG_TARGET.
BG_TARGET = 0.015
trim_table = []; nb_sol = None
for nb in range(0, 41):
    sol = solve(nb)
    if sol is None: continue
    Vs, Vn, Vt, base = sol; nb_sol = nb
    g_, b_, W_ = cgcb(base + [foam_bay('nose', Vn), foam_bay('tail', Vt)])
    trim_table.append((nb, float(b_[2] - g_[2]), float(W_), float((Vs + Vn + Vt) * 1e3)))
    if b_[2] - g_[2] >= BG_TARGET: break
extra = base + [foam_bay('nose', Vn), foam_bay('tail', Vt)]
cs, Vw, Vd, ms = budget(extra)
# water as rigid body: envelope interior (solid of revolution minus bores) minus solids inside
m_w = RHO * Vw
cw_num = RHO * (V_env * x_env) * np.array([1, 0, 0]) - sum(RHO * c['V'] * c['xyz'] for c in cs if c['inenv'])
c_w = cw_num / m_w
W = ms + m_w; B = RHO * Vd
cg = (sum(c['m'] * c['xyz'] for c in cs) + cw_num) / W
cb = (V_env * np.array([x_env, 0, 0]) + sum(c['V'] * c['xyz'] for c in cs if not c['inenv'])) / Vd
# inertia about link origin (x=0 hull mid, on axis), solids
def par(I, m, r):
    r = np.asarray(r); return I + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
I_sol = sum(par(c['I'], c['m'], c['xyz']) for c in cs)
# envelope interior water (rigid, upper bound) by slabs
wr = RHO * np.pi * rs ** 2 * dx
I_env = np.diag([np.sum(wr * rs ** 2 / 2), np.sum(wr * (rs ** 2 / 4 + xs ** 2)), np.sum(wr * (rs ** 2 / 4 + xs ** 2))])
for v, x in zip(Vb_new, TUN):     # remove bores (approx as cylinders of the chord height)
    h = 2 * np.interp(x, xs, rs); mb = RHO * v
    I_env -= par(cyl_I(mb, new_rb, 0, h, 'z'), mb, [x, 0, 0])
I_w_rigid = I_env - sum(par(c.get('Idisp', np.zeros((3, 3))), RHO * c['V'], c['xyz']) for c in cs if c['inenv'])
# roll of free-flood water: ideal-fluid annulus is not spun up -> Ixx_w about its centroid -> 0 (lower bound)
Iw_c = I_w_rigid - par(np.zeros((3, 3)), m_w, c_w)
I_w_ideal = Iw_c.copy(); I_w_ideal[0, 0] = 0; I_w_ideal[0, 1:] = 0; I_w_ideal[1:, 0] = 0
I_w_ideal = par(I_w_ideal, m_w, c_w)
I_tot_rigid = I_sol + I_w_rigid; I_tot_ideal = I_sol + I_w_ideal
def about(I, m, c): return I - m * (np.dot(c, c) * np.eye(3) - np.outer(c, c))
OUT['mass'] = dict(solids=ms, water=m_w, total=W, V_disp=Vd, B_N=B * G, W_N=W * G, net_pct=(B / W - 1) * 100,
                   cg=cg.tolist(), cb=cb.tolist(), BG=(cb - cg).tolist(), foam_L=dict(saddle=Vs * 1e3, nose=Vn * 1e3, tail=Vt * 1e3), bay_capacity_L={k: v * 1e3 for k, v in BAY_CAP.items()}, foam_xyz={c['name']: c['xyz'].tolist() for c in cs if c['name'].startswith('foam')}, ballast_n=nb_sol, X_H=X_H, trim_table=trim_table, BG_target=BG_TARGET,
                   I_origin_ideal=I_tot_ideal.tolist(), I_origin_rigidwater=I_tot_rigid.tolist(),
                   I_cg_ideal=about(I_tot_ideal, W, cg).tolist(), I_cg_rigidwater=about(I_tot_rigid, W, cg).tolist(),
                   in_water_weight={c['name']: c['m'] - RHO * c['V'] for c in cs},
                   components=[(c['name'], round(c['m'], 4), round(c['V'] * 1e3, 4), np.round(c['xyz'], 3).tolist(), c['src']) for c in cs if not c['name'].startswith('decor_')],
                   decor_mass=sum(c['m'] for c in cs if c['name'].startswith('decor_')))

# ---------------- 2-D BEM added mass (constant source panels, Katz & Plotkin 2001 sec. 10.2.1) ----------------
def bem(contours, rho=RHO):
    P1 = np.vstack(contours); P2 = np.vstack([np.roll(c, -1, 0) for c in contours])
    d = P2 - P1; Lp = np.hypot(d[:, 0], d[:, 1]); t = d / Lp[:, None]; n = np.stack([t[:, 1], -t[:, 0]], 1)
    mid = (P1 + P2) / 2
    rx = mid[:, None, 0] - P1[None, :, 0]; ry = mid[:, None, 1] - P1[None, :, 1]
    X = rx * t[None, :, 0] + ry * t[None, :, 1]; Z = rx * n[None, :, 0] + ry * n[None, :, 1]
    X2 = X - Lp[None, :]
    r1 = X ** 2 + Z ** 2; r2 = X2 ** 2 + Z ** 2
    th = np.arctan2(Z, X2) - np.arctan2(Z, X)
    th = (th + np.pi) % (2 * np.pi) - np.pi
    u = np.log(r1 / r2) / (4 * np.pi); wv = th / (2 * np.pi)
    ii = np.arange(len(Lp)); u[ii, ii] = 0; wv[ii, ii] = 0.5
    A = (u * t[None, :, 0] + wv * n[None, :, 0]) * n[:, None, 0] + (u * t[None, :, 1] + wv * n[None, :, 1]) * n[:, None, 1]
    with np.errstate(divide='ignore', invalid='ignore'):
        phi = (X * np.log(r1) - X2 * np.log(r2) + 2 * Z * th) / (4 * np.pi)
    phi[ii, ii] = Lp * np.log(Lp / 2) / (2 * np.pi)
    modes = np.stack([n[:, 0], n[:, 1], -mid[:, 1] * n[:, 0] + mid[:, 0] * n[:, 1]], 1)  # sway(y), heave(z), roll about origin
    sig = np.linalg.solve(A, modes); ph = phi @ sig
    return -rho * (ph * Lp[:, None]).T @ modes     # 3x3 [y,z,roll]
def circle(cy, cz, r, n):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False); return np.stack([cy + r * np.cos(a), cz + r * np.sin(a)], 1)
def body_with_fins(r, fins, ds=0.001):
    """CCW contour: circle radius r with plate fins [(angle, tip_radius, thickness)] attached radially."""
    pts = []
    fins = sorted(fins)
    if not fins:
        return circle(0, 0, r, max(64, int(2 * np.pi * r / ds)))
    seg = []
    for k, (al, a, tk) in enumerate(fins):
        de = math.asin(min(0.99, tk / 2 / r))
        seg.append((al - de, al + de, al, a, tk))
    for k, (s0, s1, al, a, tk) in enumerate(seg):
        e = np.array([math.cos(al), math.sin(al)]); p = np.array([-e[1], e[0]])
        # fin: from circle at s0 out along edge -tk/2, tip, back along +tk/2 to s1
        r0 = math.sqrt(max(r * r - (tk / 2) ** 2, 0.0))
        nr = max(2, int((a - r0) / ds))
        for q in np.linspace(r0, a, nr, endpoint=False): pts.append(q * e - tk / 2 * p)
        pts.append(a * e - tk / 2 * p); pts.append(a * e + tk / 2 * p)
        for q in np.linspace(a, r0, nr, endpoint=False)[1:]: pts.append(q * e + tk / 2 * p)
        nxt = seg[(k + 1) % len(seg)][0] + (2 * np.pi if k + 1 == len(seg) else 0)
        na = max(4, int(r * (nxt - s1) / ds))
        for q in np.linspace(s1, nxt, na, endpoint=False): pts.append(r * np.array([math.cos(q), math.sin(q)]))
    return np.array(pts)
# BEM validation against closed forms
val = {}
val['circle_r0.1'] = (bem([circle(0, 0, 0.1, 400)], 1000)[0, 0], 1000 * np.pi * 0.01)
pl = np.array([[-0.1, -0.002], [0.1, -0.002], [0.1, 0.002], [-0.1, 0.002]])
plc = np.vstack([np.linspace(pl[i], pl[(i + 1) % 4], 200 if i % 2 == 0 else 4, endpoint=False) for i in range(4)])
val['plate_2a0.2_heave'] = (bem([plc], 1000)[1, 1], 1000 * np.pi * 0.01)
bf = body_with_fins(0.05, [(0.0, 0.15, 0.003), (np.pi, 0.15, 0.003)])
val['circle+2fins_heave(Blevins)'] = (bem([bf], 1000)[1, 1], 1000 * np.pi * (0.15 ** 2 - 0.05 ** 2 + 0.05 ** 4 / 0.15 ** 2))
OUT['bem_validation'] = {k: dict(bem=v[0], exact=v[1], err_pct=(v[0] / v[1] - 1) * 100) for k, v in val.items()}

# fin outlines from the mesh by slicing triangles with planes x = const
def slice_extent(ids, x, comp_idx):
    tri = Vm[Fm[ids]]; vals = []
    for t in tri:
        for i, j in ((0, 1), (1, 2), (2, 0)):
            a, b = t[i], t[j]
            if (a[0] - x) * (b[0] - x) < 0:
                s = (x - a[0]) / (b[0] - a[0]); vals.append(a[comp_idx] + s * (b[comp_idx] - a[comp_idx]))
    return (min(vals), max(vals)) if vals else None
fin_shells = [(m, ids) for m, ids in shells if m == 'k_fin']
dorsal = [ids for m, ids in fin_shells if abs(meshlib.props(Vm, Fm, ids)[1][1]) < 0.01][0]
pect = [ids for m, ids in fin_shells if meshlib.props(Vm, Fm, ids)[1][1] > 0.05][0]
ST = np.linspace(-L / 2 + 0.005, L / 2 - 0.005, 131); dxs = ST[1] - ST[0]
S_dorsal = meshlib.props(Vm, Fm, dorsal)[0] / 0.004; S_pect = meshlib.props(Vm, Fm, pect)[0] / 0.004
trapz = getattr(np, 'trapezoid', None) or np.trapz
def ell_E(k):
    """complete elliptic integral of the 2nd kind E(k), k = eccentricity."""
    th = np.linspace(0, np.pi / 2, 20001); return float(trapz(np.sqrt(1 - (k * np.sin(th)) ** 2), th))
def lamb(a, b):
    """Lamb (1932) Hydrodynamics sec. 373: prolate spheroid k1 (axial), k2 (lateral), k' (rotation), a > b."""
    e = math.sqrt(1 - (b / a) ** 2); lg = math.log((1 + e) / (1 - e))
    al0 = 2 * (1 - e ** 2) / e ** 3 * (0.5 * lg - e); be0 = 1 / e ** 2 - (1 - e ** 2) / (2 * e ** 3) * lg
    kp = e ** 4 * (be0 - al0) / ((2 - e ** 2) * (2 * e ** 2 - (2 - e ** 2) * (be0 - al0)))
    return al0 / (2 - al0), be0 / (2 - be0), kp
def slice_ext(ids, val, ax, ci):
    """extent of shell `ids` along coordinate ci on the plane coordinate[ax] = val."""
    tri = Vm[Fm[ids]]; a = tri[:, [0, 1, 2, 0], :]
    p, q = a[:, :3, :], a[:, 1:, :]
    m = (p[..., ax] - val) * (q[..., ax] - val) < 0
    if not m.any(): return None
    s = (val - p[..., ax][m]) / (q[..., ax][m] - p[..., ax][m]); v = p[..., ci][m] + s * (q[..., ci][m] - p[..., ci][m])
    return float(v.min()), float(v.max())
Z3 = np.zeros((3, 3))
sec = []; secA = []
for x in ST:
    r = SC * hull_r(x); dfin = []; pfin = []
    e = slice_ext(dorsal, x, 0, 2)
    if e and e[1] > r + 0.004: dfin.append((np.pi / 2, e[1], 0.004))
    e = slice_ext(pect, x, 0, 1)
    if e and e[1] > r + 0.004: pfin = [(0.0, e[1], 0.004), (np.pi, e[1], 0.004)]
    pod = abs(x + 0.585) <= 0.0565
    pyl = [(0.0, 0.098, 0.016), (np.pi, 0.098, 0.016)] if pod else []
    hz_plate = sum(max(0, a - r) for al, a, tk in dfin)
    hy_plate = sum(max(0, a - r) for al, a, tk in pfin + pyl)
    sec.append((x, None, (2 * r, hz_plate, 0), (2 * r, hy_plate, 0), dfin + pfin + pyl, pod))
    Mb = np.diag([RHO * np.pi * r * r] * 2 + [0.0]); Md = Mpy = Mpod = Z3
    if r > 0.004 and (dfin or pod):
        Mc = bem([body_with_fins(r, [])])
        if dfin: Md = bem([body_with_fins(r, dfin)]) - Mc
        if pod:
            Mpy = bem([body_with_fins(r, pyl)]) - Mc
            Mpod = bem([body_with_fins(r, pyl), circle(0.15, 0, 0.05, 160), circle(-0.15, 0, 0.05, 160)]) - Mc - Mpy
    secA.append((Mb, Md, Mpy, Mpod))
Itg = lambda f: float(np.sum(f) * dxs)
def strip(Ms):
    Ms = np.array(Ms); myy, mzz, m44, my4 = Ms[:, 0, 0], Ms[:, 1, 1], Ms[:, 2, 2], Ms[:, 0, 2]
    A = np.zeros((6, 6))
    A[1, 1] = Itg(myy); A[2, 2] = Itg(mzz); A[3, 3] = Itg(m44)
    A[1, 5] = A[5, 1] = Itg(ST * myy); A[2, 4] = A[4, 2] = -Itg(ST * mzz)
    A[4, 4] = Itg(ST ** 2 * mzz); A[5, 5] = Itg(ST ** 2 * myy)
    A[1, 3] = A[3, 1] = Itg(my4); A[3, 5] = A[5, 3] = Itg(ST * my4)
    return A
# (1) body of revolution: exact 2-D circle strips x Lamb 3-D factors of the equal-length, equal-volume spheroid
a_s = L / 2; b_s = math.sqrt(V_rev / (4 / 3 * np.pi * a_s))
k1, k2, kpr = lamb(a_s, b_s); c_rot = kpr * (1 + b_s ** 2 / a_s ** 2)
Ab = strip([s[0] for s in secA])
for i, j in ((1, 1), (2, 2), (1, 5), (5, 1), (2, 4), (4, 2)): Ab[i, j] *= k2
Ab[4, 4] *= c_rot; Ab[5, 5] *= c_rot
Ab[0, 0] = k1 * RHO * V_rev
# (2) dorsal: 2-D BEM increments (sections cut across its long axis = chord) x 1/E(e) of the image-doubled
#     area-equivalent ellipse (exact elliptic plate / strip theory = 1/E(e); Lamb 1932 sec. 373, Kanwal 1955)
S_d_exp = Itg(np.array([s[2][1] for s in sec]))
sp_d = max(s[2][1] for s in sec); a_d = 2 * S_d_exp / (np.pi * sp_d)
assert a_d > sp_d, 'dorsal: chord must be the long axis for x-strips'
e_d = math.sqrt(1 - (sp_d / a_d) ** 2); f_d = 1 / ell_E(e_d)
Ad = strip([s[1] for s in secA]) * f_d
# (3) pylons: 2-D BEM increments, unscaled (chord 0.17 >> exposed span; upper bound)
Apy = strip([s[2] for s in secA])
# (4) T200 pods: 2-D BEM increments (incl. interaction) x finite-cylinder factor, transverse, L/D = 113/100
#     DNV-RP-C205 (2010) Table A-2 / Patton 1965: Ca = 0.62 at L/D 1.2, 0.78 at 2.5 -> 0.61 at 1.13 (linear extrapolation)
LD_pod = 0.113 / 0.100; f_pod = 0.62 + (0.78 - 0.62) / (2.5 - 1.2) * (LD_pod - 1.2)
Apod = strip([s[3] for s in secA]) * f_pod
# (5) pectorals: spanwise strips (sections cut across their long axis = span), 2-D plate rho*pi*(c/2)^2,
#     x 1/E(e) of the composite tip-to-tip area-equivalent ellipse; exposed span only (root at the hull skin)
pv = Vm[np.unique(Fm[pect])]; y_tip = pv[:, 1].max(); xm_p = 0.5 * (pv[:, 0].min() + pv[:, 0].max())
y_root = SC * hull_r(xm_p)
YS = np.linspace(y_root + 0.001, y_tip - 0.001, 200); dys = YS[1] - YS[0]
ch = []; xm = []
for y in YS:
    e = slice_ext(pect, y, 1, 0); ch.append(e[1] - e[0] if e else 0.0); xm.append(0.5 * (e[0] + e[1]) if e else 0.0)
ch = np.array(ch); xm = np.array(xm)
S_p_exp = float(np.sum(ch) * dys)
c_root = float(np.interp(y_root, YS, ch)) if ch[0] > 0 else 0.0
S_comp = 2 * S_p_exp + 2 * y_root * ch[0]
b_p = S_comp / (np.pi * y_tip); e_p = math.sqrt(1 - (b_p / y_tip) ** 2); f_p = 1 / ell_E(e_p)
m2 = RHO * np.pi * (ch / 2) ** 2
Ap = np.zeros((6, 6))
Ap[2, 2] = 2 * f_p * np.sum(m2) * dys
Ap[3, 3] = 2 * f_p * np.sum(m2 * YS ** 2) * dys
Ap[2, 4] = Ap[4, 2] = -2 * f_p * np.sum(m2 * xm) * dys
Ap[4, 4] = 2 * f_p * np.sum(m2 * xm ** 2 + RHO * np.pi * (ch / 2) ** 4 / 8) * dys
# alternative (bracket): composite elliptic plate (4/3) pi rho a b^2 / E minus the k2-corrected body strip over the root chord
m_plate = 4 / 3 * np.pi * RHO * y_tip * b_p ** 2 / ell_E(e_p)
pect_alt = m_plate - k2 * RHO * np.pi * y_root ** 2 * ch[0]
pect_blev = m_plate * (y_tip ** 2 - 2 * y_root ** 2 + y_root ** 4 / y_tip ** 2) / y_tip ** 2
# (6) water in the two open vertical tunnels: enclosed sideways -> moves with the hull in x, y (and yaw);
#     free to flow through vertically -> not entrained in heave
Abore = np.zeros((6, 6))
for v, xb, ch_b in zip(Vb_new, TUN, chords_new):
    mb = RHO * v
    Abore[0, 0] += mb; Abore[1, 1] += mb; Abore[1, 5] += mb * xb; Abore[5, 1] += mb * xb; Abore[5, 5] += mb * xb ** 2
    Abore[3, 3] += mb * ch_b ** 2 / 12
A = Ab + Ad + Apy + Apod + Ap + Abore
ev = np.linalg.eigvalsh(A)
# BEM convergence: dorsal section at ds 2 mm vs 1 mm
xc = -0.2; rc = SC * hull_r(xc); ec = slice_ext(dorsal, xc, 0, 2)
conv = dict(x=xc, ds2=float(bem([body_with_fins(rc, [(np.pi / 2, ec[1], 0.004)], 0.001)])[0, 0]),
            ds1=float(bem([body_with_fins(rc, [(np.pi / 2, ec[1], 0.004)], 0.0005)])[0, 0]))
conv['diff_pct'] = (conv['ds2'] / conv['ds1'] - 1) * 100
OUT['added_mass'] = dict(origin='link origin: x=0 hull mid-length, on the hull axis; FLU (REP-103); positive-definite SDF <fluid_added_mass> convention',
                         A=A.tolist(), eig=ev.tolist(), parts=dict(body=Ab.tolist(), dorsal=Ad.tolist(), pylons=Apy.tolist(), pods=Apod.tolist(), pect=Ap.tolist(), bores=Abore.tolist()),
                         lamb=dict(a=a_s, b=b_s, k1=k1, k2=k2, kprime=kpr, rot_factor=c_rot),
                         dorsal=dict(S_exp=S_d_exp, span=sp_d, a_equiv=a_d, e=e_d, factor=f_d),
                         pod=dict(L_over_D=LD_pod, factor=f_pod),
                         pect=dict(y_root=y_root, y_tip=y_tip, c_root=ch[0], S_exp_each=S_p_exp, S_composite=S_comp, b_equiv=b_p, e=e_p, factor=f_p,
                                   A33=Ap[2, 2], bracket_plate_minus_body=pect_alt, bracket_blevins_ratio=pect_blev),
                         bem_convergence=conv, ratio_to_mass=[A[i, i] / W for i in range(3)])

# ---------------- damping ----------------
U_REF = 1.0
Re = U_REF * L / NU
Cf = 0.075 / (math.log10(Re) - 2) ** 2                     # ITTC-1957
DL = 2 * SC * R / L
Cd_wet = Cf * (1 + 1.5 * DL ** 1.5 + 7 * DL ** 3)          # Hoerner 1965, streamlined body of revolution (wetted-area based)
S_wet_body = S_skin
CdA_body = Cd_wet * S_wet_body
# fins: both faces friction with Hoerner thickness factor (t/c ~ 0.004/0.3)
S_dorsal = meshlib.props(Vm, Fm, dorsal)[0] / 0.004; S_pect = meshlib.props(Vm, Fm, pect)[0] / 0.004
CdA_fins = Cf * 2 * (S_dorsal + 2 * S_pect) * (1 + 2 * 0.013 + 60 * 0.013 ** 4)
# pylons (16 mm thick plates, chord 0.17 -> 0.113 with T200) friction, both faces
S_pyl = 2 * (0.098 - SC * R * 0.9) * 0.113
CdA_pyl = Cf * 2 * S_pyl * (1 + 2 * 0.14 + 60 * 0.14 ** 4)
# T200 pods at zero command: bounds. low: nozzle friction inside+outside; high: + disc blockage Cd 1.0 on hub+blades disc
S_noz = 2 * np.pi * 0.05 * 0.113 * 2
CdA_pod_lo = 2 * Cf * S_noz
CdA_pod_hi = CdA_pod_lo + 2 * 1.0 * np.pi * 0.038 ** 2
X_uu_lo = -0.5 * RHO * (CdA_body + CdA_fins + CdA_pyl + CdA_pod_lo)
X_uu_hi = -0.5 * RHO * (CdA_body + CdA_fins + CdA_pyl + CdA_pod_hi)
CD_CYL, CD_PLATE = 1.2, 1.17   # Hoerner 1965: 2-D circular cylinder (subcritical); flat plate normal to flow, AR ~ 1
def cross(kind):
    k = 2 if kind == 'v' else 3
    f = np.array([(CD_CYL * s[k][0] + CD_PLATE * s[k][1] + CD_CYL * s[k][2] / 2 * 0) if isinstance(s[k], tuple) else 0 for s in sec])
    pod = np.array([CD_CYL * 0.1 * 2 if (isinstance(s[k], tuple) and s[5]) else 0 for s in sec])
    return f + pod
fv = cross('v'); fw = cross('w')
D = {}
D['yVabsV'] = -0.5 * RHO * Itg(fv); D['zWabsW'] = -0.5 * RHO * Itg(fw)
D['nRabsR'] = -0.5 * RHO * Itg(fv * np.abs(ST) ** 3); D['mQabsQ'] = -0.5 * RHO * Itg(fw * np.abs(ST) ** 3)
D['yRabsR'] = -0.5 * RHO * Itg(fv * ST * np.abs(ST)); D['nVabsV'] = -0.5 * RHO * Itg(fv * ST)
D['zQabsQ'] = +0.5 * RHO * Itg(fw * ST * np.abs(ST)); D['mWabsW'] = +0.5 * RHO * Itg(fw * ST)
# roll: dorsal fin (lateral flow -p z), pectorals and pods (vertical flow p y)
kp = 0.0
for s in sec:
    r = SC * hull_r(s[0])
    for al, a, tk in s[4]:
        if a <= r: continue
        if abs(al - np.pi / 2) < 0.1: kp += CD_PLATE * (a ** 4 - r ** 4) / 4 * dxs
        elif tk < 0.01: kp += CD_PLATE * (a ** 4 - r ** 4) / 4 * dxs
    if s[5]: kp += 2 * CD_CYL * 0.1 * 0.15 ** 3 * dxs
D['kPabsP'] = -0.5 * RHO * kp
D['xUabsU'] = 0.5 * (X_uu_lo + X_uu_hi); D['xUabsU_bounds'] = [X_uu_lo, X_uu_hi]
# fin lift (Helmbold low-AR lift slope with root-image AR doubling)
def helm(AR): return 2 * np.pi * AR / (2 + math.sqrt(AR ** 2 + 4))
sp_d = 0.245 - SC * R; AR_d = 2 * sp_d ** 2 / S_dorsal
sp_p = 0.34 - 0.083; AR_p = 2 * sp_p ** 2 / S_pect
cd_d = meshlib.props(Vm, Fm, dorsal)[1]; cp = meshlib.props(Vm, Fm, pect)[1]
Yuv = -0.5 * RHO * S_dorsal * helm(AR_d); Zuw = -0.5 * RHO * 2 * S_pect * helm(AR_p)
D['yVabsU_fin'] = Yuv; D['nVabsU_fin'] = cd_d[0] * Yuv; D['kVabsU_fin'] = -cd_d[2] * Yuv
D['zWabsU_fin'] = Zuw; D['mWabsU_fin'] = -cp[0] * Zuw
OUT['damping'] = dict(U_ref=U_REF, Re=Re, Cf=Cf, Cd_wet=Cd_wet, CdA=dict(body=CdA_body, fins=CdA_fins, pylons=CdA_pyl, pods_lo=CdA_pod_lo, pods_hi=CdA_pod_hi),
                      S_dorsal=S_dorsal, S_pect_each=S_pect, AR_dorsal=AR_d, AR_pect=AR_p, CLa_dorsal=helm(AR_d), CLa_pect=helm(AR_p),
                      coeffs=D, Cd_cyl=CD_CYL, Cd_plate=CD_PLATE)

# ---------------- T200 table ----------------
t = json.load(open(T200))
tt = {}
for k, rows in t.items():
    if not k.endswith('V'): continue
    a = np.array([r[:7] for r in rows[1:] if len(r) >= 7 and all(isinstance(q, (int, float)) for q in r[:7])], float)
    pwm, rpm, cur, F = a[:, 0], a[:, 1], a[:, 2], a[:, 5] * G
    fw = F > 0; rv = F < 0
    n = rpm / 60
    cf = np.sum(F[fw] * n[fw] ** 2) / np.sum(n[fw] ** 4); cr = np.sum(-F[rv] * n[rv] ** 2) / np.sum(n[rv] ** 4)
    db = pwm[np.abs(F) < 1e-9]
    tt[k] = dict(F_max=F.max(), F_min=F.min(), I_max=cur.max(), rpm_max=rpm.max(), c_fwd=cf, c_rev=cr, KT0_fwd=cf / (1000 * 0.076 ** 4),
                 KT0_rev=cr / (1000 * 0.076 ** 4), deadband=[db.min(), db.max()] if len(db) else None,
                 res_fwd=float(np.max(np.abs(F[fw] - cf * n[fw] ** 2))), res_rev=float(np.max(np.abs(-F[rv] - cr * n[rv] ** 2))))
OUT['t200'] = tt
OUT["_note"] = "see sea.md"
json.dump(OUT, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'derived_xh%+.3f.json' % X_H), 'w'), indent=1, default=float)
# compact print
f = lambda v: np.round(np.array(v, float), 4).tolist()
print('rho', round(RHO, 3), 'nu', NU)
print('env V_rev %.4f L, old-bores %.4f L (mesh 22.3309), new %.4f L, x_env %.4f, S_skin %.4f' % (V_rev * 1e3, V_env_old * 1e3, V_env * 1e3, x_env, S_skin))
print('mesh check: V_rev - 2 bores(r 0.047) = %.4f L' % (V_env_r047 * 1e3))
print('decor', OUT['decor']['n'], 'Vtot %.4f L Vout %.4f L replaced %.4f L' % (OUT['decor']['V_total'] * 1e3, V_decor_out * 1e3, OUT['decor']['replaced_V'] * 1e3))
mm = OUT['mass']
print('solids %.3f water %.3f total %.3f kg  Vdisp %.4f L net %.3f%%  foam %s ballast %d' % (ms, m_w, W, Vd * 1e3, mm['net_pct'], {k: round(v, 3) for k, v in mm['foam_L'].items()}, nb_sol))
print('cg', f(cg), 'cb', f(cb), 'BG', f(cb - cg))
print('I_cg ideal diag', f(np.diag(about(I_tot_ideal, W, cg))), 'rigid-water diag', f(np.diag(about(I_tot_rigid, W, cg))))
print('BEM val', {k: round(v['err_pct'], 2) for k, v in OUT['bem_validation'].items()})
print('A diag', f(np.diag(A)), 'A26 %.3f A35 %.3f A24 %.3f A46 %.3f k1 %.4f' % (A[1, 5], A[2, 4], A[1, 3], A[3, 5], k1))
am=OUT['added_mass']
for k_,v_ in am['parts'].items(): print(' part', k_, f(np.diag(np.array(v_))))
print(' lamb', {k_: round(v_, 4) for k_, v_ in am['lamb'].items()}, 'dorsal', {k_: round(v_, 4) for k_, v_ in am['dorsal'].items()}, 'pod', am['pod'])
print(' pect', {k_: round(float(v_), 4) for k_, v_ in am['pect'].items()}, 'conv', am['bem_convergence'], 'eigmin', min(am['eig']))
print('trim_table', [(a, round(b * 1e3, 2), round(d, 3)) for a, b, c, d in trim_table], 'bay cap L', {k: round(v * 1e3, 3) for k, v in BAY_CAP.items()}, 'foam xyz', {k: f(v) for k, v in OUT['mass']['foam_xyz'].items()})
print('damp', {k: (round(v, 3) if not isinstance(v, list) else f(v)) for k, v in D.items()})
print('Re %.3g Cf %.5f Cdwet %.5f' % (Re, Cf, Cd_wet), 'CdA', {k: round(v, 5) for k, v in OUT['damping']['CdA'].items()}, 'Sd %.4f Sp %.4f ARd %.2f ARp %.2f' % (S_dorsal, S_pect, AR_d, AR_p))
for k in ('12 V', '14 V', '16 V'):
    v = tt[k]; print(k, {a: (round(b, 4) if isinstance(b, float) else b) for a, b in v.items()})
