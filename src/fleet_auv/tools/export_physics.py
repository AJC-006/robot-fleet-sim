"""Turn the S1 derivation result (config/knucker_derived.json, produced by tools/derive.py) into the
simulation inputs config/knucker_physics.yaml: rigid body, <fluid_added_mass>, Hydrodynamics terms and the
buoyancy collision primitives.

usage: python3 -I tools/export_physics.py            (run from the package directory; needs numpy, pyyaml)

Buoyancy primitives. gz-sim's graded Buoyancy system integrates the volume of the collision shapes (meshes are
not supported in graded mode), so the primitives must reproduce the displaced volume V and its centroid, the
centre of buoyancy. Construction:
  1. hull of revolution (profile of shape.hull_r x 0.985, as built by blender/auv/build_auv.py) cut into
     `slabs` equal-length cylinders along x; each slab has the volume of the real slice minus the vertical
     tunnel bores (radius 0.058 m), so the slice volume is exact;
  2. four T200 bodies as solid cylinders of equal displacement ((0.427-0.239) kg / rho = 0.188 L each) and the two
     PETG tunnel walls as solid vertical cylinders of equal volume (derive.py: tunnel_wall_vol);
  3. the remaining volume (decor shells outside the envelope, DVL, sensors: V_total minus the above) is spread
     over the slabs as a smooth correction r^2 -> r^2 (1 + a + b x/L) with a, b solved so that BOTH the total
     volume and the x position of the centroid equal the derived values; z is matched by one common offset.
The centroid must match to 0.1 mm: 1 mm of CB x error tilts the vehicle by 3.8 degrees (docs/notes/sea.md).
"""
import json
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "..")
sys.path.insert(0, PKG)
from fleet_auv.shape import hull_r  # noqa: E402

N_SLABS = 13
SC = 0.985            # hull profile scale used by build_auv.py (lathe of hull_r * 0.985)
R_BORE = 0.058
THRUSTER_DISPLACED_L = (0.427 - 0.239) / 0.999103     # kg / (kg/L), catalog bluerobotics_t200


def bore_removed(x0, x1, xc, hull):
    """Volume of the body of revolution inside a vertical cylinder (radius R_BORE at x=xc) for x0 < x < x1."""
    d = 0.0005
    xs = np.arange(x0 + d / 2, x1, d)
    ys = np.arange(-R_BORE + d / 2, R_BORE, d)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    r = np.array([SC * hull_r(float(x), hull) for x in xs])[:, None]
    chord = 2 * np.sqrt(np.clip(r ** 2 - Y ** 2, 0, None))
    inside = (X - xc) ** 2 + Y ** 2 < R_BORE ** 2
    return float((chord * inside).sum() * d * d)


def main():
    cfg = yaml.safe_load(open(os.path.join(PKG, "config", "knucker.yaml")))
    D = json.load(open(os.path.join(PKG, "config", "knucker_derived.json")))
    hull = cfg["hull"]
    L = hull["length"]
    V_tot = D["mass"]["V_disp"]
    cb = D["mass"]["cb"]
    th = cfg["thrusters"]
    tun_x = [th["vfore"]["pos"][0], th["vaft"]["pos"][0]]

    # --- slabs
    dx = L / N_SLABS
    slabs = []
    for i in range(N_SLABS):
        x0, x1 = -L / 2 + i * dx, -L / 2 + (i + 1) * dx
        xs = np.linspace(x0, x1, 201)
        v = float(np.trapz([math.pi * (SC * hull_r(float(x), hull)) ** 2 for x in xs], xs))
        for xc in tun_x:
            if x1 > xc - R_BORE and x0 < xc + R_BORE:
                v -= bore_removed(x0, x1, xc, hull)
        slabs.append({"x": 0.5 * (x0 + x1), "l": dx, "v": v})
    # --- fixed bodies
    fixed = []
    for key, t in th.items():
        v = THRUSTER_DISPLACED_L * 1e-3
        if abs(t["dir"][0]) > 0.5:      # horizontal thruster: cylinder along x, length 0.113 m
            l = 0.113
            fixed.append({"name": "t200_" + key, "type": "cylinder", "axis": "x", "r": math.sqrt(v / (math.pi * l)),
                          "l": l, "pos": list(t["pos"]), "v": v})
        else:
            l = 0.113
            fixed.append({"name": "t200_" + key, "type": "cylinder", "axis": "z", "r": math.sqrt(v / (math.pi * l)),
                          "l": l, "pos": list(t["pos"]), "v": v})
    for key, vw, ch in zip(("vfore", "vaft"), D["envelope"]["tunnel_wall_vol"], D["envelope"]["tunnel_chord"]):
        fixed.append({"name": "tunnel_wall_" + key, "type": "cylinder", "axis": "z", "r": math.sqrt(vw / (math.pi * ch)),
                      "l": ch, "pos": list(th[key]["pos"]), "v": vw})
    Vf = sum(f["v"] for f in fixed)
    Mf = [sum(f["v"] * f["pos"][k] for f in fixed) for k in range(3)]
    # --- solve a, b: volume and x-moment of the slab group (slab volume v_i (1 + a + b x_i / L))
    V_slabs_target = V_tot - Vf
    Mx_target = V_tot * cb[0] - Mf[0]
    v = np.array([s["v"] for s in slabs])
    x = np.array([s["x"] for s in slabs])
    A = np.array([[v.sum(), (v * x / L).sum()], [(v * x).sum(), (v * x * x / L).sum()]])
    rhs = np.array([V_slabs_target - v.sum(), Mx_target - (v * x).sum()])
    a, b = np.linalg.solve(A, rhs)
    scale = 1 + a + b * x / L
    v_fit = v * scale
    prims = []
    for i, s in enumerate(slabs):
        r = math.sqrt(v_fit[i] / (math.pi * s["l"]))
        prims.append({"name": "hull%02d" % i, "type": "cylinder", "axis": "x", "r": float(r), "l": float(s["l"]), "pos": [float(s["x"]), 0.0, 0.0], "v": float(v_fit[i])})
    prims += fixed
    # z: one common offset of the hull slabs so that the centroid z equals the derived CB z
    Vtot_p = sum(p["v"] for p in prims)
    zc = sum(p["v"] * p["pos"][2] for p in prims) / Vtot_p
    dz = (cb[2] - zc) * Vtot_p / sum(p["v"] for p in prims if p["name"].startswith("hull"))
    for p in prims:
        if p["name"].startswith("hull"):
            p["pos"][2] += dz
    xc = sum(p["v"] * p["pos"][0] for p in prims) / Vtot_p
    zc = sum(p["v"] * p["pos"][2] for p in prims) / Vtot_p
    out_prims = [{k: (round(val, 6) if isinstance(val, float) else ([round(q, 6) for q in val] if isinstance(val, list) else val))
                  for k, val in p.items() if k != "v"} for p in prims]
    # --- rigid body and hydrodynamics
    M = D["mass"]
    I = M["I_cg_ideal"]
    Aam = D["added_mass"]["A"]
    names = ["x", "y", "z", "p", "q", "r"]
    am = {}
    for i in range(6):
        for j in range(i, 6):
            if abs(Aam[i][j]) > 1e-12:
                am[names[i] + names[j]] = round(Aam[i][j], 5)
    hydro = {  # gz Hydrodynamics tag names; values from docs/notes/sea.md 'Damping' (mid estimate, ITTC-1957 fit for surge)
        "xU": -0.218, "xUabsU": -4.455, "yVabsV": -154.78, "zWabsW": -183.98, "kPabsP": -0.574, "mQabsQ": -11.05,
        "nRabsR": -9.304, "yRabsR": 5.285, "nVabsV": 11.246, "zQabsQ": -2.851, "mWabsW": 2.061,
        "yVabsU": -36.77, "nVabsU": 8.278, "kVabsU": 4.618, "zWabsU": -127.63, "mWabsU": 28.93,
    }
    out = {
        "_generated_by": "tools/export_physics.py from config/knucker_derived.json (tools/derive.py); do not edit by hand",
        "fluid": {"temperature_c": D["fluid"]["T_C"], "density": round(D["fluid"]["rho"], 4), "air_density": 1.225,
                  "gravity": D["fluid"]["g"], "source": "Tanaka et al. 2001 Metrologia 38:301; ISA sea level; ISO standard gravity"},
        "rigid_body": {
            "mass": round(M["total"], 5), "cg": [round(c, 6) for c in M["cg"]],
            "inertia_cg": {"ixx": round(I[0][0], 6), "iyy": round(I[1][1], 6), "izz": round(I[2][2], 6),
                           "ixy": round(I[0][1], 6), "ixz": round(I[0][2], 6), "iyz": round(I[1][2], 6)},
            "displaced_volume": round(V_tot, 7), "cb": [round(c, 6) for c in cb],
        },
        "fluid_added_mass": am,
        "hydrodynamics": hydro,
        "buoyancy_primitives": out_prims,
        "buoyancy_check": {"volume_l": round(Vtot_p * 1e3, 4), "target_volume_l": round(V_tot * 1e3, 4),
                           "cb_x": round(xc, 6), "cb_z": round(zc, 6), "target_cb": [round(c, 6) for c in cb],
                           "slab_correction_a": round(float(a), 5), "slab_correction_b": round(float(b), 5)},
    }
    path = os.path.join(PKG, "config", "knucker_physics.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None, width=140)
    print("wrote", path)
    print(json.dumps(out["buoyancy_check"]))
    print("min/max slab radius", min(p["r"] for p in prims[:N_SLABS]), max(p["r"] for p in prims[:N_SLABS]))


if __name__ == "__main__":
    main()
