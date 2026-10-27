#!/usr/bin/env python3
"""Mass, centre of mass and inertia of every ground robot (G3), written to
src/fleet_description/config/mass_properties.yaml (read by the xacro; a test fails when it is stale).

    python3 tools/mass_properties.py            # rewrite the yaml
    python3 tools/mass_properties.py --check    # exit 1 when the committed yaml differs

Method (docs/notes/ground.md, G3):
  components  catalog mass and outline (datasheet) placed at the mounting pose of robots.yaml:
              boxes for batteries, drivers, servos, sensors; a solid cylinder along the axle for each wheel motor.
  structure   wheeled: every mesh part the Blender builder creates (blender/robots/part_stats/*.json, made by
              tools/export_part_stats.py) is a thin shell (area x wall x density) or a solid (closed volume x
              density) of the material class in config/structure_materials.yaml; second moments are the exact
              triangle sums stored in the json.
              legged: no per-part mesh statistics exist yet, so the structure is built from the parameters of
              robots.yaml: body = closed box shell of body.inertia_box (head, tail and wings of the visual are
              NOT in it), each link = aluminium tube between its joints, the foot = rubber sphere shell.
  assembly    parallel-axis sum about the common centre of mass (I_o = sum I_i + m_i(|c_i|^2 1 - c_i c_i^T)).
Not in the budget (unpublished or no product selected): single-board computer, wiring, fasteners, PCBs,
battery holder and BMS (a note in each robot's `not_budgeted`).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
DESC = REPO / "src" / "fleet_description" / "config"
CAT = REPO / "src" / "fleet_components" / "components"
STATS = REPO / "blender" / "robots" / "part_stats"
OUT = DESC / "mass_properties.yaml"
G = 9.80665
NOT_BUDGETED = ("single-board computer, wiring, fasteners, PCBs, battery holder and BMS: unpublished or no "
                "product selected (the real robot is heavier by their mass)")


def load(p):
    with open(p) as f:
        return yaml.safe_load(f)


def cat(category, name):
    return load(CAT / category / f"{name}.yaml")


class Acc:
    """Accumulates mass, first and second moments about the frame origin."""

    def __init__(self):
        self.m = 0.0
        self.mc = np.zeros(3)
        self.Io = np.zeros((3, 3))
        self.items = []

    def add(self, name, m, c, I_com, source=""):
        c = np.asarray(c, float)
        self.m += m
        self.mc += m * c
        self.Io += np.asarray(I_com, float) + m * (c @ c * np.eye(3) - np.outer(c, c))
        self.items.append({"item": name, "mass": round(m, 5), "source": source})

    def add_moment(self, name, m, c, S_over_m_shape, source=""):
        """Part with second-moment matrix about the origin given as inertia tensor Io of the part."""
        self.m += m
        self.mc += m * np.asarray(c, float)
        self.Io += S_over_m_shape
        self.items.append({"item": name, "mass": round(m, 5), "source": source})

    def result(self):
        com = self.mc / self.m
        I = self.Io - self.m * (com @ com * np.eye(3) - np.outer(com, com))
        return com, I


def box_I(m, x, y, z):
    return np.diag([m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y)])


def cyl_y_I(m, r, h):
    """Solid cylinder with its axis along y."""
    ia = m * r * r / 2
    it = m / 12 * (3 * r * r + h * h)
    return np.diag([it, ia, it])


def inertia_dict(I):
    return {"ixx": float(I[0, 0]), "iyy": float(I[1, 1]), "izz": float(I[2, 2]),
            "ixy": float(I[0, 1]), "ixz": float(I[0, 2]), "iyz": float(I[1, 2])}


def r6(x):
    if isinstance(x, dict):
        return {k: r6(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, np.ndarray)):
        return [r6(v) for v in x]
    if isinstance(x, (float, np.floating)):
        return float(f"{x:.6g}")
    return x


def sensor_size(entry):
    sp = entry["specs"]
    return sp.get("size") or sp["size_estimate"]


def mat_of(part, mats):
    name = part["name"].split(".")[0]
    cls = mats["part_class"].get(name)
    if cls is None:
        cls = mats["material_class"][part["materials"][0]]
    return cls


def structure_part(acc, part, cls, mats):
    """Add one mesh part (shell or solid) to acc; returns its mass."""
    mat = mats["materials"][cls]
    rho = mat["density"]
    if mat["mode"] == "shell":
        k = rho * mat["thickness"]
        m = k * part["area"]
        c = part["area_centroid"]
        S = np.array(part["area_second_moment"]) * k
    else:
        k = rho
        m = k * part["volume"]
        c = part["volume_centroid"]
        S = np.array(part["volume_second_moment"]) * k
    Io = np.trace(S) * np.eye(3) - S
    acc.add_moment(f"{part['name']} ({cls})", m, c, Io, mat["source"])
    return m


def wheeled(name, rc, mats):
    comp = rc["components"]
    r = rc["wheel_radius"]
    w = rc["wheel_width"]
    sep = rc["wheel_separation"]
    stats = json.load(open(STATS / f"{name}.json"))
    motor = cat("dc_motor", comp["wheel_motor"])["specs"]
    driver = cat("motor_driver", comp["motor_driver"])["specs"]
    cell = cat("battery", comp["battery_cell"])["specs"]
    pack = rc["battery_pack"]
    mounts = rc["mounts"]
    n_wheels = 2 * len(rc["wheel_x"])
    acc = Acc()
    balls = []
    for part in stats["parts"]:
        pname = part["name"].split(".")[0]
        if mats["part_class"].get(pname) == "exclude":
            continue
        if pname == "ball":
            balls.append(part)
            continue
        structure_part(acc, part, mat_of(part, mats), mats)
    # wheel motors: solid cylinder along the axle, inboard of the tyre
    ml = motor["body_length"]
    for x in rc["wheel_x"]:
        for side in (1, -1):
            y = side * (sep / 2 - w / 2 - ml / 2)
            acc.add("wheel motor " + comp["wheel_motor"], motor["mass"], [x, y, 0.0],
                    cyl_y_I(motor["mass"], motor["motor_diameter"] / 2, ml), "catalog dc_motor")
    n_drv = math.ceil(n_wheels / 2)          # 2 channels per RoboClaw
    dx, dy, dz = driver["size"]
    for i in range(n_drv):
        c = np.array(mounts["driver"]) + np.array([0.08 * (i - (n_drv - 1) / 2), 0.0, 0.0])
        acc.add("motor driver " + comp["motor_driver"], driver["mass"], c,
                box_inertia(driver["mass"], dx, dy, dz), "catalog motor_driver")
    ncell = pack["series"] * pack["parallel"]
    bm = ncell * cell["mass_max"]
    acc.add(f"battery {pack['series']}S{pack['parallel']}P {comp['battery_cell']}", bm, mounts["battery"],
            box_inertia(bm, cell["length"], pack["series"] * cell["diameter"], pack["parallel"] * cell["diameter"]),
            "catalog battery (cell mass_max x cells)")
    base_com, base_I = acc.result()
    base_m = acc.m
    # wheel link (mesh about its axle, one tyre-rubber shell; hub parts are not separated in the mesh)
    ws = stats["wheel"]
    rub = mats["materials"]["tyre_rubber"]
    k = rub["density"] * rub["thickness"]
    wm = k * ws["area"]
    S = np.array(ws["area_second_moment"]) * k
    wI = np.trace(S) * np.eye(3) - S
    # caster: solid steel ball
    caster = None
    if balls:
        bm_ = np.mean([structure_ball(b, mats) for b in balls])
        rad = rc["casters"][0]["radius"]
        caster = {"mass": float(bm_), "inertia": float(0.4 * bm_ * rad * rad)}
    # sensors (separate links, catalog mass)
    sens = {}
    mass_total = base_m + n_wheels * wm
    if caster:
        mass_total += len(rc["casters"]) * caster["mass"]
    for role, cat_name in (("lidar", "lidar"), ("imu", "imu"), ("camera", "depth_camera")):
        e = cat(cat_name, comp[role])
        mass_total += e["specs"]["mass"]
        sens[role] = {"component": comp[role], "mass": e["specs"]["mass"], "size": sensor_size(e)}
    items = acc.items + [{"item": f"{n_wheels} wheels (tyre rubber shell)", "mass": round(n_wheels * wm, 5),
                          "source": "structure_materials.yaml tyre_rubber"}]
    for role, s in sens.items():
        items.append({"item": f"{role} {s['component']}", "mass": s["mass"], "source": "catalog"})
    if caster:
        items.append({"item": f"{len(rc['casters'])} caster balls (steel solid)",
                      "mass": round(len(rc["casters"]) * caster["mass"], 5), "source": "structure_materials.yaml"})
    return {
        "total_mass": mass_total,
        "base_link": {"mass": base_m, "com": base_com, "inertia": inertia_dict(base_I)},
        "wheel": {"mass": wm, "inertia": inertia_dict(wI)},
        **({"caster": caster} if caster else {}),
        "sensors": sens,
        "not_budgeted": NOT_BUDGETED,
        "budget": items,
    }


def structure_ball(b, mats):
    return mats["materials"]["steel_solid"]["density"] * b["volume"]


def box_inertia(m, x, y, z):
    return box_I(m, x, y, z)


def tube_I(m, r, L):
    """Thin-wall tube along x about its centre: Ixx = m r^2, Iyy = Izz = m (r^2/2 + L^2/12)."""
    return np.diag([m * r * r, m * (r * r / 2 + L * L / 12), m * (r * r / 2 + L * L / 12)])


def legged(name, rc, mats):
    comp = rc["components"]
    lk = rc["link"]
    cell = cat("battery", comp["battery_cell"])["specs"]
    pack = rc["battery_pack"]
    act = {j: cat("servo", comp["joint_actuators"][j])["specs"] for j in ("coxa", "femur", "tibia")}
    al = mats["materials"]["aluminium_tube"]
    abs_ = mats["materials"]["abs_housing"]
    rub = mats["materials"]["tyre_rubber"]
    base_z = float(rc["standing_height"])
    bx, by, bz = rc["body"]["inertia_box"]
    bc = rc["body"]["com"]
    acc = Acc()
    area = 2 * (bx * by + by * bz + bx * bz)
    mb = area * abs_["thickness"] * abs_["density"]
    # closed box shell: faces as thin plates about the centre
    Ib = box_shell_I(mb, bx, by, bz)
    acc.add(f"body shell box {bx}x{by}x{bz} ({abs_['source']})", mb, bc, Ib, "structure_materials.yaml")
    ncell = pack["series"] * pack["parallel"]
    bm = ncell * cell["mass_max"]
    acc.add(f"battery {pack['series']}S{pack['parallel']}P", bm, rc["mounts"]["battery"],
            box_I(bm, cell["length"], pack["series"] * cell["diameter"], pack["parallel"] * cell["diameter"]),
            "catalog battery")
    legs = rc["legs"]
    sc = act["coxa"]
    for L in legs:
        acc.add(f"coxa servo {L['name']}", sc["mass"], L["hip"], box_I(sc["mass"], *sc["size"]), "catalog servo")
    base_com, base_I = acc.result()
    base_m = acc.m

    def tube(length, radius):
        m = 2 * math.pi * radius * length * al["thickness"] * al["density"]
        return m

    def link_totals(parts):
        """parts: list of (mass, com_x, I_com 3x3 diag) along x; returns mass, com_x, Ixx, Iyy, Izz."""
        m = sum(p[0] for p in parts)
        cx = sum(p[0] * p[1] for p in parts) / m
        Ixx = Iyy = Izz = 0.0
        for pm, px, pI in parts:
            d = px - cx
            Ixx += pI[0, 0]
            Iyy += pI[1, 1] + pm * d * d
            Izz += pI[2, 2] + pm * d * d
        return m, cx, Ixx, Iyy, Izz

    out_legs = {}
    items = list(acc.items)
    lc, l1, l2 = lk["coxa"], lk["femur"], lk["tibia"]
    # coxa link: structure tube (length lc) + the femur servo at the end (femur joint)
    s1 = act["femur"]
    mt = tube(lc, lk["coxa_radius"])
    parts = [(mt, lc / 2, tube_I(mt, lk["coxa_radius"], lc)),
             (s1["mass"], lc, box_I(s1["mass"], *s1["size"]))]
    out_legs["coxa"] = leg_dict(link_totals(parts))
    # femur link: tube + tibia servo at the tibia joint
    s2 = act["tibia"]
    mt = tube(l1, lk["femur_radius"])
    parts = [(mt, l1 / 2, tube_I(mt, lk["femur_radius"], l1)),
             (s2["mass"], l1, box_I(s2["mass"], *s2["size"]))]
    out_legs["femur"] = leg_dict(link_totals(parts))
    # tibia link: tube + rubber foot sphere shell
    rf = lk["foot_radius"]
    mt = tube(l2 - rf, lk["tibia_radius"])
    mf = 4 * math.pi * rf * rf * rub["thickness"] * rub["density"]
    parts = [(mt, (l2 - rf) / 2, tube_I(mt, lk["tibia_radius"], l2 - rf)),
             (mf, l2, np.diag([2 / 3 * mf * rf * rf] * 3))]
    out_legs["tibia"] = leg_dict(link_totals(parts))
    n = len(legs)
    leg_mass = sum(out_legs[j]["mass"] for j in out_legs)
    items += [{"item": f"{n} x femur servo {comp['joint_actuators']['femur']} (in coxa links)",
               "mass": round(n * s1["mass"], 5), "source": "catalog servo"},
              {"item": f"{n} x tibia servo {comp['joint_actuators']['tibia']} (in femur links)",
               "mass": round(n * s2["mass"], 5), "source": "catalog servo"},
              {"item": f"{n} x leg tubes + foot caps (aluminium_tube, tyre_rubber)",
               "mass": round(n * (leg_mass - s1["mass"] - s2["mass"]), 5), "source": "structure_materials.yaml"}]
    sens = {}
    mass_total = base_m + n * leg_mass
    for role, cat_name in (("lidar", "lidar"), ("imu", "imu"), ("camera", "depth_camera")):
        e = cat(cat_name, comp[role])
        mass_total += e["specs"]["mass"]
        sens[role] = {"component": comp[role], "mass": e["specs"]["mass"], "size": sensor_size(e)}
        items.append({"item": f"{role} {comp[role]}", "mass": e["specs"]["mass"], "source": "catalog"})
    return {
        "total_mass": mass_total,
        "base_link": {"mass": base_m, "com": base_com, "inertia": inertia_dict(base_I)},
        "leg_links": out_legs,
        "sensors": sens,
        "not_budgeted": NOT_BUDGETED + "; the visual head, tail and wings of the body are not in the shell box",
        "budget": items,
    }


def box_shell_I(m, x, y, z):
    """Closed thin box shell about its centre: six plates of equal areal density."""
    dims = [x, y, z]
    sig = m / (2 * (x * y + y * z + x * z))
    I = np.zeros(3)
    for ax in range(3):                      # two plates normal to axis `ax`, at +-dims[ax]/2
        u, v = [i for i in range(3) if i != ax]
        mp = sig * dims[u] * dims[v]
        shift = mp * (dims[ax] / 2) ** 2
        I[ax] += 2 * mp / 12 * (dims[u] ** 2 + dims[v] ** 2)
        I[u] += 2 * (mp / 12 * dims[v] ** 2 + shift)
        I[v] += 2 * (mp / 12 * dims[u] ** 2 + shift)
    return np.diag(I)


def leg_dict(t):
    m, cx, Ixx, Iyy, Izz = t
    return {"mass": m, "com_x": cx, "ixx": Ixx, "iyy": Iyy, "izz": Izz}


def generate():
    robots = load(DESC / "robots.yaml")
    mats = load(DESC / "structure_materials.yaml")
    out = {}
    for name, rc in robots.items():
        if rc.get("kind", "wheeled") == "legged":
            out[name] = legged(name, rc, mats)
        else:
            out[name] = wheeled(name, rc, mats)
    return r6(out)


HEADER = ("# GENERATED by tools/mass_properties.py from robots.yaml, structure_materials.yaml, the catalog and\n"
          "# blender/robots/part_stats/*.json. Do not edit; rerun the tool (test_mass_properties fails when stale).\n"
          "# Frames: base_link = robots.yaml convention (axle height for wheeled, standing height for legged);\n"
          "# inertia about the centre of mass in base_link axes; leg link values in the link frame (x along the link).\n")


def dump(data):
    return HEADER + yaml.safe_dump(data, sort_keys=False, width=110)


if __name__ == "__main__":
    text = dump(generate())
    if "--check" in sys.argv:
        sys.exit(0 if OUT.exists() and OUT.read_text() == text else 1)
    OUT.write_text(text)
    for k, v in load(OUT).items():
        print(f"{k:7s} total {v['total_mass']:.3f} kg, base_link {v['base_link']['mass']:.3f} kg, "
              f"com {v['base_link']['com']}")
