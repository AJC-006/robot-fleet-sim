"""Unit tests for the model generator and the catalog wiring (no simulator needed)."""
import xml.etree.ElementTree as ET

import numpy as np

from fleet_auv import catalog, model
from fleet_auv.params import load_physics
from fleet_auv.shape import load_cfg


def test_total_mass_cg_inertia_preserved():
    phys, cfg = load_physics(), load_cfg()
    th = cfg["thrusters"]
    mb, cb, Ib = model.base_inertial(phys, th)
    rb = phys["rigid_body"]
    assert abs(mb + model.PROP_MASS * len(th) - rb["mass"]) < 1e-9
    cg = (mb * cb + sum(model.PROP_MASS * np.array(t["pos"]) for t in th.values())) / rb["mass"]
    assert np.allclose(cg, rb["cg"], atol=1e-9)
    I = Ib + mb * model._pa(cb - cg) + sum(model.PROP_MASS * model._pa(np.array(t["pos"]) - cg) for t in th.values())
    i = rb["inertia_cg"]
    ref = np.array([[i["ixx"], i["ixy"], i["ixz"]], [i["ixy"], i["iyy"], i["iyz"]], [i["ixz"], i["iyz"], i["izz"]]])
    assert np.allclose(I, ref, atol=1e-9)


def test_buoyancy_primitives_match_displacement_and_cb():
    phys = load_physics()
    v = 0.0
    mx = mz = 0.0
    for c in phys["buoyancy_primitives"]:
        vol = np.pi * c["r"] ** 2 * c["l"]
        v += vol
        mx += vol * c["pos"][0]
        mz += vol * c["pos"][2]
    # tunnel bores are inside the hull slabs, so the cylinders' sum is the displaced volume up to the decor correction
    assert abs(v - phys["rigid_body"]["displaced_volume"]) < 5e-5
    assert abs(mx / v - phys["rigid_body"]["cb"][0]) < 1e-4       # CB x must match to 0.1 mm


def test_sdf_structure():
    root = ET.fromstring(model.sdf())
    plugins = {p.get("name") for p in root.iter("plugin")}
    assert "fleet_marine::T200Thruster" in plugins and "fleet_marine::LiIonBattery" in plugins
    assert "gz::sim::systems::Hydrodynamics" in plugins
    assert "gz::sim::systems::Thruster" not in plugins            # the generic gz thruster is not used
    assert sum(1 for p in root.iter("plugin") if p.get("name") == "fleet_marine::T200Thruster") == 4
    assert root.find(".//fluid_added_mass") is not None
    txt = model.sdf()
    assert "max_am_ratio" not in txt and "gpu_lidar" not in txt


def test_catalog_references_resolve():
    cfg = load_cfg()
    for cat, stem in cfg["components"].items():
        assert catalog.load(cat, stem)["category"] == cat
    t = cfg["components"]["thruster"]
    assert catalog.path("thruster", t, catalog.load("thruster", t)["specs"]["performance_table"])
    assert "unpublished" not in catalog.load("battery", cfg["components"]["battery"])["sim"].get("ocv_table_per_cell", "unpublished")
