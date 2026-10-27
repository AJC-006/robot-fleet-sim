"""Checks that the wyvern model.sdf carries the catalog-derived constants and that the derivations hold.

Run: python3 -m pytest src/fleet_drone/test/test_air_catalog.py   (no simulator needed)
"""
import csv
import math
import os
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
CAT = os.path.join(ROOT, "src", "fleet_components", "components")
SDF = os.path.join(ROOT, "src", "fleet_drone", "models", "wyvern", "model.sdf")


def cat(p):
    with open(os.path.join(CAT, p)) as fh:
        return yaml.safe_load(fh)


def test_motor_constants_from_table():
    rows = list(csv.reader(l for l in open(os.path.join(CAT, "propeller", "tmotor_t1045_air2216ii_kv920_16v.csv"))
                           if not l.startswith("#")))[1:]
    d = [[float(x) for x in r] for r in rows]
    w = [r[4] * 2 * math.pi / 60 for r in d]
    T = [r[1] * 9.80665e-3 for r in d]
    kT = sum(t * x * x for t, x in zip(T, w)) / sum(x ** 4 for x in w)
    sim = cat("propeller/tmotor_t1045.yaml")["sim"]
    assert abs(kT - sim["motor_constant"]) / kT < 0.005
    assert abs(w[-1] - sim["max_rot_velocity"]) / w[-1] < 0.005
    # 2.0 kg vehicle, 4 motors: hover speed below max and command in a sane range
    w_hover = math.sqrt(2.0 * 9.80665 / 4 / kT)
    assert 0.5 < (w_hover - 150) / (sim["max_rot_velocity"] - 150) < 0.65


def test_sdf_matches_catalog():
    m = ET.parse(SDF).getroot().find("model")
    sim = cat("propeller/tmotor_t1045.yaml")["sim"]
    tc = cat("bldc_motor/tmotor_air2216ii_kv920.yaml")["sim"]
    plugs = [p for p in m.findall("plugin") if p.get("name").endswith("MulticopterMotorModel")]
    assert len(plugs) == 4
    for p in plugs:
        assert math.isclose(float(p.findtext("motorConstant")), sim["motor_constant"], rel_tol=1e-6)
        assert math.isclose(float(p.findtext("maxRotVelocity")), sim["max_rot_velocity"], rel_tol=1e-6)
        assert math.isclose(float(p.findtext("momentConstant")), sim["moment_constant"], rel_tol=1e-6)
        assert math.isclose(float(p.findtext("timeConstantUp")), tc["time_constant_up"], rel_tol=1e-6)


def test_imu_noise_formulas():
    c = cat("fc_imu/tdk_icm_42688_p.yaml")
    sp, sm = c["specs"], c["sim"]
    rate = sm["update_rate"]
    assert math.isclose(sm["gyro_noise_stddev"], sp["gyro_noise_density"] * math.sqrt(rate), rel_tol=2e-3)
    assert math.isclose(sm["accel_noise_stddev_xy"], sp["accel_noise_density_xy"] * math.sqrt(rate), rel_tol=2e-3)
    assert math.isclose(sm["accel_noise_stddev_z"], sp["accel_noise_density_z"] * math.sqrt(rate), rel_tol=2e-3)
    assert math.isclose(sm["gyro_bias_stddev"], sp["gyro_zro_initial"] / 3, rel_tol=2e-3)
    imu = ET.parse(SDF).getroot().find("model").find("link[@name='base_link']").find("sensor[@name='imu_sensor']")
    n = imu.find("imu/angular_velocity/x/noise")
    assert math.isclose(float(n.findtext("stddev")), sm["gyro_noise_stddev"], rel_tol=1e-6)
    assert math.isclose(float(n.findtext("dynamic_bias_stddev")), sm["gyro_dynamic_bias_stddev"], rel_tol=1e-6)


def test_gnss_sigma_from_cep():
    g = cat("gnss/ublox_max_m10s.yaml")
    assert math.isclose(g["sim"]["horizontal_stddev_m"], g["specs"]["position_accuracy_cep"] / 1.1774, rel_tol=2e-3)
    assert math.isclose(g["sim"]["vertical_stddev_m"], 1.5 * g["sim"]["horizontal_stddev_m"], rel_tol=2e-3)
