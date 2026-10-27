"""Unit tests of the adapter conversions (no ROS graph)."""
import math

from fleet_sensor_sim import catalog
from fleet_sensor_sim.camera_driver_sim import depth_to_mm, intrinsics
from fleet_sensor_sim.imu_driver_sim import quat_from_rpy, quat_mul
from fleet_sensor_sim.lidar_driver_sim import convert
import numpy as np


def test_lidar_bins_and_reversal():
    n_in = 500
    ranges = np.full(n_in, 2.0)
    out = convert(ranges, -math.pi, 2 * math.pi / n_in, 2, 0.05, 12.0, 0.00025)
    assert len(out) == 720 and np.all(out == 2.0)
    # ray straight ahead: theta 0 -> device 180 deg -> bins 360, 361 -> reversed 358, 359
    ranges = np.full(n_in, np.inf)
    ranges[n_in // 2] = 1.0
    out = convert(ranges, -math.pi, 2 * math.pi / n_in, 2, 0.05, 12.0, 0.00025)
    assert np.isfinite(out).sum() == 2
    assert np.isfinite(out[358]) and np.isfinite(out[359])


def test_lidar_limits_and_quantum():
    out = convert([0.01, 13.0, 1.00011], -math.pi, 2 * math.pi / 3, 1, 0.05, 12.0, 0.00025)
    assert np.isinf(out).sum() == 359
    assert abs(out[out < 9][0] - 1.0) <= 0.000125


def test_catalog_bins_match_multiple():
    for m in ('slamtec_rplidar_c1', 'slamtec_rplidar_s2'):
        s = catalog.load('lidar', m)['sim']
        assert s['output_bins'] == 360 * int(s['samples_per_rev'] / 360 + 1)


def test_intrinsics_d435():
    k = intrinsics(640, 360, 1.211259)
    assert abs(k[0] - 462.14) < 0.05 and k[2] == 320 and k[5] == 180


def test_depth_noise_and_invalid():
    rng = np.random.default_rng(1)
    z = np.full((200, 200), 2.0)
    mm = depth_to_mm(z, 3.581e-3, 0.195, 10.0, rng)
    assert abs(mm.std() - 14.3) < 0.5 and abs(mm.mean() - 2000) < 0.5
    bad = depth_to_mm(np.array([[0.1, np.inf, 11.0]]), 3.581e-3, 0.195, 10.0, rng)
    assert not bad.any()


def test_quaternion_yaw():
    q = quat_mul(quat_from_rpy(0, 0, 0.3), quat_from_rpy(0, 0, 0.2))
    assert abs(2 * math.atan2(q[2], q[3]) - 0.5) < 1e-12
