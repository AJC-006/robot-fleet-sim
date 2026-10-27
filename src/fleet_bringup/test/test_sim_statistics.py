"""
Short headless run of one wheeled robot: sensor output statistics against the catalog entries.

Set FLEET_SIM_TESTS=0 to skip (no Gazebo available). ROS_DOMAIN_ID and GZ_PARTITION are taken
from the environment; FLEET_SIM_LAUNCH_PREFIX (e.g. a simulation-slot wrapper) is prepended to
the launch command.
"""
import math
import os
import shlex
import signal
import subprocess
import time

from fleet_sensor_sim import catalog
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

ROBOT = 'round'
LIDAR, IMU, CAMERA = 'slamtec_rplidar_c1', 'microstrain_3dm_cv7_ahrs', 'intel_realsense_d435'
COLLECT_S = 15.0


def _stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def _rate(msgs):
    t = np.array([_stamp(m) for m in msgs])
    return (len(t) - 1) / (t[-1] - t[0])


@pytest.fixture(scope='module')
def data():
    if os.environ.get('FLEET_SIM_TESTS', '1') == '0':
        pytest.skip('FLEET_SIM_TESTS=0')
    os.environ.setdefault('ROS_DOMAIN_ID', '91')
    os.environ.setdefault('GZ_PARTITION', 'ground')
    cmd = shlex.split(os.environ.get('FLEET_SIM_LAUNCH_PREFIX', '')) + [
        'ros2', 'launch', 'fleet_bringup', 'sim.launch.py', f'robot:={ROBOT}', 'world:=lab',
        'nav:=false', 'gui:=false', 'rviz:=false', 'camera:=true']
    proc = subprocess.Popen(cmd, start_new_session=True, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('sensor_statistics_test')
    msgs = {}
    topics = (('scan', LaserScan, 'scan'), ('raw', Imu, 'imu/data_raw'), ('imu', Imu, 'imu/data'),
              ('depth', Image, 'camera/camera/depth/image_rect_raw'),
              ('color', Image, 'camera/camera/color/image_raw'),
              ('dinfo', CameraInfo, 'camera/camera/depth/camera_info'),
              ('cinfo', CameraInfo, 'camera/camera/color/camera_info'))
    for key, typ, topic in topics:
        node.create_subscription(typ, topic, lambda m, k=key: msgs.setdefault(k, []).append(m),
                                 QoSProfile(depth=10))
    try:
        # wait for the sensors, then discard the start-up transient and collect a window
        deadline = time.time() + 150
        while time.time() < deadline and len(msgs.get('depth', [])) < 3:
            rclpy.spin_once(node, timeout_sec=0.2)
        assert len(msgs.get('depth', [])) >= 3, 'sensors never came up'
        msgs.clear()
        end = time.time() + COLLECT_S
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        yield msgs
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)


def test_lidar_matches_sllidar_c1(data):
    cat = catalog.load('lidar', LIDAR)
    scans = data['scan']
    s = scans[0]
    assert s.header.frame_id == cat['ros_driver']['frame_id']
    assert len(s.ranges) == cat['sim']['output_bins']
    assert abs(_rate(scans) - cat['ros_driver']['rate_hz']) < 0.3
    assert abs(s.angle_min + math.pi) < 1e-6 and abs(s.angle_max - math.pi) < 1e-6
    assert abs(s.range_min - cat['sim']['range_min']) < 1e-6
    assert abs(s.range_max - cat['sim']['range_max']) < 1e-6
    r = np.array([m.ranges for m in scans])
    finite = np.isfinite(r)
    assert finite.any() and r[finite].max() <= cat['sim']['range_max']
    assert r[finite].min() >= cat['sim']['range_min']
    stable = finite.all(axis=0)
    sigma = np.median(r[:, stable].std(axis=0))
    assert abs(sigma - cat['sim']['range_noise_stddev']) < 0.0015, sigma


def test_imu_matches_cv7(data):
    cat = catalog.load('imu', IMU)['sim']
    for key, orientation_cov0 in (('raw', -1.0), ('imu', cat['orientation_covariance_diag'][0])):
        m = data[key]
        assert m[0].header.frame_id == 'imu_link'
        assert abs(_rate(m) - cat['update_rate']) < 1.0
        a = np.array([[x.linear_acceleration.x, x.linear_acceleration.y, x.linear_acceleration.z]
                      for x in m])
        w = np.array([[x.angular_velocity.x, x.angular_velocity.y, x.angular_velocity.z]
                      for x in m])
        assert np.allclose(a.std(axis=0), cat['accel_noise_stddev'], rtol=0.15)
        assert np.allclose(w.std(axis=0), cat['gyro_noise_stddev'], rtol=0.15)
        assert abs(np.linalg.norm(a.mean(axis=0)) - 9.80665) < 0.02
        assert m[0].orientation_covariance[0] == pytest.approx(orientation_cov0)
        assert m[0].linear_acceleration_covariance[0] == pytest.approx(
            cat['linear_acceleration_covariance_diag'][0])
    q = data['imu'][-1].orientation
    assert abs(math.hypot(q.x, q.y, q.z, q.w) - 1.0) < 1e-6
    # spawn yaw 0: heading error is within 5 sigma of the static accuracy
    assert abs(2 * math.atan2(q.z, q.w)) < 5 * cat['orientation_error_stddev_rad'][2]


def test_depth_camera_matches_d435(data):
    cat = catalog.load('depth_camera', CAMERA)
    sim = cat['sim']
    d, c = data['depth'][0], data['color'][0]
    assert d.header.frame_id == cat['ros_driver']['frame_id']['depth']
    assert c.header.frame_id == cat['ros_driver']['frame_id']['color']
    assert (d.encoding, d.width, d.height) == (
        '16UC1', sim['depth']['width'], sim['depth']['height'])
    assert (c.encoding, c.width, c.height) == (
        'rgb8', sim['color']['width'], sim['color']['height'])
    assert abs(data['dinfo'][0].k[0] - sim['depth_fx']) < 0.05
    assert abs(data['cinfo'][0].k[0] - sim['color_fx']) < 0.05
    z = np.array([np.frombuffer(bytes(m.data), np.uint16) for m in data['depth']]).astype(float)
    valid = (z > 0).all(axis=0)
    assert valid.any()
    assert z[z > 0].min() >= sim['depth_clip_near'] * 1000 - 1
    assert z.max() <= sim['depth_clip_far'] * 1000
    # static scene: the temporal std is the noise, compare with sigma_z(z) = coeff z^2
    coeff = sim['depth_noise_subpixel'] / (sim['depth_fx'] * cat['specs']['baseline'])
    zm = z[:, valid].mean(axis=0) / 1000.0
    sigma = z[:, valid].std(axis=0) / 1000.0
    ratio = np.median(sigma / (coeff * zm ** 2))
    assert 0.8 < ratio < 1.2, ratio
