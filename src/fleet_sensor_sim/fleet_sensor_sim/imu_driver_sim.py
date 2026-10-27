"""
Reproduce microstrain_inertial_driver outputs from a raw gz IMU message.

imu/data_raw: angular velocity and linear acceleration (gz sensor with the catalog noise model);
              orientation unknown, so orientation_covariance[0] = -1 (REP-145)
imu/data:     the same rates plus an ENU orientation = ground truth times one fixed error
              drawn per run from the catalog static accuracy (1 sigma); covariances from the
              catalog (REP-145)
"""
import math

from fleet_sensor_sim import catalog
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Imu


def quat_mul(a, b):
    """Hamilton product of quaternions (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_from_rpy(r, p, y):
    """Quaternion (x, y, z, w) of the fixed-axis roll, pitch, yaw rotation (REP-103)."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def diag_cov(d):
    """Row-major 3x3 covariance from its diagonal."""
    return [float(d[0]), 0.0, 0.0, 0.0, float(d[1]), 0.0, 0.0, 0.0, float(d[2])]


class ImuDriverSim(Node):
    """gz/imu -> imu/data_raw, imu/data."""

    def __init__(self):
        super().__init__('imu_driver_sim')
        self.declare_parameter('component', 'microstrain_3dm_cv7_ahrs')
        self.declare_parameter('catalog_dir', '')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('seed', -1)
        entry = catalog.load('imu', self.get_parameter('component').value,
                             self.get_parameter('catalog_dir').value)
        sim = entry['sim']
        self.frame_id = (self.get_parameter('frame_id').value
                         or entry['ros_driver']['frame_id'])
        seed = int(self.get_parameter('seed').value)
        rng = np.random.default_rng(None if seed < 0 else seed)
        sigma = sim['orientation_error_stddev_rad']
        self.offset = quat_from_rpy(*(rng.normal(0.0, sigma[i]) for i in range(3)))
        self.cov_o = diag_cov(sim['orientation_covariance_diag'])
        self.cov_w = diag_cov(sim['angular_velocity_covariance_diag'])
        self.cov_a = diag_cov(sim['linear_acceleration_covariance_diag'])
        qos = QoSProfile(depth=10)
        self.pub_raw = self.create_publisher(Imu, 'imu/data_raw', qos)
        self.pub = self.create_publisher(Imu, 'imu/data', qos)
        self.create_subscription(Imu, 'gz/imu', self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        raw = Imu()
        raw.header.stamp = msg.header.stamp
        raw.header.frame_id = self.frame_id
        raw.angular_velocity = msg.angular_velocity
        raw.linear_acceleration = msg.linear_acceleration
        raw.orientation_covariance[0] = -1.0
        raw.angular_velocity_covariance = self.cov_w
        raw.linear_acceleration_covariance = self.cov_a
        self.pub_raw.publish(raw)

        out = Imu()
        out.header = raw.header
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        o = msg.orientation
        q = quat_mul((o.x, o.y, o.z, o.w), self.offset)
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q
        out.orientation_covariance = self.cov_o
        out.angular_velocity_covariance = self.cov_w
        out.linear_acceleration_covariance = self.cov_a
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriverSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
