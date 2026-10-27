"""Driver-compatible sensor nodes. Each reproduces the topics, frames, rates, QoS and covariances of the real
component's ROS 2 driver (catalog entries in fleet_components name the driver and every number used here).

  waterlinked_dvl_driver  Water Linked DVL A50   ~/velocity_report ~/odom ~/dead_reckoning_report   frame dvl_link (FRD)
  microstrain_imu         MicroStrain 3DM-CV7    imu/data imu/data_raw imu/mag                       frame imu_link (ENU/FLU)
  bar30_driver            Blue Robotics Bar30    pressure temperature depth_pose                     frame pressure_link

Inputs are the raw simulator streams bridged from Gazebo (dvl_gz, imu_gz, odometry_gt); only the Bar30 uses the
simulated truth pose (there is no Gazebo pressure sensor), exactly as its model: p = p_atm + rho g depth.
"""
import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu, MagneticField, Temperature

from . import catalog
from .shape import load_cfg

try:
    from marine_acoustic_msgs.msg import Dvl
except ImportError:   # pragma: no cover
    Dvl = None


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def rpy_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class WaterlinkedDvl(Node):
    # beam_unit_vec as published by waterlinked_dvl_driver (catalog dvl/waterlinked_dvl_a50.yaml, limitations)
    BEAMS = ((0.6532815, 0.6532815, 0.3826834), (-0.6532815, 0.6532815, 0.3826834),
             (-0.6532815, -0.6532815, 0.3826834), (0.6532815, -0.6532815, 0.3826834))

    def __init__(self):
        super().__init__("waterlinked_dvl_driver")
        cfg = load_cfg()
        spec = catalog.load("dvl", cfg["components"]["dvl"])["specs"]
        self.declare_parameter("frame_id", "dvl_link")
        self.declare_parameter("speed_of_sound", 1481.0)
        self.declare_parameter("velocity_stddev", float(cfg["sensors"]["dvl"]["velocity_noise"]))
        self.declare_parameter("scale_error", 0.0)        # fractional, constant per run, |e| <= 0.0101 (A50 spec)
        self.frame = self.get_parameter("frame_id").value
        self.c = float(self.get_parameter("speed_of_sound").value)
        self.sd = float(self.get_parameter("velocity_stddev").value)
        self.scale = 1.0 + float(self.get_parameter("scale_error").value)
        self.res = spec["velocity_resolution"]
        self.alt_min, self.alt_max = spec["altitude_min"], spec["altitude_max"]
        self.vmax = 3.75
        self.pub_v = self.create_publisher(Dvl, "~/velocity_report", 10)
        self.pub_o = self.create_publisher(Odometry, "~/odom", 10)
        self.pub_dr = self.create_publisher(PoseWithCovarianceStamped, "~/dead_reckoning_report", 10)
        self.sub = self.create_subscription(Dvl, "dvl_gz/velocity", self.cb, 10)
        self.sub_i = self.create_subscription(Imu, "imu_gz", self.cb_imu, qos_profile_sensor_data)
        self.R = np.eye(3)
        self.pos = np.zeros(3)
        self.t_prev = None

    def cb_imu(self, m):
        o = m.orientation
        self.R = quat_to_R((o.x, o.y, o.z, o.w))

    def cb(self, m):
        v = np.array([m.velocity.x, m.velocity.y, m.velocity.z]) * self.scale
        v = np.round(v / self.res) * self.res
        alt = m.altitude
        valid = self.alt_min <= alt <= self.alt_max and np.linalg.norm(v) <= self.vmax
        out = Dvl()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self.frame
        out.velocity_mode = Dvl.DVL_MODE_BOTTOM
        out.dvl_type = Dvl.DVL_TYPE_PHASED_ARRAY
        out.sound_speed = float(self.c)
        out.beam_ranges_valid = bool(valid)
        out.beam_velocities_valid = bool(valid)
        out.num_good_beams = 4 if valid else 0
        cov = [0.0] * 9
        for i in (0, 4, 8):
            cov[i] = self.sd ** 2
        out.velocity_covar = cov
        if valid:
            out.velocity.x, out.velocity.y, out.velocity.z = (float(x) for x in v)
            out.altitude = float(alt)
        else:
            out.altitude = float("nan")
        for i, b in enumerate(self.BEAMS):
            out.beam_unit_vec[i].x, out.beam_unit_vec[i].y, out.beam_unit_vec[i].z = b
        self.pub_v.publish(out)
        od = Odometry()
        od.header = out.header
        od.child_frame_id = self.frame
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = (float(x) for x in v)
        c6 = [0.0] * 36
        for i in range(3):
            c6[i * 7] = self.sd ** 2
        for i in range(3, 6):
            c6[i * 7] = -1.0     # rotation not measured
        od.twist.covariance = c6
        od.pose.covariance = [-1.0] * 36 if not valid else od.pose.covariance
        if valid:
            self.pub_o.publish(od)
            self._dead_reckon(m.header.stamp, v)

    def _dead_reckon(self, stamp, v):
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.t_prev is not None and 0 < t - self.t_prev < 1.0:
            # world-frame displacement; orientation from imu_gz stands in for the A50's internal AHRS
            self.pos += (self.R @ (np.diag([1, -1, -1]) @ v)) * (t - self.t_prev)
        self.t_prev = t
        p = PoseWithCovarianceStamped()
        p.header.stamp = stamp
        p.header.frame_id = "odom"
        p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z = (float(x) for x in self.pos)
        p.pose.pose.orientation.w = 1.0
        self.pub_dr.publish(p)


class MicrostrainImu(Node):
    def __init__(self):
        super().__init__("microstrain_imu")
        cfg = load_cfg()
        sp = catalog.load("ahrs", cfg["components"]["ahrs"])["specs"]
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("imu_orientation_cov", [0.01] * 9)      # microstrain_inertial_driver defaults
        self.declare_parameter("imu_angular_cov", [0.01] * 9)
        self.declare_parameter("imu_linear_cov", [0.01] * 9)
        self.declare_parameter("imu_data_raw_rate", 1.0)
        self.declare_parameter("mag_rate", 0.0)                         # driver default: magnetometer output off
        self.frame = self.get_parameter("frame_id").value
        self.ocov = list(self.get_parameter("imu_orientation_cov").value)
        self.gcov = list(self.get_parameter("imu_angular_cov").value)
        self.acov = list(self.get_parameter("imu_linear_cov").value)
        # Orientation error: constant per run, sigma = static accuracy (spec bound used as 1 sigma; the error
        # process dynamics are unpublished, see docs/notes/sea.md).
        self.err = rpy_quat(random.gauss(0, sp["roll_pitch_accuracy_static"]), random.gauss(0, sp["roll_pitch_accuracy_static"]),
                            random.gauss(0, sp["heading_accuracy_static"]))
        self.pub = self.create_publisher(Imu, "imu/data", 10)
        self.pub_raw = self.create_publisher(Imu, "imu/data_raw", 10)
        self.rate_raw = float(self.get_parameter("imu_data_raw_rate").value)
        self.t_raw = -1e9
        self.pub_mag = None
        if float(self.get_parameter("mag_rate").value) > 0:
            self.pub_mag = self.create_publisher(MagneticField, "imu/mag", 10)
        self.create_subscription(Imu, "imu_gz", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        o = m.orientation
        q = quat_mul((o.x, o.y, o.z, o.w), self.err)
        out = Imu()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self.frame
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q
        out.angular_velocity = m.angular_velocity
        out.linear_acceleration = m.linear_acceleration
        out.orientation_covariance = self.ocov
        out.angular_velocity_covariance = self.gcov
        out.linear_acceleration_covariance = self.acov
        self.pub.publish(out)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.rate_raw > 0 and t - self.t_raw >= 1.0 / self.rate_raw - 1e-6:
            self.t_raw = t
            raw = Imu()
            raw.header = out.header
            raw.angular_velocity = m.angular_velocity
            raw.linear_acceleration = m.linear_acceleration
            raw.orientation_covariance = [-1.0] + [0.0] * 8       # REP-145: orientation not provided
            raw.angular_velocity_covariance = self.gcov
            raw.linear_acceleration_covariance = self.acov
            self.pub_raw.publish(raw)


class Bar30(Node):
    G = 9.80665
    P_ATM = 101325.0

    def __init__(self):
        super().__init__("bar30_driver")
        cfg = load_cfg()
        sp = catalog.load("pressure", cfg["components"]["pressure"])["specs"]
        self.declare_parameter("frame_id", "pressure_link")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("water_level", 0.0)        # world z of the water surface
        self.declare_parameter("fluid_density", 999.103)  # keep in step with the world's graded buoyancy
        self.declare_parameter("offset_pa", 0.0)          # constant per run, within +/-5000 Pa (MS5837 spec)
        self.frame = self.get_parameter("frame_id").value
        self.surface = float(self.get_parameter("water_level").value)
        self.rho = float(self.get_parameter("fluid_density").value)
        self.off = float(self.get_parameter("offset_pa").value)
        self.sd = sp["resolution_rms_osr8192"]
        self.lsb = 10.0                                    # Pa, 0.1 mbar ADC step of the MS5837-30BA
        self.pos = np.array(cfg["sensors"]["pressure"]["pos"], float)
        self.R = np.eye(3)
        self.p0 = np.zeros(3)
        self.have = False
        self.pub_p = self.create_publisher(FluidPressure, "pressure", qos_profile_sensor_data)
        self.pub_t = self.create_publisher(Temperature, "temperature", qos_profile_sensor_data)
        self.pub_d = self.create_publisher(PoseWithCovarianceStamped, "depth_pose", 10)
        self.create_subscription(Odometry, "odometry_gt", self.cb, 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)

    def cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.p0 = np.array([p.x, p.y, p.z])
        self.R = quat_to_R((q.x, q.y, q.z, q.w))
        self.have = True

    def tick(self):
        if not self.have:
            return
        z = (self.p0 + self.R @ self.pos)[2]
        depth = max(self.surface - z, 0.0)
        pa = self.P_ATM + self.rho * self.G * depth + self.off + random.gauss(0, self.sd)
        pa = round(pa / self.lsb) * self.lsb
        now = self.get_clock().now().to_msg()
        m = FluidPressure()
        m.header.stamp, m.header.frame_id = now, self.frame
        m.fluid_pressure, m.variance = pa, self.sd ** 2
        self.pub_p.publish(m)
        t = Temperature()
        t.header = m.header
        t.temperature = 15.0 + random.gauss(0, 0.0022)     # lake temperature (S1); resolution 0.0022 K
        t.variance = 0.0022 ** 2
        self.pub_t.publish(t)
        # depth derived from the pressure like ArduSub does (surface calibrated), as a z measurement for the EKF
        d = PoseWithCovarianceStamped()
        d.header.stamp, d.header.frame_id = now, "odom"
        # z of base_link in the odom frame (origin at the water surface), lever arm to the sensor assumed level
        d.pose.pose.position.z = -(pa - self.P_ATM - self.off) / (self.rho * self.G) - float(self.pos[2])
        d.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[14] = (self.sd / (self.rho * self.G)) ** 2
        d.pose.covariance = cov
        self.pub_d.publish(d)


def _run(cls):
    rclpy.init()
    n = cls()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


def dvl_main():
    _run(WaterlinkedDvl)


def imu_main():
    _run(MicrostrainImu)


def bar30_main():
    _run(Bar30)
