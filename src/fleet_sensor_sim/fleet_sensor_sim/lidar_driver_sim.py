"""
Reproduce sllidar_ros2 (sllidar_node.cpp, angle_compensate) LaserScan output from a raw gz scan.

Driver behaviour copied (verified in the driver source, see docs/notes/ground.md G3):
  * N = 360 * multiple bins, multiple = int(points_per_circle / 360 + 1)
  * device angle a (deg, clockwise) fills bins int(a * multiple) .. + multiple - 1
  * output reversed (ranges[N - 1 - i]); angle_min -pi, angle_max pi, increment 2 pi / (N - 1)
  * range in 0.25 mm units; no return -> 0 -> +inf
  * QoS KeepLast(10) reliable; stamp = scan start; scan_time = revolution time
"""
import math

from fleet_sensor_sim import catalog
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan


def convert(ranges, angle_min, angle_increment, multiple, range_min, range_max, quantum):
    """Bin gz rays (ROS angle theta) into the driver's reversed, angle-compensated array."""
    n_out = 360 * multiple
    bins = np.zeros(n_out)
    ranges = np.asarray(ranges, dtype=float)
    theta = angle_min + angle_increment * np.arange(len(ranges))
    dev_deg = np.mod(np.degrees(math.pi - theta), 360.0)
    idx = np.minimum((dev_deg * multiple).astype(int), n_out - 1)
    ok = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    dist = np.round(ranges / quantum) * quantum
    for i in np.nonzero(ok)[0]:
        bins[idx[i]:min(idx[i] + multiple, n_out)] = dist[i]
    out = bins[::-1].copy()
    out[out == 0.0] = math.inf
    return out


class LidarDriverSim(Node):
    """gz/scan -> scan."""

    def __init__(self):
        super().__init__('lidar_driver_sim')
        self.declare_parameter('component', 'slamtec_rplidar_c1')
        self.declare_parameter('catalog_dir', '')
        self.declare_parameter('frame_id', '')
        entry = catalog.load('lidar', self.get_parameter('component').value,
                             self.get_parameter('catalog_dir').value)
        sim = entry['sim']
        self.frame_id = (self.get_parameter('frame_id').value
                         or entry['ros_driver']['frame_id'])
        self.multiple = int(sim['angle_compensate_multiple'])
        self.range_min = float(sim['range_min'])
        self.range_max = float(sim['range_max'])
        self.quantum = float(sim['range_quantisation'])
        self.scan_time = 1.0 / float(sim['update_rate'])
        self.pub = self.create_publisher(LaserScan, 'scan', QoSProfile(depth=10))
        self.create_subscription(LaserScan, 'gz/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        out = convert(msg.ranges, msg.angle_min, msg.angle_increment, self.multiple,
                      self.range_min, self.range_max, self.quantum)
        n = len(out)
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.frame_id
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / (n - 1)
        scan.scan_time = self.scan_time
        scan.time_increment = self.scan_time / (n - 1)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = out.astype(np.float32).tolist()
        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDriverSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
