"""
Reproduce realsense2_camera topics from raw gz renders.

  camera/camera/color/image_raw        rgb8
  camera/camera/color/camera_info      pinhole K from the catalog, zero distortion
  camera/camera/depth/image_rect_raw   16UC1 millimetres, 0 = invalid
  camera/camera/depth/camera_info
Depth noise: sigma_z = subpixel / (fx * baseline) * z^2
(catalog depth_noise_subpixel, depth_fx, baseline), applied per pixel;
z below the minimum range or beyond the maximum is reported as 0 (invalid).
"""
import math

from fleet_sensor_sim import catalog
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import CameraInfo, Image


def intrinsics(width, height, hfov):
    """Pinhole K (row-major) with square pixels and the principal point at the image centre."""
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    return [fx, 0.0, width / 2.0, 0.0, fx, height / 2.0, 0.0, 0.0, 1.0]


def depth_to_mm(z, coeff, z_min, z_max, rng):
    """Add sigma = coeff z^2 noise to a float depth image in metres, return uint16 millimetres."""
    z = np.asarray(z, dtype=np.float64)
    valid = np.isfinite(z) & (z >= z_min) & (z <= z_max)
    noisy = z + rng.normal(0.0, 1.0, z.shape) * coeff * np.where(valid, z, 0.0) ** 2
    mm = np.round(noisy * 1000.0)
    mm[~valid | (mm < 0)] = 0
    return np.minimum(mm, 65535).astype(np.uint16)


class CameraDriverSim(Node):
    """gz/camera/{color,depth}/image -> realsense2_camera topics."""

    def __init__(self):
        super().__init__('camera_driver_sim')
        self.declare_parameter('component', 'intel_realsense_d435')
        self.declare_parameter('catalog_dir', '')
        self.declare_parameter('seed', -1)
        entry = catalog.load('depth_camera', self.get_parameter('component').value,
                             self.get_parameter('catalog_dir').value)
        sim = entry['sim']
        frames = entry['ros_driver']['frame_id']
        self.color_frame, self.depth_frame = frames['color'], frames['depth']
        self.coeff = float(sim['depth_noise_subpixel']) / (
            float(sim['depth_fx']) * float(entry['specs']['baseline']))
        self.z_min, self.z_max = float(sim['depth_clip_near']), float(sim['depth_clip_far'])
        seed = int(self.get_parameter('seed').value)
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.info = {
            k: self.make_info(sim[k]['width'], sim[k]['height'], sim[k]['hfov_rad'])
            for k in ('color', 'depth')}
        qos = QoSProfile(depth=10)
        ns = 'camera/camera/'
        self.pub = {
            'color': self.create_publisher(Image, ns + 'color/image_raw', qos),
            'color_info': self.create_publisher(CameraInfo, ns + 'color/camera_info', qos),
            'depth': self.create_publisher(Image, ns + 'depth/image_rect_raw', qos),
            'depth_info': self.create_publisher(CameraInfo, ns + 'depth/camera_info', qos)}
        self.create_subscription(Image, 'gz/camera/color/image', self.on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, 'gz/camera/depth/image', self.on_depth,
                                 qos_profile_sensor_data)

    @staticmethod
    def make_info(width, height, hfov):
        info = CameraInfo()
        info.width, info.height = int(width), int(height)
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = intrinsics(width, height, hfov)
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        k = info.k
        info.p = [k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def publish_info(self, key, stamp, frame):
        info = self.info[key]
        info.header.stamp = stamp
        info.header.frame_id = frame
        self.pub[key + '_info'].publish(info)

    def on_color(self, msg):
        out = Image()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.color_frame
        out.height, out.width, out.encoding = msg.height, msg.width, 'rgb8'
        out.step, out.data = msg.step, msg.data
        self.pub['color'].publish(out)
        self.publish_info('color', msg.header.stamp, self.color_frame)

    def on_depth(self, msg):
        z = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        mm = depth_to_mm(z, self.coeff, self.z_min, self.z_max, self.rng)
        out = Image()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.depth_frame
        out.height, out.width, out.encoding = msg.height, msg.width, '16UC1'
        out.step = msg.width * 2
        out.data = mm.tobytes()
        self.pub['depth'].publish(out)
        self.publish_info('depth', msg.header.stamp, self.depth_frame)


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
