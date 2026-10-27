"""wyvern_odom: PX4 vehicle odometry (NED/FRD) -> nav_msgs/Odometry (ENU/FLU) + TF for RViz.

Publishes
  /wyvern/odom           nav_msgs/Odometry, frame `odom_frame` (PX4 local origin, ENU), child `base_frame`
  /wyvern/path           nav_msgs/Path in `map_frame` (flown trajectory, decimated)
  /wyvern/marker         visualization_msgs/Marker: the wyvern body mesh at the estimated pose
  TF  map -> odom (static: PX4 local origin in world coordinates, parameter `origin`), odom -> base_link
Stamps use this node's clock (use_sim_time: the gz /clock that PX4 is lockstepped to).
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker

from fleet_drone import px4io as P


class OdomBridge(Node):
    def __init__(self):
        super().__init__("wyvern_odom")
        self.origin = list(self.declare_parameter("origin", [0.0, 0.0, 0.0]).value)
        # PX4 local metres -> world metres (WGS84 vs. PX4's spherical projection, see px4io.geo_scale)
        self.k = P.geo_scale(float(self.declare_parameter("geo_lat", 0.0).value),
                             self.declare_parameter("geo_model", "EARTH_WGS84").value)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.mesh = self.declare_parameter("mesh", "").value
        self.base = self.declare_parameter("px4_topic", "/fmu/out/vehicle_odometry").value
        self.pub = self.create_publisher(Odometry, "/wyvern/odom", 10)
        self.path_pub = self.create_publisher(Path, "/wyvern/path", 2)
        self.mk_pub = self.create_publisher(Marker, "/wyvern/marker", 2)
        self.tf = TransformBroadcaster(self)
        self.stf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = self.map_frame, self.odom_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, self.origin)
        t.transform.rotation.w = 1.0
        self.stf.sendTransform(t)
        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.n = 0
        self.sub = None
        self.timer = self.create_timer(1.0, self._discover)

    def _discover(self):
        name = P.find_topic(self, self.base)
        if name:
            self.sub = self.create_subscription(VehicleOdometry, name, self._cb, P.QOS_PX4_OUT)
            self.get_logger().info("odometry from %s" % name)
            self.timer.cancel()

    def _cb(self, m):
        stamp = self.get_clock().now().to_msg()
        p = P.ned_to_enu(np.asarray(m.position) / self.k)
        q = P.q_ned_frd_to_enu_flu(m.q)
        if m.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            v_body = P.frd_to_flu(m.velocity)
        else:
            v_body = P.qrot([q[0], -q[1], -q[2], -q[3]], P.ned_to_enu(np.asarray(m.velocity) / self.k))
        w = P.frd_to_flu(m.angular_velocity)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, self.odom_frame, self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, p)
        o.pose.pose.orientation.w, o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z = map(float, q)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = map(float, v_body)
        o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z = map(float, w)
        pv, ov, vv = m.position_variance, m.orientation_variance, m.velocity_variance
        for i, val in enumerate((pv[1], pv[0], pv[2], ov[0], ov[1], ov[2])):
            o.pose.covariance[i * 7] = float(val)
        for i, val in enumerate((vv[1], vv[0], vv[2])):
            o.twist.covariance[i * 7] = float(val)
        self.pub.publish(o)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self.odom_frame, self.base_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
        t.transform.rotation = o.pose.pose.orientation
        self.tf.sendTransform(t)
        self.n += 1
        if self.n % 10 == 0:   # PX4 odometry ~100 Hz -> path 10 Hz, marker 10 Hz
            ps = PoseStamped()
            ps.header.stamp, ps.header.frame_id = stamp, self.map_frame
            ps.pose.position.x = float(p[0] + self.origin[0])
            ps.pose.position.y = float(p[1] + self.origin[1])
            ps.pose.position.z = float(p[2] + self.origin[2])
            ps.pose.orientation = o.pose.pose.orientation
            self.path.poses.append(ps)
            if len(self.path.poses) > 5000:
                self.path.poses = self.path.poses[-5000:]
            self.path.header.stamp = stamp
            self.path_pub.publish(self.path)
            if self.mesh:
                mk = Marker()
                mk.header.stamp, mk.header.frame_id = stamp, self.base_frame
                mk.ns, mk.id, mk.type, mk.action = "wyvern", 0, Marker.MESH_RESOURCE, Marker.ADD
                mk.mesh_resource, mk.mesh_use_embedded_materials = self.mesh, True
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = mk.scale.z = 1.0
                mk.color.a = 1.0
                self.mk_pub.publish(mk)


def main():
    rclpy.init()
    n = OdomBridge()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:  # publishing during launch shutdown races the context teardown
        if rclpy.ok():
            raise
        n.get_logger().debug("shutdown: %s" % e)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
