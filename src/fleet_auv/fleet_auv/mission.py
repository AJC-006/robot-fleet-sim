"""Waypoint mission for the knucker: integral LOS guidance in the horizontal plane, depth reference per waypoint.

Reads a yaml (`mission_file`, optional `section`, default 'auv' if present) with `water_level` and
`waypoints: [{name, x, y, z}]` (z = world height, depth = water_level - z). Navigation uses the
odometry from the EKF (odometry/filtered, odom frame). `odom_origin` [x, y] is the map->odom translation (the launch file passes
the spawn point, so map = world). Publishes ref at 10 Hz for the controller and RViz markers.
Exit code 0 when every waypoint was reached within `accept_radius` (3D), 1 on failure/timeout.
"""
import csv
import math
import sys

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from .common import clamp, quat_to_rpy, wrap


class Mission(Node):
    def __init__(self):
        super().__init__("auv_mission")
        d = self.declare_parameter
        d("mission_file", "")
        d("section", "auv")
        d("accept_radius", 0.8)
        d("cruise", 0.7)
        d("lookahead", 4.0)
        d("sigma", 0.0)
        d("leg_timeout", 240.0)
        d("start_delay", 6.0)
        d("log_csv", "")
        d("odom_origin", [0.0, 0.0])
        d("exit_on_done", True)
        gp = lambda k: self.get_parameter(k).value
        with open(gp("mission_file")) as f:
            y = yaml.safe_load(f)
        sec = y.get(gp("section"), y)
        self.wl = float(sec.get("water_level", 0.0))
        self.wps = [(w["name"], float(w["x"]), float(w["y"]), self.wl - float(w["z"])) for w in sec["waypoints"]]
        self.radius = float(gp("accept_radius"))
        self.cruise = float(gp("cruise"))
        self.delta = float(gp("lookahead"))
        self.sigma = float(gp("sigma"))
        self.leg_timeout = float(gp("leg_timeout"))
        self.start_delay = float(gp("start_delay"))
        self.exit_on_done = bool(gp("exit_on_done"))
        self.pos = None
        self.yaw = 0.0
        self.idx = 0
        self.leg_from = None
        self.leg_t0 = None
        self.t_start = None
        self.yint = 0.0
        self.min_d = 1e9
        self.results = []
        self.done = False
        self.ok = None
        self.csv = None
        if gp("log_csv"):
            self.csv = csv.writer(open(gp("log_csv"), "w"))
            self.csv.writerow(["t", "x", "y", "z", "yaw", "wp", "depth_ref", "heading_ref", "speed_ref", "dist"])
        self.create_subscription(Odometry, "odometry/filtered", self.on_odom, 10)
        self.pref = self.create_publisher(Float64MultiArray, "ref", 10)
        self.pmk = self.create_publisher(MarkerArray, "mission_markers", 1)
        self.create_timer(0.1, self.step)
        self.create_timer(1.0, self.markers)
        self.get_logger().info("mission: %d waypoints, water level %.2f, accept %.2f m" % (len(self.wps), self.wl, self.radius))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        p = m.pose.pose
        ox, oy = self.get_parameter("odom_origin").value
        self.pos = (p.position.x + ox, p.position.y + oy, self.wl + p.position.z)
        q = p.orientation
        self.yaw = quat_to_rpy(q.x, q.y, q.z, q.w)[2]

    def step(self):
        if self.pos is None or self.done:
            return
        t = self.now()
        x, y, z = self.pos
        depth = self.wl - z
        if self.t_start is None:
            self.t_start = t
            self.hold_yaw = self.yaw
        if t - self.t_start < self.start_delay:
            self.send(depth, self.hold_yaw, 0.0)
            return
        name, wx, wy, wd = self.wps[self.idx]
        if self.leg_from is None:
            self.leg_from = (x, y, depth)
            self.leg_t0 = t
            self.yint = 0.0
            self.min_d = 1e9
        dist = math.sqrt((wx - x) ** 2 + (wy - y) ** 2 + (wd - depth) ** 2)
        self.min_d = min(self.min_d, dist)
        if dist < self.radius:
            self.results.append((name, True, dist, t - self.leg_t0))
            self.get_logger().info("reached %d/%d %s: %.2f m (%.0f s)" % (self.idx + 1, len(self.wps), name, dist, t - self.leg_t0))
            self.idx += 1
            self.leg_from = (x, y, depth)
            self.leg_t0 = t
            self.yint = 0.0
            if self.idx >= len(self.wps):
                self.finish(True)
                return
            name, wx, wy, wd = self.wps[self.idx]
            dist = math.sqrt((wx - x) ** 2 + (wy - y) ** 2 + (wd - depth) ** 2)
        if t - self.leg_t0 > self.leg_timeout:
            self.results.append((name, False, self.min_d, t - self.leg_t0))
            self.get_logger().error("FAILED waypoint %s: timeout, closest %.2f m" % (name, self.min_d))
            self.finish(False)
            return
        # integral LOS
        x0, y0, d0 = self.leg_from
        L = math.hypot(wx - x0, wy - y0)
        if L < 0.5:                           # vertical-only leg: keep heading, change depth
            heading = self.yaw
            xe = 0.0
            s = 0.0
        else:
            gam = math.atan2(wy - y0, wx - x0)
            dx, dy = x - x0, y - y0
            s = dx * math.cos(gam) + dy * math.sin(gam)
            xe = -dx * math.sin(gam) + dy * math.cos(gam)
            self.yint = clamp(self.yint + 0.1 * self.delta * xe / (self.delta ** 2 + (xe + self.sigma * self.yint) ** 2), -5.0, 5.0)
            heading = wrap(gam - math.atan2(xe + self.sigma * self.yint, self.delta))
            if math.hypot(wx - x, wy - y) < 2.0 * self.delta:     # close: aim straight at the waypoint
                aim = math.atan2(wy - y, wx - x)
                k = clamp(1.0 - math.hypot(wx - x, wy - y) / (2.0 * self.delta), 0.0, 1.0)
                heading = wrap(heading + k * wrap(aim - heading))
        f = clamp(s / max(0.6 * L, 1.0), 0.0, 1.0) if L >= 0.5 else 1.0
        dref = d0 + (wd - d0) * f
        herr = wrap(heading - self.yaw)
        hdist = math.hypot(wx - x, wy - y)
        speed = self.cruise * max(0.0, math.cos(min(abs(herr), 1.4))) ** 2
        speed = min(speed, 0.25 + 0.45 * hdist / 4.0)
        if hdist < 0.6 * self.radius:
            speed = 0.0
        self.send(dref, heading, speed)
        if self.csv:
            self.csv.writerow(["%.2f" % t, "%.3f" % x, "%.3f" % y, "%.3f" % z, "%.4f" % self.yaw, self.idx, "%.3f" % dref,
                               "%.4f" % heading, "%.3f" % speed, "%.3f" % dist])

    def send(self, depth, heading, speed):
        m = Float64MultiArray()
        m.data = [float(depth), float(heading), float(speed)]
        self.pref.publish(m)

    def finish(self, ok):
        self.ok = ok
        self.done = True
        n_ok = sum(1 for r in self.results if r[1])
        self.get_logger().info("MISSION %s: %d/%d waypoints reached, worst %.2f m" % (
            "SUCCEEDED" if ok else "FAILED", n_ok, len(self.wps), max([r[2] for r in self.results] or [0.0])))
        self.send(self.wl - self.pos[2], self.yaw, 0.0)
        self.markers()

    def markers(self):
        if not self.wps:
            return
        arr = MarkerArray()
        line = Marker()
        line.header.frame_id = "world"
        line.ns, line.id, line.type, line.action = "path", 0, Marker.LINE_STRIP, Marker.ADD
        line.scale.x = 0.15
        line.color = ColorRGBA(r=0.8, g=0.4, b=1.0, a=0.9)
        for _, wx, wy, wd in self.wps:
            line.points.append(Point(x=wx, y=wy, z=self.wl - wd))
        arr.markers.append(line)
        for i, (name, wx, wy, wd) in enumerate(self.wps):
            reached = any(r[0] == name and r[1] for r in self.results)
            s = Marker()
            s.header.frame_id = "world"
            s.ns, s.id, s.type, s.action = "wp", i, Marker.SPHERE, Marker.ADD
            s.pose.position.x, s.pose.position.y, s.pose.position.z = wx, wy, self.wl - wd
            s.scale.x = s.scale.y = s.scale.z = 2 * self.radius
            s.color = ColorRGBA(r=0.2, g=1.0, b=0.5, a=0.7) if reached else ColorRGBA(r=1.0, g=0.2, b=0.8, a=0.7)
            arr.markers.append(s)
            tx = Marker()
            tx.header.frame_id = "world"
            tx.ns, tx.id, tx.type, tx.action = "wpname", i, Marker.TEXT_VIEW_FACING, Marker.ADD
            tx.pose.position.x, tx.pose.position.y, tx.pose.position.z = wx, wy, self.wl - wd + 2.0
            tx.scale.z = 1.2
            tx.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            tx.text = "%d %s" % (i + 1, name)
            arr.markers.append(tx)
        self.pmk.publish(arr)


def main():
    rclpy.init()
    n = Mission()
    code = 2
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.2)
            if n.done and n.exit_on_done:
                for _ in range(10):
                    rclpy.spin_once(n, timeout_sec=0.05)
                code = 0 if n.ok else 1
                break
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
