#!/usr/bin/env python3
"""G2 world check, sim already running: drive_check.py <world> <robot>.

Drives straight then spins in place; compares /diff_drive_controller/odom with Gazebo ground truth
(/world/<world>/pose/info) and reports the median RTF. One JSON line on stdout.
"""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.world_stats_pb2 import WorldStatistics

world, robot = sys.argv[1], sys.argv[2]
st = {'gt': None, 'rtf': [], 'odom': None}


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def on_pose(msg):
    for p in msg.pose:
        if p.name == robot:
            st['gt'] = (p.position.x, p.position.y, yaw_q(p.orientation))


def on_stats(msg):
    st['rtf'].append(msg.real_time_factor)


gz = GzNode()
gz.subscribe(Pose_V, f'/world/{world}/pose/info', on_pose)
gz.subscribe(WorldStatistics, f'/world/{world}/stats', on_stats)
rclpy.init()
n = Node('g2_drive_check', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
pub = n.create_publisher(TwistStamped, '/cmd_vel', 10)
n.create_subscription(Odometry, '/diff_drive_controller/odom',
                      lambda m: st.__setitem__('odom', (m.pose.pose.position.x, m.pose.pose.position.y,
                                                        yaw_q(m.pose.pose.orientation))), 10)
t_wall = time.time()
while (st['odom'] is None or st['gt'] is None) and time.time() - t_wall < 150:
    rclpy.spin_once(n, timeout_sec=0.1)
if st['odom'] is None or st['gt'] is None:
    print(json.dumps(dict(world=world, ok=False, why='no odom/gt', gt=st['gt'], odom=st['odom'])))
    sys.exit(1)


def now():
    return n.get_clock().now().nanoseconds * 1e-9


def hold(vx, wz, dur, track=None):
    t0 = now()
    while now() - t0 < dur:
        m = TwistStamped()
        m.header.stamp = n.get_clock().now().to_msg()
        m.twist.linear.x, m.twist.angular.z = float(vx), float(wz)
        pub.publish(m)
        end = time.time() + 0.05
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.01)
            if track is not None and st['gt'] and st['odom']:
                track.append((st['gt'][2], st['odom'][2]))


def unwrap(seq):
    out, acc = [seq[0]], seq[0]
    for a, b in zip(seq, seq[1:]):
        acc += math.atan2(math.sin(b - a), math.cos(b - a))
        out.append(acc)
    return out


hold(0, 0, 1.0)
st['rtf'].clear()
g0, o0 = st['gt'], st['odom']
hold(0.3, 0, 3.0)
hold(0, 0, 1.0)
g1, o1 = st['gt'], st['odom']
d_gt, d_od = math.dist(g0[:2], g1[:2]), math.dist(o0[:2], o1[:2])
trk = [(g1[2], o1[2])]
hold(0, 1.0, 3.0, trk)
hold(0, 0, 1.0, trk)
yg, yo = unwrap([a for a, _ in trk]), unwrap([b for _, b in trk])
dyg, dyo = yg[-1] - yg[0], yo[-1] - yo[0]
rtf = sorted(st['rtf'])[len(st['rtf']) // 2] if st['rtf'] else None
print(json.dumps(dict(world=world, robot=robot, ok=True, straight_gt_m=round(d_gt, 4), straight_odom_m=round(d_od, 4),
                      straight_ratio=round(d_gt / d_od, 4) if d_od > 1e-3 else None,
                      heading_drift_deg=round(math.degrees(math.atan2(math.sin(g1[2] - g0[2]), math.cos(g1[2] - g0[2]))), 3),
                      spin_gt_rad=round(dyg, 4), spin_odom_rad=round(dyo, 4),
                      yaw_ratio=round(dyg / dyo, 4) if abs(dyo) > 1e-3 else None, rtf_median=rtf)))
rclpy.shutdown()
