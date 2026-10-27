#!/usr/bin/env python3
"""Walking test against Gazebo ground truth.

  ros2 run fleet_legged walk_test.py --robot hound --world lab [--speed 0.6] [--seconds 6] [--stress 120]

Run it while sim.launch.py is up (nav:=false is fine). It waits for the robot to stand, checks it
stays put at rest, then commands forward, sideways (when max_vel_y > 0), backward and turning in
place at `speed` x the robot's max_vel_*, and compares the body-frame velocity measured from the
ground-truth model pose (gz topic /world/<world>/pose/info) with the command, the EKF output
(/odometry/filtered) and the leg odometry (/leg_odom). Passing: direction right (the off-axis drift
under 25 % of the commanded speed) and speed within 25 %; at rest under 5 mm / 0.01 rad of drift.
--stress N adds N sim seconds of random combined commands (what a planner sends) and fails on a fall
or a body tilt over 25 deg. The robot is kept inside a circle (--stress-radius) around a point
--stress-ahead metres in front of where it stood: outside it, the linear command points back to the
centre, so a long run never walks into a wall (that, not the gait, is what tipped the first long runs
over in the lab). On a fall the last 4 s (sim time, command, ground-truth attitude, joint
commands and measured joints) go to --dump (CSV).
"""
import argparse
import math
import sys
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def roll_pitch(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class GroundTruth:
    """Latest model pose from gz transport (needs GZ_PARTITION set like the simulation)."""

    def __init__(self, world, model):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        self.model = model
        self.lock = threading.Lock()
        self.sample = None      # (sim_time, x, y, z, yaw, roll, pitch)
        self.node = GzNode()
        ok = self.node.subscribe(Pose_V, f'/world/{world}/pose/info', self.cb)
        if not ok:
            raise RuntimeError('cannot subscribe to the gz pose topic')

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == self.model:
                q = p.orientation
                r, pt = roll_pitch(q)
                with self.lock:
                    self.sample = (t, p.position.x, p.position.y, p.position.z, yaw_of(q), r, pt)
                return

    def get(self):
        with self.lock:
            return self.sample


class Tester(Node):
    def __init__(self):
        super().__init__('walk_test')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.ekf = None
        self.leg = None
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: setattr(self, 'ekf', m), 10)
        self.create_subscription(Odometry, '/leg_odom', lambda m: setattr(self, 'leg', m), 10)
        self.joints = None
        self.jcmd = None
        self.create_subscription(JointState, '/joint_states', lambda m: setattr(self, 'joints', m), 10)
        self.create_subscription(Float64MultiArray, '/leg_controller/commands',
                                 lambda m: setattr(self, 'jcmd', list(m.data)), 10)
        self.cmd = (0.0, 0.0, 0.0)
        self.create_timer(0.05, self.send)

    def send(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = self.cmd
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', required=True)
    ap.add_argument('--world', required=True)
    ap.add_argument('--speed', type=float, default=0.6, help='fraction of max_vel_*')
    ap.add_argument('--seconds', type=float, default=6.0, help='sim seconds per motion segment')
    ap.add_argument('--settle', type=float, default=1.5, help='sim seconds skipped at segment start')
    ap.add_argument('--yaml', default='')
    ap.add_argument('--stress', type=float, default=0.0,
                    help='sim seconds of random combined commands (vx+vy+wz, held 0.3-1.5 s, like a planner) '
                         'after the axis tests; fails on a fall (tilt > 45 deg or body below half height)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--only-stress', action='store_true', help='skip the axis tests')
    ap.add_argument('--dump', default='', help='CSV for the last 4 s before a fall in the stress test')
    ap.add_argument('--stress-radius', type=float, default=1.0, help='stress test: stay within this radius (m)')
    ap.add_argument('--stress-ahead', type=float, default=1.0,
                    help='stress test: circle centre this far ahead of the start pose (m)')
    a = ap.parse_args()
    path = a.yaml
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('fleet_description'), 'config', 'robots.yaml')
    spec = yaml.safe_load(open(path))[a.robot]
    vx, vy, wz = (a.speed * float(spec['max_vel_x']), a.speed * float(spec.get('max_vel_y', 0.0)),
                  a.speed * float(spec['max_vel_theta']))

    rclpy.init()
    node = Tester()
    gt = GroundTruth(a.world, a.robot)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    def wait_sim(seconds):
        s0 = gt.get()
        while s0 is None:
            time.sleep(0.1)
            s0 = gt.get()
        while True:
            s = gt.get()
            if s[0] - s0[0] >= seconds:
                return
            time.sleep(0.02)

    def record(cmd, seconds):
        node.cmd = cmd
        wait_sim(a.settle)
        s0 = gt.get()
        e_sum, l_sum, n = [0.0] * 3, [0.0] * 3, 0
        rp_max = 0.0
        z_min = 9.0
        t_end = s0[0] + seconds - a.settle
        while True:
            s = gt.get()
            if s[0] >= t_end:
                break
            rp_max = max(rp_max, abs(s[5]), abs(s[6]))
            z_min = min(z_min, s[3])
            if node.ekf is not None and node.leg is not None:
                for i, m in enumerate((node.ekf, node.leg)):
                    tw = m.twist.twist
                    acc = e_sum if i == 0 else l_sum
                    acc[0] += tw.linear.x
                    acc[1] += tw.linear.y
                    acc[2] += tw.angular.z
                n += 1
            time.sleep(0.02)
        s1 = gt.get()
        dt = s1[0] - s0[0]
        dx, dy = s1[1] - s0[1], s1[2] - s0[2]
        dyaw = math.atan2(math.sin(s1[4] - s0[4]), math.cos(s1[4] - s0[4]))
        ym = s0[4] + dyaw / 2   # mean heading
        bx = (math.cos(ym) * dx + math.sin(ym) * dy) / dt
        by = (-math.sin(ym) * dx + math.cos(ym) * dy) / dt
        n = max(n, 1)
        return {'gt': (bx, by, dyaw / dt), 'ekf': tuple(v / n for v in e_sum), 'leg': tuple(v / n for v in l_sum),
                'dt': dt, 'tilt': rp_max, 'zmin': z_min, 'disp': math.hypot(dx, dy), 'dyaw': dyaw}

    fmt = lambda v: '(' + ', '.join(f'{x:+.3f}' for x in v) + ')'
    results = []
    print(f'[walk_test] {a.robot} in {a.world}: waiting for the robot to stand ...', flush=True)
    wait_sim(6.0)
    rest = record((0.0, 0.0, 0.0), 4.0 + a.settle)
    ok_rest = rest['disp'] < 0.005 and abs(rest['dyaw']) < 0.01
    print(f'rest      : drift {rest["disp"] * 1000:.1f} mm, yaw {math.degrees(rest["dyaw"]):+.2f} deg, '
          f'tilt max {math.degrees(rest["tilt"]):.1f} deg, z {rest["zmin"]:+.3f}  -> {"PASS" if ok_rest else "FAIL"}', flush=True)
    results.append(ok_rest)
    segs = [('forward', (vx, 0.0, 0.0)), ('backward', (-vx, 0.0, 0.0))]
    if vy > 0:
        segs += [('left', (0.0, vy, 0.0)), ('right', (0.0, -vy, 0.0))]
    segs += [('turn_ccw', (0.0, 0.0, wz)), ('turn_cw', (0.0, 0.0, -wz))]
    if a.only_stress:
        segs = []
    for name, cmd in segs:
        r = record(cmd, a.seconds)
        node.cmd = (0.0, 0.0, 0.0)
        g = r['gt']
        if cmd[2] != 0.0:
            ratio = g[2] / cmd[2]
            off = math.hypot(g[0], g[1]) / max(0.05, abs(cmd[2]) * 0.2)
            ok = 0.75 <= ratio <= 1.25
        else:
            ax = 0 if cmd[0] != 0.0 else 1
            ratio = g[ax] / cmd[ax]
            off = abs(g[1 - ax]) / abs(cmd[ax])
            ok = 0.75 <= ratio <= 1.25 and off < 0.25
        results.append(ok)
        print(f'{name:10s}: cmd {fmt(cmd)} gt {fmt(g)} ekf {fmt(r["ekf"])} leg {fmt(r["leg"])}  ratio {ratio:.2f} '
              f'off-axis {off:.2f} tilt max {math.degrees(r["tilt"]):.1f} deg z_min {r["zmin"]:+.3f} -> {"PASS" if ok else "FAIL"}',
              flush=True)
        wait_sim(1.5)
    if a.stress > 0:
        import random
        rnd = random.Random(a.seed)
        h0 = float(spec['standing_height'])
        mx, my, mw = float(spec['max_vel_x']), float(spec.get('max_vel_y', 0.0)), float(spec['max_vel_theta'])
        s0 = gt.get()
        z0 = s0[3]      # model origin = base_footprint, ~0 while standing
        t_end, t_next, worst, fell = s0[0] + a.stress, s0[0], (0.0, None, None), False
        cx, cy = s0[1] + a.stress_ahead * math.cos(s0[4]), s0[2] + a.stress_ahead * math.sin(s0[4])
        ring, homing = [], False
        while True:
            s = gt.get()
            if s[0] >= t_end or fell:
                break
            out = math.hypot(s[1] - cx, s[2] - cy) > a.stress_radius
            if s[0] >= t_next or (out and not homing):
                node.cmd = (rnd.uniform(-0.3, 1.0) * mx, rnd.uniform(-1, 1) * my, rnd.uniform(-1, 1) * mw)
                t_next = s[0] + rnd.uniform(0.3, 1.5)
                homing = out
                if out:     # head back to the centre: body-frame direction, random speed within the limits
                    b = math.atan2(math.sin(math.atan2(cy - s[2], cx - s[1]) - s[4]),
                                   math.cos(math.atan2(cy - s[2], cx - s[1]) - s[4]))
                    ux, uy = math.cos(b), math.sin(b)
                    if my > 0.0:
                        k = rnd.uniform(0.4, 1.0) / max(abs(ux) / mx, abs(uy) / my)
                        node.cmd = (k * ux, k * uy, node.cmd[2])
                    else:           # no sideways walking: turn towards the centre, walk once facing it
                        node.cmd = (mx * rnd.uniform(0.4, 1.0) if abs(b) < 0.6 else 0.0, 0.0,
                                    math.copysign(0.8 * mw, math.atan2(uy, ux)))
            elif not out:
                homing = False
            tilt = max(abs(s[5]), abs(s[6]))
            if tilt > worst[0]:
                worst = (tilt, s[0] - s0[0], node.cmd)
            js = node.joints
            ring.append((s, node.cmd, list(js.position) if js else [], node.jcmd or []))
            while ring and ring[0][0][0] < s[0] - 4.0:
                ring.pop(0)
            if tilt > math.radians(45) or s[3] < z0 - 0.5 * h0:
                fell = True
            time.sleep(0.01)
        if fell and a.dump:
            names = list(node.joints.name) if node.joints else []
            with open(a.dump, 'w') as f:
                f.write('t,vx,vy,wz,x,y,z,yaw,roll,pitch,' + ','.join(f'q_{n}' for n in names) + ','
                        + ','.join(f'c{i}' for i in range(len(ring[-1][3]))) + '\n')
                for smp, c, q, jc in ring:
                    f.write(','.join(f'{v:.4f}' for v in (smp[0], *c, *smp[1:])) + ',' +
                            ','.join(f'{v:.4f}' for v in q) + ',' + ','.join(f'{v:.4f}' for v in jc) + '\n')
            print(f'stress    : fall data in {a.dump}', flush=True)
        node.cmd = (0.0, 0.0, 0.0)
        ok = not fell and worst[0] < math.radians(25)
        results.append(ok)
        print(f'stress    : {a.stress:.0f} s random combined commands (seed {a.seed}): tilt max '
              f'{math.degrees(worst[0]):.1f} deg at t={worst[1]:.1f} s cmd {fmt(worst[2] or (0, 0, 0))}'
              f'{"  FELL" if fell else ""} -> {"PASS" if ok else "FAIL"}', flush=True)
        wait_sim(2.0)
    stop = record((0.0, 0.0, 0.0), 3.0 + a.settle)
    ok_stop = stop['disp'] < 0.005 and abs(stop['dyaw']) < 0.01
    print(f'rest again: drift {stop["disp"] * 1000:.1f} mm, yaw {math.degrees(stop["dyaw"]):+.2f} deg -> '
          f'{"PASS" if ok_stop else "FAIL"}', flush=True)
    results.append(ok_stop)
    print(f'[walk_test] {sum(results)}/{len(results)} passed', flush=True)
    node.cmd = (0.0, 0.0, 0.0)
    time.sleep(0.3)
    rclpy.shutdown()
    sys.stdout.flush()
    # skip interpreter teardown: destroying the gz-transport node from a finalizer aborts with
    # "terminate called without an active exception"
    os._exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
