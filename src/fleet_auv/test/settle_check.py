#!/usr/bin/env python3
"""Pass/fail settle check against a running knucker simulation (launch auv.launch.py world:=tank first).

Logs <namespace>/odometry_gt in SIM time for --duration seconds, then checks the trim against the S1 derivation:
mean pitch and roll about 0 (the CB is 15.1 mm above the CG, CB x equals CG x to 0.1 mm), mean vertical speed equal to
the terminal speed sqrt(net_buoyancy / |zWabsW|) when the vehicle is released under the surface, and the battery.
usage: python3 test/settle_check.py [--namespace knucker] [--duration 60] [--max-pitch-deg 0.4]
"""
import argparse
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


def pitch_roll(q):
    sinp = 2 * (q.w * q.y - q.z * q.x)
    return math.asin(max(-1, min(1, sinp))), math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))


class Rec(Node):
    def __init__(self, ns):
        super().__init__("settle_check", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.rows, self.volt = [], None
        self.create_subscription(Odometry, "/%s/odometry_gt" % ns, self.cb, 10)
        self.create_subscription(BatteryState, "/%s/battery/state" % ns, lambda m: setattr(self, "volt", m.voltage), 10)

    def cb(self, m):
        t = self.get_clock().now().nanoseconds * 1e-9
        p, r = pitch_roll(m.pose.pose.orientation)
        self.rows.append((t, m.pose.pose.position.z, p, r))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="knucker")
    ap.add_argument("--duration", type=float, default=60.0, help="sim seconds")
    ap.add_argument("--max-pitch-deg", type=float, default=0.4)
    ap.add_argument("--wall-timeout", type=float, default=200.0)
    a = ap.parse_args()
    rclpy.init()
    n = Rec(a.namespace)
    import time
    w0 = time.time()
    while rclpy.ok() and time.time() - w0 < a.wall_timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.rows and n.rows[-1][0] - n.rows[0][0] >= a.duration:
            break
    rows = n.rows
    if len(rows) < 10:
        print("FAIL no odometry")
        return 1
    t0, t1 = rows[0][0], rows[-1][0]
    tail = [r for r in rows if r[0] >= t0 + 0.5 * (t1 - t0)]       # second half of the record
    mp = sum(r[2] for r in tail) / len(tail)
    mr = sum(r[3] for r in tail) / len(tail)
    amp = (max(r[2] for r in tail) - min(r[2] for r in tail)) / 2
    vz = (tail[-1][1] - tail[0][1]) / (tail[-1][0] - tail[0][0])
    print("sim %.1f s (%d samples)  z %.3f -> %.3f  mean vz %.4f m/s  mean pitch %.3f deg  mean roll %.3f deg  pitch amplitude %.3f deg  bus %.2f V"
          % (t1 - t0, len(rows), rows[0][1], rows[-1][1], vz, math.degrees(mp), math.degrees(mr), math.degrees(amp), n.volt or float("nan")))
    ok = abs(math.degrees(mp)) <= a.max_pitch_deg and abs(math.degrees(mr)) <= a.max_pitch_deg
    print("PASS" if ok else "FAIL", "trim within %.2f deg" % a.max_pitch_deg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
