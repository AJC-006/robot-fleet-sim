"""Headless AUV test suite. Run against a live sim:  ros2 launch fleet_auv auv.launch.py world:=tank mission:=false
then  ros2 run fleet_auv auv_tests [--only depth,heading,surge,roll,surface] [--out docs/img] [--water 0.0]
Logs ground-truth odometry + controller status, prints metrics, writes plots auv_<test>.png, exits 0 if all pass."""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray

from .common import quat_to_rpy, wrap


class Rec(Node):
    def __init__(self):
        super().__init__("auv_tests", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.od = None
        self.st = None
        self.log = []
        self.create_subscription(Odometry, "/knucker/odometry_gt", self.on_od, 20)
        self.create_subscription(Float64MultiArray, "/knucker/status", self.on_st, 20)
        self.pref = self.create_publisher(Float64MultiArray, "/knucker/ref", 10)
        self.pen = self.create_publisher(Bool, "/knucker/enable", 10)
        self.water = 0.0

    def on_od(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        r, pi, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.od = (t, p.x, p.y, p.z, r, pi, y, v.x, v.y, v.z)
        self.log.append(self.od + (self.st[2] if self.st else float("nan"), self.st[4] if self.st else float("nan"),
                                   self.st[8] if self.st else float("nan")))

    def on_st(self, m):
        self.st = list(m.data)

    def spin(self, secs):
        t0 = self.od[0] if self.od else 0.0
        end = time.time() + 600
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.od and self.od[0] - t0 >= secs:
                return

    def ref(self, depth, heading, surge):
        m = Float64MultiArray()
        m.data = [float(depth), float(heading), float(surge)]
        self.pref.publish(m)

    def hold(self, depth, heading, surge, secs):
        t0 = self.od[0]
        while self.od[0] - t0 < secs:
            self.ref(depth, heading, surge)
            self.spin(0.1)

    def enable(self, on):
        for _ in range(3):
            self.pen.publish(Bool(data=bool(on)))
            self.spin(0.1)


def gz_set_pose(world, x, y, z, roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2),
                              math.cos(yaw / 2), math.sin(yaw / 2))
    qw, qx, qy, qz = cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy
    req = 'name: "knucker", position: {x: %g, y: %g, z: %g}, orientation: {x: %g, y: %g, z: %g, w: %g}' % (x, y, z, qx, qy, qz, qw)
    subprocess.run(["gz", "service", "-s", "/world/%s/set_pose" % world, "--reqtype", "gz.msgs.Pose", "--reptype",
                    "gz.msgs.Boolean", "--timeout", "3000", "--req", req], capture_output=True)


def step_metrics(ts, ys, y0, y1, tol_settle):
    """overshoot %, settling time (2% of step or tol), steady-state error over the last 10% of the record, 10-90 rise."""
    span = y1 - y0
    sgn = 1.0 if span >= 0 else -1.0
    over = max(0.0, max((y - y1) * sgn for y in ys)) / abs(span) * 100.0
    n = len(ys)
    ss = sum(ys[int(n * 0.9):]) / max(1, n - int(n * 0.9))
    band = max(tol_settle, 0.02 * abs(span))
    ts_ = ts[-1]
    for i in range(n - 1, -1, -1):
        if abs(ys[i] - y1) > band:
            ts_ = ts[min(i + 1, n - 1)]
            break
    else:
        ts_ = ts[0]
    return over, ts_ - ts[0], ss - y1


def plot(path, title, rows, series, hlines=()):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(len(series), 1, figsize=(9, 2.6 * len(series)), sharex=True)
    fig.patch.set_facecolor("#14101c")
    if len(series) == 1:
        axs = [axs]
    for ax, (label, curves) in zip(axs, series):
        ax.set_facecolor("#1d1729")
        for name, xs, ys, col, ls in curves:
            ax.plot(xs, ys, color=col, ls=ls, lw=1.6, label=name)
        ax.set_ylabel(label, color="#d9c8f0")
        ax.tick_params(colors="#b9a6d6")
        ax.grid(alpha=0.25, color="#6a4f94")
        for s in ax.spines.values():
            s.set_color("#4a3a66")
        ax.legend(facecolor="#1d1729", edgecolor="#4a3a66", labelcolor="#e6d9f7", fontsize=8, loc="best")
    axs[0].set_title(title, color="#f0e4ff")
    axs[-1].set_xlabel("time [s]", color="#d9c8f0")
    fig.tight_layout()
    fig.savefig(path, dpi=110, facecolor=fig.get_facecolor())
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="depth,heading,surge,roll,surface")
    ap.add_argument("--out", default="docs/img")
    ap.add_argument("--world", default="tank")
    ap.add_argument("--water", type=float, default=0.0, help="water surface z")
    ap.add_argument("--json", default="")
    ap.add_argument("--surface-secs", dest="surface_secs", type=float, default=200.0)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    n = Rec()
    n.water = a.water
    while n.od is None or n.st is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    res = {}
    tests = a.only.split(",")
    W = a.water
    D = lambda z: W - z  # depth from z

    def cut(i0):
        return n.log[i0:]

    if "depth" in tests:
        n.hold(2.0, n.st[3], 0.0, 25)  # settle at 2 m
        i0 = len(n.log)
        n.hold(4.0, n.st[3], 0.0, 45)  # 2 m step
        L = cut(i0)
        ts = [r[0] - L[0][0] for r in L]
        d = [D(r[3]) for r in L]
        ov, st, ess = step_metrics(ts, d, 2.0, 4.0, 0.05)
        res["depth"] = dict(step="2.0->4.0 m", overshoot_pct=ov, settle_s=st, sse_m=ess, pass_=abs(ess) < 0.15)
        plot(os.path.join(a.out, "auv_depth_step.png"), "knucker depth step 2 m", L, [
            ("depth [m]", [("ground truth", ts, d, "#c77dff", "-"), ("reference", ts, [r[10] for r in L], "#ffd166", "--")]),
            ("pitch [deg]", [("pitch", ts, [math.degrees(r[5]) for r in L], "#7bdff2", "-")]),
            ("heave w [m/s]", [("w world", ts, [r[9] for r in L], "#ff8fab", "-")])])
    if "heading" in tests:
        n.hold(4.0, n.st[3], 0.0, 15)
        h0 = n.st[3]
        h1 = wrap(h0 + math.pi / 2)
        i0 = len(n.log)
        n.hold(4.0, h1, 0.0, 35)
        L = cut(i0)
        ts = [r[0] - L[0][0] for r in L]
        yaw = [math.degrees(wrap(r[6] - h0)) for r in L]
        ov, st, ess = step_metrics(ts, yaw, 0.0, 90.0, 2.0)
        res["heading"] = dict(step="+90 deg", overshoot_pct=ov, settle_s=st, sse_deg=ess, pass_=abs(ess) < 5.0)
        plot(os.path.join(a.out, "auv_heading_step.png"), "knucker heading step 90 deg", L, [
            ("heading rel [deg]", [("ground truth", ts, yaw, "#c77dff", "-"),
                                   ("reference", ts, [math.degrees(wrap(r[11] - h0)) for r in L], "#ffd166", "--")]),
            ("depth [m]", [("depth", ts, [D(r[3]) for r in L], "#7bdff2", "-")])])
    if "surge" in tests:
        hh = n.st[3]
        n.hold(4.0, hh, 0.0, 10)
        i0 = len(n.log)
        n.hold(4.0, hh, 0.5, 40)
        L = cut(i0)
        ts = [r[0] - L[0][0] for r in L]
        u = [r[7] * math.cos(r[5]) + 0 for r in L]
        ov, st, ess = step_metrics(ts, u, 0.0, 0.5, 0.025)
        dev = max(abs(D(r[3]) - 4.0) for r in L)
        res["surge"] = dict(step="0->0.5 m/s", overshoot_pct=ov, settle_s=st, sse_ms=ess, max_depth_dev_m=dev,
                            pass_=abs(ess) < 0.05 and dev < 0.3)
        plot(os.path.join(a.out, "auv_surge_step.png"), "knucker surge step 0.5 m/s", L, [
            ("surge u [m/s]", [("body u", ts, u, "#c77dff", "-"), ("reference", ts, [r[12] for r in L], "#ffd166", "--")]),
            ("depth [m]", [("depth", ts, [D(r[3]) for r in L], "#7bdff2", "-")]),
            ("heading [deg]", [("yaw", ts, [math.degrees(r[6]) for r in L], "#ff8fab", "-")])])
        n.hold(4.0, hh, 0.0, 15)
    if "roll" in tests:
        n.enable(False)
        n.spin(1.0)
        p = n.od
        i0 = len(n.log)
        gz_set_pose(a.world, p[1], p[2], p[3], math.radians(25), 0.0, p[6])
        n.spin(25)
        L = cut(i0)
        ts = [r[0] - L[0][0] for r in L]
        roll = [math.degrees(r[4]) for r in L]
        peak = max(abs(r) for r in roll)
        t_half = next((t for t, r in zip(ts, roll) if t > ts[roll.index(max(roll, key=abs))] and abs(r) < peak / 2), float('nan'))
        end = sum(abs(r) for r in roll[-50:]) / 50
        res["roll"] = dict(initial_deg=peak, half_time_s=t_half, final_deg=end, pass_=end < 2.0)
        plot(os.path.join(a.out, "auv_roll_decay.png"), "knucker passive roll recovery (thrusters off)", L,
             [("roll [deg]", [("roll", ts, roll, "#c77dff", "-")])])
        n.enable(True)
    if "surface" in tests:
        n.hold(3.0, n.st[3], 0.0, 25)
        n.enable(False)
        i0 = len(n.log)
        n.spin(a.surface_secs)
        L = cut(i0)
        ts = [r[0] - L[0][0] for r in L]
        d = [D(r[3]) for r in L]
        res["surface"] = dict(start_depth=d[0], final_depth=d[-1], rise_ms=(d[0] - d[len(d) // 4]) / ts[len(d) // 4],
                              pass_=d[-1] < 0.4)
        plot(os.path.join(a.out, "auv_surfacing.png"), "knucker surfacing with thrusters off", L,
             [("depth [m]", [("depth", ts, d, "#c77dff", "-")]),
              ("pitch [deg]", [("pitch", ts, [math.degrees(r[5]) for r in L], "#7bdff2", "-")])])
        n.enable(True)
    ok = all(v["pass_"] for v in res.values())
    print(json.dumps(res, indent=1))
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
