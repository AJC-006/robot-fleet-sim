#!/usr/bin/env python3
"""Log map-frame pose error (tf map->base_footprint vs Gazebo ground truth) while a sim runs.

    python3 tools/gt_monitor.py --world keep --robot round --spawn X Y YAW --out gt.csv [--period 1.0]

Ground truth: gz topic /world/<w>/dynamic_pose/info (entity <robot>) in the world frame; the map frame
is assumed equal to the world frame (the maps are exported with world-frame origins). Stops on SIGINT/SIGTERM,
then prints one summary line:  GT n=.. err_max=.. err_mean=.. err_final=.. last_gt=(x,y)
"""
import argparse, math, signal, sys, threading, time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True); ap.add_argument('--robot', required=True)
    ap.add_argument('--out', required=True); ap.add_argument('--period', type=float, default=1.0)
    a, _ = ap.parse_known_args()
    gt = {}
    def cb(msg):
        for p in msg.pose:
            if p.name == a.robot:
                q = p.orientation
                gt['p'] = (p.position.x, p.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    gz = GzNode(); gz.subscribe(Pose_V, f'/world/{a.world}/dynamic_pose/info', cb)
    rclpy.init(); n = Node('gt_monitor', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    buf = Buffer(); TransformListener(buf, n)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *x: stop.set()); signal.signal(signal.SIGTERM, lambda *x: stop.set())
    th = threading.Thread(target=rclpy.spin, args=(n,), daemon=True); th.start()
    errs = []; f = open(a.out, 'w'); f.write('wall,gt_x,gt_y,gt_yaw,map_x,map_y,err\n')
    last = None
    while not stop.is_set():
        time.sleep(a.period)
        if 'p' not in gt: continue
        try:
            t = buf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            continue
        mx, my = t.transform.translation.x, t.transform.translation.y
        g = gt['p']; e = math.hypot(mx - g[0], my - g[1]); errs.append(e); last = g
        f.write(f'{time.time():.1f},{g[0]:.2f},{g[1]:.2f},{g[2]:.2f},{mx:.2f},{my:.2f},{e:.3f}\n'); f.flush()
    if errs:
        print(f'GT n={len(errs)} err_max={max(errs):.2f} err_mean={sum(errs)/len(errs):.2f} err_final={errs[-1]:.2f} last_gt=({last[0]:.1f},{last[1]:.1f})', flush=True)
    else:
        print('GT n=0', flush=True)
    f.close(); sys.stdout.flush(); import os; os._exit(0)

main()
