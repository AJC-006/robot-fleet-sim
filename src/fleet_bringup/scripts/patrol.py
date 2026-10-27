#!/usr/bin/env python3
"""Send the robot through a world's goals with Nav2. Doubles as the end-to-end test.

  ros2 run fleet_bringup patrol.py --world keep [--goals hall,vault] [--loop] [--timeout 240]
                                     [--initial-pose]

Goals come from fleet_worlds/worlds/<world>.yaml. Exit code 0 when every goal succeeded.
--initial-pose re-sends the spawn pose to AMCL first (only right after launch).
"""
import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def wait_active(node, names, call_timeout=3.0):
    """Block until every lifecycle node in `names` reports `active`, retrying lost service calls."""
    from lifecycle_msgs.srv import GetState
    for name in names:
        cli = node.create_client(GetState, f'/{name}/get_state')
        while not cli.wait_for_service(timeout_sec=2.0):
            node.get_logger().info(f'{name}/get_state not available, waiting ...')
        while True:
            fut = cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=call_timeout)
            if fut.done() and fut.result() is not None:
                label = fut.result().current_state.label
                if label == 'active':
                    break
                node.get_logger().info(f'{name} is {label}, waiting ...')
            else:
                cli.remove_pending_request(fut)
                node.get_logger().info(f'{name}/get_state did not answer, asking again')
            time.sleep(1.0)
        node.destroy_client(cli)
    node.get_logger().info('Nav2 is ready for use!')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', required=True)
    ap.add_argument('--goals', help='comma-separated goal names (default: all, in file order)')
    ap.add_argument('--loop', action='store_true', help='repeat forever')
    ap.add_argument('--timeout', type=float, default=240.0, help='seconds per goal (sim time)')
    ap.add_argument('--initial-pose', action='store_true')
    args, _ = ap.parse_known_args()

    meta_path = os.path.join(get_package_share_directory('fleet_worlds'), 'worlds', f'{args.world}.yaml')
    with open(meta_path) as f:
        meta = yaml.safe_load(f)
    goals = meta['goals']
    if args.goals:
        wanted = args.goals.split(',')
        goals = [g for name in wanted for g in meta['goals'] if g['name'] == name]
        if len(goals) != len(wanted):
            sys.exit(f'unknown goal in {wanted}; have {[g["name"] for g in meta["goals"]]}')

    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    if args.initial_pose:
        s = meta['spawn']
        nav.setInitialPose(make_pose(nav, s['x'], s['y'], s['yaw']))
    # Waiting on controller_server rather than amcl skips BasicNavigator's habit of publishing a
    # zero initial pose, and works the same with slam_toolbox. Not waitUntilNav2Active(): it blocks
    # forever when a get_state response is lost (the server logs "failed to send response ...
    # (timeout)" when the machine is loaded), so ask again with a timeout instead.
    wait_active(nav, ['controller_server', 'bt_navigator'])

    results = []
    while True:
        for g in goals:
            nav.goToPose(make_pose(nav, g['x'], g['y'], g['yaw']))
            start = nav.get_clock().now()
            while not nav.isTaskComplete():
                elapsed = (nav.get_clock().now() - start).nanoseconds / 1e9
                if elapsed > args.timeout:
                    nav.cancelTask()
                time.sleep(0.2)
            elapsed = (nav.get_clock().now() - start).nanoseconds / 1e9
            result = nav.getResult()
            ok = result == TaskResult.SUCCEEDED
            results.append((g['name'], ok, elapsed))
            print(f'[patrol] {g["name"]:>14}: {"OK  " if ok else "FAIL"} {result.name:<10} {elapsed:6.1f} s',
                  flush=True)
        if not args.loop:
            break

    passed = sum(ok for _, ok, _ in results)
    print(f'[patrol] {args.world}: {passed}/{len(results)} goals reached', flush=True)
    nav.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed == len(results) else 1)


if __name__ == '__main__':
    main()
