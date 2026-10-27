#!/usr/bin/env python3
"""Print the Nav2 footprint (swept stance envelope + 1 cm) and robot_radius of a legged robot.

  ros2 run fleet_legged envelope.py hound [--yaml path/to/robots.yaml] [--margin 0.01]
"""
import argparse
import os

from fleet_legged.envelope import extents, footprint
from fleet_legged.kinematics import load


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('robot', nargs='+')
    ap.add_argument('--yaml', default='')
    ap.add_argument('--margin', type=float, default=0.01)
    a = ap.parse_args()
    path = a.yaml
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('fleet_description'), 'config', 'robots.yaml')
    for name in a.robot:
        rb = load(name, path)
        poly, r = footprint(rb, a.margin)
        x0, x1, y0, y1 = extents(rb)
        print(f'{name}: feet x {x0:.3f}..{x1:.3f} y {y0:.3f}..{y1:.3f}')
        print(f'  footprint: {[list(p) for p in poly]}'.replace(' ', '').replace(',[', ', [').replace(':', ': '))
        width = max(p[1] for p in poly) - min(p[1] for p in poly)
        print(f'  robot_radius: {r:.3f}   width {width:.3f} m')


if __name__ == '__main__':
    main()
