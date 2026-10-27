import os
from glob import glob

from setuptools import find_packages, setup

pkg = "fleet_drone"


def tree(src, dst):
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join("share", pkg, dst, os.path.relpath(root, src)).rstrip("/."),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=pkg,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + pkg]),
        ("share/" + pkg, ["package.xml", "CREDITS.md"]),
        ("share/" + pkg + "/launch", glob("launch/*.py")),
        ("share/" + pkg + "/rviz", glob("rviz/*")),
        ("share/" + pkg + "/config/missions", glob("config/missions/*.yaml")),
        ("share/" + pkg + "/config/routes", glob("config/routes/*.yaml")),
    ] + tree("models", "models"),
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Alan",
    maintainer_email="65161979+AJC-006@users.noreply.github.com",
    description="wyvern PX4 SITL quadcopter",
    license="Apache-2.0 AND BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "wyvern_odom = fleet_drone.odom_bridge:main",
            "wyvern_mission = fleet_drone.mission:main",
            "wyvern_ulog = fleet_drone.ulog_report:main",
            "wyvern_gz_capture = fleet_drone.gz_capture:main",
            "wyvern_route_check = fleet_drone.route_check:main",
        ],
    },
)
