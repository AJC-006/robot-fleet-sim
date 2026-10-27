import os
from glob import glob

from setuptools import find_packages, setup

pkg = "fleet_auv"
setup(
    name=pkg,
    version="0.1.0",
    tests_require=["pytest"],
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + pkg]),
        ("share/" + pkg, ["package.xml"]),
        ("share/" + pkg + "/launch", glob("launch/*.py")),
        ("share/" + pkg + "/config", glob("config/*")),
        ("share/" + pkg + "/rviz", glob("rviz/*")),
        ("share/" + pkg + "/worlds", glob("worlds/*")),
        ("share/" + pkg + "/meshes/knucker", glob("meshes/knucker/*")),
        ("share/" + pkg + "/tools", glob("tools/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alan",
    maintainer_email="65161979+AJC-006@users.noreply.github.com",
    description="knucker AUV",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "auv_controller = fleet_auv.controller:main",
            "auv_mission = fleet_auv.mission:main",
            "waterlinked_dvl_driver = fleet_auv.sensor_drivers:dvl_main",
            "microstrain_imu = fleet_auv.sensor_drivers:imu_main",
            "bar30_driver = fleet_auv.sensor_drivers:bar30_main",
            "auv_tests = fleet_auv.tests_run:main",
            "auv_gen_model = fleet_auv.model:main",
        ],
    },
)
