from setuptools import find_packages, setup

package_name = 'fleet_sensor_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alan',
    maintainer_email='65161979+AJC-006@users.noreply.github.com',
    description='Real-driver adapters for simulated sensors.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_driver_sim = fleet_sensor_sim.lidar_driver_sim:main',
            'imu_driver_sim = fleet_sensor_sim.imu_driver_sim:main',
            'camera_driver_sim = fleet_sensor_sim.camera_driver_sim:main',
        ],
    },
)
