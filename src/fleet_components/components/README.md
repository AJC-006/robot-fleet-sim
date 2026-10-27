# Component catalog

One YAML file per real, purchasable component: `components/<category>/<vendor>_<model>.yaml`
(lower case, underscores), e.g. `lidar/slamtec_rplidar_s2.yaml`, `dc_motor/pololu_37d_50to1_12v.yaml`.
Robots pick components by file stem in their own config; swapping a sensor means changing one name.

Required keys:

```yaml
model: Slamtec RPLIDAR S2          # exact product name and variant
category: lidar                    # same as the directory
vendor: Slamtec
datasheet: https://...             # primary source
sources:                           # every other source used, and what for
  - {url: https://..., used_for: range noise}
specs:                             # datasheet values converted to SI units; keep the original
  range_max: 30.0                  # m   (datasheet: "30 m @ 70% reflectivity")
  ...
ros_driver:                        # interface of the real component's ROS 2 driver, which the sim reproduces
  package: sllidar_ros2
  topics: {scan: sensor_msgs/msg/LaserScan}
  frame_id: laser
  rate_hz: 10
  qos: sensor_data
sim:                               # how specs map to Gazebo sensor / plugin parameters, with derivations
  ...
limitations: >-                    # what the simulation does not reproduce
  ...
```

Rules: every number in `specs` comes from a listed source; every `sim` value is either a spec or
derived from specs with the derivation written next to it. No tuning factors chosen to make a test
pass. Values the vendor does not publish are marked `unpublished` with the estimate and how it was
obtained (paper, teardown, measurement).

Matching xacro macros live in `urdf/<category>.xacro` and take the component name, parent link,
origin and a name prefix.
