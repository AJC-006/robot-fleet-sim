# Add a wheeled robot

A wheeled robot is one entry in `src/fleet_description/config/robots.yaml` that picks catalog components by
name, plus meshes drawn by a Blender builder module. Mass, centre of mass and inertia are computed, never typed.
Everything below runs from the workspace root. Legged robots follow the same entry pattern with `kind: legged`
(see the `hound` entry); this guide covers wheeled differential/skid drive.

## 1. Copy a similar entry

Copy the `round` entry (a small two-wheel robot with two casters) to a new key, for example `scout`. Keys, in
the order they matter:

```yaml
scout:
  label: "Scout"
  builder: pebble              # module in blender/robots/ that draws the meshes (step 2)
  components:                  # catalog file stems, src/fleet_components/components/<category>/<stem>.yaml
    lidar: slamtec_rplidar_c1
    imu: microstrain_3dm_cv7_ahrs
    camera: intel_realsense_d435
    wheel_motor: pololu_37d_12v_50to1_64cpr
    motor_driver: basicmicro_roboclaw_2x15a
    battery_cell: samsung_inr18650_35e
    tyre: rubber_on_concrete
  battery_pack: {series: 3, parallel: 1}
  mounts: {battery: [0.0, 0.0, 0.02], driver: [-0.06, 0.0, 0.045]}   # base_link frame, m
  wheel_radius: 0.035          # m
  wheel_width: 0.022
  wheel_separation: 0.23       # geometric, centre to centre
  wheel_separation_multiplier: 1.0   # keep 1.0; a different value would be a fudge factor
  left_wheels: [left_wheel]
  right_wheels: [right_wheel]
  wheel_x: [0.0]
  casters: [{name: front, x: 0.10, y: 0.0, radius: 0.015}, {name: rear, x: -0.10, y: 0.0, radius: 0.015}]
  body: {wheel_style: pebble, collisions: [{shape: cylinder, radius: 0.14, length: 0.075, xyz: [0, 0, 0.0275]}]}
  lidar: {x: 0.0, y: 0.0, z: 0.17}        # sensor positions, m (REP-103 FLU, z above the floor)
  imu: {x: 0.0, y: 0.0, z: 0.06}
  camera: {x: 0.143, y: 0.0, z: 0.075, pitch: 0.0}
  footprint: [[0.158, 0.058], [0.063, 0.153], [-0.063, 0.153], [-0.153, 0.063], [-0.153, -0.063], [-0.063, -0.153], [0.063, -0.153], [0.158, -0.058]]
  robot_radius: 0.158          # Nav2
  max_vel_x: 0.5               # m/s, must be reachable with the chosen motor, gearing, wheel radius and pack voltage
  max_vel_theta: 2.0           # rad/s
  height: 0.19
```

Full key meaning: [../configuration.md](../configuration.md#robot-config-robotsyaml). `builder:` is optional for
the seven shipped robots; for a new key it is required (`build_robots.py` stops with a message when it is missing).

Rules the numbers must obey: parts come from the catalog (to add a part see [swap_a_sensor.md](swap_a_sensor.md)
and `src/fleet_components/components/README.md`); the speed limits must be reachable with the motor's
no-load speed at the pack voltage; do not add a mass or an inertia anywhere.

## 2. Meshes (Blender, headless)

Existing builder modules: `pebble` (disc), `brick` (box), `rover`, `truck` (see `MODULES` in
`blender/build_robots.py`). To get your own shape, copy `blender/robots/pebble.py` to `blender/robots/<mod>.py`
and edit `build(cfg, ctx)`; set `builder: <mod>` in the entry. Then:

```bash
blender -b --factory-startup -P blender/build_robots.py -- --robot scout --no-render
```

This writes `src/fleet_description/meshes/scout/{base,wheel}.obj` (+ `.mtl`) and
`blender/blend/robot_scout.blend`. Drop `--no-render` to also write a preview image into `blender/previews/`.
The committed OBJ files are what the ROS build uses, so users without Blender can still build.
No Blender, or your own CAD meshes? Put `base.obj` (frame: base_link origin, x forward, z up, SI metres)
and `wheel.obj` (axle along y, centred at the origin) in `src/fleet_description/meshes/<name>/` yourself.

## 3. Mass properties

The mass budget needs the volume and inertia of each structural mesh part, then adds the catalog parts at their
`mounts`:

```bash
blender -b --factory-startup -P tools/export_part_stats.py -- --robot scout   # writes blender/robots/part_stats/scout.json
python3 tools/mass_properties.py                                              # rewrites config/mass_properties.yaml
```

Without Blender, `mass_properties.py` cannot size the structure: the part-stats JSON must exist. The test
`test_mass_properties_current` fails when `mass_properties.yaml` is stale, so commit both files.

## 4. Build and launch

```bash
colcon build --symlink-install --packages-select fleet_description fleet_bringup
source install/setup.bash
ros2 launch fleet_bringup sim.launch.py robot:=scout world:=lab gui:=false rviz:=false
```

Ready when Nav2 prints "Managed nodes are active" and `/odometry/filtered` publishes. Then send a goal as in
the README quickstart. The launch writes the URDF with xacro from your entry; expand it yourself to check:

```bash
xacro src/fleet_description/urdf/robot.urdf.xacro robot:=scout | check_urdf /dev/stdin
```

Topics of wheeled and legged robots are at the root (`/scan`, `/imu/data`, `/odometry/filtered`, `/cmd_vel`): no
`namespace` argument yet (G7). With the catalog's RPLIDAR C1, `/scan` is `sensor_msgs/msg/LaserScan` at about 10 Hz.

Proved with a temporary robot built exactly by these steps (builder `pebble`, `wheel_separation` 0.23 m): xacro
expanded and `check_urdf` parsed it, Nav2 reached "Managed nodes are active" in the lab, `/scan` ran at 9.8 Hz and
`/odometry/filtered` published. The temporary robot has been removed from the repository.

## 5. What you get for free

TF `odom -> base_footprint` (REP-105), driver-compatible sensor topics for the catalog parts, the diff-drive
controller, EKF, AMCL and Nav2 with your `footprint`, `robot_radius` and speed limits. The robot is in every
`robot:=` choice and in the tests that loop over `robots.yaml` (`colcon test --packages-select fleet_description`
checks component names and mass properties for it).

## Common mistakes

- A component name with a typo: the test `test_component_names_resolve` names it.
- Footprint larger than a doorway of the world: use `tools/check_world.py <world> --inflate <robot_radius>`.
- Forgot to regenerate `mass_properties.yaml`: the robot loads without mass or the test fails.
