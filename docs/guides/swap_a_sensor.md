# Swap a sensor (or any catalog part)

Robots pick parts by catalog file stem. Changing one name changes the Gazebo sensor parameters, the
driver-emulator parameters, the mass budget and the frame/topic conventions together, because all of them are
generated from the same catalog entry.

## Swap among existing entries

```bash
ls src/fleet_components/components/lidar        # slamtec_rplidar_c1.yaml  slamtec_rplidar_s2.yaml
```

Edit `components:` of the robot in `src/fleet_description/config/robots.yaml`:

```yaml
round:
  components:
    lidar: slamtec_rplidar_s2      # was slamtec_rplidar_c1
```

Then:

```bash
python3 tools/mass_properties.py                                   # new part mass and size
colcon build --symlink-install --packages-select fleet_description
ros2 launch fleet_bringup sim.launch.py robot:=round world:=lab gui:=false rviz:=false
ros2 topic info /scan -v | head -3; ros2 topic hz /scan
```

Checked on a temporary robot with `xacro ... robot:=<name>`: the C1 gives `samples` 500, `max` 12.0 m, the S2 gives
`samples` 3200, `max` 30.0 m, and swapping back restores the C1 values. If you forget `mass_properties.py`, the
`test_mass_properties_current` test and `python3 tools/mass_properties.py --check` fail (exit 1), so a swap cannot
leave a stale mass budget unnoticed.

What changes (S2 versus C1 as example): range 30 m vs 12 m, 32 000 vs 5 000 samples per second, 10 Hz scan,
noise and quantisation from the datasheet. The topic name, type (`sensor_msgs/msg/LaserScan`), frame id and
REP-117 handling (`inf` out of range, `NaN` invalid) follow the catalog `ros_driver` block, which is what the
real vendor driver publishes. Replacing the simulation with the real driver (`sllidar_ros2`) needs no other change.

Roles and categories: `lidar` (lidar), `imu` (imu or ahrs), `camera` (depth_camera), `wheel_motor` (dc_motor),
`motor_driver`, `battery_cell` (battery), `tyre`; legged `joint_actuators` (servo).

## Add a new part

1. Copy the closest entry in `src/fleet_components/components/<category>/` to `<vendor>_<model>.yaml`
   (lower case, underscores). Schema and rules: `src/fleet_components/components/README.md`.
2. Fill in from the datasheet, in SI, with the original units in a comment: `model`, `category`, `vendor`,
   `datasheet`, `sources` (every URL used and what for), `specs`, `ros_driver` (the real driver's package,
   topic names and types, `frame_id`, `rate_hz`, QoS), `sim` (how specs map to simulator parameters, with the
   derivation), `limitations` (what is not reproduced).
3. Derivation conventions: IMU white noise stddev = noise density x sqrt(sample rate); bias stddev from the
   bias instability or zero-offset bound; lidar noise from the stated accuracy and say how you read it (1 sigma
   or 3 sigma). If a number is not published, mark it `# ASSUMPTION` and say so in `limitations`; never hide it.
4. Use the new stem in `robots.yaml`, rebuild as above. The tests in `src/fleet_description/test` and
   `src/fleet_sensor_sim/test` check that the names resolve and that the generated sensor matches the entry.

A part category with no xacro macro yet (for example a new sensor type) needs a macro in
`src/fleet_components/urdf/<category>.xacro` that loads the entry with `xacro.load_yaml` (rules in
`src/fleet_components/urdf/README.md`) and, when Gazebo's raw output differs from the real driver's output, an
adapter node in `src/fleet_sensor_sim` (see `lidar_driver_sim.py`, `imu_driver_sim.py`,
`camera_driver_sim.py`). If a physical effect cannot be done with stock Gazebo, write a gz system plugin
(examples: `src/fleet_gz_plugins`, `src/fleet_marine_plugins`, `src/fleet_aircraft`).

## Other kinds

- AUV: change the stems in `src/fleet_auv/config/knucker.yaml`, see [auv.md](auv.md).
- Wyvern: sensor stems in `blender/drone/make_sdf.py`, see [aircraft.md](aircraft.md).
