# The underwater robot (knucker)

Knucker is a 25.5 kg four-thruster hover AUV. Everything physical is derived and cited: mass, inertia, buoyancy,
added mass (strip theory), damping and fin lift from `src/fleet_auv/tools/derive.py`, thrusters from the Blue
Robotics T200 table, battery, DVL, AHRS and Bar30 from the component catalog. Status and open problems are in
[../notes/sea.md](../notes/sea.md); read the README realism table first.

## Run it

```bash
source install/setup.bash
ros2 launch fleet_auv auv.launch.py world:=tank namespace:=knucker       # 20 m deep test tank, headless
ros2 launch fleet_auv auv.launch.py world:=keep                           # the keep lake
ros2 launch fleet_auv auv.launch.py --show-args                           # every argument with its description
```

Arguments: `world` (tank, keep or any `fleet_worlds` world with water), `namespace`, `gui`, `rviz`, `camera`,
`localization` (EKF, default true), `controller` (depth/heading controller), `mission` (waypoint mission from
the world yaml), `cruise`, `log_csv`, `spawn_delay`, `exit_on_done`.
**Known limitation:** `controller:=true` and `mission:=true` use a controller written for the earlier, unrealistic
model; it is unstable on the current one (the lake mission is XFAIL in the tests). Driving the thrusters by hand
works and the vehicle trims correctly at rest. Retuning is slice S3 in `docs/notes/sea.md`.

## Interface (all inside the namespace, default `knucker`)

| topic | type | source |
|---|---|---|
| `thrusters/{port,stbd,vfore,vaft}/pwm_us` | `std_msgs/Float64` | command, microseconds like a real ESC (1500 stop, 1100-1900) |
| `waterlinked_dvl_driver/velocity_report`, `.../odom`, `.../dead_reckoning_report` | `marine_acoustic_msgs/Dvl`, `nav_msgs/Odometry`, `PoseWithCovarianceStamped` (Water Linked DVL A50 driver, private-name topics) | `waterlinked_dvl_driver` |
| `imu/data`, `imu/data_raw`, `imu/mag` | `sensor_msgs/Imu`, `MagneticField` | `microstrain_imu` (3DM-CV7-AHRS driver layout) |
| `pressure`, `temperature`, `depth_pose` | `FluidPressure`, `Temperature`, `PoseWithCovarianceStamped` | `bar30_driver` |
| `battery/state` | `sensor_msgs/BatteryState` | `LiIonBattery` plugin |
| `thrusters/<name>/{thrust,current,rpm}` | `std_msgs/Float64` | `T200Thruster` plugin outputs (N, A, rpm) |
| `imu_gz`, `dvl_gz/velocity`, `odometry_gt` | raw Gazebo sensor outputs | inputs of the drivers above; `odometry_gt` is ground truth, tests only |
| `odometry/filtered`, TF `odom -> base_link` | `nav_msgs/Odometry` | robot_localization EKF (`config/ekf.yaml`) |

```bash
ros2 topic list | grep knucker
ros2 topic pub --once /knucker/thrusters/vfore/pwm_us std_msgs/msg/Float64 "{data: 1550.0}"
```

TF follows REP-105 (map -> odom static, odom -> base_link from the EKF); ground truth is `odometry_gt` and is for
tests only. Check the live list with `ros2 topic list | grep knucker` after a launch.

## Change the vehicle

| change | where |
|---|---|
| swap thruster, ESC, battery, DVL, AHRS or pressure sensor | stems under `components:` in `src/fleet_auv/config/knucker.yaml`; new parts per [swap_a_sensor.md](swap_a_sensor.md) |
| thruster positions or directions | `thrusters:` in the same file |
| hull shape, mass budget, buoyancy | geometry in `fleet_auv/shape.py` and `blender/auv/build_auv.py` (meshes), then `tools/derive.py` and `tools/export_physics.py` regenerate `config/knucker_physics.yaml` and `knucker_derived.json` |
| regenerate SDF and URDF | `ros2 run fleet_auv auv_gen_model` (also done at launch) |

Rebuild after a change: `colcon build --symlink-install --packages-select fleet_auv`. Do not type masses,
inertias or hydrodynamic coefficients by hand: change the inputs and let `derive.py` produce them, so the derivation
stays next to the value (tests in `src/fleet_auv/test` check the SDF against the derivation).

## Plugins

`src/fleet_marine_plugins`: `T200Thruster` (PWM in, vendor table bilinear in PWM and bus voltage, rotor
dynamics, current out) and `LiIonBattery` (4S6P, open-circuit voltage by state of charge, internal resistance).
To add another thruster, add a catalog entry with its table and reuse the plugin parameters.

## Tests

`colcon test --packages-select fleet_auv fleet_marine_plugins` (unit tests, no simulator);
`scripts/run_all_tests.sh --quick` includes the tank ready-state check. The live trim check is
`src/fleet_auv/test/settle_check.py`.
