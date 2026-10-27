# robot-fleet-sim architecture

Custom robots in custom worlds, simulated in Gazebo Harmonic (gz-sim 8) with ROS 2 Jazzy. Geometry is
modelled by headless Blender 5.2 Python scripts; every export is committed, so the ROS side builds without
Blender. Physical numbers come from a catalog of real parts (datasheets) or from derivations recorded
next to the value. Sensors reproduce the ROS 2 drivers of the real products, so a real driver can replace
the simulated one. Status of each realism item and its limits: [docs/notes/](notes/) (one file per lane) and the
"Realism status" table in the [README](../README.md). Day-to-day use: [configuration.md](configuration.md) and
[guides/](guides/).

## Robot kinds

| kind | robots | simulator-side drive | ROS side | launch |
| ---- | ------ | -------------------- | -------- | ------ |
| wheeled | round, box, rover, truck, wyrm (wheeled shell) | `gz_ros2_control` + `diff_drive_controller`, `CoulombFriction` on wheels | Nav2 + `robot_localization` EKF | `fleet_bringup sim.launch.py` |
| legged | hound, spider, wyrm (legs) | `JointGroupPositionController` on 3-DOF legs | `fleet_legged` gait node, leg odometry, EKF, Nav2 | `fleet_bringup sim.launch.py` |
| AUV | knucker | `Hydrodynamics`, `Buoyancy`, `fluid_added_mass`, `T200Thruster`, `LiIonBattery` | depth/pitch/heading/surge controller, LOS mission, EKF | `fleet_auv auv.launch.py` |
| aircraft | wyvern (PX4 quadcopter) | PX4 SITL, `GnssError` plugin | uXRCE-DDS `/fmu/*`, odometry bridge, offboard mission | `fleet_drone drone.launch.py` |

Wheeled and legged robots share one launch; the `kind:` key of the robot's `robots.yaml` entry picks the
controller setup (`wheeled_kind`, `legged_kind` in `sim.launch.py`). Namespaces: only the AUV launch has a
`namespace` argument (the wheeled/legged stack and the drone use global names; see the open items below).

## Data flow

```
Blender (blender/*.py)          meshes, collision primitives, part statistics
   |  committed exports
   v
robots.yaml + mass_properties.yaml + catalog (fleet_components)     <- one config entry per robot
   |  xacro (fleet_description urdf/, fleet_components urdf/)
   v
URDF / SDF  ->  gz-sim (DART + dantzig; worlds from fleet_worlds)
   |  plugins: CoulombFriction, T200Thruster, LiIonBattery, GnssError, gz_ros2_control
   v
raw gz topics (ros_gz_bridge, under /gz/...)
   |  adapters: fleet_sensor_sim, fleet_auv sensor_drivers  (real-driver topic names, frames, covariances, QoS)
   v
ROS 2 topics  ->  robot_localization EKF  ->  Nav2 | auv controller/mission | PX4 offboard mission
```

Per kind:

- **Wheeled.** `build_robots.py` writes OBJ meshes and `part_stats`; `tools/mass_properties.py` turns them
  and the catalog parts into `mass_properties.yaml` (mass, COM, inertia by parallel axis). `robot.urdf.xacro`
  reads `robots.yaml` with `xacro.load_yaml`, instantiates the sensor macros from the catalog, and
  `ros2_control.xacro` the wheel joints. `sim.launch.py` starts Gazebo, spawns the URDF, bridges `/clock` and
  the sensors, starts the sensor adapters, controllers, EKF and (with `nav:=true`) Nav2 on the world map.
  `/cmd_vel` (`TwistStamped`) is the one input.
- **Legged.** Same description pipeline (`legs.xacro`, structure mass built from `robots.yaml` parameters).
  `fleet_legged.gait_node` stands the robot up, turns `/cmd_vel` into foot targets (`gait.py`), solves leg
  inverse kinematics (`kinematics.py`) and publishes joint position commands and leg odometry for the EKF.
- **AUV.** `blender/auv/` and `fleet_auv.model` (`auv_gen_model`) generate the SDF from the catalog and
  `config/knucker_physics.yaml` (S1 derivations in `docs/notes/sea.md`). `auv.launch.py` starts gz (tank or
  keep lake), bridges thrusters and sensors, runs the driver emulators (`waterlinked_dvl_driver`,
  `microstrain_imu`, `bar30_driver`), the EKF, `auv_controller` and `auv_mission`.
- **Aircraft.** `blender/drone/make_sdf.py` writes the wyvern SDF with Pixhawk-class sensors from the catalog;
  `drone.launch.py` starts PX4 SITL (working dir under `.px4/`, never inside the PX4 checkout), the XRCE
  agent, `odom_bridge` and the waypoint mission node.

## Packages (`src/`)

| package | role |
| ------- | ---- |
| `fleet_description` | `config/robots.yaml` (single source of robot numbers and component picks), `mass_properties.yaml` (generated), `structure_materials.yaml`; xacro in `urdf/`; OBJ meshes in `meshes/<robot>/`; `view_robot.launch.py` |
| `fleet_components` | component catalog `components/<category>/<vendor>_<model>.yaml` (schema in `components/README.md`) and xacro macros (`urdf/README.md`); categories: lidar, imu, ahrs, depth_camera, dvl, gnss, barometer, pressure, magnetometer, fc_imu, dc_motor, bldc_motor, servo, thruster, esc, esc_air, propeller, motor_driver, battery, tyre |
| `fleet_worlds` | `worlds/<world>.sdf` + `.yaml` (spawn, goals), `models/<world>_env/`, `maps/` (keep, apartment, warehouse, lab) |
| `fleet_bringup` | `sim.launch.py` (wheeled + legged), EKF, Nav2 overrides, bridge configs, behaviour trees, `patrol.py` |
| `fleet_sensor_sim` | adapter nodes: `lidar_driver_sim` (sllidar_ros2), `imu_driver_sim` (microstrain), `camera_driver_sim` (realsense2_camera) |
| `fleet_legged` | gait, IK, envelope, walking and lidar-plane check tools |
| `fleet_auv` | knucker model generator, controller, mission, sensor emulators, `auv.launch.py`, tank and lake worlds |
| `fleet_drone` | wyvern models, PX4 I/O helpers, `odom_bridge`, `mission`, ULog report, `drone.launch.py` |
| `fleet_gz_plugins` | `CoulombFriction` (isotropic Coulomb cone for DART). `actuator_models.cpp`, `gz_joint_io.hpp`, `roboclaw_system.cpp` are unfinished G4 work and are not in `CMakeLists.txt` |
| `fleet_marine_plugins` | `T200Thruster` (voltage/current-limited, from the vendor table), `LiIonBattery` |
| `fleet_aircraft` | `GnssError` (receiver error model: bias walk, noise, rate, latency) |
| `px4_msgs` | PX4 message definitions, copied from the PX4 tree (version in its package.xml) |

Outside `src/`: `blender/` (model scripts, `blend/` scenes, `previews/`), `tools/` (mass_properties, check_world,
make_map, export_part_stats, fit_motor_prop, imu_noise_params, leg_demand, audit_visual, run_matrix/run_combo),
`scripts/` (install_deps.sh, install_xrce_agent.sh, run_all_tests.sh), `docs/`.

## Frame tree

REP-103 (SI, ENU world, FLU body) and REP-105 (`map -> odom -> base_link`; `base_footprint` for ground robots).

```
map -> odom -> base_footprint -> base_link -> wheel_*_link, caster_*_link, laser_frame, imu_link, camera_link (+ realsense frames)
map -> odom -> base_link -> thruster_*, dvl_link, imu_link, pressure sensor links               (AUV, namespaced)
map -> odom (static: PX4 local origin) -> base_link                                              (wyvern, from odom_bridge)
```

- `map -> odom`: AMCL (Nav2) or slam_toolbox; static for the AUV and wyvern.
- `odom -> base_footprint`: the EKF (`config/ekf.yaml`); the diff-drive controller does not publish it
  (`enable_odom_tf: false`).
- PX4 uses NED/FRD; the conversion to ENU/FLU is done in one place, `fleet_drone/odom_bridge.py`.
- Sensor frame ids follow the real drivers (`laser_frame`, `imu_link`, `camera_link`).
- Everything runs on `/clock` (`use_sim_time: true`).

## Component catalog

One YAML per purchasable product under `fleet_components/components/<category>/`: datasheet values with
their source URL, the real driver it imitates (package, message types, topics, frame ids, rate, QoS) and the
simulation mapping. A robot picks parts by name in `robots.yaml` (`components:`); xacro macros and the adapter
nodes read the same file, so swapping a sensor is a one-name change followed by `tools/mass_properties.py`
(`--check` in the tests catches stale mass entries). Procedure: [guides/swap_a_sensor.md](guides/swap_a_sensor.md).

## Physics settings

Worlds run DART with the dantzig solver (`<engine>` in the Physics system element), step 2 ms. Friction cone
isotropy comes from `CoulombFriction`; floor friction is cited in `docs/notes/ground.md`. Hydrodynamics,
added mass and buoyancy use the `knucker_physics.yaml` values derived in `docs/notes/sea.md`.

## Where the tests live

| what | where | run |
| ---- | ----- | --- |
| description, catalog, mass budget | `src/fleet_description/test/` | `colcon test --packages-select fleet_description` |
| sensor adapter units | `src/fleet_sensor_sim/test/` | colcon test |
| plugin numerics | `src/fleet_marine_plugins/test/`, `src/fleet_aircraft/test/`, `src/fleet_gz_plugins/test/` | colcon test |
| AUV model, drone catalog | `src/fleet_auv/test/`, `src/fleet_drone/test/` | colcon test |
| headless statistics (round) | `src/fleet_bringup/test/` | colcon test |
| world geometry | `tools/check_world.py` | via `scripts/run_all_tests.sh` |
| launch-to-ready, Nav2 goals, AUV and wyvern missions | `scripts/run_all_tests.sh` (`--quick`, `--full`, `--only REGEX`) | writes `docs/notes/test_results.md` |

## Known gaps (details in docs/notes)

- No `namespace` argument for the wheeled/legged launch (G7) and none possible for the drone.
- G4 (DC motor/driver/encoder plugin), G5 (skid steer without measured factors), G6 (legged actuators from real
  Dynamixel limits), G7 (Nav2/EKF from specs), S3 (AUV controller retune; the lake mission is XFAIL), A2-A6 (VTOL,
  ornithopter) were not done in this pass.
- Dragon-themed visuals (keep world, wyrm, wyvern, knucker) are kept as they are.
