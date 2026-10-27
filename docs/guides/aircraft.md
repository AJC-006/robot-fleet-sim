# The aircraft (wyvern, PX4)

Wyvern is a PX4 software-in-the-loop quadrotor (airframe 4001, x500 geometry). Its sensors and propulsion come
from catalog entries: ICM-42688-P IMU, MS5611 barometer, IST8310 magnetometer, u-blox MAX-M10S GNSS with an
error plugin, T-Motor AIR2216II KV920 motors with T1045 propellers. Status and open items:
[../notes/air.md](../notes/air.md). VTOL and flapping-wing airframes are not done yet (slices A2-A6 in that file).

## Requirements

PX4-Autopilot built for `px4_sitl_default` (read only, `px4_dir:=` or `$PX4_DIR`), and the Micro-XRCE-DDS-Agent
(`bash scripts/install_xrce_agent.sh`, installs into `.deps/`). `px4_msgs` is in this workspace.

## Run it

```bash
source install/setup.bash
ros2 launch fleet_drone drone.launch.py world:=lab mission:=true                    # world's waypoint mission
ros2 launch fleet_drone drone.launch.py world:=keep mode:=hover hover_alt:=3.0 hover_time:=20
ros2 launch fleet_drone drone.launch.py --show-args                                  # all 25 arguments
```

The launch starts Gazebo headless, the model, the XRCE agent, PX4 SITL (lockstep on the Gazebo clock),
the odometry/TF bridge and the mission node. PX4's own topics are under `/fmu/in/*` and `/fmu/out/*`
(NED/FRD, `px4_msgs`); the conversion to ROS conventions happens in one place, `fleet_drone/odom_bridge.py`,
which publishes `odometry` (ENU/FLU) and TF. This is why there is no `namespace` argument. Ready when PX4 prints
"Ready for takeoff" and `/fmu/out/vehicle_odometry` publishes.

Missions: waypoints in the world yaml (`drone:` section; see [add_a_world.md](add_a_world.md)) or `mission_file:=`
with the same schema. GNSS errors are on outdoors only (`gnss_errors:=auto`); indoor worlds have no GNSS.

## Change the sensors or motors

`python3 blender/drone/make_sdf.py` regenerates `src/fleet_drone/models/wyvern/model.sdf` from PX4's x500
models and the catalog; change the stems it reads (IMU, barometer, magnetometer, GNSS, motor, propeller), do not
edit the SDF. Add a part as in [swap_a_sensor.md](swap_a_sensor.md). A new motor/propeller pair needs a published
thrust table (CSV in `components/propeller/`), then `tools/fit_motor_prop.py` fits `motorConstant`,
`momentConstant`, maximum rotor speed and time constants. Set `SIM_GZ_EC_MAX1..4` in the launch to the fitted
maximum speed. Sensor names must match what PX4's gz bridge looks up under `base_link`: `imu_sensor`,
`magnetometer_sensor`, `air_pressure_sensor`, `navsat_sensor`.
Test: `python3 -m pytest src/fleet_drone/test/test_air_catalog.py`.

## Add another PX4 airframe

The pattern, following the wyvern:

1. Choose the PX4 airframe whose `PX4_SYS_AUTOSTART` id and gz model your aircraft uses
   (`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`). Mixers and controllers stay PX4's.
2. Write the model SDF from the catalog (copy `blender/drone/make_sdf.py` to a new generator): physical links,
   joints and rotor plugins from PX4's gz model for that airframe, sensors and actuator constants from catalog
   entries, visuals from a Blender mesh (`blender/drone/build_drone.py` is the pattern).
3. Copy `src/fleet_drone/launch/drone.launch.py`, change the model path/name, `PX4_SYS_AUTOSTART`,
   `PX4_GZ_MODEL_NAME` and the `PX4_PARAM_*` values the airframe needs. Declare every argument with a description.
4. Fixed-wing and VTOL airframes also need the aerodynamic data (lift, drag, control-surface coefficients) from a
   source such as AVL or a wind-tunnel report, the airspeed sensor in the catalog (MS4525 class), and a check of
   the airspeed topic in PX4's gz bridge. This is the open work of A2-A6.

## Tests

`scripts/run_all_tests.sh --quick` checks that the wyvern reaches "Ready for takeoff" in the lab;
`--full` flies the lab mission and the keep route (see [../notes/test_results.md](../notes/test_results.md)).
