# wyvern quadcopter (package fleet_drone, blender/drone)

PX4 SITL x500 (airframe 4001_gz_x500, PX4 v1.18.0-beta1 from a local PX4-Autopilot checkout, run read-only) with the
x500 physics, sensors and motor plugins unchanged and only the visuals replaced by a carbon/violet-scale dragon body
(bat-wing membranes between the arms, horned head with the camera in its maw, tail fin, glowing heart vent).
Rotors stay separate visuals on the x500 rotor links; all collisions are the x500 primitives.
ROS 2 link: uXRCE-DDS (px4_msgs built from the same PX4 checkout, Micro-XRCE-DDS Agent v2.4.3 built from source).

## Build / run
    blender -b --factory-startup -P blender/drone/build_drone.py -- [--no-render] [--review <dir>] [--hero <png>] [--res 1600x900] [--samples 96]
        # meshes/wyvern_{body,rotor_ccw,rotor_cw}.glb + textures, docs/img/wyvern_hero.png; --review: ortho views with collision boxes/prop discs
    python3 blender/drone/make_sdf.py          # regenerates models/wyvern/model.sdf from PX4's x500_base/x500 (only if those change)
    bash scripts/install_xrce_agent.sh         # not needed if .deps/xrce_agent exists (no sudo; builds into <ws>/.deps)
    colcon build --symlink-install --packages-select px4_msgs fleet_drone
    export ROS_DOMAIN_ID=81 GZ_PARTITION=drone       # anything, but set both (user default is 7/shared)
    ros2 launch fleet_drone drone.launch.py world:=lab mission:=true gui:=false rviz:=false shutdown_on_done:=true result_file:=/tmp/r.json
    ros2 launch fleet_drone drone.launch.py world:=lab mode:=hover hover_alt:=2.0 hover_time:=30 shutdown_on_done:=true result_file:=/tmp/h.json
    ros2 launch fleet_drone drone.launch.py world:=keep mission:=false        # PX4 idles on the ground; fly it yourself
    ros2 run fleet_drone wyvern_ulog <.px4/<world>/fs/log/<date>/<t>.ulg> --tag <name> --world <world> --out docs/img --json <metrics.json>
    ros2 run fleet_drone wyvern_route_check [world ...]     # clearance of every mission leg against the world's collision primitives
    ... capture_dir:=<dir>                     # adds a chase camera; frames <dir>/gz_drone_<world>_NNN.png while airborne
QGroundControl is never started; run it yourself and it connects to PX4 on UDP 14550 (MAVLink GCS link of the SITL).
Launch args (full list in the launch docstring): world, mission, mode (mission|hover), hover_alt, hover_time, spawn
(takeoff | a world spawn name), x/y/yaw, mission_file, speed (defaults lab 1.0, apartment 0.8, warehouse 3.0, keep 5.0 m/s),
accept_radius (0.5), shutdown_on_done, result_file, capture_dir, gz_ip (127.0.0.1), fresh_params (true), px4_dir, agent,
xrce_port (8888), spawn_delay (6), px4_delay (4), gui, rviz.
The mission node exits 0 only if every waypoint was reached within accept_radius, the vehicle landed and disarmed.

What the launch starts: `gz sim -s -r --headless-rendering` on a temp copy of the world (magnetometer patch, optional
capture camera), CLI /clock bridge, pose bridge, `ros_gz_sim create` of models/wyvern at the takeoff point
(z = ground + 0.24 m, ground from the world's collision primitives), MicroXRCEAgent udp4 8888, PX4
(`PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME=wyvern PX4_GZ_WORLD=<world>`, working dir
`<ws>/.px4/<world>`, lockstep on the gz clock), wyvern_odom, wyvern_mission (mission:=true), RViz (rviz:=true).
Both nodes get origin (spawn base_link in world coordinates) and geo_lat/geo_model (from the world's
`<spherical_coordinates>`).
PX4 params set through `PX4_PARAM_*`: NAV_DLL_ACT=0, COM_RC_IN_MODE=4, COM_RCL_EXCEPT=4, COM_OF_LOSS_T=1.0 (no RC, no GCS),
SENS_BAR_AUTOCAL=0 (see gotchas).

## Interfaces (ROS)
- `/wyvern/odom` (nav_msgs/Odometry, frame odom -> base_link, ENU/FLU, from PX4 `/fmu/out/vehicle_odometry`), TF
  `map -> odom` (static, = spawn base_link pose in the world) and `odom -> base_link`. The odom origin is the spawn
  base_link, so map coordinates = world coordinates.
- `/wyvern/path` (Path, map), `/wyvern/marker` (mesh marker of the body), `/wyvern/mission_markers` (MarkerArray, latched:
  route line, accept-radius spheres, waypoint labels), `/wyvern/ground_truth/pose` (PoseStamped from gz `/model/wyvern/pose`).
- PX4 in: `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint` (position + velocity + acceleration feed-forward,
  NED, 50 Hz), `/fmu/in/vehicle_command`. PX4 out used: `vehicle_status_v4`, `vehicle_local_position_v1`,
  `vehicle_land_detected`, `vehicle_odometry`, `vehicle_command_ack`.
- All nodes use_sim_time; /clock comes from gz.
- Missions: the `drone:` section of `fleet_worlds/worlds/<world>.yaml` (takeoff x/y, cruise_altitude, waypoints with
  world z), merged with via points from `config/routes/<world>.yaml`; `config/missions/apartment.yaml` is a fallback
  because apartment.yaml has no drone section. Route: takeoff -> cruise altitude -> waypoints (trapezoidal legs,
  yaw towards the leg) -> back over the takeoff point -> land -> disarm.

## Gotchas
- **/clock**: a config-file parameter_bridge that also ran with use_sim_time:=true created the /clock publisher but
  delivered nothing; everything waiting on sim time hung. The launch runs a separate CLI-form clock bridge
  (`/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`, no use_sim_time) plus a config-file bridge for the pose.
- **GZ_IP**: with tailscale up, gz-transport picked 100.x for discovery and in one run PX4's navsat subscription never
  connected: no sensor_gnss, EKF2 xy_valid stayed false, arming refused ("not ready to arm after 120 s"). The launch
  sets GZ_IP=127.0.0.1 for every gz process (gz_ip:= to change); since then PX4 is ready ~7 s after start.
- **Magnetometer units**: PX4 expects the gz Magnetometer system in Tesla with an ENU earth frame. PX4's own worlds get
  that from its server.config; the project's worlds declare the system inline (`<plugin filename="gz-sim-magnetometer-system"/>`),
  so the launch rewrites it in its temp copy to
  `<use_units_gauss>false</use_units_gauss><use_earth_frame_ned>false</use_earth_frame_ned>`. Without it EKF2 rejects
  the mag and never aligns heading.
- **Geodetic scale (WGS84 vs. PX4 sphere)**: gz NavSat turns world metres into lat/lon on the WGS84 ellipsoid, PX4's
  EKF2 projects lat/lon back onto a 6371 km sphere. PX4 local metres = k * world metres, k_north = R/M, k_east = R/N;
  at the worlds' 47.397742 deg N k_east = 0.99707 (east 0.29 % short), k_north = 1.00015. Unnoticed in lab/warehouse
  (< 0.1 m), it put the keep's lake waypoints 0.68-0.93 m off in ground truth while PX4 reported 0.20 m (keep1, fit of
  est - gt against position: -0.306 %/m east, -0.01 % north). The launch reads `<spherical_coordinates>` from the world
  (latitude, surface model) and passes geo_lat/geo_model to wyvern_mission and wyvern_odom, which apply
  `px4io.geo_scale` in both directions; a world without the block gets gz's default (lat 0, WGS84: k_north 1.0056).
- **Vertical speed limits**: PX4 clamps offboard velocity to MPC_Z_VEL_MAX_UP/DN (3.0/1.5 m/s). A 5 m/s leg that drops
  36 m over 32 m asked for 3.7 m/s down; the vehicle lagged 1.05 m for 20 s (keep1). Each leg's peak speed is now capped
  so its vertical component stays below climb_speed (2.0) / vertical_speed (1.2 m/s).
- **Ground-truth timing**: PX4 runs directly on the gz clock, but the gz pose and /clock reach ROS through the bridges;
  est - gt showed a constant ~0.10 s (sim time, at any RTF) x velocity term (0.5 m at 5 m/s), i.e. the pose of an
  instant arrives ~0.1 s after PX4's estimate of it. PX4 message timestamps cannot be used to align (uXRCE-DDS
  timesync maps them onto the agent's wall-clock epoch). wyvern_mission therefore fits one constant offset tau
  (least squares of logged estimate vs. pose history while flying, `gt_time_offset_s` in the result) and evaluates
  ground truth at record time + tau for waypoints, CSV and hover stats; `gt_err_raw` keeps the error against the
  newest pose at the moment of reaching, `gt_stamp_age_ms` the median age of the newest pose vs. the node clock.
- **Baro auto-calibration**: PX4 1.18 SENS_BAR_AUTOCAL shifts the baro to the GNSS altitude (world elevation 488 m)
  ~5-10 s after GNSS lock, on the ground. When the mission armed before that (lab4: ready at t=17.6 s, armed 20.3 s),
  EKF2's height jumped 2.3 m up while the vehicle sat on the floor: setpoint below the estimate, no lift-off,
  "Disarmed by auto preflight disarming". The launch sets SENS_BAR_AUTOCAL=0 (baro AMSL then reads ~0 m, EKF2 height
  is relative anyway), and wyvern_mission only arms after the estimate has been at rest (|v| < 0.1 m/s) for 2 s.
- **Versioned uXRCE topics** (`vehicle_status_v4`, `vehicle_local_position_v1`): resolved at runtime by
  `px4io.find_topic`, so a PX4 update that bumps a version does not break the nodes. Messages must come from the same
  PX4 checkout's `msg/` (src/px4_msgs).
- **Environment**: the user's shell exports ROS_DOMAIN_ID=7, GZ_PARTITION=shared, FASTDDS_BUILTIN_TRANSPORTS=UDPv4;
  override both IDs per sim. PX4 derives UXRCE_DDS_DOM_ID from ROS_DOMAIN_ID.
- **fresh_params:=true** deletes `.px4/<world>/parameters*.bson` so stale values from an older run never leak in.
- MicroXRCEAgent ignores SIGINT: launch escalates to SIGTERM after 5 s (harmless "process has died ... -15" at shutdown).
- "Accel #0 TIMEOUT" / "preflight fail: ekf2 missing data" lines in the first seconds after PX4 starts are benign.
- The project's worlds step at 2 ms (PX4's x500 world uses 4 ms); PX4 lockstep handles both, the IMU runs at 250 Hz.
- PX4's logger keeps `vehicle_local_position_setpoint` at 10 Hz and `trajectory_setpoint` at 5 Hz, so wyvern_ulog
  interpolates the setpoint linearly (a sample-and-hold showed a speed-proportional sawtooth that was not real).
- Lift-off: PX4 runs its own takeoff ramp; the first leg is slow (<= 0.8 m/s, 0.5 m/s^2) and a reference governor slows
  the leg clock while the vehicle lags the setpoint (lift-off leg: stops at 0.40 m lag; other legs at 0.9 + 0.08 v m),
  so the setpoint never runs away from a vehicle still on its skids.
- **Chase camera set_pose**: an in-process `Node.request()` from the Python capture tool timed out on every call in
  the lab (RTF 0.9, image + pose callbacks busy; it had worked in the keep at RTF 0.3). wyvern_gz_capture now calls
  `gz service -s /world/<w>/set_pose` as a subprocess (~0.5-0.9 s, 3 tries).
- After touchdown EKF2's height is off by ~0.3-0.4 m (lab3, lab6, keep3; most likely the baro ground-effect deadzone,
  EKF2_GND_EFF_DZ = 4 m). The vehicle is landed and disarmed by then; the land point itself is reached within 0.1 m.
- ULogs: `<ws>/.px4/<world>/fs/log/<date>/*.ulg` (25-70 MB each, logger mode all); delete old ones now and then.

## Permanent world changes needed (fleet_worlds; not edited here, the launch patches a temp copy)
1. Magnetometer system in every world sdf: replace `<plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>`
   with the version carrying `<use_units_gauss>false</use_units_gauss><use_earth_frame_ned>false</use_earth_frame_ned>`
   (affects nothing else that reads the magnetometer in these worlds; the launch keeps working either way).
2. `apartment.yaml` (currently `drone: null`): fill the `drone:` section (copy of `src/fleet_drone/config/missions/apartment.yaml`) so the
   fallback can go.
3. (done by the worlds' owner meanwhile: every world now has `<spherical_coordinates>` at 47.397742 N, 8.545594 E,
   488 m, WGS84; the launch reads it for the geodetic scale, so keep it in sync with any change.)

## Planned .gitignore entries
    .deps/          # MicroXRCEAgent build + install (scripts/install_xrce_agent.sh)
    .px4/           # PX4 SITL working dirs per world: params, dataman, ULogs
    build/ install/ log/
    blender/drone/__pycache__/  src/fleet_drone/**/__pycache__/

## Test results (headless, one sim at a time, lockstep)
Final code (as on disk), 2026-10-24. Errors are distances to the route point at the moment it counts as reached
(est = PX4 estimate, gt = gz ground truth time-aligned with the fitted offset); accept radius 0.5 m. RTF is sim/wall
during the flight; another agent's simulation (warehouse or keep) ran on the same machine during most runs
(load average 11-14 on this laptop), so the keep's RTF is load-bound, not a model property. Lockstep keeps the flight
itself identical at any RTF.

| run | world / mission | waypoints | max est err | max gt err | flight (sim) | wall | RTF flight | gt offset |
|---|---|---|---|---|---|---|---|---|
| lab6 | lab, cage loop (chase cam on) | 6/6 | 0.10 m | 0.10 m | 65.7 s | 108 s total | 0.86 | 0.080 s |
| wh2 | warehouse, aisles | 8/8 | 0.18 m | 0.21 m | 113.8 s | 128 s | 0.98 | 0.095 s |
| keep4 | keep, court/hall/lake, up to 48 m | 8/8 | 0.20 m | 0.30 m | 355.5 s | 828 s | 0.44 | 0.090 s |
| apt1 | apartment (fallback mission) | 4/4 | 0.14 m | 0.13 m | 31.3 s | 64 s total | 0.99 | 0.090 s |
| keepcap | keep courtyard loop, chase cam (frames only) | 4/4 | 0.17 m | 0.17 m | ~85 s | 331 s total | - | - |

All landed and disarmed, mission node exit 0, no crash/failsafe. Every non-waypoint route point (climb, vias, return,
land) was within 0.30 m by the estimate and 0.35 m by ground truth (vias are passed at speed, accept radius 0.5 m).

Hover hold (hover2, lab, 2.0 m, 30 s, 1501 samples at 50 Hz, RTF 0.98): position std dev
est [0.015, 0.016, 0.010] m, ground truth [0.020, 0.029, 0.026] m (x, y, z); gt mean 2.017 m above the floor.

PX4 ULog analysis (wyvern_ulog, setpoint vs. estimate, airborne):

| run | rms | p95 | max | horiz rms | max speed | max alt | max tilt | motors mean / max | saturation |
|---|---|---|---|---|---|---|---|---|---|
| lab6 | 0.131 m | 0.379 m | 0.432 m | 0.065 m | 1.20 m/s | 3.2 m | 7.5 deg | 0.65 / 0.86 | 0 |
| wh2 | 0.149 m | 0.234 m | 0.429 m | 0.119 m | 3.18 m/s | 8.0 m | 8.3 deg | 0.69 / 0.86 | 0 |
| keep4 | 0.136 m | 0.267 m | 0.403 m | 0.127 m | 5.25 m/s | 47.8 m | 9.7 deg | 0.72 / 0.87 | 0 |

The max (0.40-0.43 m) is the lift-off: the governor holds the setpoint 0.35-0.4 m above the vehicle for the ~7 s PX4
needs to spool up and ramp; after that the error stays under 0.3 m.

History (why the fixes above exist): keep1 (before the geodetic scale and vertical cap) reached every waypoint by the
estimate (<= 0.20 m) but ground truth was 0.68-0.93 m off at the lake and the ULog showed 1.05 m plateaus on the
descents (rms 0.37 m, p95 1.03 m); keep3 with both fixes: gt <= 0.30 m, ULog rms 0.145, p95 0.28, max 0.45 m. lab4 never
lifted off (baro auto-calibration, see gotchas); lab5 = the same mission after that fix (6/6, gt <= 0.15 m).
Older lab3/wh1/hover1 (before the worlds got spherical coordinates): lab 6/6 gt <= 0.35 m, warehouse 8/8 gt <= 0.15 m,
hover gt std [0.016, 0.026, 0.090] m.

Plots: docs/img/drone_{lab,warehouse,keep}_{tracking,path,attitude,motors}.png (wyvern_ulog, from the lab6, wh2 and keep4 ULogs).
Gazebo frames: docs/img/gz_drone_lab_cage.png (lab, leaving the takeoff pad in the cage), docs/img/gz_drone_lab_arm.png (lab, in the
cage by the robot-arm cell), docs/img/gz_drone_keep_court.png (keep courtyard, throne hall behind); captured by
wyvern_gz_capture from `gz sim -s --headless-rendering` (1600x900, ogre2). Hero render: docs/img/wyvern_hero.png (EEVEE, 96 samples).

## Open issues
- Magnetometer units: only patched in the launch's temp world copy; the permanent change (item 1 above) is not made.
- apartment.yaml still has `drone: null`; the launch uses config/missions/apartment.yaml (item 2 above).
- The keep's wall time is 14 min for a 6 min flight while another simulation runs (RTF 0.44; 0.28 with the chase camera
  on as well). Results are unaffected (lockstep), only wall time.
- Lift-off takes ~7 s from arming (PX4 spool-up/ramp under offboard); it dominates the max tracking error in the ULog
  stats (0.40-0.43 m). A velocity-only takeoff setpoint could shorten it; not attempted.
- EKF2 height is ~0.3-0.4 m off after touchdown (see gotchas); harmless for these tests, not investigated further.
- gz pose / clock reach ROS ~0.08-0.10 s after PX4's estimate; handled by the fitted offset (`gt_time_offset_s`),
  not removed. `gt_err_raw` (newest pose) is reported as well and differs by <= 0.02 m.
- MicroXRCEAgent ignores SIGINT; ros2 launch escalates to SIGTERM after 5 s (harmless error line at shutdown).
- QGroundControl was never started (per instructions); the UDP 14550 link is PX4's standard GCS MAVLink instance and
  untested here.
- ULogs are 17-86 MB per flight under .px4/ (logger mode all); nothing prunes them.
- .gitignore is not written (planned entries above); the workspace is not a git repo here.
