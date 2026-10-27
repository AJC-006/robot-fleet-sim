# ground: wheeled + legged robots, actuators, sensors, physics

Scope: `src/fleet_description`, `src/fleet_legged`, `src/fleet_bringup`, `src/fleet_gz_plugins`,
catalog categories lidar, imu, camera, depth_camera, wheel_encoder, dc_motor, gearbox, motor_driver, servo,
leg_actuator, battery, tyre; physics settings of the four worlds.

## Status / next

Updated 2026-10-25 after G3 (components, sensors, mass budget). G2 status is under "G2 summary".

Done (G3), evidence in "G3: components, sensors, mass":
- Catalog: lidar C1/S2, IMU CV7-AHRS, depth camera D435, dc_motor Pololu 37D 50:1/70:1, RoboClaw 2x15A,
  servo XM430-W210/XH540-W150, battery Samsung 35E, tyre (material model). All with sources.
- `src/fleet_components/urdf/{lidar,imu,depth_camera}.xacro` read the catalog; robots pick parts by
  name in `robots.yaml` `components:`. Old `sensors.xacro` deleted. Frames: laser_frame, imu_link,
  camera_link and the realsense frame tree.
- New package `fleet_sensor_sim` (ament_python): `lidar_driver_sim` (sllidar_ros2 binning, 720/3240 bins),
  `imu_driver_sim` (imu/data_raw, imu/data, catalog covariances), `camera_driver_sim` (realsense topics, 16UC1
  depth with z^2 noise). Raw gz topics are bridged under /gz/. sim.launch.py starts the adapters.
- Mass: `tools/mass_properties.py` writes `config/mass_properties.yaml` (parallel-axis budget from part
  statistics, catalog parts at their mounts). Old hand `mass`, `link_mass`, `wheel_mass`, `com` removed.
  Totals kg: round 2.01, box 3.12, rover 8.11, truck 8.81, wyrm 2.65, hound 4.03, spider 3.50.
- wheel_mu now comes from the tyre catalog entry (0.725, min with the floor in DART).
- Tests (colcon test): description 31, sensor_sim 8, bringup 4 (headless statistics of round), all pass.
  fleet_gz_plugins fails cpplint on the unfinished G4 files (see below), not on G3 files.
- All 7 robots launch headless and reach ready (controllers active, scan flowing, legs standing).
- `tools/leg_demand.py` (moved from scratch, reads mass_properties) rerun at the real masses: see G3.

Open issues / limitations (details in G3):
- No `namespace` launch argument: Nav2, controllers and EKF use global names. Needs G7.
- Legged structure mass is built from robots.yaml parameters, not mesh statistics.
- S2 outline unpublished: size_estimate. Tyre is a material model, no vendor wheel selected.
- Duplicate catalog entry `components/ahrs/microstrain_3dm_cv7_ahrs.yaml` (sea lane's); left alone.
- Lidar intensities not published (gpu_lidar has none). Camera depth/colour run below 30 Hz sim time
  on this iGPU (measured 7.7 and 19.8 Hz), camera stays `camera:=true` opt-in.
- Rolling resistance (tyre 0.0125) is recorded but not applied until G4.
- Not budgeted: SBC, wiring, fasteners, PCBs, battery holder/BMS.
- Other agent's unfinished files in fleet_gz_plugins/src (actuator_models.cpp, gz_joint_io.hpp,
  roboclaw_system.cpp, not in CMakeLists) fail cpplint; G4 must finish or lint them.

For G4/G5: skid-steer yaw (rover, wz 0.5 for 4 s, mu 0.725, 8.1 kg) gt/odom 0.876 with the 1.234
multiplier in place (G2 measured 0.810 at 6 kg, mu 0.85 without it): retune in G5. Round: gt/odom 1.010.
For G6: leg torque at the real masses exceeds the chosen servos' continuous curve, see G3 table.
For the usability slice: namespace argument, per-robot wheel product, docs for adding a component.

## G3: components, sensors, mass (2026-10-25)

Catalog and macros: `components/<category>/<model>.yaml` (schema in components/README.md), macros in
`fleet_components/urdf/`. A robot lists `components:` in robots.yaml (lidar, imu, camera, wheel_motor,
motor_driver, battery_cell, tyre; legged: joint_actuators {coxa, femur, tibia}, battery_cell) plus
`battery_pack {series, parallel}` and `mounts`. To add a sensor: add a YAML in the category, name it in robots.yaml.

Sensor adapters (verified against sllidar_node.cpp and the D435/CV7 docs, sources in the catalog entries):
- Lidar: gz gpu_lidar (min_angle -pi, max_angle pi - 2pi/N, sigma 0.010 m, 10 Hz) -> scan with
  N = 360*int(samples/360 + 1) bins (C1 720, S2 3240), reversed order, 0.25 mm quantum, no return = inf,
  QoS keep-last 10 reliable, frame laser_frame. Intensities empty.
- IMU: gz imu, ENU, white noise N*sqrt(fs), Gauss-Markov bias (tau 1000 s assumed) and turn-on bias per the
  catalog. imu/data_raw has orientation_covariance[0] = -1; imu/data has ground-truth orientation times one
  per-run error draw (0.25/0.25/0.5 deg 1 sigma). Covariances from the catalog.
- Camera: color 640x360 and depth 848x480 gz sensors; depth to 16UC1 mm, sigma_z = 3.581e-3 z^2, invalid below
  0.195 m; camera_info from the catalog (zero distortion). Topics camera/camera/{color,depth}/....

Measured (round, lab, 15-20 s, sim-time stamps; test_sim_statistics.py asserts these against the catalog):
- scan 10.00 Hz, 720 bins, per-bin std 9.9 mm (catalog 10.0), range limits 0.05-12 m, frame laser_frame.
- imu 100.00 Hz both topics; accel std 2.89/2.97/2.95e-3 (catalog 2.94e-3); gyro std 4.11/4.04/4.10e-4
  (4.12e-4); |a| 9.808; ENU heading with spawn yaw 1.5708 read 1.5767 (offset 0.0059, within 1 sigma).
- depth 7.68 Hz, color 19.8 Hz (30 Hz nominal; gz render load on the iGPU); depth temporal std 9.1 mm at a median
  1.76 m (model 11.1 mm), K fx 446.80 / 462.14 match the catalog.

Mass budget (tools/mass_properties.py; `--check` fails when the yaml is stale, test_description runs it):
| robot | total kg (old hand value) | base_link kg | wheels | casters |
|---|---|---|---|---|
| round | 2.010 (2.5) | 1.467 | 4 x 0.071 | 2 x 0.105 |
| box | 3.121 (3.0) | 2.233 | 4 x 0.222 | 2 x 0.253 |
| rover | 8.109 (6.0) | 6.032 | 6 x 0.302 | none |
| truck | 8.806 (5.0) | 6.872 | 4 x 0.417 | none |
| wyrm | 2.654 (3.5) | 1.032 | legs | none |
| hound | 4.033 (5.0) | 1.536 | legs | none |
| spider | 3.498 (3.0) | 1.193 | legs | none |
Method: structure from the Blender part statistics (area x wall x density, or volume x density), catalog parts
as boxes at their mounts, wheel motor as a solid cylinder on the axle, parallel-axis assembly; full inertia
tensor about the CoM. Legged structure: ABS shell box, aluminium tubes (1.5 mm wall), rubber foot shell from
the robots.yaml dimensions (no per-part statistics exist). Not budgeted: SBC, wiring, fasteners, PCBs, holder, BMS.

Wheeled checks at the new masses (10.8 V, same method as G1, a = 1 m/s^2, 10 % slope, mu 0.725):
round 0.5 m/s avail 0.45 N m vs demand 0.07, traction 0.25; box 0.70 vs 0.15, 0.55; rover 1.16 vs 0.15,
traction 0.53; truck 0.94 vs 0.30, traction 1.10 (above the available torque at 0.6 m/s: the truck is
motor-limited at full speed on grippy floors). Point turns have 1.2-1.6 N m available.

Legged demand (tools/leg_demand.py, speed x0.8, periods 0.8/0.7/0.7), rms joint torque vs the chosen servo's
speed-torque limit at the rms speed (N m): wyrm femur 1.07 vs XH540-W150 0.56, tibia 0.14 vs XM430-W210 0.68, coxa 0.00;
hound coxa 0.86 vs 0.31, femur 0.16 vs 0.70, tibia 1.69 vs 0.75 (all XH540-W150); spider femur 0.87 vs 0.74,
coxa 0.00, tibia 0.27 vs 0.66 (XM430-W210). Peak: wyrm femur 1.96, hound tibia 3.11 N m. So wyrm femur, hound
coxa and tibia, and spider femur exceed the continuous curve; G6 must change servos, gaits or masses. Gaits not retargeted.

Launch check: round, box, rover, truck, wyrm, hound, spider reach ready headless (~16-22 s), adapters exit
cleanly on SIGINT. Regression: round gt/odom yaw 1.010, straight 1.755 m gt = odom.

## G2 summary (status block as of 2026-10-25, kept for detail)
Updated 2026-10-25 after G2 (physics engine and contact). G1 status moved to "G1 summary" below.

Done (G2), evidence in "G2: physics engine and contact":
- Engine DART (gz-physics-dartsim 7.8), solver dantzig, collision detector ode, written explicitly
  in all four worlds and in `blender/worlds/sdfgen.py` (`PHYSICS`). The misleading
  `<physics type="ode">` is now `type="dart"`. Step 0.002 s (keep stays at 0.004 s). Gravity
  0 0 -9.80665 in every world and in sdfgen.
- New package `src/fleet_gz_plugins`, system `fleet_gz_plugins::CoulombFriction`.
  - It turns DART's box friction (two tangent rows on the world X/Y axes) into an isotropic
    Coulomb cone: aligned with the slip when sliding, and set from last-step contact forces per
    link pair when sticking.
  - Loaded in all four worlds.
  - `test/test_coulomb_friction.py` checks 3 closed-form cases; colcon test passes 14/14 (with
    lint). Without the plugin the same pytest fails 2/3 (a = 0.673 vs 3.923 m/s^2; breakaway
    1.138 vs 0.8).
- Robots, wheels: collisions are spheres (cylinders gave yaw ratio 1.11-1.17). kp/kd/minDepth/
  maxVel/fdir1 are removed because DART does not read them (verified in source).
- Robots, friction: wheel_mu and foot_mu are a single value, 1.0 (tyre dry / road dry). Caster
  0.005 (ball transfer unit, < 0.5 % of load).
- Robots, wheel_separation_multiplier:

  | robot | was | now | basis |
  |---|---|---|---|
  | round | 0.9 | 1.0 | measured yaw ratio 1.000 |
  | box | 0.86 | 1.0 | measured yaw ratio 1.000 |
  | rover | 1.15 | 1.234 | 1/0.8101 measured skid yaw ratio, G5 to verify |
  | truck | 1.2 | 1.886 | 1/0.5303 measured skid yaw ratio, G5 to verify |

- Floors (collision c0): lab and warehouse concrete 0.85, apartment laminate 0.51 (cited). The
  table is `FLOOR_SURFACES` in sdfgen.py, picked with `W.floor(..., surface=)`.
- World check: all four worlds load headless with the plugin library mapped, 0 error lines. round
  drove 0.3 m/s for 3 s, then spun at 1 rad/s for 3 s. Ground truth = odom in every world
  (0.900-0.907 m, yaw ratio 1.000, heading drift 0.000 deg). RTF held the 1.0 target.
- Bench: `docs/notes/ground_bench/physbench.py`, with `--cone` to load the plugin.

Open issues / limitations (details in G2 section):
- DART has one friction coefficient, so static = kinetic, and no rolling resistance (G4 must add
  C_rr N r at the wheel).
- The sphere wheel sticks out r - w/2 past each sidewall.
- The stick direction lags one step. Twist-dominated multi-contact assumes uniform patch
  stiffness.
- Plugin cost: about 30 % lower RTF in tiny bench worlds. It is not visible in the full worlds,
  where the 1.0 target holds; unthrottled RTF there was not measured.
- Truck straight-line acceleration is 4 % above mu g, with or without the plugin (cause not
  isolated, G5).
- Zone floors (rubber mats, carpet, asphalt zones) have no collision, so there is no per-zone
  friction. Keep's stone floors carry no surface (no verified stone value), so the tyre mu
  governs there.
- Laminate 0.51 is an EN 13893 slider value (dynamic), not a tyre compound.
- wheel_mu 1.0 is an upper bound until G3 selects the tyre product.

Next, G3 (components/sensors): tyre and caster products, then replace wheel_mu 1.0 and caster
0.005 with datasheet or measured values. Masses per the G1 spec. G4: rolling-resistance torque in
the actuator plugin. G5: verify the 1.234/1.886 multipliers against skid-steer theory (Wong), and
the truck +4 %. G6: femur shell material (0.6 now), remove odom_scale once walking matches. G7:
move physbench cases into launch_testing.

## G2: physics engine and contact (2026-10-25)

### What gz-sim / gz-physics actually read (source-verified)

- **Engine** is chosen only by the Physics system's `<engine><filename>`, and defaults to
  `gz-physics-dartsim-plugin` (gz-sim 8.15 Physics.cc). The SDF `<physics type>` attribute is
  required by the SDF 1.9 spec (`sdformat14/1.9/physics.sdf`: default "ode", required="1") but
  gz-sim ignores it. Worlds now write `type="dart"` to match, plus an explicit `<engine>`.
- **DART solver settings** gz-sim reads: `<dart><collision_detector>` and
  `<dart><solver><solver_type>` (gz-sim src/SdfEntityCreator.cc 428-443).
- **Default collision detector** is ODE (gz-physics 7.8 dartsim/src/EntityManagementFeatures.cc 767,
  `GzOdeCollisionDetector::create`), not FCL. The comment in w_keep.py was corrected.
- **Surface elements dartsim reads** (dartsim/src/SDFFeatures.cc ~884-967): `friction/ode` mu, mu2,
  fdir1, slip1, slip2, restitution, and the collide bitmasks. It ignores kp, kd, max_vel, min_depth,
  torsional and rolling, so those were removed from the robots.
- **How DART combines surfaces:** mu = min(mu_a, mu_b), restitution = product, slip = sum.
- **Friction is a box.** There are two tangent constraint rows, each bounded by +-mu N. The first
  is along fdir1, or the world X axis when fdir1 is not set. It is not a cone.
- **The hook the plugin uses:** gz-sim emits `events::CollectContactSurfaceProperties` per contact
  for collisions that carry `EnableContactSurfaceCustomization`. The `force` argument is zero
  there, because DART fills contact forces only in applyImpulse. Last-step forces come from
  `components::ContactSensorData`, which Physics.cc fills for collisions that have it.

### Bench: `docs/notes/ground_bench/physbench.py`

The bench steps gz-sim in-process with TestFixture (no rendering). Unless stated otherwise:
step 0.002 s, body mu 0.8, ground mu 1.0, g 9.80665.

| case (expected) | DART alone | DART + CoulombFriction | others |
|---|---|---|---|
| free fall g (9.80665) | 9.8066 | 9.8066 | bfs 9.8066, bullet 9.8066, TPE 0 (no dynamics) |
| slide 0 deg / 90 deg, F = 1.5 mu m g (a = 3.9227) | 3.9227 | 3.9227 | |
| slide 30 deg (a = 3.9227 along 30 deg) | 2.346, along x | 3.9227 at 30.0 deg | |
| slide 45 deg (a = 3.9227 along 45 deg) | 0.673 (sqrt 2 box) | 3.9227 at 45.0 deg | bullet 0 (stuck) |
| slide 45 deg, F = 0.9 / 0.98 / 1.02 mu m g (0 / 0 / 0.1569) | | 0 / 0 / 0.1569 | |
| slip threshold 1e-6 / 1e-4 / 1e-2 m/s | | all exact | |
| 4 feet, breakaway ratio F/W, 0 deg (0.81)* | 0.81 | 0.81 | pgs 0.346; bfs 0.196 (torsional 1.0) |
| 4 feet, breakaway at 45 deg (0.81)* | 1.141 | 0.81 (30 deg 0.81, spider 0.81) | |
| rover accel, mu 0.5, yaw 0 / 30 deg (mu g = 4.9033) | – / 6.7405 | 4.9396 / 4.9415 | |
| truck accel, mu 0.5 | 5.11 | 5.11 (+4 %, open) | |
| spin yaw ratio, round / box / rover / truck | skid robots bang-bang 0 or 1 | 1.000 / 1.000 / 0.8101 / 0.5303 | cylinder wheels: round/box 1.11-1.17 |
| straight 0.3 m/s, heading drift (0) | | 0.000 deg (yaw0 0 and 30 deg) | |
| step 0.001 / 0.002 / 0.004 s | slide, feet and spin all exact | | RTF truck drive 2.23 / 4.05 / 7.16 |

\* The force ramp is 0.5 W/s and slip is detected at |v| > 1 mm/s. Reaching 1 mm/s after
breakaway takes t = sqrt(2 * 1e-3 / (0.5 g)) = 0.020 s, which adds 0.010 to the ratio. So 0.81
means breakaway at exactly mu = 0.80. With stick alignment off (`--match-r 0`) the 45 deg result
is 1.138.

RTF without / with the plugin in the tiny bench worlds:

| case | without | with |
|---|---|---|
| slide | 16.45 | 11.6 |
| round spin | 12.3 | 8.1 |
| hound stand | 7.43 | 5.34 |
| spider stand | – | 2.18 |

### Why bullet-featherstone fails (gz-physics 7.8 source and bench)

- **Friction source.** Friction comes from the first collision of a link only, so lumped caster or
  extra collisions act as mu = 1 skids.
- **Spinning friction.** The default torsional coefficient is 1.0 and becomes spinning friction.
  This blocks pivot turns (the earlier Nav2 turning failure) and gives foot breakaway at 0.196
  instead of 0.81.
- **Anisotropic friction.** `setAnisotropicFriction` scales the friction coefficient a second
  time, so effective mu = mu^2.
- **`<ode><mu>` is ignored.** Only `<bullet><friction>` is read.

The bullet (non-featherstone) engine let a 45 deg slide stick (a = 0) and released feet at 0.062.
**Decision:** DART. Among DART's solvers, pgs released feet at 0.346, so the solver is dantzig.

### CoulombFriction plugin (src/fleet_gz_plugins/src/coulomb_friction.cpp)

- **Sliding** (relative tangential point speed > `slip_threshold`, 1e-4 m/s): fdir1 is set along
  the slip and mu2 = 0. DART then applies exactly mu N opposite the slip.
- **Sticking:** the direction comes from the previous step's contact forces, grouped per link pair
  (centroid c, net force F, moment M, polar moment J):

  ```
  dir_i = F/m + (M x r_i)/J
  ```

  This is the incipient-slip field of a rigid patch on a uniform elastic foundation.
  - Why per link pair: with four contacts on one rigid link the problem is statically
    indeterminate, and per-contact forces from DART stayed on the world axes.
  - Result: feet at 45 deg broke away at 0.903 with the per-contact version (v2), and at 0.81
    with the per-link-pair version (v3).
- **Anisotropic collisions** (mu != mu2) are left untouched, and the plugin warns once.

### Parameters and sources

| parameter | value | source / derivation |
|---|---|---|
| gravity | 9.80665 m/s^2 | standard gravity, 3rd CGPM 1901 (BIPM SI Brochure) |
| wheel_mu, foot_mu | 1.0 | Engineering ToolBox "Tire, dry / Road, dry" 1.0; "Rubber / dry asphalt" static 0.9 (engineeringtoolbox.com/friction-coefficients-d_778.html). Upper bound so the floor pair value governs (min rule); G3 replaces it with the tyre product. |
| caster mu | 0.005 | Omnitrack ball transfer units, rolling resistance "less than 0.5 % of the conveyed load" (omnitrack.com/ball-transfer-units/spring-loaded-t-b-bars/); G3 replaces it with the chosen caster |
| femur shell mu | 0.6 | soft plastic on dry concrete (Roymech table via web.mae.ufl.edu/designlab "Friction coefficients"); G6 picks the material |
| concrete floor | 0.85 | soft rubber on dry concrete 0.85 (Roymech table, same page); Engineering ToolBox rubber/concrete 0.6-0.85 |
| laminate floor | 0.51 | EN 13893:2003 dry dynamic COF, 8 mm AC3/AC4 laminate, EPH Dresden certificate BOD-19-12-16-02 (2019) for Kastamonu Entegre FLOORPAN (kastamonuentegre.com/uploads/2022/12/017-floorpansun-artffloorsun-8mmac3ac4-2000-sliding-behaviour-test-certificate.pdf). The slider material is not verified (paid standard). |
| asphalt (table only) | 0.9 | Engineering ToolBox rubber / dry asphalt, static |
| step | 0.002 s | bench: identical results at 0.001-0.004 s |
| wheel_separation_multiplier | round/box 1.0; rover 1.234; truck 1.886 | 1 / measured spin yaw ratio with isotropic Coulomb contact |

Not used because no source could be retrieved: stone and ceramic pair values. The Demarch et al.
2019 IJMQE paper and the publisher pages were blocked.

### World check (headless, gui:=false rviz:=false, robot round)

Script: `docs/notes/ground_bench/drive_check.py`, which compares Gazebo `/world/<w>/pose/info`
with `/diff_drive_controller/odom`.

| world | straight gt / odom (m) | heading drift | spin gt / odom (rad) | yaw ratio | RTF (target 1.0) |
|---|---|---|---|---|---|
| lab | 0.9000 / 0.9000 | 0.000 deg | 2.9999 / 2.9999 | 1.000 | 1.000 |
| apartment | 0.9000 / 0.9000 | 0.000 deg | 3.0019 / 3.0019 | 1.000 | 1.000 |
| warehouse | 0.9030 / 0.9030 | 0.000 deg | 3.0498 / 3.0498 | 1.000 | 1.000 |
| keep (0.004 s) | 0.9072 / 0.9072 | 0.000 deg | 3.0120 / 3.0120 | 1.000 | 1.000 |

In every world, `libfleet_coulomb_friction.so` was mapped in the gz server process and the
launch logs contain no error lines.

Validation tests from the G1 spec:
- 1 (engine): passed, by the `<engine>` element plus the mapped library.
- 2 (free fall): passed, 9.8066.
- 4 (yaw ratio 0.97-1.03 with multiplier 1.0): passed for round and box. Skid steer moves to G5.

### Limitations (honest)

- **Friction model.** Single mu (static = kinetic), and no rolling resistance in DART.
- **Sphere wheel.** It sticks out r - w/2 past each sidewall, which matters on narrow kerbs and
  steps. Its inertia is still the real cylinder.
- **Plugin approximations.** One-step lag in the stick direction; uniform-stiffness assumption for
  twist-dominated multi-contact.
- **Floors.** No per-zone friction (zone floors have no collision). Keep has no floor surface.
- **Open measurements.** Truck +4 % acceleration is unexplained. Unthrottled RTF in the full
  worlds is not measured.

## G1 summary (2026-10-24)

Status block as left by G1 (audit, BOM, spec). The "Engine" open issue and the G2 "Next" list are
resolved in the G2 section above.

Done (G1): full audit table below; real BOM for all 7 robots with datasheet numbers and feasibility
checks; spec for G2-G7 (catalog entries, motor/servo models, mass method, sensor models, driver
interfaces, validation tests). Offline checker: `tools/leg_demand.py`
(`leg_demand.py <robot> [mass] [speed_scale] [period]`; G6 moves it into `fleet_legged/scripts`).

Decisions (numbers in the BOM / spec sections):
- Wheels: Pololu 37D 12 V gearmotor + 64 CPR encoder. round 50:1 (#4753, 3200 counts/rev),
  box/rover/truck 70:1 (#4754, 4480 counts/rev). At 3S nominal 10.8 V all max_vel are reachable
  (round 76 %, box 71 %, rover 51 %, truck 61 % of no-load speed) with torque margin.
- Driver: Basicmicro RoboClaw 2x15A (round/box 1, truck 2, rover 3). ROS 2 driver
  dumbotics/roboclaw_hardware_interface: velocity command, POSITION state only -> sim must export
  only position and diff_drive_controller uses `position_feedback: true`.
- Lidar: Slamtec RPLIDAR C1 (round, box, wyrm, spider), RPLIDAR S2 (rover, truck, hound); driver
  sllidar_ros2 (source build, not in Jazzy apt). C1 -> 720-bin scan, S2 -> 3240 bins (driver binning).
- IMU: MicroStrain 3DM-CV7-AHRS on all (8.3 g; gyro 8.5 deg/h/rtHz, BI 1.5 deg/h; accel 30 ug/rtHz,
  BI 18 ug) via ros-jazzy-microstrain-inertial-driver 4.10 (imu/data, imu/data_raw, frame imu_link, ENU).
  Phidgets MOT0110 rejected: its noise spec is a +- amplitude with no statistical definition.
- Camera: Intel RealSense D435 on all (72 g, baseline 50 mm, depth 87x58 deg, color 69x42 deg) via
  realsense2_camera 4.58; sim profiles pinned to depth 848x480x30, color 640x360x30.
- Battery: Samsung INR18650-35E packs, 3S1P (round) and 3S2P (others); 3S (12.6 V full) suits Pololu
  12 V motors and Dynamixel X (10-14.8 V; legged packs cut at 10.0 V = 3.33 V/cell).
- Legs (ROBOTIS Dynamixel X, ros2_control driver dynamixel_hardware_interface 1.5.2). Linear T-N
  check max(w/w0 + tau/tau_s) <= 0.90 at 11.1 V (= <= 1 at the 10 V pack cut-off):
  wyrm coxa+tibia XM430-W210, femur XH540-W150, m <= 4.5 kg, speed x0.8, period 0.8 s -> 0.48/0.56/0.68;
  hound 12x XH540-W150, m <= 6.5 kg, speed x0.8, period 0.7 s -> 0.46/0.70/0.79;
  spider 24x XM430-W210, m <= 4.0 kg, speed x0.8, period 0.7 s -> 0.65/0.84/0.66.
  Present configs are infeasible with any single X-series servo (wyrm femur 0.97 on XM430-W210,
  hound tibia 1.10 even on XH540-W150: the 0.45 s period gives 7.6 rad/s swing). Existing 16-25 rad/s
  velocity limits and 4-14 N m efforts have no actuator behind them.

Open issues:
- CV7 datasheet PDF could not be fetched (microstrain.com redirects, HBK/Mouser block scripts); values
  are from the vendor datasheet 8400-0141 as quoted by two sources that agree. G3 confirms by hand-
  downloading the PDF before writing the catalog entry (also ranges, max rate, AHRS accuracy).
- RPLIDAR S2 official manual blocked; mass 185 g (Waveshare wiki) vs 190 g (Core Electronics).
- Datasheet accuracy figures (C1/S2 +-30 mm) have no statistical definition; the spec models them as
  3 sigma and says so in `limitations`; replace with measured data if any becomes available.
- Unpublished: Pololu rotor inertia and inductance, gearbox backlash; RoboClaw PID loop rate;
  Dynamixel firmware loop rate. G4 searches manuals; whatever stays unpublished is listed, not invented.
- Engine: bullet-featherstone gives no wheel traction (root cause open); DART cylinder diff-drive yaw
  ratio inconsistent between preliminary runs (1.00 vs 1.11/1.17). Worlds gravity 9.8 (not 9.80665),
  floors carry no `<surface>` friction, sdfgen.py writes `<physics type="ode">`.
- Tyres/wheels: real wheel products and a cited rubber-on-floor mu / rolling resistance still to pick.
- Masses are caps from the checks, not yet budgets: G3 computes them (method below) and reruns checks.

Next, G2 (physics engine and contact) needs: the audit rows marked G2; engine test scripts
(now `docs/notes/ground_bench/`); decide DART vs bullet-featherstone by root cause, not by fit; set
gravity 9.80665 (WGS-84 standard gravity, CGPM 1901) in worlds and sdfgen.py under worlds.lock;
drop ODE-only kp/kd/minDepth/maxVel or move to an engine that reads them; floor `<surface>` friction
from a cited source; remove wheel_separation_multiplier and odom_scale only once the physics is right
(yaw ratio test below, pass 0.97-1.03 with multiplier 1.0).

## Audit

G1 snapshot (2026-10-24). Rows marked G2 are resolved in the G2 section (values now in the files).

Values as found on 2026-10-24. "plan" names the slice that replaces each one.

| parameter | where | value found | plan | source / note |
|---|---|---|---|---|
| physics engine | worlds `<physics type="ode">`, blender/worlds/sdfgen.py 106-110, 156-157 | "ode" claimed, dartsim used | G2 | gz-sim 8.15 Physics.cc: `type` unused; no `<engine>` -> gz-physics-dartsim-plugin |
| collision detector | worlds | not set (GzOdeCollisionDetector) | G2 | gz-physics 7.8 dartsim EntityManagementFeatures.cc |
| step size | worlds, sdfgen.py default | 0.002; keep.sdf 0.004 (sea lane, Buoyancy) | G2 | keep keep.sdf as is |
| gravity | all worlds, sdfgen.py 123 | 0 0 -9.8 | G2 | standard gravity 9.80665 m/s^2 |
| floor friction | world floors | no `<surface>` (engine default) | G2 | none |
| contact kp/kd/minDepth/maxVel | wheel, caster, foot `<gazebo>`; foot_kp/kd 1e5/50 robots.yaml | 1e6/100, 1e5/50 | G2 | ODE-only; dartsim SDFFeatures.cc reads only mu, mu2, fdir1, slip1/2, restitution |
| wheel mu1/mu2 | robots.yaml body.wheel_mu | round/box 1.0/1.0, rover 0.9/0.6, truck 1.0/0.7 | G2/G5 | none |
| caster mu | caster.xacro | 0.01 | G2 | none |
| foot mu / femur mu | robots.yaml / legs.xacro | 1.0 / 0.6 | G2/G6 | none |
| wheel joint damping | wheel.xacro | 0.01 N m s | G4 | replaced by motor model (back-EMF, I0) |
| wheel_separation_multiplier | robots.yaml | round 0.9, box 0.86, rover 1.15, truck 1.2 | G2/G5 | test-fit; remove |
| wheel velocity cmd limit | ros2_control.xacro | +-30 rad/s | G4 | no-load speed at pack voltage |
| wheel collision | wheel.xacro | cylinder r x w | G2 | |
| robot masses | robots.yaml mass | 2.5/3.0/6.0/5.0/3.5/5.0/3.0 kg | G3 | component + structure budget |
| base inertia | robots.yaml inertia_box; base_mass = mass - parts - 0.0001 | solid box | G3 | mesh-based method |
| wheel mass | robots.yaml | 0.08/0.15/0.25/0.3 kg | G3 | wheel product |
| caster/lidar/imu/camera mass | robot.urdf.xacro | 0.02/0.1/0.01/0.03 kg | G3 | catalog |
| sensor_link / camera_optical inertia | sensors.xacro | 0.04 m box / 0.0001 kg | G3 | catalog dims |
| leg link inertia | legs.xacro 78-127 | boxes/cylinders of link_mass | G3/G6 | servo + bracket masses |
| diff_drive limits | sim.launch.py 87-120 | 1.5 m/s^2, 4.0 rad/s^2, vx_min -0.4 vx | G5 | from motor torque / traction |
| diff_drive rates | sim.launch.py | update 100, publish 50, cmd_vel_timeout 0.5 | G5 | keep, document |
| diff_drive covariances | sim.launch.py | pose/twist 0.001, 0.03 | G7 | encoder resolution + slip test |
| max_vel_x / theta | robots.yaml | 0.5/2.0, 0.5/2.0, 0.4/1.5, 0.6/2.0 | G4 | all reachable (BOM check) |
| lidar | sensors.xacro | gpu_lidar 360 samples, 0.12-20 m, sigma 0.01, 10 Hz, frame lidar_link | G3 | C1/S2 |
| IMU | sensors.xacro | topic imu, 100 Hz, gyro 5e-4, accel 5e-3, no bias | G3 | CV7 |
| camera | sensors.xacro | 640x480, hfov 1.2, 15 Hz, near 0.05 far 20, no distortion | G3 | D435 |
| gz bridge | gz_bridge.yaml, gz_bridge_camera.yaml | absolute /clock /scan /imu /camera/* | G3 | driver topics, relative, namespaced |
| leg joint effort | robots.yaml joint.effort | 6/14/4 N m | G4/G6 | servo model |
| leg joint velocity | robots.yaml joint.velocity | 16/25/20 rad/s | G4/G6 | servo model |
| leg joint damping | robots.yaml | 0.02/0.03/0.01 | G4 | servo model |
| position_proportional_gain | sim.launch.py | 0.5 x 250 Hz | G4 | Dynamixel firmware PID |
| servo_lead | gait_node.py | 8 ms | G4/G6 | remove with the above |
| legged controller rates | sim.launch.py | controller 250, joint_state_broadcaster 125, hold_joints | G6 | |
| gait node params | gait_node.py | odom_alpha 0.25, odom_trim 0.12, level_tau 0.08, stand_time 2.0 | G6 | |
| odom_scale | robots.yaml gait | wyrm 1.03, hound 1.075/1.04, spider 1.01/1.02 | G6 | fit; remove |
| leg odom covariance | gait_node.py | pose 0.01, twist 0.004, yaw 0.05 | G6/G7 | |
| gait accel limits | sim.launch.py | 0.5 m/s^2, 1.5 rad/s^2 | G6 | |
| EKF | ekf.yaml | 50 Hz, two_d_mode, odom0 vx,vy, imu0 vyaw on /imu, sensor_timeout 0.2 | G7 | CV7 topic imu/data |
| AMCL / costmap | nav2_overrides.yaml | alpha 0.05/0.02, sigma_hit 0.1, max_beams 120, obstacle_max_range 5.0 | G7 | from lidar spec |
| inflation / plan radius | sim.launch.py | max(0.45, r+0.25) / inscribed+0.10 | G7 | |

## Bill of materials

Every value carries its source; "derived" values are computed below.

### Sources

| item | datasheet / page |
|---|---|
| Pololu 37D gearmotors | https://www.pololu.com/file/0J1736/pololu-37d-metal-gearmotors-rev-1-2.pdf (rev 1.2), https://www.pololu.com/product/4753, /4754 |
| RoboClaw 2x15A | https://downloads.basicmicro.com/docs/roboclaw_datasheet_2x15A.pdf (v3.0) |
| RoboClaw ROS 2 driver | https://github.com/dumbotics/roboclaw_hardware_interface (Apache-2.0) |
| RPLIDAR C1 | https://cdn.shopify.com/s/files/1/0058/3145/8904/files/SLAMTEC_rplidar_datasheet_C1_v1.1_en.pdf |
| RPLIDAR S2 | https://www.slamtec.com/en/s2 ; https://www.waveshare.com/wiki/RPLIDAR_S2 |
| sllidar_ros2 | https://github.com/Slamtec/sllidar_ros2 (src/sllidar_node.cpp) |
| 3DM-CV7-AHRS | https://www.mouser.com/datasheet/2/1083/Parker_8400_0141_3DM_CV7_Datasheet-3218130.pdf (8400-0141) |
| MicroStrain driver | https://github.com/LORD-MicroStrain/microstrain_inertial , microstrain_inertial_driver_common/config/params.yml |
| RealSense D435 | https://www.realsenseai.com/wp-content/uploads/2020/06/Intel-RealSense-D400-Series-Datasheet-June-2020.pdf (337029-009) |
| realsense2_camera | https://github.com/IntelRealSense/realsense-ros (README, ros2-master) |
| Samsung INR18650-35E | https://www.orbtronic.com/content/samsung-35e-datasheet-inr18650-35e.pdf (Samsung SDI spec v1.1) |
| Dynamixel X | https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/ , /xh540-w150/ , /xh540-w270/ , /xl430-w250/ |
| Dynamixel ROS 2 driver | https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface (jazzy branch) |

### Per robot

| robot | actuators | driver | battery | lidar | IMU | camera | mass |
|---|---|---|---|---|---|---|---|
| round | 2x Pololu 37D 50:1 64 CPR | 1x RoboClaw 2x15A | 3S1P 35E | RPLIDAR C1 | CV7-AHRS | D435 | G3 budget (now 2.5) |
| box | 2x Pololu 37D 70:1 64 CPR | 1x RoboClaw | 3S2P | C1 | CV7 | D435 | G3 (3.0) |
| rover | 6x 37D 70:1 (skid) | 3x RoboClaw | 3S2P | S2 | CV7 | D435 | G3 (6.0) |
| truck | 4x 37D 70:1 (skid) | 2x RoboClaw | 3S2P | S2 | CV7 | D435 | G3 (5.0) |
| wyrm | 8x XM430-W210 (coxa, tibia) + 4x XH540-W150 (femur) | U2D2 + dynamixel_hardware_interface | 3S2P | C1 | CV7 | D435 | cap 4.5 |
| hound | 12x XH540-W150 | same | 3S2P (G6 checks current) | S2 | CV7 | D435 | cap 6.5 |
| spider | 24x XM430-W210 | same | 3S2P | C1 | CV7 | D435 | cap 4.0 |

### Key values

Pololu 37D 12 V (datasheet tables; motor alone 10,000 rpm, 0.2 A no-load, 5.5 A stall at 12 V; with
encoder 190-210 g by ratio; recommended continuous <= 100 kg mm = 0.98 N m, instantaneous <= 250 kg mm;
stall values extrapolated; keep current <= 25 % of stall):
- 50:1 #4753: 200 rpm, 210 kg mm (2.06 N m) stall, 5.5 A; graph fit w = 200 - 1.0 tau [rpm, kg mm],
  I = 0.080 + 0.026 tau [A]; 3200 counts/output rev.
- 70:1 #4754: 150 rpm, 270 kg mm (2.65 N m), 5.2 A; fit w = 150 - 0.53 tau, I = 0.088 + 0.018 tau;
  4480 counts/output rev.
- derived (motor section): 50:1 R 2.22 ohm, keN 0.5604 V s/rad, NetaKt 0.377 N m/A, eta 0.673;
  70:1 R 2.31, keN 0.7477, NetaKt 0.545, eta 0.729.

Wheeled checks at 10.8 V (no-load 18.85 rad/s for 50:1, 14.14 for 70:1; available torque on the
linear T-N line at the required speed; demand = m(a + g sin) r / n_driven with a = 1 m/s^2, 10 % slope):
round 0.5 m/s -> 14.29 rad/s, 0.45 N m available vs ~0.10 needed; box 0.5 m/s -> 10.0 rad/s,
0.69 vs ~0.16; rover 0.4 m/s -> 7.27 rad/s, traction-limited demand mu(mg/6)r = 0.43 N m
at mu 0.8, point turn 4.6 rad/s -> 1.5 N m available; truck 0.6 m/s -> 8.57 rad/s (61 %), traction
0.69 N m (< 0.98 continuous), point turn 4.86 rad/s -> 1.56 N m. mu 0.8 is a placeholder (Wong,
Theory of Ground Vehicles, dry concrete peak 0.8-0.9) until G5 picks the tyre source.

RoboClaw 2x15A: 6-34 V, 15 A continuous / 30 A peak per channel, 4.3 mohm on-resistance, 62 g,
quadrature up to 9.8 M PPS, 32-bit counters, programmable current limit, velocity PID with
auto-tune, regenerative. PID loop rate unpublished in the datasheet (G4: user manual).

RPLIDAR C1: 0.05-12 m (70 % reflectivity), 0.05-6 m (10 %), 5 kHz, 8-12 Hz (10 typ), 0.72 deg,
accuracy +-30 mm, resolution 15 mm, flatness 0-1.5 deg, 40 klux, 905 nm class 1, 110 g, UART 460800.
RPLIDAR S2: 0.05-30 m (90 %), 10 m (10 %), 32 kHz, 10 Hz, 0.1125 deg, accuracy +-30 mm, resolution
13 mm, 80 klux, IP65, 1 Mbaud, 185-190 g.

3DM-CV7-AHRS: accel noise density 30 ug/rtHz, bias instability 18 ug; gyro noise density
8.5 deg/h/rtHz (= ARW 0.14 deg/rth), bias instability 1.5 deg/h; 8.3 g, 38 x 24 x 8.6 mm.
(HBK page lists 20 ug/rtHz and 0.3 deg/rth "random walk": G3 resolves from the PDF.)

D435 (337029-009): baseline 50 mm; depth FOV 87+-3 x 58+-1 deg (HD), 75 x 62 (VGA); color OV2740
rolling shutter 69+-1 x 42+-1 deg, up to 1920x1080; depth imagers global shutter, f 1.93 mm;
Min-Z 280 mm @1280x720, 195 @848x480, 175 @640x480; Z-accuracy <= 2 %, RMS spatial noise <= 2 %,
temporal noise <= 1 % (up to 2 m, 80 % ROI, HD); 90 x 25 x 25 mm, 72 g (68.4-75.6).

INR18650-35E: 3.6 V nominal, 4.2 V charge, 2.65 V cut-off, 3350 mAh min, 8 A continuous (13 A non-
continuous), <= 35 mohm AC 1 kHz, 50 g max, 18.55 x 65.25 mm. 3S1P 36.2 Wh, 8 A; 3S2P 72.4 Wh, 16 A.

Dynamixel (e-manual; 4096 pulse/rev, velocity unit 0.229 rpm, current unit 2.69 mA, PWM unit 0.113 %):

| model | mass | ratio | stall N m (A) @11.1 / 12 / 14.8 V | no-load rpm @11.1/12/14.8 | defaults |
|---|---|---|---|---|---|
| XM430-W210 | 82 g | 212.6 | 2.7 (2.1) / 3.0 (2.3) / 3.7 (2.7) | 70 / 77 / 95 | P 800, I 0, D 0; vel limit 330; current limit 1193 |
| XH540-W150 | 165 g | 152.3 | 6.6 (4.5) / 7.1 (4.9) / 8.5 (5.9) | 66 / 70 / 86 | vel limit 300; current limit 2047 |
| XH540-W270 | 165 g | 272.5 | 9.2 (4.5) / 9.9 (4.9) / 11.7 (5.9) | 36 / 39 / 46 | P 800; vel limit 167; current limit 2047 |
| XL430-W250 | 57.2 g | 258.5 | 1.0 @9 V, 1.4 @11.1, 1.5 @12 | 47 / 57 / 61 | 6.5-12 V: 3S full 12.6 V exceeds; no current sensing; rejected |

XM430/XH540 input 10.0-14.8 V (recommended 12). ROBOTIS: stall rating differs from continuous output;
the performance graph is closer to real behaviour (G4 digitizes the 12 V graph).

## Spec for G2-G7

### Catalog entries (src/fleet_components/components/<category>/)

- dc_motor/pololu_37d_12v_50to1_64cpr.yaml: Pololu #4753; 0J1736; 200 rpm, 2.06 N m, 5.5 A, R 2.22, keN 0.5604, NetaKt 0.377, I0 0.080 A, 3200 counts/rev, 64 CPR motor-side quadrature (gearbox + encoder blocks inside; integrated product).
- dc_motor/pololu_37d_12v_70to1_64cpr.yaml: #4754; 150 rpm, 2.65 N m, 5.2 A, R 2.31, keN 0.7477, NetaKt 0.545, I0 0.088 A, 4480 counts/rev.
- motor_driver/basicmicro_roboclaw_2x15a.yaml: 2 ch, 15/30 A, 6-34 V, 4.3 mohm, 62 g; ros_driver roboclaw_hardware_interface (velocity cmd, position state, params serial_port, address, qppr, motor_type).
- lidar/slamtec_rplidar_c1.yaml: values above; scan_mode Standard, 500 samples/rev, 720 published bins.
- lidar/slamtec_rplidar_s2.yaml: scan_mode DenseBoost, 3200 samples/rev, 3240 bins, range_max 30.
- imu/microstrain_3dm_cv7_ahrs.yaml: noise densities and BI above, 8.3 g.
- depth_camera/intel_realsense_d435.yaml: values above; profiles depth 848x480x30, color 640x360x30.
- battery/samsung_inr18650_35e.yaml: cell values; pack series/parallel set per robot.
- servo/robotis_xm430_w210.yaml, servo/robotis_xh540_w150.yaml: table values, control-table units and defaults.
- tyre/<vendor>_<model>.yaml: G3/G5 after the wheel products are picked (diameter, width, mass, mu, rolling resistance with source).

### DC motor + gearbox + encoder (G4 plugin, wheels)

Quasi-static (inductance unpublished, so the electrical pole is dropped and listed as a limitation):
V = d * V_batt (d = driver PWM duty); I = (V - keN w) / R, clamped to the driver current limit;
tau_out = NetaKt (I - I0 sgn w). From the Pololu graph fits (tau in kg mm, I in A, w in rpm, 12 V):
R = 12 / I_stall; keN = 12 / w_noload(rad/s); NetaKt = 9.80665e-3 / slope_I; eta = NetaKt / keN.
Encoder: position state quantized to 2 pi / counts_per_rev; no velocity state (RoboClaw driver).
RoboClaw velocity PID on counts/s with QPPS feed-forward (form and loop rate from the user manual,
G4). Unpublished, listed in `limitations`: rotor inertia, gearbox backlash, inductance.

### Leg actuator (G4 plugin, Dynamixel X)

Same DC model per servo with R = V / I_stall, keN = V / w0, kt_out = stall N m/A from the table
(XH540-W150 @12 V: R 2.45 ohm, keN 1.637 V s/rad, kt_out 1.449 N m/A; XM430-W210: 5.22, 1.488, 1.304).
Firmware: position PID on pulse error (KPP = P/128, KPI = I/65536, KPD = D/16 per e-manual) -> PWM
(limit 885) -> voltage; current limit, velocity limit, profile velocity/acceleration (0 = none).
States: Present Position (4096/rev), Present Velocity (0.229 rpm units), Present Current (2.69 mA).
T-N check rule: max over the gait of w/w0 + tau/tau_s <= 0.90 at 11.1 V. Swing-leg gravity and
inertia torques are not yet in leg_demand.py (G6 adds them once link masses are known).

### Mass, inertia, CoM (G3)

Generator script (fleet_description/scripts/mass_properties.py) writes a committed yaml per robot;
a test fails when it is stale (xacro cannot import Python). Per part: catalog components as boxes or
cylinders at their mounting pose with datasheet mass and dimensions; structure from the Blender mesh:
shell area x wall thickness x density (printed/sheet parts) or closed volume x density (solid
parts), material and thickness stated per part with source (e.g. PLA 1.24 g/cm^3 from the filament
datasheet). Parallel-axis sum gives total mass, CoM and inertia. robots.yaml `mass` becomes an output.

### Sensor models (G3)

- Lidar: gpu_lidar at device-native samples/rev (C1 500, S2 3200), 10 Hz; range noise gaussian with
  sigma = 30 mm / 3 = 10 mm (datasheet +-30 mm taken as 3 sigma, stated as an interpretation).
  A driver-emulation node (C++, fleet_gz_plugins) reproduces sllidar_ros2: angle_compensate binning
  (multiple = int(points_per_circle/360 + 1): C1 2 -> 720 bins, S2 9 -> 3240), each sample fills
  `multiple` bins, reversed (ROS angle = pi - device angle, so the URDF frame is yawed pi from the
  housing front), angle_min -pi, angle_max pi, increment 2 pi/(N-1), range_min 0.05, range_max from
  scan mode, no return -> +inf (REP-117), ranges quantized 0.25 mm, scan_time = rev time,
  time_increment = scan_time/(N-1), stamp = scan start.
- IMU: per-sample white noise sigma = N sqrt(f_s) (Allan definition: ADEV(1 s) = N). At 100 Hz:
  gyro 4.12e-5 rad/s/rtHz -> 4.12e-4 rad/s; accel 2.94e-4 m/s^2/rtHz -> 2.94e-3 m/s^2. Bias as a
  first-order Gauss-Markov process (gz dynamic_bias_stddev / correlation_time) tuned so the simulated
  Allan floor equals the datasheet BI (gyro 7.27e-6 rad/s, accel 1.77e-4 m/s^2).
- D435: depth fx = (W/2)/tan(HFOV/2) = 446.8 px at 848x480 (HFOV 87 deg), color fx = 465.6 px at
  640x360 (69 deg); square pixels, principal point at centre, distortion zero (verify against a real
  camera_info in G3). Depth noise sigma(z) = z^2 * 0.08 / (fx * 0.05) (subpixel 0.08, Intel whitepaper
  "Tuning depth cameras for best performance") = 14.3 mm at 2 m (<= 2 % spec); z < Min-Z (0.195 m)
  -> 0; output 16UC1 mm, 0 = invalid.

### Real driver interfaces the sim reproduces

| product | package | topics (type) | frame_id | rate | QoS |
|---|---|---|---|---|---|
| RPLIDAR C1/S2 | sllidar_ros2 | scan (sensor_msgs/LaserScan) | param frame_id (default laser_frame) | 10 Hz | reliable, keep last 10 |
| CV7-AHRS | microstrain_inertial_driver | imu/data (Imu, 100 Hz default), imu/data_raw (Imu, default 1 Hz, set 100) | imu_link | params imu_data_rate, imu_data_raw_rate | G3 verify from source |
| D435 | realsense2_camera | camera/camera/color/image_raw, .../color/camera_info, .../depth/image_rect_raw (16UC1), .../depth/camera_info | camera_link, camera_color_optical_frame, camera_depth_optical_frame | 30 Hz | SYSTEM_DEFAULT (param <stream>_qos) |
| RoboClaw | roboclaw_hardware_interface (ros2_control) | joint velocity command, position state | - | controller rate | - |
| Dynamixel | dynamixel_hardware_interface (ros2_control) | joint position command; position/velocity/effort states; GPIO per servo (ID, Goal Position, Present Position/Velocity/Current, gains); dxl_state topic | - | controller rate | - |

All topics relative and namespaced; frames REP-105/REP-103 (CV7 use_enu_frame true); use_sim_time.

### Validation tests (pass criteria)

1. Engine: loaded physics plugin = configured one (log check).
2. Free fall: measured g = 9.80665 +- 0.1 %.
3. Wheel motor, robot lifted: no-load speed at 12 V within 5 % of datasheet; blocked torque at the
   driver current limit within 5 % of the model; position state step = 2 pi/counts.
4. Diff drive yaw ratio (ground-truth / commanded) 0.97-1.03 with wheel_separation_multiplier 1.0.
5. Skid steer: turning torque vs Wong M_r = mu W L / 4 within 15 %.
6. 10 % slope at full mass holds max_vel; motor current <= driver limit.
7. Lidar vs analytic ranges in a known room: mean error < 5 mm, sigma 10 mm +- 10 %; bin count, angle
   fields, inf and range_min behaviour identical to the driver rules above.
8. IMU, 1 h static: ADEV(1 s) within 10 % of N; floor within 30 % of BI; |accel| = g +- noise.
9. D435: camera_info K matches derived fx; depth RMS at 1 m and 2 m within 20 % of the z^2 model.
10. Servo: stall torque and no-load speed within 5 % of the table at 12 V; quantization 4096/rev.
11. Legged walk: joint effort vs leg_demand.py prediction within 20 %; T-N utilisation <= 0.90.
12. Interface test: `ros2 topic info -v` (type, QoS) and frame_id match each catalog ros_driver block.
13. Mass properties: generated yaml up to date; inertia positive definite, triangle inequality holds.

## Details and gotchas

- Joint speed in the existing gait is set by the swing (step_height pi / swing time) more than by the
  command speed: hound tibia ~7 rad/s at any speed with period 0.45 s; slowing the period to 0.7 s
  is what makes the XH540-W150 feasible. Stance torque scales with mass and posture (hound knee arm
  ~0.12 m at standing_height 0.27).
- Offline leg demand at current configs (half-sine GRF peak pi m g /(2 duty n), Alexander 2003):
  wyrm 3.5 kg peak GRF 22.5 N, max |tau| femur 2.58 N m; hound 5 kg 32.1 N, tibia 3.85 N m;
  spider 3 kg 9.6 N, femur 1.53 N m.
- RPLIDAR coordinate system is left-handed clockwise (x = device front); the driver reverses it.
- sllidar_ros2 launch files default frame_id 'laser' while the node default is 'laser_frame'.
- RealSense profile defaults depend on the device; the launch must set them explicitly.
- CV7 driver covariances (imu_*_cov, default 0.01 diag) are config values, not measurements: real and
  sim share one yaml whose values come from the noise model above.
- Phidgets MOT0110 values (kept for reference): accel noise +-3 mg @1 ms, gyro +-0.2 deg/s @1 ms.
