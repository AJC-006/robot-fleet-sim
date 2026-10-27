# legged: walking robots (wyrm, hound, spider)

Scope: `kind: legged` entries in `src/fleet_description` (robots.yaml, legs.xacro, meshes),
`src/fleet_legged` (new), the legged kind in `src/fleet_bringup/launch/sim.launch.py`,
`blender/robots/{wyrm,hound,spider,legkit}.py`, `blender/build_robots.py`.

| robot | legs | mass | standing h | gait | max vx / vy / wz | lidar z | footprint w | robot_radius |
|---|---|---|---|---|---|---|---|---|
| wyrm (dragon hatchling) | 4 sprawling (yaw) | 3.5 kg | 0.14 m | trot 0.6 s, duty 0.6 | 0.18 / 0.08 / 0.6 | 0.27 m | 0.509 m | 0.382 |
| hound (robot dog) | 4 mammal (roll) | 5.0 kg | 0.27 m | trot 0.45 s, duty 0.6 | 0.25 / 0.12 / 0.7 | 0.40 m | 0.383 m | 0.369 |
| spider (octopod, lidar mast) | 8 sprawling (yaw) | 3.0 kg | 0.12 m | tetrapod 0.5 s, duty 0.6 | 0.25 / 0.15 / 0.8 | 0.30 m | 0.538 m | 0.329 |

All three are 3-DOF per leg (coxa, femur, tibia) with closed-form IK, and are omnidirectional.

## Interfaces

Same launch command as the wheeled robots:

    ros2 launch fleet_bringup sim.launch.py robot:=hound world:=lab nav:=true gui:=false rviz:=false

Topics (legged):
- `/cmd_vel` geometry_msgs/TwistStamped (vx, vy, wz). The gait node clamps it to `max_vel_*` and
  applies an acceleration limit (0.5 m/s^2, 1.5 rad/s^2).
- `/leg_controller/commands` std_msgs/Float64MultiArray: joint positions for a
  `position_controllers/JointGroupPositionController`. Joint order = robots.yaml legs order x
  (coxa, femur, tibia), named `<leg>_<coxa|femur|tibia>_joint`.
- `/leg_odom` nav_msgs/Odometry (odom -> base_footprint, twist only is meaningful, no TF). The EKF fuses
  vx, vy from it plus IMU yaw rate, the same way it fuses wheel odometry.
- `/joint_states`, `/imu` are inputs of the gait node.

`src/fleet_legged`:
- `fleet_legged/kinematics.py`: loads a robot from robots.yaml; FK/IK per leg type (yaw = sprawling,
  coxa axis z rotated by `mount_yaw_deg`; roll = mammal hip ab/ad).
- `fleet_legged/gait.py`: phase scheduler (trot / creep / wave / tetrapod from `offsets`), Raibert-like
  foot targets clamped to `max_reach`, Hermite swing curves.
- `scripts/gait_node.py` (params: `robot`, `robots_yaml`, `rate`, `cmd_timeout` 0.5 s, `stand_time` 2 s,
  `accel_lin`, `accel_ang`, `odom_topic`, `odom_frame`, `base_frame`, `odom_alpha`, `odom_trim`,
  `servo_lead`, `imu_topic`). Waits for joint states, stands up from the spawn crouch, walks when
  /cmd_vel is non-zero. Leg odometry is a least-squares fit of (vx, vy, wz) to the stance-foot velocities
  from the FK of the measured joints.
- `scripts/envelope.py <robot>`: prints the footprint (swept stance envelope + body outline + 1 cm) and
  robot_radius to paste into robots.yaml.
- `scripts/check_lidar_plane.py wyrm hound spider`: checks the lidar plane against every visual
  triangle through the stand-up and the gait at top speed (beyond 0.12 m).
- `scripts/walk_test.py --robot R --world W [--stress N] [--dump f.csv]`: forward/sideways/backward/turn
  at 0.6 x max speed vs Gazebo ground truth, EKF and leg odometry; rest drift check. `--stress` adds N s
  of random combined commands inside a containment circle (`--stress-radius`, `--stress-ahead`) and
  fails on a fall (tilt > 45 deg or the body below half its standing height) or tilt > 25 deg;
  `--dump` writes the last 4 s (pose, commands, joints) to a CSV.

robots.yaml legged schema: see the comment block above the `wyrm:` entry (leg_type, knee,
standing_height, crouch_height, link, link_mass, joint, limits, legs, gait, body.{com, inertia_box,
outline, collisions, foot_mu, foot_kp, foot_kd}, lidar/imu/camera, footprint, robot_radius, max_vel_*).

sim.launch.py `legged_kind()`: controller_manager 250 Hz, `gz_ros_control.position_proportional_gain`
0.5, spawns crouched (z = crouch - standing + 3 mm), MPPI `motion_model: Omni`, AMCL `OmniMotionModel`,
velocity_smoother/behavior_server limits from robots.yaml, progress checker 0.25 m / 20 s,
`min_y_velocity_threshold` 0.001, global obstacle layer `obstacle_max_range` 5 m. Global costmap
circle = the footprint's inscribed radius + 0.10 m (`plan_radius`); inflation radius still from
robot_radius. BT:
`fleet_bringup/config/bt/navigate_to_pose_recover_always.xml`.

## Commands

    colcon build --symlink-install --packages-select fleet_description fleet_legged fleet_bringup
    # meshes + previews (headless)
    blender -b --factory-startup -P blender/build_robots.py -- --robot wyrm     # or hound, spider
    blender -b --factory-startup -P blender/build_robots.py -- --lineup-only     # robot_lineup.png
    # checks
    ros2 run fleet_legged envelope.py spider
    ros2 run fleet_legged check_lidar_plane.py wyrm hound spider
    ros2 launch fleet_bringup sim.launch.py robot:=spider world:=warehouse gui:=false rviz:=false
    ros2 run fleet_legged walk_test.py --robot spider --world warehouse
    ros2 run fleet_bringup patrol.py --world warehouse --timeout 300

## Gotchas

- gz_ros2_control turns a position command into a velocity command, v = gain * update_rate * error,
  so the servo lags 1/(gain * rate) = 8 ms. The gait node sends q + servo_lead * dq/dt (velocity
  feed-forward) with servo_lead set by the launch from those two numbers. Raising the gain instead
  makes the legs buzz.
- The gpu_lidar sees visual meshes, the robot's own included. Horns, crest, mast and legs are kept out
  of the plane by design; rerun `check_lidar_plane.py` after any mesh change (the wyrm horn tips are
  21 mm under the plane).
- Nav2 Jazzy's default BT does not recover from `START_OCCUPIED` or `TF_ERROR`; both happened in long
  patrols (spider in a 1.2 m rack gap; hound when the EKF/AMCL transforms lagged under load), and the
  whole goal and every following goal failed. Legged robots use the gate-free BT above; both costmaps
  got `transform_tolerance: 1.0` (all robots).
- The global costmap uses a circle, never the polygon (Smac 2D with a polygon flags "Start occupied"
  near walls). For legged robots the circle is the footprint's inscribed radius plus 0.10 m
  (`plan_radius`: wyrm 0.35, hound 0.29, spider 0.36), so the planner only uses gaps ~0.2 m wider than
  the body. The circumscribed circle (feet out at the diagonals) gave "Start occupied" next to racks;
  "inscribed - 0.10" let Smac route the 0.54 m spider through a 0.70 m conveyor slot where MPPI (real
  footprint) wedged it. A start that AMCL puts inside the circle is left to the BT's recoveries.
- MPPI acceleration limits: keep `ax_max`/`ay_max`/`az_max` at Nav2's defaults (3 m/s^2). MPPI
  starts each rollout from the measured speed and clamps the first control to
  measured + a_max * model_dt; with the real 0.5 m/s^2 that is only +0.025 m/s per cycle, the softmax
  average cancels it and the spider crawled at 0.04 m/s of its 0.25 (dock_north 118 s, 17 s with the
  defaults). The velocity smoother and the gait node apply the real 0.5 m/s^2 limit. A higher
  `temperature` (0.3) made it worse (0.01 m/s).
- `patrol.py` no longer uses `waitUntilNav2Active()`: a lost lifecycle `get_state` response under load
  made it hang forever; it now polls with a timeout.
- `gait_node.py` initialises rclpy without signal handlers and shuts down itself, otherwise Ctrl-C
  raised "publisher's context is invalid" and the launch reported exit code 1.
- `walk_test.py` ends with `os._exit` (gz-transport aborts with "terminate called" on normal exit).
- Blender: the OBJ/MTL export takes Kd from the Principled Base Color default value even when a node
  is linked, so the procedural bump/colour-variation (`common.FX`) only affects the previews. AgX tone
  mapping washes strong emission to pastel; glowing materials use a dark base and emission 0.5-1.8.
  The wyrm eye is a deep violet iris (emission 0.6, specular 0) with a brighter magenta core round the
  slit pupil, so it reads as a glow instead of a flat lavender disc.
- Stress-test falls were the test, not the gait: every early hound fall (16-74 s into a random walk in
  the lab) ended at the east wall, a crate or the south wall. The random walk now stays inside a
  1 m circle (homing commands point back to its centre); the fall check compares with the start height
  (the model origin is base_footprint, ground-truth z = 0 while standing).
- Joint velocity limits must exceed the gait's peak joint rate (offline replay of 600 s of random
  commands through Gait + IK: hound 14.7, spider 12.9, wyrm 9.8 rad/s); DART clamps the servo to the
  URDF velocity limit, which at the old 12 rad/s made the hound's swing legs lag and stumble. Limits
  are now hound 25, spider 20, wyrm 16 rad/s.

## Tuning notes (hound)

Kept: joint velocity 12 -> 25 rad/s, duty 0.55 -> 0.6, max speeds 0.30/0.15/0.8 -> 0.25/0.12/0.7.
Tried and dropped (each still fell in the uncontained stress test, which was hitting walls):
effort 8 N m (big wobbles), period 0.36 s / step 0.04 m, position gain 0.2 (softer servo), and a
capture-point swing offset (balance_gain * sqrt(h/g) * IMU roll/pitch rate x h): at gain 1.0 it fed a
growing roll oscillation and fell at 16 s, so it was removed.

Leg odometry is good enough for Nav2 (walk-test speed ratios 0.95-1.09 vs ground truth; drift over a
full patrol 2.6 m / 40 deg in ~270 s, which AMCL corrects; max map-frame error 0.07-0.19 m), so the
gz OdometryPublisher fallback was not needed.

## Test results (2026-10-24, headless, one sim at a time)

Walk tests (`walk_test.py --stress`, commands at 0.6 x max speed; ratio = ground-truth speed / command):

| robot @ world | rest | fwd | back | left | right | ccw | cw | stress (contained) | result |
|---|---|---|---|---|---|---|---|---|---|
| wyrm @ keep | 0 mm | 1.03 | 1.03 | 1.05 | 1.04 | 0.98 | 0.98 | 150 s, tilt max 7.1 deg | 9/9 |
| hound @ lab | 0 mm | 1.07 | 1.09 | 1.02 | 1.02 | 1.00 | 0.99 | 120 s, tilt max 6.7 deg (240 s: 9.1 deg) | 9/9 |
| spider @ warehouse | 0 mm | 1.02 | 1.02 | 1.02 | 1.03 | 0.96 | 0.95 | 150 s, tilt max 0.7 deg | 9/9 |

Nav2 (`patrol.py`; time per goal in sim seconds, RTF ~0.97 with Nav2 running):

| run | goals | result |
|---|---|---|
| wyrm @ keep, full patrol | dragon_plinth 43, throne 33, vault 75, armory 85, north_hall 81, south_hall 128 s | 6/6, max map error 0.13 m |
| hound @ lab, full patrol | arena 31, bench 60, robot_arm 24, storage 23, shelves_east 65, south_bay 54 s | 6/6, max map error 0.19 m |
| spider @ warehouse, full patrol | dock_north 35, dock_south 64, aisle_a 90, aisle_b 59, north_aisle 46, west_lane 67, staging 108 s | 7/7, max map error 0.13 m |
| wyrm @ apartment | kitchen 59, living_room 69 s (from the spawn, ~8 m apart) | 2/2 (also 2/2 at 57, 67 s with the old 10 rad/s joint limit) |
| hound @ apartment | kitchen 45, living_room 49 s | 2/2 |
| spider @ apartment | kitchen 43, living_room 51 s | 2/2 |
| round @ keep (bringup re-check) | south_hall 44, armory 73 s | 2/2 (before: 43, 74 s) |

The spider warehouse patrol and the wyrm apartment run were repeated after the joint velocity limit
raise (wyrm 16, spider 20 rad/s); the wyrm keep patrol and the hound/spider apartment runs were done
with the earlier limits (wyrm 10, spider 12; the hound already at 25).

Lidar plane (`check_lidar_plane.py`): no self hits beyond 0.12 m through stand-up and gait; closest
leg geometry below the plane: wyrm 63 mm, hound 108 mm, spider 55 mm.
Triangles (posed, all links): wyrm 76k, hound 45k, spider 54k.
