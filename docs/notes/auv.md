# knucker AUV (package fleet_auv, blender/auv)

Named after the Sussex water dragon. 4 thrusters (port/stbd aft horizontal: surge + yaw; vfore/vaft vertical: heave + pitch),
roll passively stable (CoB 2.5 cm above CoG), 27.9 kg, 1.30 m long, 28.1 l displaced, +0.75 % net buoyancy (+2 N), thrust limit +-30 N each.
All numbers come from `src/fleet_auv/config/knucker.yaml`; `fleet_auv/params.py` derives mass/inertia/CoB/Fossen coefficients
(Lamb added mass of the equivalent spheroid x appendage factors, capped at 0.7 of the rigid-body mass because the gz Hydrodynamics system is explicit).

## Build / run
    blender -b --factory-startup -P blender/auv/build_auv.py -- [--no-render]        # meshes/knucker/{hull,prop}.obj + preview
    colcon build --packages-select fleet_auv --symlink-install
    export ROS_DOMAIN_ID=74 GZ_PARTITION=auv
    ros2 launch fleet_auv auv.launch.py world:=keep gui:=false rviz:=false camera:=false mission:=true   # lake mission, exits 0 on success
    ros2 launch fleet_auv auv.launch.py world:=tank mission:=false     # 20 m test tank (surface z=0), then:
    ros2 run fleet_auv auv_tests --out docs/img [--only depth,heading,surge,roll,surface]   # step responses, plots, exit 0 = pass
Launch args: world (keep|tank) gui rviz camera mission cruise log_csv spawn_delay exit_on_done. The model SDF/URDF is generated
by `fleet_auv/model.py` (`auv_gen_model`), spawned with `ros_gz_sim create` (no gz plugin of the world needs to know about it).

## Interfaces (ROS)
- in: `/knucker/cmd_vel` (Twist) and `/knucker/cmd_vel_stamped` (TwistStamped): linear.x surge, linear.z heave rate (+up), angular.z yaw rate; `/knucker/ref` (Float64MultiArray depth m down+, heading rad, surge m/s); `/knucker/enable` (Bool).
- sensors: `/knucker/imu`, `/knucker/dvl` (marine_acoustic_msgs/Dvl from gz DVL system; odometry fallback with noise if none), `/knucker/depth` (noisy, 1 cm), `/knucker/pressure`, `/knucker/velocity` (body, TwistStamped), `/knucker/odom_gt` (ground truth), `/knucker/trail`, TF world->base_link. camera:=true adds `/knucker/camera/image` and a forward gpu_lidar imaging-sonar proxy (`/knucker/sonar/{scan,points}`).
- out: `/knucker/thrust/{port,stbd,vfore,vaft}` (Float64 N -> gz `cmd_thrust`), `/knucker/status`, `/knucker/mission_markers`.
- Controller: depth PID with buoyancy feed-forward, pitch PD, heading PID, surge PI, allocation with saturation, 50 Hz. Mission: integral-LOS in the plane, depth reference per waypoint, 3D accept radius 0.8 m, exit 0 on success.

## World changes (lake only; blender/worlds/w_keep.py, keep_lake.py)
- Graded Buoyancy plugin in the keep world, `<enable>knucker</enable>` only (ground robots untouched), density 1000 below z=-0.8, 1.2 above; DVL system added.
- Water surface is now an SDF box visual (`lake_surface`, transparency 0.45) because Gazebo ignores the glTF alpha factor: the old mesh slab was invisible and the lake looked drained from below. The `keep_water` model is gone. No collision either way.
- `maps/keep.{pgm,yaml}` md5 unchanged after the rebuild (75bf67b5..., 33ca628e...). keep.yaml `auv:` section unchanged (spawn 24,0,-2; 8 waypoints through the drowned temple).
- Not done: depth fog (gz-rendering ogre2 has no scene fog); the lake is lit by the existing `deep_*` cyan lights only, so there is no haze.

## Test results (headless, ODE, tank)
| test | result |
|---|---|
| depth step 2->4 m | overshoot 7.6 %, settle (2 %) 24.9 s, steady-state error 0.007 m (target < 0.15) |
| heading +90 deg | overshoot 1.9 %, settle 5.5 s, SSE 0.86 deg (target < 5) |
| surge 0->0.5 m/s | overshoot 16 %, settle 7.0 s, SSE 0.0005 m/s, depth deviation 4 mm |
| passive roll (thrusters off, 25 deg start) | half-time 0.7 s, final 0.0005 deg |
| surfacing, thrusters off from 3 m | rises at 0.046 m/s, final depth 0.06 m (rests at the surface) |
| lake mission keep, 8 waypoints | 8/8 reached, worst 0.80 m (3D), 323 s sim, exit 0 |
Plots: docs/img/auv_{depth_step,heading_step,surge_step,roll_decay,surfacing,mission_keep}.png. Gazebo frames: docs/img/gz_lake_knucker_{pillar,ruins}.png, gz_lake_ruins_wide.png. Hero render: docs/img/knucker_hero.png.
The step tests were run in the tank (same plugins, no ruins); the keep run exercised the same controller through the ruins (collisions: columns, drums, bed, quay as primitives).

## Gotchas
- Surfacing is slow by design: +2 N net buoyancy against quadratic heave drag gives ~0.05 m/s (a 5 m rise takes about 2 minutes).
- gz Hydrodynamics is explicit: added mass >= rigid-body mass diverges, hence the 0.7 cap. Hull collisions are a stack of cylinders because graded buoyancy integrates primitive volumes exactly for cylinders (ellipsoids were ~40 % off).
- Cameras and `gz sim -s` frame dumps need a subscriber; use `gz topic -e`. When killing your own `setsid ... &` job from a script use the PID, not `-PGID`, if it is not a group leader.
- The Buoyancy plugin is in the keep SCENE `plugins`; the keep uses default ODE physics (bullet-featherstone broke Nav2 turning, see worlds.md).
- The tests node spins `use_sim_time`; start it after the launch has been up ~25 s.
