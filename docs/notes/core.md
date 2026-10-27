# core: wheeled bringup notes

Scope: `src/fleet_bringup`, wheeled entries (round, box, rover, truck) in `src/fleet_description`.

## Interfaces

Launch (`ros2 launch fleet_bringup sim.launch.py ...`), CLI unchanged plus new args:

| arg | meaning |
|---|---|
| `robot:=` | name from `fleet_description/config/robots.yaml` |
| `world:=` | name of a world in the fleet_worlds share dir |
| `spawn:=<name>` | pose from the world yaml `spawns` dict; unknown name prints a message and falls back to `spawn` |
| `x:= y:= yaw:=` | override the chosen pose (any subset) |
| `gui:= rviz:= slam:=` | as before |

Internals (`sim.launch.py`):
- `KINDS = {'wheeled': wheeled_kind}`; `kind` in robots.yaml defaults to `wheeled`. A kind function takes the robot spec and returns a dict `{controllers, spawn, odom, ekf, nav2, post, z}`. Add a kind by adding one function and one dict entry.
- Robot and world lists are discovered (robots.yaml, share dir), not hard-coded.
- `nav2_config()` merges `config/nav2_overrides.yaml` over Nav2's default params, sets the global costmap resolution to the map's resolution (read from the map yaml), keeps the local costmap at 0.05, and with `slam:=true` starts from slam_toolbox's installed `mapper_params_online_sync.yaml` (a `slam_toolbox` section otherwise replaces its defaults wholesale).

Topics: `/cmd_vel` is TwistStamped end to end (diff_drive_controller only accepts stamped). Nav2 uses `/odometry/filtered` (EKF: wheel vx,vy + IMU yaw rate). TF: map -> odom (AMCL) -> base_footprint.

## Settings changed

- Lidar max range 20 m: `sensors.xacro` `<max>`, AMCL `laser_max_range`, costmap obstacle 19.5 / raytrace 20.0 (local and global), slam_toolbox `max_laser_range`.
- Global planner: `nav2_smac_planner::SmacPlanner2D` (large maps), tolerance 0.5, 4 s planning budget, `allow_unknown: true`.
- MPPI: `temperature: 0.05`, `vx_std: 0.3`, `batch_size: 1000`, `visualize: false`, CostCritic `consider_footprint: true`. Default temperature 0.3 capped speed near 0.15 m/s on a 0.5 m/s limit; 0.05 roughly halves patrol leg times.
- Global costmap uses `robot_radius` (circumscribed), local costmap uses the footprint polygon.
- `robots.yaml` `wheel_separation_multiplier`: round 0.9, box 0.86, rover 1.15, truck 1.2 (all give commanded/ground-truth yaw ratio about 1.00).

## Commands

Always `export ROS_DOMAIN_ID=<n> GZ_PARTITION=<name>`; use `gui:=false rviz:=false` headless.

```
colcon build --symlink-install            # yaml/xacro/launch edits are picked up without rebuild
ros2 launch fleet_bringup sim.launch.py robot:=rover world:=keep gui:=false rviz:=false
ros2 run fleet_bringup patrol.py --world keep [--goals a,b] [--timeout 240]
ros2 launch ... spawn:=hall            # named spawn (new keep)
```
Real-time factor: `gz topic -e -t /stats -n 1`.

## Gotchas

- Start `patrol.py` a few seconds after "Managed nodes are active"; started instantly it can hang on a lost get_state reply.
- Machine load matters: with load average above ~15 RTF drops to 0.2 and patrols time out spuriously. Test on a quiet machine.
- gz gpu_lidar sees visual geometry, so visual-only props appear as obstacles that are not in the map.
- Smac 2D logs at configure "Inflation layer either not found or inflation is not set sufficiently ... non-circular collision checking". Harmless for planning here (we use a circular footprint globally); not silenced.
- Truck (skid steer) drifts about 0.13 m forward and 0.2 m sideways during a 4 s spin in place; it still rotates and passes doorways.
- diff_drive_controller warns "Velocity command timed out. Braking." between goals; benign.
- `pkill -f` with a pattern present in your own command line kills your shell.

## Test results (headless, RTF about 1.0 when machine quiet)

Calibration (commanded vs ground-truth yaw rate): round 1.00, box 1.00, rover 1.00, truck 1.01. Round drives forward in the right direction.

round @ keep full patrol after the refactor: 6/6 (25.0, 24.1, 40.8, 43.5, 46.3, 70.3 s).

2-goal patrols (all succeeded, seconds per leg):

| robot | apartment (bedroom, kitchen) | keep (south_hall, armory) |
|---|---|---|
| box | 39.9, 30.4 | 45.5, 74.4 |
| rover | 43.2, 35.2 | 47.6, 79.8 |
| truck | 42.2, 36.3 | 44.7, 75.8 |

Staged new 300x220 m keep with 0.1 m/px map (worlds agent's overlay, `spawn:=hall`): round hall_carpet OK in 18 s (0.1 m global map, AMCL, Smac2D all work). hall_statue (-8,-22) failed twice: the robot stops near (-24.9,-14.7) and the planner reports "Start occupied" there although the map cell is free; likely an obstacle not in the map (a visual-only prop) at that spot. Reported to the worlds owner, not changed here.
