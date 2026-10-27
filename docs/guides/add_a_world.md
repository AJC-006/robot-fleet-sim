# Add a world

A world is a Python module that draws the place with a small builder API (boxes, walls, cylinders, floors,
lights), plus metadata (spawn, goals, optional drone mission). One command turns it into the Gazebo world, the
collision model, the occupancy map for Nav2 and a preview. Nothing else needs registering: the launch files
accept any world that exists in `fleet_worlds`.

## Files a world produces (per name `w`)

`src/fleet_worlds/worlds/w.sdf` and `w.yaml`, `models/w_env/{model.sdf,model.config,collision.json,meshes/w.glb}`,
`maps/w.{pgm,yaml}`, `blender/blend/world_w.blend`, `blender/previews/world_w.png`.

## 1. Write `blender/worlds/w_<name>.py`

Copy the smallest module (`w_apartment.py` or `w_lab.py`) and replace the content. It must define:

| name | meaning |
|---|---|
| `NAME`, `BOUNDS`, `OUTER` | name; interior `(xmin, ymin, xmax, ymax)` m; outer bounds including walls |
| `SPAWN` | `(x, y, yaw)` default robot pose (m, rad, ENU world frame, z up) |
| `GOALS` | list of `(name, x, y, yaw, zone)` Nav2 goal poses (tests drive between them) |
| `SPAWNS` (optional) | dict name -> `(x, y, yaw)`, used by `spawn:=<name>` |
| `SHORT_PAIRS` (optional) | list of `(goal_a, goal_b, distance_m)` pairs under 60 m for quick tests |
| `DRONE` (optional) | `{takeoff, cruise, waypoints: [(name, x, y, z)]}` for the wyvern mission |
| `SCENE`, `PREVIEW` | ambient/sun colours and the preview camera |
| `MATS` | materials: textured (`tex=`, `tile=`) or plain (`color=`, `rough=`, `metal=`) |
| `build(W)` | draws the world with the builder `W` |

Builder calls in `build(W)` (all dimensions in metres, `solid=True` adds a collision primitive that is also
rasterised into the occupancy map): `W.floor(mat, surface="concrete")`, `W.perimeter(mat, thickness, height)`,
`W.wall((x0, y0), (x1, y1), t, h, mat, openings=...)`, `W.box(mat, cx, cy, z0, sx, sy, h, yaw)`,
`W.cyl(mat, cx, cy, z0, r, h)`, `W.line`, `W.ring`, `W.light(...)`. Implementation: `blender/worlds/lib.py`.

Conventions that keep Nav2 and the sensors honest: keep solid obstacles at a constant footprint between 0.05 and
0.6 m of height (the 2D map and the lidar plane then agree); decoration goes above that height or inside a
footprint; every spawn and goal needs about 0.6 m clearance; floor `surface=` selects the cited friction pair
(see `docs/notes/ground.md`, G2).

## 2. Build, check, launch

```bash
blender -b --factory-startup -P blender/build_worlds.py -- --world <name> --no-preview
python3 tools/check_world.py <name> --png /tmp/<name>_map.png     # exits non-zero if a spawn/goal is blocked or unreachable
SDF_PATH=src/fleet_worlds/models gz sdf -k src/fleet_worlds/worlds/<name>.sdf   # prints Valid.
colcon build --symlink-install --packages-select fleet_worlds
ros2 launch fleet_bringup sim.launch.py robot:=round world:=<name> gui:=false rviz:=false
```

`--out DIR` on `build_worlds.py` writes into another directory (try a world without touching the package), and
`check_world.py <name> --pkg DIR` checks it there. The pattern was proven with a 10 x 8 m room with one pillar:
the build wrote all files, `check_world.py` reported both goals reachable, and `gz sdf -k` printed Valid.
A world needs only `color=` materials; textured ones (`tex=`) use the CC0 texture set the existing worlds use.
The generated SDF has DART physics with the dantzig solver and the `CoulombFriction` plugin; do not change the
physics block per world without a source.

Add the name to `ALL` in `blender/build_worlds.py` and to `WORLDS` in `scripts/run_all_tests.sh` so that the
regular tests include it. Credit any asset in `src/fleet_worlds/CREDITS.md`.

## Environments for the other robots

- AUV: add `auv:` data (start pose, mission waypoints) to `w.yaml`; the water is a `<fluid>` buoyancy region in
  the world SDF (see `src/fleet_auv/worlds/tank.sdf`) and the AUV launch takes `world:=<name>` ([auv.md](auv.md)).
- Drone: the `DRONE` data above; PX4 needs no map ([aircraft.md](aircraft.md)).
