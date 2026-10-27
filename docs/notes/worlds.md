# Worlds (apartment, warehouse, lab, keep)

Worlds are modelled by headless Blender scripts and exported to a Gazebo package.

## Build

    blender -b --factory-startup -P blender/build_worlds.py -- --world keep --out stage/fleet_worlds [--no-preview]
    python3 tools/make_map.py <world> --pkg stage/fleet_worlds [--res 0.10 --crop-seed X Y]
    python3 tools/check_world.py <world> --pkg stage/fleet_worlds --lidar [--lidar-step 3 --lidar-min 0.3]

`--out` writes a complete package copy (the repo `src/` is untouched, copy the staged package over `src/fleet_worlds` once it checks out).
Previews go to `blender/previews/world_<w>*.png`. For `gz sdf -k` set `GZ_SIM_RESOURCE_PATH` and `SDF_PATH`
to the package `models/` dir, otherwise `model://` includes are not found.

## Sizes and budgets

| world | size | tris | goals | drone wps | map |
|---|---|---|---|---|---|
| apartment | ~44x32 m | 17k | 12 | none | 0.05 |
| warehouse | ~96x64 m | 20k | 14 | 8 | 0.05 |
| lab | ~56x40 m | 9k | 13 | 6 | 0.05 |
| keep | ~330x200 m, dragon citadel | 234k (48 tiles of 40 m) | 12 | 8 | 0.10, cropped |

The keep has a lake (surface z=-0.8) with a quay and 0.7 m parapet, a 50 m dragon statue, a hoard hall,
crypt, temple, gardens. Chunked models `keep_tN_M` plus `keep_lake`, `keep_sky`, `keep_water`.

## World yaml schema

`spawns` (named poses, `spawn:=<name>`), `goals` (name, x, y, yaw, zone), `short_pairs` (goal pairs < 60 m,
use these for patrol tests), `drone` (waypoints), `auv` (spawn + waypoints, keep only).

## Gotchas

- Physics engine: SCENE accepts `engine` (e.g. `bullet-featherstone`, written into the Physics plugin).
  It cut the bare keep idle cost (0.2 vs 0.7 cores) but under Nav2 the round robot did not follow the
  path (commanded turns not executed, drove straight into a hedge, 3 runs). With default ODE the same
  run turns through the garden gap correctly. The keep ships with ODE; the key is commented out in `w_keep.py`.
- Keep lidar check (collision primitives only): min 0.30 after the clamp, 6 points in the 22 m lane
  junctions are >20 m from walls by construction (lane is 22 m for the 20 m dragon route).
- Nav2 on the 0.10 m keep map (3310x1975): nav container ~120-160% CPU, RSS 0.5 to 1.1 GB over 8 min;
  gz sim ~95% CPU, 770-870 MB.
- RTF is very load dependent. The tests ran while other agents' sims pushed load to 8-29, RTF swung
  0.06-1.4. Quiet-moment readings: apartment 0.95-1.0, lab 1.0, keep bare ~1.0.

## Test results

- `gz sdf -k`: all four valid. check_world: all OK (smaller worlds lidar hit fraction min >= 0.93).
- 2-goal patrols with the round robot: apartment entry 16.7 s, study 75.8 s; lab server_room 113.5 s,
  cage_centre 98.2 s; warehouse charging 107.3 s, dock_north 153.8 s (all 2/2).
- Keep patrol (ODE, `spawn:=courtyard`, goal court_fountain, 56 m path): SUCCEEDED in 201 s sim-wall time
  while machine load was 20+ (RTF 0.07-0.55). Run it on a quiet machine to get a real RTF for the keep.

## Credits

Self-made geometry; CC0 Poly Haven textures as listed in the package CREDITS.md.
