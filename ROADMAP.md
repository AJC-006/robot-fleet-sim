# Roadmap

Known gaps and planned work, in rough priority order. Each item has a label used in the design notes
(`G` ground, `S` sea, `A` air); the full spec for each is in the note named in the last column. The
[Realism status](README.md#realism-status) table in the README lists what is already validated.

## Open work

| label | item | what is wrong today | spec |
|---|---|---|---|
| S3 | AUV depth and heading control | The controller in `fleet_auv` is unstable on the realistic plant, so the keep-lake mission is a known failure (XFAIL). The plots in `docs/img/auv_*.png` predate the realistic model. | [sea.md](docs/notes/sea.md) |
| S3 | T200 thrust at speed | The thruster plugin uses the vendor bollard table, which is an upper bound once the vehicle moves (no KT(J) advance-ratio curve), so surge speeds are optimistic. | [sea.md](docs/notes/sea.md) |
| G4 | DC motor, RoboClaw and encoder models | Wheeled robots drive through ideal velocity interfaces. `fleet_gz_plugins/src/actuator_models.cpp` and `roboclaw_system.cpp` are started but not built or wired in; rolling resistance is recorded but not applied. | [ground.md](docs/notes/ground.md) |
| G5 | Skid-steer model | `rover` and `truck` turn correctly only because of a measured `wheel_separation_multiplier` (1.234, 1.886). | [ground.md](docs/notes/ground.md) |
| G6 | Legged actuators | Leg torque demand at the real masses exceeds every single Dynamixel X-series servo; `odom_scale` and the 16-25 rad/s joint limits are placeholders. | [ground.md](docs/notes/ground.md), [legged.md](docs/notes/legged.md) |
| G7 | Nav2 and EKF parameters, namespaces | Ground Nav2/EKF parameters are tuned by hand, not derived from the sensor specs; the wheeled/legged launch has no `namespace` argument (the AUV launch has one). | [ground.md](docs/notes/ground.md) |
| A1 follow-up | Wyvern airframe numbers | Base mass, inertia and rotor drag coefficients are still the PX4 x500 values; no vibration or battery sag. | [air.md](docs/notes/air.md) |
| A2-A6 | VTOL and flapping-wing aircraft | Specified, not started. | [air.md](docs/notes/air.md) |

## Known test failures

`scripts/run_all_tests.sh` reports these as XFAIL rather than FAIL. Each one becomes a PASS when fixed.

- `auv_lake_mission`: the S3 controller problem above.
- `world_lidar_keep_slamtec_rplidar_c1`: the RPLIDAR C1 has a 12 m range, so in the 20 m wide keep lanes it cannot
  reach the 30 % beam-hit threshold the lidar check uses. This is the real sensor's limit, not a bug: use the
  RPLIDAR S2 (30 m) in the keep.

## Tooling

- `scripts/run_all_tests.sh` reports a launch that exits at once as "not ready within N s"; read its log in the
  run directory to see the real error.
- `--full` has never been timed end to end. The measured rows add up to about 46 min on an 8-thread laptop; the
  script header estimates 3-4 h for the whole matrix.
