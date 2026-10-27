# Design notes

Working notes written while the simulation was built: audits of every parameter, derivations, measurements,
bench results and the reasons behind decisions. The user-facing documentation is the [README](../../README.md),
[configuration.md](../configuration.md), the [guides](../guides/) and [ARCHITECTURE.md](../ARCHITECTURE.md);
read these notes when you need to know where a number came from or why something is done the way it is.

| note | covers |
|---|---|
| [ground.md](ground.md) | wheeled and legged robots: parameter audit, bill of materials, physics engine choice (DART, friction cone), sensor driver emulation, mass properties; specs for G4-G7 |
| [core.md](core.md) | wheeled bringup: `fleet_bringup` and the wheeled robot entries in `fleet_description` |
| [legged.md](legged.md) | the walking robots: kinematics, gait and odometry (`fleet_legged`) |
| [worlds.md](worlds.md) | how the Blender world generator, occupancy maps and world checks work |
| [sea.md](sea.md) | the AUV: hull derivations (inertia, added mass, damping, buoyancy), T200 thruster and battery plugins, sensor emulators; spec for S3 |
| [auv.md](auv.md) | the AUV package layout and thruster arrangement |
| [drone.md](drone.md) | the quadrotor on PX4 SITL: launch, uXRCE-DDS, missions, results |
| [air.md](air.md) | aircraft realism: flight-controller sensor catalog, motor and propeller fit, GNSS error plugin; specs for A2-A6 (VTOL, flapping wing) |
| [test_results.md](test_results.md) | the table written by `scripts/run_all_tests.sh` |
| [ground_bench/](ground_bench/) | stand-alone bench scripts for the physics measurements in ground.md |

Labels: work was planned in numbered steps per area: `G1`-`G7` ground, `S1`-`S3` sea, `A1`-`A6` air. The notes
refer to them by label; what is still open is in [ROADMAP.md](../../ROADMAP.md). Dates are when a value was
measured or a decision was made.
