# fleet_gz_plugins

Gazebo (gz-sim 8, Harmonic) system plugins for robot-fleet-sim. Each one models something stock
Gazebo gets wrong or leaves out, with the physics written down and checked by tests.

## CoulombFriction (`fleet_coulomb_friction`)

DART, the default gz-physics engine, treats friction as a box rather than a cone. It uses two
independent tangent directions, each limited to `mu N`, and lines them up with the world X axis.
As a result:

- a body pushed along a world diagonal can hold up to `sqrt(2) mu N` before it slides;
- once sliding, the friction force points the wrong way (a block pushed at 30 deg slid along x);
- wheel traction depends on the robot's heading (accel at yaw 30 deg was 6.74 m/s^2, against
  4.90 for mu = 0.5).

The plugin turns this into isotropic Coulomb friction. It does this through gz-sim's
contact-surface hook (`events::CollectContactSurfaceProperties`) and changes nothing else:

- **Sliding contact** (relative tangential speed above `slip_threshold`): the first friction
  direction is set along the slip, and the second coefficient to 0. DART then applies exactly
  `mu N` against the motion.
- **Sticking contact:** the friction direction is set from the previous step's contact forces.
  These are grouped per link pair, with the incipient-slip field of a rigid uniform contact
  patch. A multi-contact body (legs, wheels with casters) therefore breaks away at `mu W` in
  every direction.

### Use

Add the plugin next to the Physics system in a world. The project's worlds already do this.

```xml
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
  <engine><filename>gz-physics-dartsim-plugin</filename></engine>
</plugin>
<plugin filename="fleet_coulomb_friction" name="fleet_gz_plugins::CoulombFriction">
  <slip_threshold>1e-4</slip_threshold>       <!-- m/s, sliding vs sticking (default 1e-4) -->
  <stick_match_radius>0.02</stick_match_radius> <!-- m, match a contact to last step's (default 0.02) -->
</plugin>
```

Collisions keep their usual `<surface><friction><ode><mu>` and `<mu2>`. DART combines two
surfaces with `min(mu_a, mu_b)`. The package's environment hook adds its `lib` directory to
`GZ_SIM_SYSTEM_PLUGIN_PATH`, so a sourced workspace finds the plugin by name.

### Limitations

- **Single coefficient.** DART has one coefficient, so static and kinetic friction are equal.
- **No rolling resistance.** DART does not model rolling resistance; that needs an actuator-side
  model.
- **Isotropic surfaces only.** A collision with `mu != mu2` is left to DART's box, and the plugin
  warns once.
- **One-step lag.** The stick direction comes from the previous step's forces.
- **Twist-dominated multi-contact.** This case assumes uniform contact stiffness.
- **Speed cost.** About 30 % fewer steps per second in small test worlds.

Measurements, sources and the engine comparison are in `docs/notes/ground.md` (G2).

### Tests

Run the tests with:

```
colcon test --packages-select fleet_gz_plugins
```

`test/test_coulomb_friction.py` steps small worlds in-process and checks three closed-form
results:

- a 45 deg slide at 1.5x the friction limit;
- sticking at 0.9x the limit;
- four-foot breakaway at 45 deg, at `mu W` within 2 %.
