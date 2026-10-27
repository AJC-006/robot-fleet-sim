"""Writers for model.sdf / model.config / world sdf / world yaml / collision.json (pure python)."""
import json
import os


# Physics settings, chosen from measurements in docs/notes/ground.md (G2):
# - engine DART (gz-physics-dartsim): exact free fall and exact Coulomb slide/breakaway with the
#   CoulombFriction plugin (src/fleet_gz_plugins). bullet-featherstone ignores <ode><mu>, gives mu^2
#   through its anisotropic-friction call, takes friction from a link's first collision only, and its
#   default spinning friction (torsional 1.0) blocks pivot turns; the bullet engine let a 45 deg slide stick.
# - solver dantzig: pgs released a mu 0.8 foot at a 0.35 force ratio instead of 0.81.
# - collision detector ode: the dartsim default (EntityManagementFeatures.cc), written out explicitly.
# - step 0.002 s: 0.001/0.002/0.004 s gave identical slide, spin and drive results; RTF scales with
#   step (truck 2.2/4.1/7.2), so a world may raise it (keep uses 0.004).
# gz-sim 8 reads <dart><collision_detector> and <dart><solver><solver_type> (SdfEntityCreator.cc); the
# engine itself comes from the Physics plugin's <engine>; the required <physics type> is set to match.
PHYSICS = {"engine": "dartsim", "collision_detector": "ode", "solver": "dantzig", "max_step_size": "0.002"}
# source: standard acceleration of gravity g_n = 9.80665 m/s^2 (3rd CGPM, 1901; BIPM SI Brochure).
GRAVITY = "0 0 -9.80665"
# Floor friction = rubber/floor pair coefficient (dry). DART uses min(mu_a, mu_b) per contact; the robots'
# rubber (tyres, feet) carries an upper-bound mu, so these pair values govern. A collision prim gets one
# with prim["surface"] = key.
FLOOR_SURFACES = {
    # source: soft rubber on dry concrete 0.85 (Roymech friction table, reproduced by U. Florida Design Lab,
    # web.mae.ufl.edu/designlab/Class Projects/Background Information/Friction coefficients.htm);
    # Engineering ToolBox rubber/dry concrete 0.6-0.85.
    "concrete": 0.85,
    # source: EN 13893:2003 dry dynamic coefficient of friction of an 8 mm AC3/AC4 laminate, mu = 0.51
    # (EPH Dresden certificate BOD-19-12-16-02, 2019, for Kastamonu Entegre FLOORPAN). Standard slider,
    # not a tyre compound; a dynamic value, so peak traction on laminate is likely underestimated.
    "laminate": 0.51,
    # source: Engineering ToolBox, rubber / dry asphalt, static 0.9 (kinetic 0.5-0.8).
    "asphalt": 0.9,
}


def f(x):
    return ("%.4f" % x).rstrip("0").rstrip(".") if abs(x) > 1e-9 else "0"


def write_collision_json(path, name, bounds, prims, extra=None):
    d = {"world": name, "bounds": list(bounds), "primitives": prims}
    if extra:
        d.update(extra)
    with open(path, "w") as fh:
        json.dump(d, fh, indent=1)


def write_tile_model(model_dir, mname, mesh_rel):
    """Visual-only static model (one chunk of a big world)."""
    os.makedirs(model_dir, exist_ok=True)
    with open(os.path.join(model_dir, "model.sdf"), "w") as fh:
        fh.write("""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="%s">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://%s/%s</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
""" % (mname, mname, mesh_rel))
    with open(os.path.join(model_dir, "model.config"), "w") as fh:
        fh.write("""<?xml version="1.0"?>
<model>
  <name>%s</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>robot-fleet-sim</name></author>
  <description>Visual chunk (Blender export).</description>
</model>
""" % mname)


def write_model(model_dir, name, mesh_rel, prims):
    cols = []
    for i, p in enumerate(prims):
        x, y, z, yaw = p["pose"]
        rp = p.get("rp", (0, 0))
        pose = "%s %s %s %s %s %s" % (f(x), f(y), f(z), f(rp[0]), f(rp[1]), f(yaw))
        if p["type"] == "box":
            geom = "<box><size>%s %s %s</size></box>" % tuple(f(v) for v in p["size"])
        else:
            geom = "<cylinder><radius>%s</radius><length>%s</length></cylinder>" % (f(p["radius"]), f(p["length"]))
        surf = ""
        if p.get("surface"):
            mu = f(FLOOR_SURFACES[p["surface"]])
            surf = "<surface><friction><ode><mu>%s</mu><mu2>%s</mu2></ode></friction></surface>" % (mu, mu)
            cols.append("      <!-- %s floor: rubber/%s friction, source in blender/worlds/sdfgen.py FLOOR_SURFACES -->"
                        % (p["surface"], p["surface"]))
        cols.append('      <collision name="c%d"><pose>%s</pose><geometry>%s</geometry>%s</collision>' % (i, pose, geom, surf))
    vis = ""
    if mesh_rel:
        vis = """      <visual name="visual">
        <cast_shadows>true</cast_shadows>
        <geometry><mesh><uri>model://%s_env/%s</uri></mesh></geometry>
      </visual>
""" % (name, mesh_rel)
    sdf = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="%s_env">
    <static>true</static>
    <link name="link">
%s%s
    </link>
  </model>
</sdf>
""" % (name, vis, "\n".join(cols))
    with open(os.path.join(model_dir, "model.sdf"), "w") as fh:
        fh.write(sdf)
    with open(os.path.join(model_dir, "model.config"), "w") as fh:
        fh.write("""<?xml version="1.0"?>
<model>
  <name>%s_env</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>robot-fleet-sim</name></author>
  <description>Procedural %s environment (Blender export, primitive collisions).</description>
</model>
""" % (name, name))


def write_world_sdf(path, name, scene, lights, tiles=()):
    """scene may override PHYSICS keys (engine, collision_detector, solver, max_step_size)."""
    phys = dict(PHYSICS, **{k: scene[k] for k in PHYSICS if k in scene})
    amb, bg = scene["ambient"], scene["background"]
    sun = scene["sun"]
    lt = []
    for l in lights:
        lt.append("""    <light type="point" name="%s">
      <pose>%s %s %s 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <diffuse>%s %s %s 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>%s</range><constant>%s</constant><linear>%s</linear><quadratic>%s</quadratic></attenuation>
      <intensity>%s</intensity>
    </light>""" % (l["name"], *[f(v) for v in l["pos"]], *[f(v) for v in l["color"]], f(l["range"]),
                   *[f(v) for v in l.get("atten", (0.3, 0.08, 0.01))], f(l["intensity"])))
    sdf = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="%(name)s">
    <!-- physics: DART, chosen from measurements (docs/notes/ground.md G2, blender/worlds/sdfgen.py PHYSICS) -->
    <physics name="default_physics" type="%(ptype)s">
      <max_step_size>%(step)s</max_step_size>
      <real_time_factor>1.0</real_time_factor>%(cd)s
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">%(eng)s</plugin>%(cone)s
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>%(plugins)s
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>%(gravity)s</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397742</latitude_deg>
      <longitude_deg>8.545594</longitude_deg>
      <elevation>488</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <scene>
      <ambient>%(amb)s 1</ambient>
      <background>%(bg)s 1</background>
      <shadows>%(shadows)s</shadows>
    </scene>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>%(sunshadow)s</cast_shadows>
      <diffuse>%(sd)s 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>%(sdir)s</direction>
    </light>
%(lights)s
    <include>
      <uri>model://%(name)s_env</uri>
      <name>%(name)s_env</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
%(tiles)s
  </world>
</sdf>
""" % {"plugins": ("\n    " + "\n    ".join(scene["plugins"])) if scene.get("plugins") else "",
       "eng": "<engine><filename>gz-physics-%s-plugin</filename></engine>" % phys["engine"],
       "ptype": "dart" if phys["engine"] == "dartsim" else "bullet" if phys["engine"].startswith("bullet") else phys["engine"],
       "step": phys["max_step_size"],
       "cd": ("\n      <dart><collision_detector>%s</collision_detector><solver><solver_type>%s</solver_type></solver></dart>"
              % (phys["collision_detector"], phys["solver"])) if phys["engine"] == "dartsim" else "",
       # isotropic Coulomb friction (cone) for DART's box friction: src/fleet_gz_plugins
       "cone": ('\n    <plugin filename="fleet_coulomb_friction" name="fleet_gz_plugins::CoulombFriction"/>'
                if phys["engine"] == "dartsim" else ""),
       "gravity": GRAVITY,
       "name": name, "amb": " ".join(f(v) for v in amb), "bg": " ".join(f(v) for v in bg),
       "sd": " ".join(f(v) for v in sun["color"]), "sdir": " ".join(f(v) for v in sun["dir"]),
       "lights": "\n".join(lt), "tiles": "\n".join(
           "    <include><uri>model://%s</uri><name>%s</name></include>" % (t, t) for t in tiles),
       "sunshadow": "true" if sun.get("shadows", True) else "false", "shadows": "true" if scene.get("shadows", True) else "false"}
    with open(path, "w") as fh:
        fh.write(sdf)


def write_world_yaml(path, name, spawn, goals, drone=None, extra=None):
    """goals: (name, x, y, yaw[, zone]); extra: {"spawns": {name: (x,y,yaw)}, "auv": {...}, "short_pairs": [[a,b]..]}"""
    extra = extra or {}
    lines = ["# generated by blender/build_worlds.py", "spawn: {x: %s, y: %s, yaw: %s}" % (f(spawn[0]), f(spawn[1]), f(spawn[2])),
             "map: %s.yaml" % name]
    sp = extra.get("spawns")
    if sp:
        lines.append("spawns:")
        for k, v in sp.items():
            lines.append("  %s: {x: %s, y: %s, yaw: %s}" % (k, f(v[0]), f(v[1]), f(v[2])))
    lines.append("goals:")
    for g in goals:
        z = (", zone: %s" % g[4]) if len(g) > 4 else ""
        lines.append("  - {name: %s, x: %s, y: %s, yaw: %s%s}" % (g[0], f(g[1]), f(g[2]), f(g[3]), z))
    if extra.get("short_pairs"):
        lines.append("short_pairs:  # goal pairs < 60 m apart (quick tests)")
        for a, b, d in extra["short_pairs"]:
            lines.append("  - [%s, %s]  # %.0f m" % (a, b, d))
    if not drone:
        lines.append("drone: null")
    else:
        lines.append("drone:")
        lines.append("  takeoff: {x: %s, y: %s}" % (f(drone["takeoff"][0]), f(drone["takeoff"][1])))
        lines.append("  cruise_altitude: %s" % f(drone["cruise"]))
        lines.append("  waypoints:")
        for w in drone["waypoints"]:
            lines.append("    - {name: %s, x: %s, y: %s, z: %s}" % (w[0], f(w[1]), f(w[2]), f(w[3])))
    auv = extra.get("auv")
    if auv:
        sx = auv["spawn"]
        lines.append("auv:")
        lines.append("  water_level: %s" % f(auv["water_level"]))
        lines.append("  spawn: {x: %s, y: %s, z: %s, yaw: %s}" % tuple(f(v) for v in sx))
        lines.append("  waypoints:")
        for w in auv["waypoints"]:
            lines.append("    - {name: %s, x: %s, y: %s, z: %s}" % (w[0], f(w[1]), f(w[2]), f(w[3])))
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
