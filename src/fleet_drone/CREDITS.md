# fleet_drone credits

| Asset | Source | Licence |
|---|---|---|
| `models/wyvern/model.sdf`: links, inertia, joints, collisions, IMU / magnetometer / barometer / GNSS sensors and the four `MulticopterMotorModel` plugins | PX4-Autopilot `Tools/simulation/gz/models/x500_base` and `x500` (Copyright (c) 2022 Rudis Laboratories, Benjamin Perseghetti), copied unchanged by `blender/drone/make_sdf.py` so PX4 airframe `4001_gz_x500` tuning applies | BSD 3-Clause, see `models/wyvern/LICENSE` |
| `models/wyvern/meshes/*.glb` (wyvern body, rotors) and their textures | self-made, `blender/drone/build_drone.py` (procedural geometry and procedurally generated PBR maps) | same as the project |
| PX4 SITL, airframe 4001, uXRCE-DDS client | PX4-Autopilot (your own checkout, `px4_dir:=` or `$PX4_DIR`; not redistributed) | BSD 3-Clause |
| `src/px4_msgs` (sibling package) | generated from PX4-Autopilot `msg/` and `srv/` of the same checkout | BSD 3-Clause |
| Micro-XRCE-DDS Agent (built into `.deps/xrce_agent`, not redistributed) | eProsima Micro-XRCE-DDS-Agent v2.4.3 | Apache 2.0 |
