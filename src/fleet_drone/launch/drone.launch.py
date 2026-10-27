"""wyvern: PX4 SITL quadcopter (airframe 4001_gz_x500) in a fleet_worlds world, linked to ROS 2 over uXRCE-DDS.

    ros2 launch fleet_drone drone.launch.py world:=lab mission:=true gui:=false rviz:=false

Starts: gz sim server (headless rendering; gui:=true adds the Gazebo GUI client), the wyvern model, the
Micro-XRCE-DDS Agent (UDP 8888), PX4 SITL in standalone gz mode (lockstep on the gz clock), ros_gz_bridge
(/clock, ground-truth pose), the odometry/TF bridge and optionally the offboard mission node and RViz.

Arguments
  world:=lab|warehouse|keep|apartment     fleet_worlds world
  mission:=true|false                     fly the world's drone waypoints (false: PX4 idles on the ground)
  spawn:=takeoff|<world spawn name>       where the wyvern starts (default: the mission's takeoff point)
  x:= y:= yaw:=                           explicit spawn pose (overrides spawn:=; yaw in rad, ENU)
  mission_file:=<yaml>                    custom mission (same schema as a world yaml's drone: section)
  speed:=<m/s>                            cruise speed along the legs (default per world)
  accept_radius:=0.5                      waypoint acceptance radius [m]
  mode:=mission|hover                     hover: climb to hover_alt:=2.0, hold hover_time:=30 s, land (stats in result)
  shutdown_on_done:=true|false            stop the whole launch when the mission node exits
  result_file:=<json>                     mission node writes its result/metrics here
  capture_dir:=<dir>                      add a chase camera to the world copy and save frames (wyvern_gz_capture)
  gz_ip:=127.0.0.1                        GZ_IP for all gz processes ('' keeps the environment's)
  fresh_params:=true                      delete PX4's saved parameters in .px4/<world> before starting
  gnss_errors:=auto|true|false            GNSS error model (fleet_aircraft::GnssError, MAX-M10S numbers): auto = on
                                          outdoors (keep) only; false = ideal position source (indoor worlds have no GNSS)
  xrce_port:=8888 spawn_delay:=6 px4_delay:=4
  gui:=false rviz:=false                  windows (both off by default)
  px4_dir:=<PX4-Autopilot checkout>       default $PX4_DIR or ~/PX4-Autopilot (build/px4_sitl_default)
  agent:=<MicroXRCEAgent binary>          default $XRCE_AGENT, <workspace>/.deps/xrce_agent/bin/MicroXRCEAgent, PATH
  px4_dir is only read: PX4 runs with its working directory in <workspace>/.px4/<world>.
QGroundControl is never started; it can connect to PX4 on UDP 14550 if you run it yourself.
"""
import os
import re
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fleet_drone import route as R

BASE_Z = 0.24       # x500 base_link height above the ground when resting on its skids
DEFAULT_SPEED = {"lab": 1.0, "apartment": 0.8, "warehouse": 3.0, "keep": 5.0}

MAG_PATCH = ('<plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer">'
             '<use_units_gauss>false</use_units_gauss><use_earth_frame_ned>false</use_earth_frame_ned></plugin>')


def _b(ctx, name):
    return LaunchConfiguration(name).perform(ctx).lower() in ("1", "true", "yes", "on")


def _s(ctx, name):
    return LaunchConfiguration(name).perform(ctx)


def _workspace_root(share):
    # <ws>/install/fleet_drone/share/fleet_drone -> <ws>
    p = os.path.realpath(share)
    for _ in range(4):
        p = os.path.dirname(p)
    return p


# chase-camera distances [m] (cycled: close-up, wider), elevation [deg], sim seconds between frames
CAPTURE = {"lab": ((2.2, 3.6), 14.0, 4.0), "apartment": ((1.8, 2.6), 12.0, 4.0), "warehouse": ((2.6, 6.0), 14.0, 5.0),
           "keep": ((3.0, 9.0), 12.0, 6.0)}


def patch_world(src, dst):
    """Copy a world sdf, giving the Magnetometer system the units PX4 expects (Tesla, ENU earth frame).

    Returns the world name. PX4's own worlds get these settings from Tools/simulation/gz/server.config;
    the project's worlds declare their systems inline so the server config is never read.
    """
    with open(src) as fh:
        s = fh.read()
    name = re.search(r'<world\s+name="([^"]+)"', s).group(1)
    s2, n = re.subn(r'<plugin\s+filename="gz-sim-magnetometer-system"\s+name="gz::sim::systems::Magnetometer"\s*/>',
                    MAG_PATCH, s)
    if n == 0 and "use_units_gauss" not in s:
        s2 = s.replace("</world>", "  " + MAG_PATCH + "\n  </world>", 1)
    with open(dst, "w") as fh:
        fh.write(s2)
    return name


def world_geo(sdf):
    """(latitude_deg, surface_model) of a world file's <spherical_coordinates> (gz defaults: 0, EARTH_WGS84)."""
    with open(sdf) as fh:
        s = fh.read()
    lat, model = 0.0, "EARTH_WGS84"
    m = re.search(r"<spherical_coordinates>(.*?)</spherical_coordinates>", s, re.S)
    if m:
        a = re.search(r"<latitude_deg>\s*([-+0-9.eE]+)\s*<", m.group(1))
        b = re.search(r"<surface_model>\s*(\w+)\s*<", m.group(1))
        lat = float(a.group(1)) if a else lat
        model = b.group(1) if b else model
    return lat, model


def _setup(context):
    pkg = get_package_share_directory("fleet_drone")
    wpkg = get_package_share_directory("fleet_worlds")
    ws = _workspace_root(pkg)
    world = _s(context, "world")
    gui, rviz, mission = _b(context, "gui"), _b(context, "rviz"), _b(context, "mission")
    tmp = tempfile.mkdtemp(prefix="wyvern_%s_" % world)

    # world copy with the magnetometer units PX4 needs
    wsdf = os.path.join(tmp, world + ".sdf")
    wname = patch_world(os.path.join(wpkg, "worlds", world + ".sdf"), wsdf)
    geo_lat, geo_model = world_geo(wsdf)
    geo = {"geo_lat": geo_lat, "geo_model": geo_model}
    capture_dir = _s(context, "capture_dir")
    if capture_dir:
        from fleet_drone.gz_capture import camera_model_sdf
        with open(wsdf) as fh:
            s = fh.read()
        with open(wsdf, "w") as fh:
            fh.write(s.replace("</world>", camera_model_sdf() + "  </world>", 1))

    # mission + spawn pose
    mfile = _s(context, "mission_file")
    try:
        m = R.load_mission(world, mfile)
    except RuntimeError:
        m = None
    spawn = _s(context, "spawn")
    sx, sy, syaw = None, None, 0.0
    if spawn and spawn != "takeoff":
        import yaml
        with open(os.path.join(wpkg, "worlds", world + ".yaml")) as fh:
            wy = yaml.safe_load(fh)
        sp = wy.get("spawns", {}).get(spawn) if spawn != "default" else wy.get("spawn")
        if sp is None:
            raise RuntimeError("world %s has no spawn %r" % (world, spawn))
        sx, sy, syaw = float(sp["x"]), float(sp["y"]), float(sp.get("yaw", 0.0))
    elif m is not None:
        sx, sy, syaw = m["takeoff"]["x"], m["takeoff"]["y"], m["takeoff"].get("yaw", 0.0)
    if _s(context, "x"):
        sx = float(_s(context, "x"))
    if _s(context, "y"):
        sy = float(_s(context, "y"))
    if _s(context, "yaw"):
        syaw = float(_s(context, "yaw"))
    if sx is None:
        raise RuntimeError("no spawn pose: give x:= y:= or a world with a drone section")
    ground = R.ground_height(R.load_primitives(world), sx, sy)
    origin = (sx, sy, ground + BASE_Z)

    # PX4, agent
    px4_dir = _s(context, "px4_dir") or os.environ.get("PX4_DIR") or os.path.expanduser("~/PX4-Autopilot")
    px4_build = os.path.join(px4_dir, "build", "px4_sitl_default")
    px4_bin = os.path.join(px4_build, "bin", "px4")
    agent = _s(context, "agent") or os.environ.get("XRCE_AGENT", "")
    if not agent:
        cand = os.path.join(ws, ".deps", "xrce_agent", "bin", "MicroXRCEAgent")
        agent = cand if os.path.exists(cand) else (shutil.which("MicroXRCEAgent") or "MicroXRCEAgent")
    px4_wd = os.path.join(ws, ".px4", world)
    os.makedirs(px4_wd, exist_ok=True)
    if _b(context, "fresh_params"):
        for f in ("parameters.bson", "parameters_backup.bson"):
            for d in (px4_wd, os.path.join(px4_wd, "fs")):
                if os.path.exists(os.path.join(d, f)):
                    os.remove(os.path.join(d, f))

    res_path = os.pathsep.join(p for p in [os.path.join(pkg, "models"), os.path.join(wpkg, "models"),
                                           os.environ.get("GZ_SIM_RESOURCE_PATH", "")] if p)
    gz_env = {"GZ_SIM_RESOURCE_PATH": res_path}
    try:  # fleet_aircraft plugins (GnssError); an installed workspace's env hook also sets this
        from ament_index_python.packages import get_package_prefix
        gz_env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(
            p for p in [os.path.join(get_package_prefix("fleet_aircraft"), "lib"),
                        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")] if p)
    except Exception:
        pass
    # model copy: GNSS errors off -> zero stddevs, the plugin then republishes the truth unchanged
    gmode = (_s(context, "gnss_errors") or "auto").lower()
    gnss_on = (world == "keep") if gmode == "auto" else gmode in ("true", "1", "yes")
    msdf = os.path.join(pkg, "models", "wyvern", "model.sdf")
    if not gnss_on:
        with open(msdf) as fh:
            txt = fh.read()
        txt = re.sub(r"<(horizontal_stddev_m|vertical_stddev_m|velocity_stddev_mps)>[^<]*<", r"<\1>0<", txt)
        msdf = os.path.join(tmp, "wyvern_model.sdf")
        with open(msdf, "w") as fh:
            fh.write(txt)
        # the model's mesh URIs are model:// so the copy still resolves them through GZ_SIM_RESOURCE_PATH

    # /clock gets its own bridge in CLI form without use_sim_time: a config-file bridge that also runs with
    # use_sim_time:=true created the /clock publisher but delivered nothing to ROS subscribers (see notes).
    bridge = [
        dict(ros_topic_name="/wyvern/ground_truth/pose", gz_topic_name="/model/wyvern/pose",
             ros_type_name="geometry_msgs/msg/PoseStamped", gz_type_name="gz.msgs.Pose", direction="GZ_TO_ROS"),
    ]
    import yaml
    bcfg = os.path.join(tmp, "bridge.yaml")
    with open(bcfg, "w") as fh:
        yaml.safe_dump(bridge, fh)

    speed = float(_s(context, "speed") or DEFAULT_SPEED.get(world, 2.0))
    sim = {"use_sim_time": True}
    delay = float(_s(context, "spawn_delay"))
    acts = []
    if _s(context, "gz_ip"):
        # gz-transport discovery on loopback only: on hosts with VPN/extra interfaces the default interface choice
        # occasionally left one PX4 subscription (navsat) unconnected, so EKF2 never got GPS (see notes)
        acts.append(SetEnvironmentVariable("GZ_IP", _s(context, "gz_ip")))
    # --force-version 8: Gazebo Harmonic even when a newer Gazebo is also installed (the plugins are built against gz-sim 8)
    gz_cmd = ["gz", "sim", "--force-version", "8", "-s", "-r", "--headless-rendering", "-v", "2", wsdf]
    acts.append(ExecuteProcess(cmd=gz_cmd, output="screen", additional_env=gz_env, name="gz_server"))
    if gui:
        acts.append(ExecuteProcess(cmd=["gz", "sim", "--force-version", "8", "-g", "-v", "2"], output="log", additional_env=gz_env, name="gz_gui"))
    acts.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge", output="log",
                     arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]))
    acts.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="pose_bridge", output="log",
                     parameters=[{"config_file": bcfg}]))
    acts.append(TimerAction(period=delay, actions=[
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", wname, "-file", msdf, "-name", "wyvern",
                        "-x", "%.3f" % sx, "-y", "%.3f" % sy, "-z", "%.3f" % (ground + BASE_Z + 0.02), "-Y", "%.4f" % syaw])]))
    acts.append(ExecuteProcess(cmd=[agent, "udp4", "-p", _s(context, "xrce_port")], output="log", name="xrce_agent"))
    px4_env = {
        "PX4_SYS_AUTOSTART": "4001",
        "PX4_GZ_STANDALONE": "1",
        "PX4_GZ_MODEL_NAME": "wyvern",
        "PX4_GZ_WORLD": wname,
        "PX4_UXRCE_DDS_PORT": _s(context, "xrce_port"),
        "GZ_SIM_RESOURCE_PATH": res_path,
        # no RC / GCS in this setup: no data-link-loss failsafe, stick input disabled, offboard keeps running
        "PX4_PARAM_NAV_DLL_ACT": "0",
        "PX4_PARAM_COM_RC_IN_MODE": "4",
        "PX4_PARAM_COM_RCL_EXCEPT": "4",
        "PX4_PARAM_COM_OF_LOSS_T": "1.0",
        # GNSS baro auto-calibration steps the baro by the world elevation (488 m) some seconds after GNSS lock; if the
        # vehicle is armed by then, EKF2's height jumps ~2 m and it never lifts off (auto preflight disarm)
        "PX4_PARAM_SENS_BAR_AUTOCAL": "0",
        # the wyvern motor constants come from the T-Motor T1045 table: max rotor speed 1032.2 rad/s (see
        # docs/notes/air.md); PX4's gz bridge maps command 0..1 linearly onto [SIM_GZ_EC_MIN, SIM_GZ_EC_MAX]
        "PX4_PARAM_SIM_GZ_EC_MAX1": "1032", "PX4_PARAM_SIM_GZ_EC_MAX2": "1032",
        "PX4_PARAM_SIM_GZ_EC_MAX3": "1032", "PX4_PARAM_SIM_GZ_EC_MAX4": "1032",
    }
    px4_cmd = [px4_bin, "-d", "-w", px4_wd, "-s", "etc/init.d-posix/rcS", os.path.join(px4_build, "etc")]
    px4 = ExecuteProcess(cmd=px4_cmd, cwd=px4_wd, output="screen", additional_env=px4_env, name="px4")
    acts.append(TimerAction(period=delay + float(_s(context, "px4_delay")), actions=[px4]))
    acts.append(Node(package="fleet_drone", executable="wyvern_odom", output="screen",
                     parameters=[sim, geo, {"origin": list(origin), "mesh": "package://fleet_drone/models/wyvern/meshes/wyvern_body.glb"}]))
    if mission:
        if m is None:
            raise RuntimeError("mission:=true but world %s has no drone mission" % world)
        mnode = Node(package="fleet_drone", executable="wyvern_mission", output="screen",
                     parameters=[sim, geo, {"world": world, "mission_file": mfile, "origin": list(origin), "speed": speed,
                                       "result_file": _s(context, "result_file"),
                                       "mode": _s(context, "mode"),
                                       "hover_alt": float(_s(context, "hover_alt")),
                                       "hover_time": float(_s(context, "hover_time")),
                                       "accept_radius": float(_s(context, "accept_radius"))}])
        acts.append(TimerAction(period=delay + float(_s(context, "px4_delay")) + 2.0, actions=[mnode]))
        if _b(context, "shutdown_on_done"):
            acts.append(RegisterEventHandler(OnProcessExit(target_action=mnode, on_exit=[EmitEvent(event=Shutdown(reason="mission done"))])))
    if capture_dir:
        cap = CAPTURE.get(world, CAPTURE["lab"])
        args = ["--world", wname, "--out", capture_dir, "--prefix", "gz_drone_%s" % world,
                "--dist", *[str(d) for d in cap[0]], "--elev", str(cap[1]), "--period", str(cap[2]),
                "--min-cam-z", str(ground + 0.4)]
        acts.append(TimerAction(period=delay + 2.0, actions=[
            Node(package="fleet_drone", executable="wyvern_gz_capture", output="screen", arguments=args)]))
    if rviz:
        acts.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(pkg, "rviz", "drone.rviz")],
                         parameters=[sim], output="log"))
    return acts


def generate_launch_description():
    a = DeclareLaunchArgument
    return LaunchDescription([
        a("world", default_value="lab", description="fleet_worlds world: apartment | lab | warehouse | keep"),
        a("mission", default_value="true", description="fly the world's waypoint mission (false: PX4 idles on the ground)"),
        a("gui", default_value="false", description="open the Gazebo GUI (default: server only)"),
        a("rviz", default_value="false", description="open RViz2"),
        a("spawn", default_value="takeoff", description="takeoff (mission start) or a named spawn of the world yaml"),
        a("x", default_value="", description="spawn x override (m, world frame)"), a("y", default_value="", description="spawn y override (m)"),
        a("yaw", default_value="", description="spawn yaw override (rad)"),
        a("mission_file", default_value="", description="mission yaml instead of the world yaml's drone section"),
        a("speed", default_value="", description="cruise speed m/s (default per world: lab 1.0, apartment 0.8, warehouse 3.0, keep 5.0)"),
        a("accept_radius", default_value="0.5", description="waypoint acceptance radius (m)"),
        a("shutdown_on_done", default_value="false", description="shut the launch down when the mission ends"),
        a("result_file", default_value="", description="write the mission result JSON here"),
        a("px4_dir", default_value="", description="PX4-Autopilot checkout built for px4_sitl_default (default $PX4_DIR or ~/PX4-Autopilot, read only)"),
        a("agent", default_value="", description="MicroXRCEAgent binary (default: <workspace>/.deps/xrce_agent, else PATH)"),
        a("xrce_port", default_value="8888", description="uXRCE-DDS agent UDP port"),
        a("fresh_params", default_value="true", description="wipe the PX4 parameter store of this world before starting"),
        a("spawn_delay", default_value="6", description="seconds to wait for the Gazebo server before spawning the model"),
        a("px4_delay", default_value="4", description="seconds between the model spawn and the PX4 start"),
        a("gz_ip", default_value="127.0.0.1", description="GZ_IP for every gz process ('' keeps the environment)"),
        a("capture_dir", default_value="", description="adds a chase camera and saves frames there (wyvern_gz_capture)"),
        a("mode", default_value="mission", description="mission | hover (climb to hover_alt, hold hover_time s, land)"),
        a("gnss_errors", default_value="auto", description="auto (outdoor keep only) | true | false"),
        a("hover_alt", default_value="2.0", description="mode:=hover climb altitude (m)"),
        a("hover_time", default_value="30.0", description="mode:=hover hold time (s)"),
        OpaqueFunction(function=_setup),
    ])
