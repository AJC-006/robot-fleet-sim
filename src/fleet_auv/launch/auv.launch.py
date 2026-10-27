"""knucker AUV: Gazebo (headless by default), spawn, ros_gz_bridge, robot_state_publisher, driver-compatible sensor
nodes, robot_localization EKF (REP-105: map -> odom static, odom -> base_link from the EKF).

    ros2 launch fleet_auv auv.launch.py world:=keep gui:=false rviz:=false
    ros2 launch fleet_auv auv.launch.py world:=tank namespace:=knucker

Thruster command: std_msgs/Float64 ESC pulse width in microseconds on <namespace>/thrusters/<port|stbd|vfore|vaft>/pwm_us
(1100 full reverse, 1500 stop, 1900 full forward, like a real ESC).
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fleet_auv import model as M
from fleet_auv.shape import load_cfg


def _b(ctx, name):
    return LaunchConfiguration(name).perform(ctx).lower() in ("1", "true", "yes", "on")


def _s(ctx, name):
    return LaunchConfiguration(name).perform(ctx)


def _setup(context):
    pkg = get_package_share_directory("fleet_auv")
    cfg = load_cfg()
    mname = cfg["name"]
    ns = _s(context, "namespace")
    world = _s(context, "world")
    camera, gui = _b(context, "camera"), _b(context, "gui")
    tmp = tempfile.mkdtemp(prefix="knucker_")
    if world == "tank":
        wsdf = os.path.join(pkg, "worlds", "tank.sdf")
        wyaml = os.path.join(pkg, "config", "tank_mission.yaml")
    else:
        wp = get_package_share_directory("fleet_worlds")
        wsdf = os.path.join(wp, "worlds", world + ".sdf")
        wyaml = os.path.join(wp, "worlds", world + ".yaml")
    with open(wyaml) as f:
        wy = yaml.safe_load(f)
    sec = wy.get("auv", wy)
    sp = sec["spawn"]
    water = float(sec.get("water_level", 0.0))
    spawn = (sp["x"], sp["y"], sp["z"], sp.get("yaw", 0.0))
    sdf_path = os.path.join(tmp, mname + ".sdf")
    with open(sdf_path, "w") as f:
        f.write(M.sdf(camera=camera, spawn=(0, 0, 0, 0), name=mname))
    urdf = M.urdf(camera=camera, name=mname)
    g = "/model/" + mname
    E = lambda r, gz, rt, gt, d: dict(ros_topic_name=r, gz_topic_name=gz, ros_type_name=rt, gz_type_name=gt, direction=d)
    ent = [
        E("/clock", "/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS"),
        E("odometry_gt", g + "/odometry_gt", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
        E("imu_gz", g + "/imu_gz", "sensor_msgs/msg/Imu", "gz.msgs.IMU", "GZ_TO_ROS"),
        E("dvl_gz/velocity", g + "/dvl_gz", "marine_acoustic_msgs/msg/Dvl", "gz.msgs.DVLVelocityTracking", "GZ_TO_ROS"),
        E("battery/state", g + "/battery/state", "sensor_msgs/msg/BatteryState", "gz.msgs.BatteryState", "GZ_TO_ROS"),
    ]
    for n in cfg["thrusters"]:
        ent.append(E("thrusters/%s/pwm_us" % n, g + "/thrusters/%s/pwm_us" % n, "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"))
        for k in ("thrust", "current", "rpm"):
            ent.append(E("thrusters/%s/%s" % (n, k), g + "/thrusters/%s/%s" % (n, k), "std_msgs/msg/Float64", "gz.msgs.Double", "GZ_TO_ROS"))
    if camera:
        ent.append(E("camera/image", g + "/camera", "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"))
    bcfg = os.path.join(tmp, "bridge.yaml")
    with open(bcfg, "w") as f:
        yaml.safe_dump(ent, f)
    sim = {"use_sim_time": True}
    kw = dict(namespace=ns, output="screen")
    acts = []
    # --force-version 8: Gazebo Harmonic even when a newer Gazebo is also installed (the plugins are built against gz-sim 8)
    gz_cmd = (["gz", "sim", "--force-version", "8", "-r", "-v", "2", wsdf] if gui else
              ["gz", "sim", "--force-version", "8", "-s", "-r", "--headless-rendering", "-v", "2", wsdf])
    acts.append(ExecuteProcess(cmd=gz_cmd, output="screen"))
    wname = "tank" if world == "tank" else world
    acts.append(TimerAction(period=float(_s(context, "spawn_delay")), actions=[
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", wname, "-file", sdf_path, "-name", mname, "-x", str(spawn[0]), "-y", str(spawn[1]),
                        "-z", str(spawn[2]), "-Y", str(spawn[3])])]))
    acts.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
                     parameters=[{"config_file": bcfg, "use_sim_time": True}], **kw))
    acts.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                     parameters=[{"robot_description": urdf, "use_sim_time": True}], **kw))
    acts.append(Node(package="fleet_auv", executable="waterlinked_dvl_driver", parameters=[sim], **kw))
    acts.append(Node(package="fleet_auv", executable="microstrain_imu", parameters=[sim], **kw))
    acts.append(Node(package="fleet_auv", executable="bar30_driver",
                     parameters=[sim, {"water_level": water, "fluid_density": 999.103}], **kw))
    if _b(context, "localization"):
        acts.append(Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom",
                         arguments=["--x", str(spawn[0]), "--y", str(spawn[1]), "--z", str(water),
                                    "--frame-id", "map", "--child-frame-id", "odom"], parameters=[sim], **kw))
        acts.append(Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
                         parameters=[os.path.join(pkg, "config", "ekf.yaml")], **kw))
    if _b(context, "controller"):
        acts.append(Node(package="fleet_auv", executable="auv_controller", parameters=[sim], **kw))
    if _b(context, "mission"):
        acts.append(Node(package="fleet_auv", executable="auv_mission",
                         parameters=[sim, {"mission_file": wyaml, "cruise": float(_s(context, "cruise")),
                                           "log_csv": _s(context, "log_csv"), "odom_origin": [float(spawn[0]), float(spawn[1])], "exit_on_done": _b(context, "exit_on_done")}], **kw))
    if _b(context, "rviz"):
        acts.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(pkg, "rviz", "auv.rviz")],
                         parameters=[sim], output="log"))
    return acts


def generate_launch_description():
    a = DeclareLaunchArgument
    return LaunchDescription([
        a("world", default_value="keep", description="World: 'keep' (fleet_worlds lake) or 'tank' (20 m test tank), or another fleet_worlds world name"),
        a("namespace", default_value="knucker", description="ROS namespace of every robot node and topic (the Gazebo model name stays knucker)"),
        a("gui", default_value="false", description="Open the Gazebo GUI (default is the headless server only)"),
        a("rviz", default_value="false", description="Open RViz2"),
        a("camera", default_value="false", description="Add the forward camera sensor"),
        a("localization", default_value="true", description="Run the robot_localization EKF and the static map->odom transform (REP-105)"),
        a("controller", default_value="false", description="Run the depth/heading/surge controller (commands the T200 ESC pulse widths; gains are the pre-realism values, S3 retunes)"),
        a("mission", default_value="false", description="Run the waypoint mission node (needs controller:=true)"),
        a("cruise", default_value="0.7", description="Mission cruise speed in m/s"),
        a("log_csv", default_value="", description="Mission log file ('' disables)"),
        a("spawn_delay", default_value="12", description="Seconds to wait for the Gazebo server before spawning the AUV"),
        a("exit_on_done", default_value="true", description="Mission node exits when the mission completes"),
        OpaqueFunction(function=_setup),
    ])
