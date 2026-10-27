"""knucker SDF and URDF generators.

Inputs: config/knucker.yaml (layout, component names), config/knucker_physics.yaml (derived rigid body, added mass,
damping, buoyancy primitives) and the fleet_components catalog (thruster table, battery, sensor specs).
"""
import math

import numpy as np

from . import catalog
from .params import load_physics
from .shape import load_cfg

PI2 = math.pi / 2
PROP_MASS = 0.01     # kg per propeller link: visual spin only (the T200 mass is inside the base link inertia)
PROP_R = 0.038


def default_mesh_dir():
    import os
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory("fleet_auv"), "meshes", "knucker")
        if os.path.exists(os.path.join(d, "hull.obj")):
            return d
    except Exception:
        pass
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "meshes", "knucker"))


def _pose(p, rpy=(0, 0, 0)):
    return "%.6f %.6f %.6f %.6f %.6f %.6f" % (tuple(p) + tuple(rpy))


def thr_rpy(d):
    """Link rpy that turns the link x axis onto the thrust direction d (x or z only)."""
    if abs(d[0]) > 0.5:
        return (0, 0, 0) if d[0] > 0 else (0, 0, math.pi)
    return (0, -PI2 if d[2] > 0 else PI2, 0)


def _pa(d):
    """Parallel-axis tensor |d|^2 I - d d^T."""
    d = np.asarray(d, float)
    return d.dot(d) * np.eye(3) - np.outer(d, d)


def base_inertial(phys, th):
    """Base link mass, centre and inertia tensor so that the whole model (base + propeller links) keeps the derived
    total mass, CG and full inertia tensor. The propeller discs' own inertia (7e-6 kg m^2) is neglected."""
    rb = phys["rigid_body"]
    M, cg = rb["mass"], np.array(rb["cg"])
    i = rb["inertia_cg"]
    I = np.array([[i["ixx"], i["ixy"], i["ixz"]], [i["ixy"], i["iyy"], i["iyz"]], [i["ixz"], i["iyz"], i["izz"]]])
    mb = M - PROP_MASS * len(th)
    cb = (M * cg - sum(PROP_MASS * np.array(t["pos"]) for t in th.values())) / mb
    Ib = I - mb * _pa(cb - cg) - sum(PROP_MASS * _pa(np.array(t["pos"]) - cg) for t in th.values())
    return mb, cb, Ib


def _geom(c):
    if c["axis"] == "x":
        rpy = (0, PI2, 0)
    elif c["axis"] == "y":
        rpy = (PI2, 0, 0)
    else:
        rpy = (0, 0, 0)
    return "<cylinder><radius>%.6f</radius><length>%.6f</length></cylinder>" % (c["r"], c["l"]), rpy


def _noise(tag, sd, bias_sd=0.0, dyn_sd=0.0, tau=100.0):
    return ("<%s><noise type=\"gaussian\"><mean>0</mean><stddev>%.6g</stddev><bias_mean>0</bias_mean>"
            "<bias_stddev>%.6g</bias_stddev><dynamic_bias_stddev>%.6g</dynamic_bias_stddev>"
            "<dynamic_bias_correlation_time>%g</dynamic_bias_correlation_time></noise></%s>" % (tag, sd, bias_sd, dyn_sd, tau, tag))


def sdf(camera=False, mesh_dir=None, cfg=None, spawn=(0, 0, -2.0, 0.0), name=None, phys=None):
    import os
    cfg = cfg or load_cfg()
    phys = phys or load_physics()
    name = name or cfg["name"]
    mesh_dir = os.path.abspath(mesh_dir or default_mesh_dir())
    comp = cfg["components"]
    t200 = catalog.load("thruster", comp["thruster"])
    esc = catalog.load("esc", comp["esc"])
    bat = catalog.load("battery", comp["battery"])
    ahrs = catalog.load("ahrs", comp["ahrs"])
    dvl = catalog.load("dvl", comp["dvl"])
    table = catalog.path("thruster", comp["thruster"], t200["specs"]["performance_table"])
    th, S = cfg["thrusters"], cfg["sensors"]
    rb = phys["rigid_body"]
    mb, cb, Ib = base_inertial(phys, th)
    o = []
    a = o.append
    a('<?xml version="1.0"?>\n<sdf version="1.11">\n<model name="%s">' % name)
    a('<pose>%s</pose>' % _pose(spawn[:3], (0, 0, spawn[3])))
    a('<link name="base_link">')
    a('<inertial><pose>%s</pose><mass>%.6f</mass><inertia><ixx>%.8f</ixx><iyy>%.8f</iyy><izz>%.8f</izz>'
      '<ixy>%.8f</ixy><ixz>%.8f</ixz><iyz>%.8f</iyz></inertia>' % (
          _pose(cb), mb, Ib[0, 0], Ib[1, 1], Ib[2, 2], Ib[0, 1], Ib[0, 2], Ib[1, 2]))
    # Added mass is about the link origin (base_link frame); see docs/notes/sea.md.
    am = phys["fluid_added_mass"]
    a('<fluid_added_mass>' + "".join("<%s>%.6f</%s>" % (k, v, k) for k, v in am.items()) + '</fluid_added_mass></inertial>')
    a('<visual name="hull"><geometry><mesh><uri>file://%s/hull.obj</uri></mesh></geometry></visual>' % mesh_dir)
    for c in phys["buoyancy_primitives"]:
        g, rpy = _geom(c)
        a('<collision name="%s"><pose>%s</pose><geometry>%s</geometry></collision>' % (c["name"], _pose(c["pos"], rpy), g))
    # IMU: CV7, noise from the catalog (sigma = density * sqrt(rate)), turn-on bias and bias instability
    s = ahrs["sim"]
    rate = float(ahrs["ros_driver"]["rate_hz"])
    a('<sensor name="imu" type="imu"><pose>%s</pose><always_on>1</always_on><update_rate>%g</update_rate>'
      '<topic>/model/%s/imu_gz</topic><imu><orientation_reference_frame><localization>ENU</localization></orientation_reference_frame>'
      '<angular_velocity>%s%s%s</angular_velocity><linear_acceleration>%s%s%s</linear_acceleration></imu></sensor>' % (
          _pose(S["imu"]["pos"]), rate, name,
          *[_noise(ax, s["gyro_noise_stddev_100hz"], s["gyro_bias_stddev"], s["gyro_dynamic_bias_stddev"]) for ax in "xyz"],
          *[_noise(ax, s["accel_noise_stddev_100hz"], s["accel_bias_stddev"], s["accel_dynamic_bias_stddev"]) for ax in "xyz"]))
    # DVL: gz DopplerVelocityLog. Beams 22.5 deg from the vertical (A50 datasheet), sensor frame looks along -z,
    # output rotated to FRD by reference_frame (roll pi) like the A50 driver's dvl_link.
    ds = dvl["specs"]
    ang = 22.5
    beams = "".join('<beam><aperture>%g</aperture><rotation>%g</rotation><tilt>%g</tilt></beam>' % (
        S["dvl"]["beam_width_deg"], r, ang) for r in (45, 135, -135, -45))
    a('<sensor name="dvl" type="custom" gz:type="dvl" xmlns:gz="http://gazebosim.org/schema"><pose>%s</pose>'
      '<always_on>1</always_on><update_rate>%g</update_rate><topic>/model/%s/dvl_gz</topic>'
      '<gz:dvl><type>phased_array</type><arrangement degrees="true">%s</arrangement>'
      '<tracking><bottom_mode><when>best</when><noise type="gaussian"><stddev>%g</stddev></noise>'
      '<visualize>false</visualize></bottom_mode></tracking>'
      '<resolution>%g</resolution><maximum_range>%g</maximum_range><minimum_range>%g</minimum_range>'
      '<reference_frame degrees="false">0 0 0 %.6f 0 0</reference_frame></gz:dvl></sensor>' % (
          _pose(S["dvl"]["pos"]), float(S["dvl"]["rate_hz"]), name, beams, S["dvl"]["velocity_noise"],
          S["dvl"]["ray_spacing_rad"], ds["altitude_max"], ds["altitude_min"], math.pi))
    if camera:
        cp = S["camera"]
        a('<sensor name="camera" type="camera"><pose>%s</pose><always_on>1</always_on><update_rate>10</update_rate>'
          '<topic>/model/%s/camera</topic><camera><horizontal_fov>%g</horizontal_fov>'
          '<image><width>640</width><height>360</height></image><clip><near>0.05</near><far>60</far></clip></camera></sensor>'
          % (_pose(cp["pos"]), name, cp["hfov"]))
    a('</link>')
    for key, t in th.items():
        rpy = thr_rpy(t["dir"])
        a('<link name="prop_%s"><pose>%s</pose>'
          '<inertial><mass>%g</mass><inertia><ixx>%g</ixx><iyy>%g</iyy><izz>%g</izz></inertia></inertial>'
          '<visual name="v"><pose>0 0 0 0 %.6f 0</pose><geometry><mesh><uri>file://%s/prop.obj</uri></mesh></geometry></visual></link>'
          % (key, _pose(t["pos"], rpy), PROP_MASS, 0.5 * PROP_MASS * PROP_R ** 2, 0.25 * PROP_MASS * PROP_R ** 2,
             0.25 * PROP_MASS * PROP_R ** 2, PI2, mesh_dir))
        a('<joint name="%s" type="revolute"><parent>base_link</parent><child>prop_%s</child>'
          '<axis><xyz>1 0 0</xyz><limit><lower>-1e12</lower><upper>1e12</upper></limit></axis></joint>' % (t["joint"], key))
    # fluid: graded buoyancy lives in the world; the Hydrodynamics plugin gives drag and fin lift
    hyd = phys["hydrodynamics"]
    a('<plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics"><link_name>base_link</link_name>'
      "<namespace>%s</namespace>" % name + "".join("<%s>%.6g</%s>" % (k, v, k) for k, v in hyd.items()) + '</plugin>')
    sp = esc["specs"]
    for key, t in th.items():
        a('<plugin filename="fleet_t200_thruster" name="fleet_marine::T200Thruster"><thruster_name>%s</thruster_name>'
          '<table>%s</table><body_link>base_link</body_link><prop_link>prop_%s</prop_link><joint_name>%s</joint_name>'
          '<axis>1 0 0</axis><pwm_min_us>%g</pwm_min_us><pwm_stop_us>%g</pwm_stop_us><pwm_max_us>%g</pwm_max_us>'
          '<deadband_us>%g</deadband_us><initial_voltage_v>%g</initial_voltage_v></plugin>' % (
              key, table, key, t["joint"], sp["pwm_min_us"], sp["pwm_stop_us"], sp["pwm_max_us"], cfg["thruster_deadband_us"],
              bat["specs"]["voltage_nominal"]))
    bs = bat["specs"]
    a('<plugin filename="fleet_liion_battery" name="fleet_marine::LiIonBattery"><cells_series>4</cells_series>'
      '<cells_parallel>6</cells_parallel><capacity_ah>%g</capacity_ah><pack_resistance_ohm>%g</pack_resistance_ohm>'
      '<hotel_power_w>%g</hotel_power_w><current_continuous_a>%g</current_continuous_a><current_burst_a>%g</current_burst_a>'
      '<burst_duration_s>10</burst_duration_s><initial_soc>1.0</initial_soc><ocv_table>%s</ocv_table>%s</plugin>' % (
          bs["capacity"], bat["sim"]["pack_resistance"], bat["sim"]["hotel_power"], bs["current_continuous"],
          bs["current_burst_10s"], bat["sim"]["ocv_table_per_cell"],
          "".join("<current_topic>/model/%s/thrusters/%s/current</current_topic>" % (name, k) for k in th)))
    a('<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">'
      '<publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose></plugin>')
    a('<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
      '<odom_frame>world</odom_frame><robot_base_frame>base_link</robot_base_frame><dimensions>3</dimensions>'
      '<odom_publish_frequency>50</odom_publish_frequency><odom_topic>/model/%s/odometry_gt</odom_topic></plugin>' % name)
    a('</model>\n</sdf>')
    return "\n".join(o)


def urdf(camera=False, mesh_dir=None, cfg=None, name=None):
    """URDF for robot_state_publisher: hull, propellers (continuous joints driven by /joint_states if wanted) and the
    sensor frames. dvl_link is FRD (roll pi) as published by the Water Linked driver; the others are FLU/ENU."""
    import os
    cfg = cfg or load_cfg()
    name = name or cfg["name"]
    mesh_dir = os.path.abspath(mesh_dir or default_mesh_dir())
    th, S = cfg["thrusters"], cfg["sensors"]
    o = ['<?xml version="1.0"?>\n<robot name="%s">' % name,
         '<link name="base_link"><visual><geometry><mesh filename="file://%s/hull.obj"/></geometry></visual></link>' % mesh_dir]
    for key, t in th.items():
        o.append('<link name="thruster_%s_link"><visual><origin rpy="0 %.6f 0"/><geometry><mesh filename="file://%s/prop.obj"/>'
                 '</geometry></visual></link>' % (key, PI2, mesh_dir))
        o.append('<joint name="%s" type="continuous"><parent link="base_link"/><child link="thruster_%s_link"/>'
                 '<origin xyz="%s" rpy="%s"/><axis xyz="1 0 0"/></joint>'
                 % (t["joint"], key, " ".join("%g" % v for v in t["pos"]), " ".join("%.6f" % v for v in thr_rpy(t["dir"]))))
    rpy = {"dvl": "%.6f 0 0" % math.pi}
    names = ["imu", "dvl", "pressure", "battery"] + (["camera"] if camera else [])
    for n in names:
        o.append('<link name="%s_link"/><joint name="%s_joint" type="fixed"><parent link="base_link"/><child link="%s_link"/>'
                 '<origin xyz="%s" rpy="%s"/></joint>' % (n, n, n, " ".join("%g" % v for v in S[n]["pos"]), rpy.get(n, "0 0 0")))
    o.append("</robot>")
    return "\n".join(o)


def main():
    import sys
    print(sdf(camera="--camera" in sys.argv) if "--urdf" not in sys.argv else urdf())


if __name__ == "__main__":
    main()
