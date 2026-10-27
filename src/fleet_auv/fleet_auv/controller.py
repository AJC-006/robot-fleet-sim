"""knucker low-level controller: depth PID (+buoyancy feed-forward), pitch PD, heading PID, surge PI, allocation to the
4 thrusters, then thrust -> ESC pulse width through the T200 vendor table at the present bus voltage.

All topics are relative to the node namespace.
in : odometry/filtered (robot_localization EKF: depth from z, orientation, body-frame velocity and rates),
     battery/state (bus voltage for the thrust table),
     ref (Float64MultiArray [depth m (down +), heading rad (ENU yaw), surge m/s]),
     cmd_vel (Twist) and cmd_vel_stamped (TwistStamped): linear.x surge, linear.z heave rate (+ up),
     angular.z yaw rate, enable (Bool)
out: thrusters/{port,stbd,vfore,vaft}/pwm_us (Float64, microseconds), status (Float64MultiArray, see STATUS)
Physics (net buoyancy, restoring stiffness, damping for feed-forward, lever arms) come from config/knucker_physics.yaml
and config/knucker.yaml; the controller gains from the `control` section of knucker.yaml. S3 retunes the gains for the
T200 plant (asymmetric thrust, dead band, dynamics); until then they are the pre-realism values.
"""
import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64, Float64MultiArray

from . import catalog
from .common import clamp, quat_to_rpy, rot_body_to_world, wrap
from .params import load_physics
from .shape import load_cfg
from .t200_table import T200Table

STATUS = ["t", "depth", "depth_ref", "heading", "heading_ref", "pitch", "roll", "u", "u_ref", "w_world", "r", "q",
          "T_port", "T_stbd", "T_vfore", "T_vaft", "Z", "M", "X", "N"]
NAMES = ("port", "stbd", "vfore", "vaft")


class Controller(Node):
    def __init__(self):
        super().__init__("auv_controller")
        cfg = load_cfg()
        phys = load_physics()
        self.C = cfg["control"]
        comp = cfg["components"]
        t200 = catalog.load("thruster", comp["thruster"])
        self.table = T200Table(catalog.path("thruster", comp["thruster"], t200["specs"]["performance_table"]))
        self.pwm_stop = float(t200["specs"].get("pwm_stop_us", 1500))
        self.volt = float(self.declare_parameter("nominal_bus_voltage", 16.0).value)
        self.hyd = phys["hydrodynamics"]
        self.dt = 1.0 / self.C["rate_hz"]
        rb, fl = phys["rigid_body"], phys["fluid"]
        g = fl["gravity"]
        buoy = fl["density"] * g * rb["displaced_volume"]
        self.net_b = buoy - rb["mass"] * g                       # N, up +
        self.k_rest = buoy * (rb["cb"][2] - rb["cg"][2])         # N m/rad pitch restoring stiffness (BG * B)
        th = cfg["thrusters"]
        cg = rb["cg"]
        self.b_yaw = abs(th["stbd"]["pos"][1])           # N = b_yaw (Ts - Tp) for thrusters either side of the axis
        # pitch moment about +y (nose down +) of a positive vertical thrust: M = -(x - x_cg) T
        self.b_pitch = (-(th["vfore"]["pos"][0] - cg[0]), -(th["vaft"]["pos"][0] - cg[0]))
        # state
        self.depth_f = None
        self.rpy = None
        self.gyro = (0.0, 0.0, 0.0)
        self.vel = (0.0, 0.0, 0.0)
        self.vel_t = -1e9
        self.enable = True
        self.tgt = {"depth": None, "heading": None, "surge": 0.0}
        self.last_ref_t = -1e9
        self.rm = {"depth": [None, 0.0], "heading": [None, 0.0], "surge": 0.0}
        self.i = {"depth": 0.0, "heading": 0.0, "surge": 0.0}
        self.t0 = None
        self.depth = None
        self.u_f = [0.0] * 4
        sub = self.create_subscription
        sub(Odometry, "odometry/filtered", self.on_odom, 20)
        sub(BatteryState, "battery/state", lambda m: setattr(self, "volt", m.voltage) if m.voltage > 5.0 else None, 5)
        sub(Float64MultiArray, "ref", self.on_ref, 10)
        sub(Twist, "cmd_vel", lambda m: self.on_cmd(m), 10)
        sub(TwistStamped, "cmd_vel_stamped", lambda m: self.on_cmd(m.twist), 10)
        sub(Bool, "enable", self.on_enable, 10)
        self.pub = [self.create_publisher(Float64, "thrusters/%s/pwm_us" % n, 10) for n in NAMES]
        self.pst = self.create_publisher(Float64MultiArray, "status", 10)
        self.create_timer(self.dt, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs
    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.rpy = quat_to_rpy(q.x, q.y, q.z, q.w)
        w = m.twist.twist.angular
        self.gyro = (w.x, w.y, w.z)
        v = m.twist.twist.linear                  # child frame (base_link, FLU)
        self.vel = (v.x, v.y, v.z)
        self.vel_t = self.now()
        self.depth = -m.pose.pose.position.z      # odom z = 0 at the water surface (Bar30 depth_pose)
        self.depth_f = self.depth if self.depth_f is None else self.depth_f + 0.35 * (self.depth - self.depth_f)

    def on_ref(self, m):
        d, h, s = m.data[:3]
        self.tgt["depth"], self.tgt["heading"], self.tgt["surge"] = d, h, s
        self.last_ref_t = self.now()

    def on_cmd(self, tw):
        t = self.now()
        dt = 0.1
        self.last_ref_t = t
        self.tgt["surge"] = tw.linear.x
        if self.tgt["depth"] is not None:
            self.tgt["depth"] = clamp(self.tgt["depth"] - tw.linear.z * dt, 0.3, 40.0)
        if self.tgt["heading"] is not None:
            self.tgt["heading"] = wrap(self.tgt["heading"] + tw.angular.z * dt)

    def on_enable(self, m):
        self.enable = m.data
        if not m.data:
            self.i = {k: 0.0 for k in self.i}

    # ---- control
    def step(self):
        if self.depth is None or self.rpy is None:
            return
        t = self.now()
        if self.t0 is None:
            self.t0 = t
        roll, pitch, yaw = self.rpy
        dt = self.dt
        # initialise holds
        if self.tgt["depth"] is None:
            self.tgt["depth"] = self.depth_f
        if self.tgt["heading"] is None:
            self.tgt["heading"] = yaw
        if self.rm["depth"][0] is None:
            self.rm["depth"][0] = self.depth_f
        if self.rm["heading"][0] is None:
            self.rm["heading"][0] = yaw
        surge_tgt = self.tgt["surge"] if t - self.last_ref_t < 1.5 else 0.0
        # body velocity -> world heave rate
        R = rot_body_to_world(roll, pitch, yaw)
        v = self.vel if t - self.vel_t < 0.5 else (0.0, 0.0, 0.0)
        w_world = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]     # up +
        u = v[0]
        p, q, r = self.gyro
        # reference models (critically damped 2nd order with rate limit)
        cd, ch = self.C["depth"], self.C["heading"]
        rd = self.rm["depth"]
        acc = cd["wn"] ** 2 * (self.tgt["depth"] - rd[0]) - 2 * cd["wn"] * rd[1]
        rd[1] = clamp(rd[1] + acc * dt, -cd["rate_max"], cd["rate_max"])
        rd[0] += rd[1] * dt
        rh = self.rm["heading"]
        acc = ch["wn"] ** 2 * wrap(self.tgt["heading"] - rh[0]) - 2 * ch["wn"] * rh[1]
        rh[1] = clamp(rh[1] + acc * dt, -ch["rate_max"], ch["rate_max"])
        rh[0] = wrap(rh[0] + rh[1] * dt)
        cs = self.C["surge"]
        du = clamp(surge_tgt - self.rm["surge"], -cs["acc_max"] * dt, cs["acc_max"] * dt)
        self.rm["surge"] += du
        u_ref = self.rm["surge"]
        # depth: heave force Z (up +). depth error positive => too deep => push up
        e_d = self.depth_f - rd[0]
        self.i["depth"] = clamp(self.i["depth"] + cd["ki"] * e_d * dt, -cd["i_max"], cd["i_max"])
        down_rate = -w_world
        # feed-forward: cancel net buoyancy (up +) and the hull drag at the reference heave rate
        Z = cd["kp"] * e_d + cd["kd"] * (down_rate - rd[1]) + self.i["depth"] - self.net_b
        # pitch: nose-down positive, M about +y
        cp = self.C["pitch"]
        M = cp["kp"] * (0.0 - pitch) - cp["kd"] * q + self.k_rest * pitch
        # heading
        e_h = wrap(yaw - rh[0])
        self.i["heading"] = clamp(self.i["heading"] - ch["ki"] * e_h * dt, -ch["i_max"], ch["i_max"])
        r_ref = rh[1]
        N = -ch["kp"] * e_h - ch["kd"] * (r - r_ref) + self.i["heading"]
        # surge
        e_u = u_ref - u
        self.i["surge"] = clamp(self.i["surge"] + cs["ki"] * e_u * dt, -cs["i_max"], cs["i_max"])
        X = cs["kp"] * e_u + self.i["surge"] - (self.hyd["xU"] * u_ref + self.hyd["xUabsU"] * abs(u_ref) * u_ref)
        # allocation: asymmetric T200 limits at the present bus voltage -----------------------------------------
        lo, hi = self.table.limits(self.volt)
        Tmax = min(hi, -lo)
        # horizontal pair: N = b_yaw (Ts - Tp), X = Tp + Ts ; yaw has priority
        dT = clamp(N / (2 * self.b_yaw), -Tmax, Tmax)       # half difference (Ts - Tp)/2
        com = clamp(X / 2, -(Tmax - abs(dT)), Tmax - abs(dT))
        Tp, Ts = com - dT, com + dT
        # vertical pair: Z = Tf + Ta, M = bf Tf + ba Ta
        bf, ba = self.b_pitch
        det = ba - bf
        Tf = (Z * ba - M) / det
        Ta = (M - Z * bf) / det
        mx = max(abs(Tf), abs(Ta))
        if mx > Tmax:
            Tf, Ta = Tf * Tmax / mx, Ta * Tmax / mx
        cmd = [Tp, Ts, Tf, Ta]
        if not self.enable:
            cmd = [0.0] * 4
        for pb, val in zip(self.pub, cmd):
            m = Float64()
            m.data = float(self.table.pwm_for_force(clamp(val, lo, hi), self.volt, self.pwm_stop))
            pb.publish(m)
        self.u_f = cmd
        s = Float64MultiArray()
        s.data = [t - self.t0, self.depth_f, rd[0], yaw, rh[0], pitch, roll, u, u_ref, w_world, r, q] + list(self.u_f) + [Z, M, X, N]
        self.pst.publish(s)


def main():
    rclpy.init()
    n = Controller()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
