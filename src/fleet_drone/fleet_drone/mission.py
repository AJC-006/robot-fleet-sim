"""wyvern_mission: PX4 offboard mission over uXRCE-DDS.

Arms, takes off vertically to the mission's cruise altitude, flies the 3D waypoints in order (smooth trapezoidal
setpoints with velocity feed-forward, yaw along the leg), returns above the takeoff point, descends, lands with
NAV_LAND and disarms. Exit code 0 when every waypoint was reached within `accept_radius` (PX4 estimate), the
vehicle landed and disarmed; 1 otherwise.

mode:=hover instead climbs to `hover_alt` above the takeoff point, holds `hover_time` seconds and lands; the
result reports the position standard deviation (PX4 estimate and gz ground truth).

Waypoints are world ENU (gz world frame). PX4's local NED origin sits at parameter `origin` (world coordinates of
the vehicle's resting base_link at boot, which the launch file passes); world <-> local also applies the geodetic
scale of px4io.geo_scale (parameters geo_lat, geo_model from the world's <spherical_coordinates>).
Ground-truth errors: the gz pose (and /clock) reach ROS ~0.1 s of sim time after PX4, which runs directly on the gz
clock, has produced its estimate of the same instant. finish() therefore fits that constant offset (least squares of
estimate vs. pose history over the flight, `gt_time_offset_s`) and evaluates ground truth at record time + offset;
`gt_err_raw` keeps the error against the newest pose at the moment of reaching.
"""
import csv
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,
                          VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

from fleet_drone import px4io as P
from fleet_drone import route as R

NAN = float("nan")


class Leg:
    """Straight segment from a to b (local NED) flown with a trapezoidal speed profile."""

    def __init__(self, a, b, vmax, acc, yaw):
        self.a, self.b = np.asarray(a, float), np.asarray(b, float)
        d = self.b - self.a
        self.L = float(np.linalg.norm(d))
        self.u = d / self.L if self.L > 1e-6 else np.zeros(3)
        self.vmax, self.acc, self.yaw = vmax, acc, yaw
        ta = vmax / acc
        if self.L < vmax * ta:          # triangle profile
            ta = math.sqrt(self.L / acc)
            self.vpk = acc * ta
            self.tc = 0.0
        else:
            self.vpk = vmax
            self.tc = (self.L - vmax * ta) / vmax
        self.ta = ta
        self.T = 2 * ta + self.tc

    def at(self, t):
        t = min(max(t, 0.0), self.T)
        a, ta, tc, v = self.acc, self.ta, self.tc, self.vpk
        if t < ta:
            s, sd, sdd = 0.5 * a * t * t, a * t, a
        elif t < ta + tc:
            s, sd, sdd = 0.5 * a * ta * ta + v * (t - ta), v, 0.0
        else:
            r = self.T - t
            s, sd, sdd = self.L - 0.5 * a * r * r, a * r, -a
        return self.a + self.u * s, self.u * sd, self.u * sdd


class Mission(Node):
    def __init__(self):
        super().__init__("wyvern_mission")
        dp = self.declare_parameter
        self.world = dp("world", "lab").value
        self.mode = dp("mode", "mission").value
        self.origin = np.array(dp("origin", [0.0, 0.0, 0.24]).value, float)
        self.k = P.geo_scale(float(dp("geo_lat", 0.0).value), dp("geo_model", "EARTH_WGS84").value)
        self.speed = float(dp("speed", 1.0).value)
        self.vz = float(dp("vertical_speed", 1.2).value)       # descent limit of a leg's vertical component
        self.vz_up = float(dp("climb_speed", 2.0).value)        # climb limit
        self.acc = float(dp("accel", 1.0).value)
        self.acc_r = float(dp("accept_radius", 0.5).value)
        self.settle_r = float(dp("settle_radius", 0.2).value)
        self.settle_t = float(dp("settle_timeout", 12.0).value)
        self.yaw_rate = math.radians(float(dp("yaw_rate_deg", 60.0).value))
        self.hover_alt = float(dp("hover_alt", 2.0).value)
        self.hover_time = float(dp("hover_time", 30.0).value)
        self.result_file = dp("result_file", "").value
        self.timeout = float(dp("timeout", 1800.0).value)
        mfile = dp("mission_file", "").value

        m = R.load_mission(self.world, mfile)
        self.m = m
        ground = self.origin[2] - 0.24
        if self.mode == "hover":
            t = m["takeoff"]
            pts = [("ground", t["x"], t["y"], ground + 0.3, False),
                   ("hover", t["x"], t["y"], ground + self.hover_alt, True),
                   ("land", t["x"], t["y"], ground + 0.3, False)]
        else:
            pts = R.route_points(m, ground)
        # local NED points; the final 'land' point is replaced by a 1 m-above-ground descent, then NAV_LAND
        self.route = []
        for (name, x, y, z, is_wp) in pts[1:]:
            if name == "land":
                z = ground + 1.0 + 0.24
            ned = P.enu_to_ned(np.array([x, y, z]) - self.origin) * self.k
            self.route.append((name, ned, is_wp, (x, y, z)))

        self.status = self.lpos = self.land = None
        self.gt = None
        self.gt_hist = []       # (gz stamp, x, y, z)
        self.clock_diag = []    # (node clock, newest gt stamp)
        self.topics = {}
        self.pub_ocm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", P.QOS_PX4_IN)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", P.QOS_PX4_IN)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", P.QOS_PX4_IN)
        self.create_subscription(PoseStamped, "/wyvern/ground_truth/pose", self._gt, qos_profile_sensor_data)

        self._publish_markers([(self.m["takeoff"]["x"], self.m["takeoff"]["y"], ground + 0.24)]
                              + [(r[3], r[2], r[0]) for r in self.route])

        self.state = "discover"
        self.t_state = None
        self.sp_pos = None
        self.sp_vel = np.zeros(3)
        self.sp_acc = None
        self.sp_yaw = None
        self.leg_i = -1
        self.leg = None
        self.leg_t0 = None
        self.phase = None
        self.results = []
        self.hover_log = []
        self.log_rows = []
        self.ok = True
        self.reason = ""
        self.t_start = None
        self.t_takeoff = None
        self.wall_takeoff = None
        self.wall0 = time.time()
        self.sim0 = None
        self.last_cmd = None
        self.retries = 0
        self.done = False
        self.exit_code = 1
        self.timer = self.create_timer(0.02, self._tick)
        self.get_logger().info("mission %s (%s): %d route points, speed %.1f m/s, origin (%.2f, %.2f, %.2f)" % (
            self.world, self.mode, len(self.route), self.speed, *self.origin))

    # ---------------------------------------------------------------- subscriptions
    def _gt(self, m):
        self.gt = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        self.gt_hist.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, *self.gt))

    def gt_offset(self, H):
        """Constant time offset tau [s] (in [-0.5, 0.5]) minimising |estimate(t) - ground truth(t + tau)| over the
        airborne log; 0 without enough data."""
        rows = [r for r in self.log_rows if r[1] == "fly"]
        if H is None or len(rows) < 50:
            return 0.0
        t = np.array([r[0] for r in rows])
        est = np.array([r[11] for r in rows])
        taus = np.arange(-0.5, 0.5001, 0.005)
        cost = [np.nanmean(np.sum((est - self.gt_at(t + x, H)) ** 2, axis=1)) for x in taus]
        return float(taus[int(np.nanargmin(cost))])

    def gt_at(self, ts, h=None):
        """gz ground truth at sim time(s) ts, interpolated in the pose history h (default: its last 80 s; the newest
        sample if ts is newer). None without data."""
        if h is None:
            if not self.gt_hist:
                return None
            h = np.asarray(self.gt_hist[-4000:])
        return np.stack([np.interp(ts, h[:, 0], h[:, i]) for i in (1, 2, 3)], axis=-1)

    def _sub(self, base, typ, attr):
        name = P.find_topic(self, base)
        if name and base not in self.topics:
            self.topics[base] = name
            self.create_subscription(typ, name, lambda msg: setattr(self, attr, msg), P.QOS_PX4_OUT)
            self.get_logger().info("subscribed %s" % name)

    def _ack(self, m):
        self.get_logger().info("ack: command %d result %d" % (m.command, m.result))

    # ---------------------------------------------------------------- outputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cmd(self, command, p1=0.0, p2=0.0, p7=0.0, p4=NAN):
        c = VehicleCommand()
        c.timestamp = 0     # PX4 stamps it on reception (sim-time vs. timesync safe)
        c.command, c.param1, c.param2, c.param4, c.param7 = command, float(p1), float(p2), float(p4), float(p7)
        c.param3 = 0.0
        c.target_system, c.target_component = 1, 1
        c.source_system, c.source_component = 1, 1
        c.from_external = True
        self.pub_cmd.publish(c)

    def stream(self):
        o = OffboardControlMode()
        o.timestamp = 0
        o.position = True
        self.pub_ocm.publish(o)
        s = TrajectorySetpoint()
        s.timestamp = 0
        s.position = [float(v) for v in self.sp_pos]
        s.velocity = [float(v) for v in self.sp_vel]
        s.acceleration = [float(v) for v in self.sp_acc] if self.sp_acc is not None else [NAN, NAN, NAN]
        s.jerk = [NAN, NAN, NAN]
        s.yaw = float(self.sp_yaw)
        s.yawspeed = NAN
        self.pub_sp.publish(s)

    def pos(self):
        return np.array([self.lpos.x, self.lpos.y, self.lpos.z])

    def world_of(self, ned):
        return P.ned_to_enu(np.asarray(ned) / self.k) + self.origin

    def goto_state(self, s):
        self.get_logger().info("state %s -> %s (t=%.1f)" % (self.state, s, self.now()))
        self.state, self.t_state = s, self.now()

    def fail(self, why):
        self.ok = False
        self.reason = (self.reason + "; " if self.reason else "") + why
        self.get_logger().error(why)

    # ---------------------------------------------------------------- main loop
    def _publish_markers(self, pts):
        """Route for RViz on /wyvern/mission_markers (map = gz world frame), latched."""
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_mk = self.create_publisher(MarkerArray, "/wyvern/mission_markers", qos)
        arr = MarkerArray()
        line = Marker()
        line.header.frame_id, line.ns, line.id, line.type = "map", "route", 0, Marker.LINE_STRIP
        line.scale.x = 0.05
        line.color.r, line.color.g, line.color.b, line.color.a = 0.75, 0.3, 1.0, 0.9
        line.pose.orientation.w = 1.0
        xyz0 = pts[0]
        line.points.append(Point(x=float(xyz0[0]), y=float(xyz0[1]), z=float(xyz0[2])))
        for i, (xyz, is_wp, name) in enumerate(pts[1:]):
            line.points.append(Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
            if not is_wp:
                continue
            for k, typ in ((0, Marker.SPHERE), (1, Marker.TEXT_VIEW_FACING)):
                mk = Marker()
                mk.header.frame_id, mk.ns, mk.id, mk.type = "map", "waypoints", 2 * i + k + 1, typ
                mk.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]) + 0.6 * k)
                mk.pose.orientation.w = 1.0
                if k == 0:
                    d = 2 * self.acc_r
                    mk.scale.x = mk.scale.y = mk.scale.z = d
                    mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.2, 0.8, 0.35
                else:
                    mk.text = name
                    mk.scale.z = 0.35
                    mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
                arr.markers.append(mk)
        arr.markers.append(line)
        self.pub_mk.publish(arr)

    def _tick(self):
        t = self.now()
        if t <= 0.0:
            return
        if self.sim0 is None:
            self.sim0, self.wall0 = t, time.time()
        if self.t_state is None:
            self.t_state = t
        if t - self.sim0 > self.timeout and self.state != "done":
            self.fail("mission timeout")
            self.finish()
            return
        st = self.state
        if st == "discover":
            self._sub("/fmu/out/vehicle_status", VehicleStatus, "status")
            self._sub("/fmu/out/vehicle_local_position", VehicleLocalPosition, "lpos")
            self._sub("/fmu/out/vehicle_land_detected", VehicleLandDetected, "land")
            if "/fmu/out/vehicle_command_ack" not in self.topics:
                n = P.find_topic(self, "/fmu/out/vehicle_command_ack")
                if n:
                    self.topics["/fmu/out/vehicle_command_ack"] = n
                    self.create_subscription(VehicleCommandAck, n, self._ack, P.QOS_PX4_OUT)
            if self.status is not None and self.lpos is not None and self.land is not None:
                self.goto_state("wait_ready")
            return
        if self.lpos is None or self.status is None:
            return
        self._log(t)
        if st == "wait_ready":
            # also wait until the estimate has been at rest for 2 s (an EKF height step on the ground would otherwise
            # leave the vehicle "airborne" in its own estimate and never lift off)
            still = max(abs(self.lpos.vx), abs(self.lpos.vy), abs(self.lpos.vz)) < 0.1
            self._still_since = (getattr(self, "_still_since", None) or t) if still else None
            if self.lpos.xy_valid and self.lpos.z_valid and self.status.pre_flight_checks_pass \
                    and self._still_since is not None and t - self._still_since >= 2.0:
                p = self.pos()
                self.sp_pos, self.sp_vel = p.copy(), np.zeros(3)
                self.sp_yaw = self.lpos.heading
                self.goto_state("prestream")
            elif t - self.t_state > 120:
                self.fail("PX4 not ready to arm after 120 s (xy_valid=%s z_valid=%s preflight=%s)" % (
                    self.lpos.xy_valid, self.lpos.z_valid, self.status.pre_flight_checks_pass))
                self.finish()
            return
        if st in ("prestream", "offboard", "arm", "fly", "descend"):
            self.stream()
        if st == "prestream":
            if t - self.t_state > 1.5:
                self.goto_state("offboard")
                self.last_cmd = None
            return
        if st == "offboard":
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.goto_state("arm")
                self.last_cmd = None
            elif self.last_cmd is None or t - self.last_cmd > 1.0:
                self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.last_cmd = t
                if t - self.t_state > 15:
                    self.fail("offboard mode rejected")
                    self.finish()
            return
        if st == "arm":
            if self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.t_takeoff, self.wall_takeoff = t, time.time()
                self.goto_state("fly")
                self.next_leg(t)
            elif self.last_cmd is None or t - self.last_cmd > 1.0:
                self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.last_cmd = t
                if t - self.t_state > 15:
                    self.fail("arming rejected")
                    self.finish()
            return
        if st == "fly":
            if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.fail("left offboard/armed during flight (nav_state %d, arming %d, failsafe %s)" % (
                    self.status.nav_state, self.status.arming_state, self.status.failsafe))
                self.goto_state("land")
                self.last_cmd = None
                return
            self.fly(t)
            return
        if st == "land":
            if self.last_cmd is None or (t - self.last_cmd > 2.0 and self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                                         and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND, p4=NAN)
                self.last_cmd = t
            if self.land is not None and self.land.landed:
                self.goto_state("disarm")
                self.last_cmd = None
            elif t - self.t_state > 90:
                self.fail("landing not detected after 90 s")
                self.finish()
            return
        if st == "disarm":
            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("landed and disarmed")
                self.finish()
            elif t - self.t_state > 4.0 and (self.last_cmd is None or t - self.last_cmd > 1.0):
                self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self.last_cmd = t
                if t - self.t_state > 20:
                    self.fail("disarm rejected")
                    self.finish()
            return

    def next_leg(self, t):
        self.leg_i += 1
        if self.leg_i >= len(self.route):
            self.goto_state("land")
            self.last_cmd = None
            return
        name, b, is_wp, wb = self.route[self.leg_i]
        a = self.sp_pos.copy()
        d = b - a
        horiz = math.hypot(d[0], d[1])
        yaw = math.atan2(d[1], d[0]) if horiz > 1.0 else self.sp_yaw
        vmax = self.speed
        acc = self.acc
        # PX4 clamps offboard velocity to MPC_Z_VEL_MAX_UP / _DN (3.0 / 1.5 m/s): keep each leg's vertical component
        # below climb_speed / vertical_speed so steep legs (the keep's 36 m descents) do not outrun the vehicle
        L = float(np.linalg.norm(d))
        if L > 1e-6 and abs(d[2]) > 1e-3 * L:
            vmax = min(vmax, (self.vz if d[2] > 0 else self.vz_up) * L / abs(d[2]))
        if self.leg_i == 0:
            # lift-off: PX4 ramps its own takeoff (MPC_TKO_RAMP_T), so a gentle first climb keeps the lag small
            vmax, acc = min(vmax, 0.8), min(acc, 0.5)
        self.leg = Leg(a, b, vmax, acc, yaw)
        self.leg_t0 = None
        self.phase = "yaw"
        self.leg_name, self.leg_wp, self.leg_world = name, is_wp, wb
        self.get_logger().info("leg %d -> %s (%.1f, %.1f, %.1f) len %.1f m, v %.1f m/s, %.0f s" % (
            self.leg_i, name, wb[0], wb[1], wb[2], self.leg.L, vmax, self.leg.T))

    def fly(self, t):
        leg = self.leg
        if self.phase == "yaw":
            err = P.wrap(leg.yaw - self.sp_yaw)
            step = self.yaw_rate * 0.02
            if abs(err) <= step:
                self.sp_yaw = leg.yaw
                if abs(P.wrap(self.lpos.heading - leg.yaw)) < math.radians(10) or t - self.t_state > 8:
                    self.phase, self.leg_t0, self.leg_tau, self.t_prev = "move", t, 0.0, t
            else:
                self.sp_yaw = P.wrap(self.sp_yaw + math.copysign(step, err))
            return
        if self.phase == "move":
            # reference governor: the leg's clock slows down (and stops) while the vehicle lags the setpoint by
            # more than lag_lo, e.g. during PX4's spool-up/takeoff ramp or a gust, so the setpoint never runs away
            dt, self.t_prev = max(0.0, t - self.t_prev), t
            lag = float(np.linalg.norm(self.pos() - self.sp_pos))
            # (tighter on the lift-off leg: PX4 only needs > 0.2 m of lead to start its takeoff ramp)
            lo, band = (0.15, 0.25) if self.leg_i == 0 else (0.3 + 0.08 * leg.vpk, 0.6)
            k = min(1.0, max(0.0, (lo + band - lag) / band))
            self.leg_tau += dt * k
            p, v, acc = leg.at(self.leg_tau)
            self.sp_pos, self.sp_vel, self.sp_acc = p, v * k, acc * k
            if self.leg_tau >= leg.T:
                self.sp_pos, self.sp_vel, self.sp_acc = leg.b.copy(), np.zeros(3), np.zeros(3)
                self.phase, self.settle_t0, self.best = "settle", t, (1e9, None, None)
            return
        if self.phase == "settle":
            e = float(np.linalg.norm(self.pos() - leg.b))
            if e < self.best[0]:
                self.best = (e, t)
            if self.leg_name == "hover":
                # the hold (and its statistics) starts once the vehicle has stayed within settle_radius for 3 s
                if not hasattr(self, "hover_t0"):
                    if e >= self.settle_r:
                        self.hover_in = None
                    elif getattr(self, "hover_in", None) is None:
                        self.hover_in = t
                    elif t - self.hover_in >= 3.0:
                        self.hover_t0 = t
                        self.get_logger().info("hover hold started")
                if hasattr(self, "hover_t0"):
                    self.hover_log.append((t, *self.world_of(self.pos())))
                    if t - self.hover_t0 >= self.hover_time:
                        self.record(t, e)
                        self.next_leg(t)
                elif t - self.settle_t0 > self.settle_t:
                    self.fail("hover point not reached (%.2f m)" % e)
                    self.next_leg(t)
                return
            if e <= self.settle_r or (t - self.settle_t0 > 3.0 and e <= self.acc_r) or (not self.leg_wp and e <= 1.0):
                self.record(t, e)
                self.next_leg(t)
            elif t - self.settle_t0 > self.settle_t:
                be, bt = self.best
                if self.leg_wp and be > self.acc_r:
                    self.fail("waypoint %s not reached: best %.2f m" % (self.leg_name, be))
                self.record(bt, be)
                self.next_leg(t)

    def record(self, t, e):
        """Log a reached route point; gt_err is provisional here (newest pose) and recomputed in finish()."""
        gt_e = None if self.gt is None else float(np.linalg.norm(self.gt - np.array(self.leg_world)))
        gt_e = None if gt_e is None else round(gt_e, 3)
        r = {"name": self.leg_name, "waypoint": bool(self.leg_wp), "target": [round(v, 3) for v in self.leg_world],
             "est_err": round(e, 3), "gt_err": gt_e, "gt_err_raw": gt_e, "t": round(t - self.t_takeoff, 2), "_t": t}
        self.results.append(r)
        self.get_logger().info("reached %s: est err %.2f m, ground-truth err %s m (t+%.1f s)" % (
            self.leg_name, e, "n/a" if gt_e is None else "%.2f" % gt_e, t - self.t_takeoff))

    def _log(self, t):
        if int(t * 10) == getattr(self, "_last_log", -1):
            return
        self._last_log = int(t * 10)
        est = self.world_of(self.pos())
        sp = self.world_of(self.sp_pos) if self.sp_pos is not None else [NAN] * 3
        self.clock_diag.append((t, self.gt_hist[-1][0] if self.gt_hist else NAN))
        # ground-truth columns are filled in finish() from the pose history (time-aligned)
        self.log_rows.append([round(t, 2), self.state, *np.round(est, 3), t, NAN, NAN, *np.round(sp, 3), est])

    def finish(self):
        if self.done:
            return
        self.done = True
        # ground truth at the estimates' own timestamps, now that the pose history covers them
        H = np.asarray(self.gt_hist) if len(self.gt_hist) > 10 else None
        tau = self.gt_offset(H)
        for r in self.results:
            t_rec = r.pop("_t")
            if H is not None:
                r["gt_err"] = round(float(np.linalg.norm(self.gt_at(t_rec + tau, H) - np.array(r["target"]))), 3)
        if self.log_rows:
            G = self.gt_at(np.array([row[5] for row in self.log_rows]) + tau, H) if H is not None else None
            for i, row in enumerate(self.log_rows):
                row[5:8] = [NAN] * 3 if G is None else [round(float(v), 3) for v in G[i]]
                del row[11:]
        landed = bool(self.land is not None and self.land.landed)
        disarmed = bool(self.status is not None and self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED)
        wps = [r for r in self.results if r["waypoint"]]
        n_expected = 1 if self.mode == "hover" else len(self.m["waypoints"])
        if len(wps) < n_expected:
            self.fail("only %d/%d waypoints visited" % (len(wps), n_expected))
        if not (landed and disarmed):
            self.fail("not landed+disarmed at the end (landed=%s disarmed=%s)" % (landed, disarmed))
        sim_dt = self.now() - (self.sim0 or 0)
        wall_dt = time.time() - self.wall0
        res = {"world": self.world, "mode": self.mode, "success": self.ok, "reason": self.reason,
               "waypoints": wps, "route": self.results, "landed": landed, "disarmed": disarmed,
               "max_est_err": max([r["est_err"] for r in wps], default=None),
               "max_gt_err": max([r["gt_err"] for r in wps if r["gt_err"] is not None], default=None),
               "max_gt_err_raw": max([r["gt_err_raw"] for r in wps if r["gt_err_raw"] is not None], default=None),
               "flight_time": None if self.t_takeoff is None else round(self.now() - self.t_takeoff, 1),
               "sim_time": round(sim_dt, 1), "wall_time": round(wall_dt, 1),
               "rtf_avg": round(sim_dt / wall_dt, 3) if wall_dt > 0 else None}
        if self.t_takeoff is not None and time.time() > self.wall_takeoff:
            res["rtf_flight"] = round((self.now() - self.t_takeoff) / (time.time() - self.wall_takeoff), 3)
        res["gt_time_offset_s"] = round(tau, 3)
        if self.clock_diag:
            c = np.array(self.clock_diag)
            res["gt_stamp_age_ms"] = round(float(np.nanmedian(c[:, 0] - c[:, 1])) * 1e3, 1)
        if self.hover_log:
            h = np.array(self.hover_log)
            h = np.c_[h, self.gt_at(h[:, 0] + tau, H) if H is not None else np.full((len(h), 3), NAN)]
            res["hover"] = {"samples": len(h), "duration": round(h[-1, 0] - h[0, 0], 1),
                            "est_std_xyz": [round(float(v), 4) for v in np.nanstd(h[:, 1:4], axis=0)],
                            "gt_std_xyz": [round(float(v), 4) for v in np.nanstd(h[:, 4:7], axis=0)],
                            "gt_mean_xyz": [round(float(v), 3) for v in np.nanmean(h[:, 4:7], axis=0)]}
        self.exit_code = 0 if self.ok else 1
        self.get_logger().info("RESULT " + json.dumps(res))
        if self.result_file:
            with open(self.result_file, "w") as fh:
                json.dump(res, fh, indent=1)
            with open(self.result_file.rsplit(".", 1)[0] + ".csv", "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t", "state", "est_x", "est_y", "est_z", "gt_x", "gt_y", "gt_z", "sp_x", "sp_y", "sp_z"])
                w.writerows(self.log_rows)
        self.goto_state("done")
        self.timer.cancel()


def main():
    rclpy.init()
    n = Mission()
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        n.fail("interrupted")
    code = n.exit_code
    n.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
