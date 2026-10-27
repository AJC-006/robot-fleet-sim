#!/usr/bin/env python3
"""Gait node for the legged robots (wyrm, hound, spider).

  ros2 run fleet_legged gait_node.py --ros-args -p robot:=hound -p use_sim_time:=true

  sub  /cmd_vel                    geometry_msgs/TwistStamped  body twist (vx, vy, wz), clamped to the
                                                               robot's max_vel_* and acceleration-limited
       /joint_states               sensor_msgs/JointState      measured joints, for the leg odometry
       /imu/data                   sensor_msgs/Imu             roll/pitch: swing feet land on the real floor
  pub  /leg_controller/commands    std_msgs/Float64MultiArray  joint positions, robot.joint_names order
       /leg_odom                   nav_msgs/Odometry           leg odometry (odom -> base_footprint), twist
                                                               from the stance feet; no TF (the EKF owns it)

Sequence: wait for /joint_states, then stand up (feet from where they are to their nominal spots
while the body rises from crouch_height to standing_height over `stand_time`), then walk whenever
/cmd_vel is non-zero (gait.py: trot / creep / wave / tetrapod, set by robots.yaml offsets).

Leg odometry: for every foot in stance (gait schedule, minus the first and last `odom_trim` of the
stance phase) the forward kinematics of the measured joints gives the foot position p_i in base_link
and its rate dp_i/dt. A foot fixed on the ground moves as dp/dt = -v - w x p, so (vx, vy, wz) is the
least-squares fit over the stance feet, low-pass filtered. While standing still every foot counts.
The feet are spheres that partly roll and slip as the leg pivots over them, so a "planted" foot centre
still creeps forward a little and the raw fit reads short (hound 7 %, wyrm 3-4 %, spider 1-2 %, from
walk_test.py against Gazebo ground truth). robots.yaml `gait.odom_scale: [sx, sy]` calibrates that
(the EKF takes yaw rate from the IMU, so wz is not scaled).
"""
import math
import os

import rclpy
import rclpy.signals
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from fleet_legged.gait import Gait
from fleet_legged.kinematics import load


def _smooth(s):
    s = 0.0 if s < 0 else 1.0 if s > 1 else s
    return s * s * (3 - 2 * s)


def _solve3(a, b):
    """Solve the 3x3 system a x = b (Cramer); None if singular."""
    det = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
           + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    if abs(det) < 1e-12:
        return None
    out = []
    for c in range(3):
        m = [row[:] for row in a]
        for r in range(3):
            m[r][c] = b[r]
        out.append((m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                    + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / det)
    return out


class GaitNode(Node):
    def __init__(self):
        super().__init__('gait_node')
        self.declare_parameter('robot', 'hound')
        self.declare_parameter('robots_yaml', '')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('stand_time', 2.0)
        self.declare_parameter('accel_lin', 0.5)       # m/s^2
        self.declare_parameter('accel_ang', 1.5)       # rad/s^2
        self.declare_parameter('odom_topic', '/leg_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_alpha', 0.25)     # low-pass factor per joint_states sample
        self.declare_parameter('odom_trim', 0.12)      # fraction of the stance phase skipped at each end
        # gz_ros2_control's position servo lags by 1 / (position_proportional_gain * update_rate); sending
        # q + lead * dq/dt cancels that lag (velocity feed-forward through the position interface)
        self.declare_parameter('servo_lead', 0.0)      # s
        self.declare_parameter('imu_topic', '/imu/data')     # roll/pitch for the swing touchdown height
        p = lambda n: self.get_parameter(n).value
        path = p('robots_yaml')
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('fleet_description'), 'config', 'robots.yaml')
        name = p('robot')
        self.robot = load(name, path)
        spec = self.robot.spec
        self.vmax = (float(spec['max_vel_x']), float(spec.get('max_vel_y', 0.0)), float(spec['max_vel_theta']))
        self.gait = Gait(self.robot)
        self.odom_scale = [float(v) for v in self.robot.gait.get('odom_scale', [1.0, 1.0])]
        self.dt_nom = 1.0 / float(p('rate'))
        self.cmd_timeout = float(p('cmd_timeout'))
        self.stand_time = float(p('stand_time'))
        self.acc = (float(p('accel_lin')), float(p('accel_lin')), float(p('accel_ang')))
        self.alpha = float(p('odom_alpha'))
        self.trim = float(p('odom_trim'))
        self.lead = float(p('servo_lead'))
        self.q_prev = None
        self.odom_frame, self.base_frame = p('odom_frame'), p('base_frame')

        self.cmd = (0.0, 0.0, 0.0)          # last /cmd_vel, clamped
        self.cmd_time = None
        self.vel = [0.0, 0.0, 0.0]          # ramped command fed to the gait
        self.state = 'wait'                 # wait -> stand -> run
        self.t_state = 0.0
        self.start_feet = None
        self.q_meas = None                  # dict joint -> position
        self.last_tick = None

        # leg odometry state
        self.prev_feet = None
        self.prev_stamp = None
        self.twist = [0.0, 0.0, 0.0]
        self.pose = [0.0, 0.0, 0.0]

        self.pub_cmd = self.create_publisher(Float64MultiArray, '/leg_controller/commands', 10)
        self.pub_odom = self.create_publisher(Odometry, p('odom_topic'), 20)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Imu, p('imu_topic'), self.on_imu, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self.timer = self.create_timer(self.dt_nom, self.tick)
        self.get_logger().info(f'{name}: {len(self.robot.legs)} legs, gait {self.robot.gait["type"]} '
                               f'period {self.gait.period}s duty {self.gait.duty}, standing {self.robot.height} m')

    # ------------------------------------------------------------------ inputs
    def on_cmd(self, msg):
        t = msg.twist
        lim = lambda v, m: max(-m, min(m, v))
        self.cmd = (lim(t.linear.x, self.vmax[0]), lim(t.linear.y, self.vmax[1]), lim(t.angular.z, self.vmax[2]))
        self.cmd_time = self.get_clock().now()

    def on_imu(self, msg):
        q = msg.orientation
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.gait.set_tilt(roll, pitch)

    def on_joints(self, msg):
        q = dict(zip(msg.name, msg.position))
        if any(n not in q for n in self.robot.joint_names):
            return
        self.q_meas = q
        self.leg_odometry(msg, q)

    # ------------------------------------------------------------------ control loop
    def tick(self):
        now = self.get_clock().now()
        if self.last_tick is None:
            self.last_tick = now
            return
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        if dt <= 0.0:
            return
        dt = min(dt, 0.05)
        if self.state == 'wait':
            if self.q_meas is None:
                return
            # start from wherever the feet are now (the spawn crouch)
            self.start_feet = [list(leg.fk([self.q_meas[n] for n in leg.joint_names])) for leg in self.robot.legs]
            self.gait.set_height(self.robot.crouch)
            self.state, self.t_state = 'stand', 0.0
            self.get_logger().info('standing up')
        if self.state == 'stand':
            self.t_state += dt
            s = _smooth(self.t_state / self.stand_time)
            h = self.robot.crouch + (self.robot.height - self.robot.crouch) * s
            k = _smooth(2.0 * self.t_state / self.stand_time)       # feet reach their spots in the first half
            feet = []
            for leg, f0 in zip(self.robot.legs, self.start_feet):
                n = leg.nominal(h)
                feet.append([f0[j] + (n[j] - f0[j]) * k for j in range(3)])
            if self.t_state >= self.stand_time:
                self.gait.set_height(self.robot.height)
                self.gait.feet = [list(leg.nominal(self.robot.height)) for leg in self.robot.legs]
                self.state = 'run'
                self.get_logger().info('standing; walking on /cmd_vel')
            self.send(feet, dt)
            return
        # run: ramp the command, time out stale ones
        target = self.cmd
        if self.cmd_time is None or (now - self.cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            target = (0.0, 0.0, 0.0)
        for i in range(3):
            d = target[i] - self.vel[i]
            step = self.acc[i] * dt
            self.vel[i] += max(-step, min(step, d))
            if abs(self.vel[i]) < 1e-4 and target[i] == 0.0:
                self.vel[i] = 0.0
        feet = self.gait.update(dt, tuple(self.vel))
        self.send(feet, dt)

    def send(self, feet, dt=None):
        out = []
        for leg, f in zip(self.robot.legs, feet):
            q, _ = leg.ik(f)
            out.extend(q)
        cmd = out
        if self.lead > 0.0 and dt and self.q_prev is not None:
            cmd = [q + self.lead * (q - q0) / dt for q, q0 in zip(out, self.q_prev)]
        self.q_prev = out
        self.pub_cmd.publish(Float64MultiArray(data=cmd))

    # ------------------------------------------------------------------ leg odometry
    def stance_weight(self, i):
        if self.state != 'run' or not self.gait.walking:
            return 1.0
        psi = (self.gait.phi + self.gait.offsets[i]) % 1.0
        duty = self.gait.duty
        if psi >= duty:
            return 0.0
        s = psi / duty
        return 1.0 if self.trim <= s <= 1.0 - self.trim else 0.0

    def leg_odometry(self, msg, q):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        feet = [leg.fk([q[n] for n in leg.joint_names]) for leg in self.robot.legs]
        if self.prev_feet is None or stamp <= self.prev_stamp:
            self.prev_feet, self.prev_stamp = feet, stamp
            return
        dt = stamp - self.prev_stamp
        if dt > 0.5:
            self.prev_feet, self.prev_stamp = feet, stamp
            return
        # normal equations of  [-1 0 py; 0 -1 -px] [vx vy wz]^T = dp/dt
        a = [[0.0] * 3 for _ in range(3)]
        b = [0.0] * 3
        nst = 0
        for i, (p, p0) in enumerate(zip(feet, self.prev_feet)):
            w = self.stance_weight(i) if self.state == 'run' else 0.0
            if w <= 0.0:
                continue
            nst += 1
            px, py = 0.5 * (p[0] + p0[0]), 0.5 * (p[1] + p0[1])
            vxf, vyf = (p[0] - p0[0]) / dt, (p[1] - p0[1]) / dt
            rows = (((-1.0, 0.0, py), vxf), ((0.0, -1.0, -px), vyf))
            for r, y in rows:
                for j in range(3):
                    b[j] += w * r[j] * y
                    for k in range(3):
                        a[j][k] += w * r[j] * r[k]
        self.prev_feet, self.prev_stamp = feet, stamp
        meas = None
        if nst >= 2:
            meas = _solve3(a, b)
            if meas is not None:
                meas[0] *= self.odom_scale[0]
                meas[1] *= self.odom_scale[1]
        if meas is None:
            meas = list(self.vel) if self.state == 'run' else [0.0, 0.0, 0.0]
        for j in range(3):
            self.twist[j] += self.alpha * (meas[j] - self.twist[j])
        vx, vy, wz = self.twist
        if self.state != 'run':
            vx = vy = wz = 0.0
        th = self.pose[2]
        self.pose[0] += (vx * math.cos(th) - vy * math.sin(th)) * dt
        self.pose[1] += (vx * math.sin(th) + vy * math.cos(th)) * dt
        self.pose[2] = math.atan2(math.sin(th + wz * dt), math.cos(th + wz * dt))
        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = self.pose[0], self.pose[1]
        od.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        od.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = vx, vy, wz
        pc = [0.0] * 36
        tc = [0.0] * 36
        for k, v in enumerate((0.01, 0.01, 1e6, 1e6, 1e6, 0.05)):
            pc[7 * k] = v
        for k, v in enumerate((0.004, 0.004, 1e6, 1e6, 1e6, 0.05)):
            tc[7 * k] = v
        od.pose.covariance, od.twist.covariance = pc, tc
        self.pub_odom.publish(od)


def main():
    # Own SIGINT handling: with rclpy's handler the context dies while a timer callback may still
    # publish ("publisher's context is invalid" on every launch shutdown).
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
