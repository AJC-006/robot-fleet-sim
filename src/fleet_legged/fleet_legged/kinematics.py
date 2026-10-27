"""Leg kinematics for the legged robots (wyrm, hound, spider) (pure Python, no ROS, no numpy).

Shared by the gait node, the lidar-plane checker and the Blender builders, so the URDF, the
meshes and the controller all agree on one convention.

Body frame B = base_link (x forward, y left, z up, origin at the body centre, standing_height above
the floor). Every leg has three revolute joints and the same planar femur/tibia chain:

  yaw leg  (sprawling: wyrm, spider)   coxa axis z, joint frame at `hip`, rotated by `mount_yaw_deg`;
                                       the femur joint sits `coxa` metres out along the coxa x axis.
  roll leg (mammal: hound)             coxa axis x (hip ab/ad), joint frame at `hip`, not rotated;
                                       the femur joint sits `coxa` metres out along +-y (side of the hip).

  femur joint  axis y, q = 0: femur along the parent x axis, q > 0 swings the femur tip down
  tibia joint  axis y, q = 0: tibia straight on from the femur, q > 0 bends it further down
  foot         sphere centre at x = tibia length on the tibia link

In the leg plane (a = outward/forward, b = downward) the foot is at
  a = L1 cos q1 + L2 cos(q1 + q2),  b = L1 sin q1 + L2 sin(q1 + q2).
`knee: +1` picks the knee-up branch (q2 > 0, spider/lizard elbows), `knee: -1` the knee-back branch
(q2 < 0, dog). Joint names: <leg>_coxa_joint, <leg>_femur_joint, <leg>_tibia_joint.
"""
import math

JOINTS = ('coxa', 'femur', 'tibia')


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ----------------------------------------------------------------------------- 4x4 helpers (row-major lists)
def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def mat_trans(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def mat_rot(axis, q):
    c, s = math.cos(q), math.sin(q)
    if axis == 'x':
        return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]
    if axis == 'y':
        return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def mat_apply(m, p):
    return tuple(m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3] for i in range(3))


# ----------------------------------------------------------------------------- leg
class Leg:
    def __init__(self, spec, robot):
        link = dict(robot['link'])
        link.update(spec.get('link', {}))
        self.name = spec['name']
        self.type = robot['leg_type']
        self.hip = tuple(float(v) for v in spec['hip'])
        self.side = 1.0 if self.hip[1] >= 0 else -1.0
        self.yaw = math.radians(float(spec.get('mount_yaw_deg', 0.0))) if self.type == 'yaw' else 0.0
        self.lc, self.l1, self.l2 = float(link['coxa']), float(link['femur']), float(link['tibia'])
        self.rf = float(link['foot_radius'])
        self.knee = float(robot.get('knee', 1))
        lim = robot['limits']
        c = [float(v) for v in lim['coxa']]
        if self.type == 'roll' and self.side < 0:     # mirrored ab/ad range on the right
            c = [-c[1], -c[0]]
        self.limits = [tuple(c), tuple(float(v) for v in lim['femur']), tuple(float(v) for v in lim['tibia'])]
        self.foot_xy = tuple(float(v) for v in spec['foot'])
        self.joint_names = [f'{self.name}_{j}_joint' for j in JOINTS]
        self.link_names = [f'{self.name}_{j}_link' for j in JOINTS]

    def nominal(self, height):
        """Foot centre in B for a body `height` above the floor."""
        return (self.foot_xy[0], self.foot_xy[1], -(height - self.rf))

    # planar 2-link solve: a outward, b down
    def _planar(self, a, b):
        l1, l2 = self.l1, self.l2
        d2 = a * a + b * b
        dmax, dmin = (l1 + l2) * 0.999, abs(l1 - l2) * 1.001 + 1e-6
        d = math.sqrt(d2)
        ok = dmin <= d <= dmax
        if not ok:      # project onto the reachable annulus, keep the direction
            k = _clamp(d, dmin, dmax) / max(d, 1e-9)
            a, b, d2 = a * k, b * k, (d * k) ** 2
        cq2 = _clamp((d2 - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1.0, 1.0)
        q2 = self.knee * math.acos(cq2)
        q1 = math.atan2(b, a) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
        return q1, q2, ok

    def ik(self, p):
        """Foot centre p (in B) -> ([q_coxa, q_femur, q_tibia], reachable). Clamped to the joint limits."""
        dx, dy, dz = p[0] - self.hip[0], p[1] - self.hip[1], p[2] - self.hip[2]
        if self.type == 'yaw':
            q0 = _wrap(math.atan2(dy, dx) - self.yaw)
            a = math.hypot(dx, dy) - self.lc
            q1, q2, ok = self._planar(a, -dz)
        else:
            rho2 = dy * dy + dz * dz - self.lc * self.lc
            ok0 = rho2 > 1e-8
            L = math.sqrt(max(rho2, 1e-8))
            q0 = _wrap(math.atan2(dz, dy) - math.atan2(-L, self.side * self.lc))
            q1, q2, ok = self._planar(dx, L)
            ok = ok and ok0
        q = [q0, q1, q2]
        for i, (lo, hi) in enumerate(self.limits):
            if not lo - 1e-6 <= q[i] <= hi + 1e-6:
                ok = False
            q[i] = _clamp(q[i], lo, hi)
        return q, ok

    # forward kinematics -------------------------------------------------------------------------
    def frames(self, q):
        """Link frames in B: [coxa_link, femur_link, tibia_link, foot] as 4x4 matrices."""
        if self.type == 'yaw':
            c = mat_mul(mat_mul(mat_trans(*self.hip), mat_rot('z', self.yaw)), mat_rot('z', q[0]))
            f = mat_mul(mat_mul(c, mat_trans(self.lc, 0, 0)), mat_rot('y', q[1]))
        else:
            c = mat_mul(mat_trans(*self.hip), mat_rot('x', q[0]))
            f = mat_mul(mat_mul(c, mat_trans(0, self.side * self.lc, 0)), mat_rot('y', q[1]))
        t = mat_mul(mat_mul(f, mat_trans(self.l1, 0, 0)), mat_rot('y', q[2]))
        foot = mat_mul(t, mat_trans(self.l2, 0, 0))
        return [c, f, t, foot]

    def fk(self, q):
        """Foot centre in B (fast path, no matrices)."""
        q0, q1, q2 = q
        a = self.l1 * math.cos(q1) + self.l2 * math.cos(q1 + q2)
        b = self.l1 * math.sin(q1) + self.l2 * math.sin(q1 + q2)
        if self.type == 'yaw':
            r = self.lc + a
            ang = self.yaw + q0
            return (self.hip[0] + r * math.cos(ang), self.hip[1] + r * math.sin(ang), self.hip[2] - b)
        y, z = self.side * self.lc, -b
        c, s = math.cos(q0), math.sin(q0)
        return (self.hip[0] + a, self.hip[1] + y * c - z * s, self.hip[2] + y * s + z * c)

    def joint_origins(self):
        """URDF joint origins (xyz, rpy) and axes for coxa, femur, tibia joints, and the foot offset."""
        if self.type == 'yaw':
            coxa = (self.hip, (0.0, 0.0, self.yaw), (0, 0, 1))
            femur = ((self.lc, 0.0, 0.0), (0.0, 0.0, 0.0), (0, 1, 0))
        else:
            coxa = (self.hip, (0.0, 0.0, 0.0), (1, 0, 0))
            femur = ((0.0, self.side * self.lc, 0.0), (0.0, 0.0, 0.0), (0, 1, 0))
        tibia = ((self.l1, 0.0, 0.0), (0.0, 0.0, 0.0), (0, 1, 0))
        return coxa, femur, tibia, (self.l2, 0.0, 0.0)


class Robot:
    """Kinematic model of one legged entry of robots.yaml."""

    def __init__(self, name, spec):
        if spec.get('kind') != 'legged':
            raise ValueError(f'{name} is not a legged robot')
        self.name = name
        self.spec = spec
        self.legs = [Leg(l, spec) for l in spec['legs']]
        self.height = float(spec['standing_height'])
        self.crouch = float(spec['crouch_height'])
        g = spec['gait']
        self.gait = dict(g)
        self.joint_names = [n for leg in self.legs for n in leg.joint_names]

    def stance_q(self, height=None):
        h = self.height if height is None else height
        out = []
        for leg in self.legs:
            q, _ = leg.ik(leg.nominal(h))
            out.append(q)
        return out


def load(name, path=None, cfg=None):
    if cfg is None:
        import yaml
        with open(path) as f:
            cfg = yaml.safe_load(f)
    return Robot(name, cfg[name])
