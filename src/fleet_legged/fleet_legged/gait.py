"""Periodic gait generator in the body frame (pure Python; used by the gait node and the checkers).

Every leg has a phase offset; the cycle phase phi advances at 1/period while walking. With
psi = (phi + offset) mod 1 a leg is in stance for psi < duty and in swing otherwise.

  stance  the foot moves opposite to the commanded body twist: p <- Rz(-wz dt) p - v dt (z fixed)
  swing   lift-off point -> touchdown target, horizontal cubic Hermite whose end velocities are the
          stance velocity (the foot is at rest in the world when it lifts and lands); height sin^2
          bump of `step_height` over a blend from the lift-off height to the floor under the target.
          The floor comes from the body tilt (set_tilt, IMU roll/pitch) and the stance feet, so a foot
          lands where the floor really is when the body has rolled about the support diagonal; in
          stance the foot height then relaxes to the level-body value (time constant level_tau),
          which levels the body without kicking it.
          Target = nominal + half the stance travel: n + (v + wz x n) * duty * period / 2,
          so the foot passes its nominal spot at mid stance. It is re-evaluated every tick.

The generator only walks while there is a command, and keeps stepping (to nominal) for one more
cycle after the command goes to zero, then freezes with every foot at its nominal spot.

Start: the cycle starts with the first leg at mid stance (phi = duty / 2 - offsets[0]), where a foot
standing on its nominal spot belongs. Legs that start inside their swing phase lift off from where
they are and squeeze the rest of the swing into the time left (sw0 = swing fraction at lift-off), so
no foot jumps and no stance foot is dragged further back than in the steady gait.
"""
import math


class Gait:
    def __init__(self, robot):
        self.robot = robot
        g = robot.gait
        self.period = float(g['period'])
        self.duty = float(g['duty'])
        self.step_h = float(g['step_height'])
        self.offsets = [float(o) for o in g['offsets']]
        self.max_reach = float(g.get('max_reach', 0.06))
        if len(self.offsets) != len(robot.legs):
            raise ValueError('gait offsets must match the legs')
        self.height = robot.height
        self.nominal = [leg.nominal(self.height) for leg in robot.legs]
        self.feet = [list(n) for n in self.nominal]
        self.swing = [False] * len(robot.legs)
        self.lift = [list(n) for n in self.nominal]
        self.sw0 = [0.0] * len(robot.legs)
        self.phi = 0.0
        self.walking = False
        self.idle_time = 0.0
        self.tilt = (0.0, 0.0)              # measured body roll, pitch (set_tilt)
        self.level_tau = float(g.get('level_tau', 0.08))

    def set_tilt(self, roll, pitch):
        """Body attitude from the IMU: swing feet then land on the real floor, not the level-body one."""
        self.tilt = (roll, pitch)

    def _ground(self):
        """Floor plane in B from the tilt and the stance feet: returns z(x, y)."""
        r, p = self.tilt
        u = (-math.sin(p), math.sin(r) * math.cos(p), math.cos(r) * math.cos(p))     # up, in B
        st = [f for i, f in enumerate(self.feet) if not self.swing[i]] or self.nominal
        h = -sum(u[0] * f[0] + u[1] * f[1] + u[2] * f[2] for f in st) / len(st)
        return lambda x, y: (-h - u[0] * x - u[1] * y) / u[2]

    def set_height(self, h):
        dz = -(h - self.height)
        self.height = h
        self.nominal = [leg.nominal(h) for leg in self.robot.legs]
        for f in self.feet:
            f[2] += dz
        for f in self.lift:
            f[2] += dz

    def stance(self, i):
        return ((self.phi + self.offsets[i]) % 1.0) < self.duty

    def _target(self, n, v):
        vx, vy, wz = v
        t = self.duty * self.period / 2
        tx, ty = (vx - wz * n[1]) * t, (vy + wz * n[0]) * t
        r = math.hypot(tx, ty)
        if r > self.max_reach:
            tx, ty = tx * self.max_reach / r, ty * self.max_reach / r
        return n[0] + tx, n[1] + ty

    def update(self, dt, v):
        """Advance by dt with body twist v = (vx, vy, wz). Returns the foot positions (B frame)."""
        moving = abs(v[0]) > 1e-3 or abs(v[1]) > 1e-3 or abs(v[2]) > 1e-3
        if moving:
            if not self.walking:
                self.phi = (self.duty / 2 - self.offsets[0]) % 1.0
                self.swing = [False] * len(self.feet)
            self.walking = True
            self.idle_time = 0.0
        elif self.walking:
            self.idle_time += dt
            settled = all(math.dist(f, n) < 0.004 for f, n in zip(self.feet, self.nominal))
            if self.idle_time > self.period and settled and all(self.stance(i) for i in range(len(self.feet))):
                self.walking = False
        if not self.walking:
            return self.feet
        self.phi = (self.phi + dt / self.period) % 1.0
        vx, vy, wz = v
        c, s = math.cos(-wz * dt), math.sin(-wz * dt)
        ground = self._ground()
        lev = min(1.0, dt / self.level_tau) if self.level_tau > 0 else 1.0
        for i, (f, n) in enumerate(zip(self.feet, self.nominal)):
            psi = (self.phi + self.offsets[i]) % 1.0
            if psi < self.duty:                      # stance
                if self.swing[i]:
                    self.swing[i] = False
                x, y = f[0], f[1]
                f[0] = c * x - s * y - vx * dt
                f[1] = s * x + c * y - vy * dt
                f[2] += (n[2] - f[2]) * lev           # back to the level-body height: levels the body
            else:                                    # swing
                sw_raw = (psi - self.duty) / (1.0 - self.duty)
                if not self.swing[i]:
                    self.swing[i] = True
                    self.lift[i] = list(f)
                    self.sw0[i] = sw_raw if sw_raw < 0.9 else 0.0
                    if self.sw0[i] < 0.05:
                        self.sw0[i] = 0.0
                sw = (sw_raw - self.sw0[i]) / (1.0 - self.sw0[i])
                sw = 0.0 if sw < 0 else 1.0 if sw > 1 else sw
                tx, ty = self._target(n, v)
                p0 = self.lift[i]
                # cubic Hermite with the stance velocity at both ends (ground speed matching): the foot
                # leaves and meets the floor at rest in the world, so nothing is dragged or kicked
                T = (1.0 - self.duty) * self.period * (1.0 - self.sw0[i])
                u0x, u0y = -(vx - wz * p0[1]), -(vy + wz * p0[0])
                u1x, u1y = -(vx - wz * ty), -(vy + wz * tx)
                s2, s3 = sw * sw, sw * sw * sw
                h00, h10, h01, h11 = 2 * s3 - 3 * s2 + 1, s3 - 2 * s2 + sw, 3 * s2 - 2 * s3, s3 - s2
                f[0] = h00 * p0[0] + h10 * T * u0x + h01 * tx + h11 * T * u1x
                f[1] = h00 * p0[1] + h10 * T * u0y + h01 * ty + h11 * T * u1y
                zs = sw * sw * (3 - 2 * sw)
                f[2] = p0[2] + (ground(tx, ty) - p0[2]) * zs + self.step_h * math.sin(math.pi * sw) ** 2
        return self.feet
