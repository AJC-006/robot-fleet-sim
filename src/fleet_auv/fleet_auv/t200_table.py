"""Blue Robotics T200 vendor bollard table (catalog CSV): force(pwm, bus voltage) and its inverse.

Used by the controller to turn a wanted thrust into an ESC pulse width, the same table the simulated thruster plugin
(fleet_marine_plugins T200Thruster) uses to turn the pulse width back into thrust.
"""
import bisect
import csv


class T200Table:
    def __init__(self, path):
        acc = {}
        with open(path) as f:
            rows = csv.DictReader(r for r in f if r.strip() and not r.startswith("#"))
            for r in rows:
                k = (float(r["bus_voltage_v"]), int(round(float(r["pwm_us"]))))
                s = acc.setdefault(k, [0.0, 0])
                s[0] += float(r["force_n"])
                s[1] += 1
        self.volts = sorted({k[0] for k in acc})
        self.pwm = {v: sorted(p for (vv, p) in acc if vv == v) for v in self.volts}
        self.f = {v: [acc[(v, p)][0] / acc[(v, p)][1] for p in self.pwm[v]] for v in self.volts}

    @staticmethod
    def _interp(xs, ys, x):
        if x <= xs[0]:
            return ys[0]
        if x >= xs[-1]:
            return ys[-1]
        i = bisect.bisect_left(xs, x)
        w = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
        return ys[i - 1] + w * (ys[i] - ys[i - 1])

    def _bracket(self, volt):
        volt = min(max(volt, self.volts[0]), self.volts[-1])
        i = max(1, bisect.bisect_left(self.volts, volt))
        v0, v1 = self.volts[i - 1], self.volts[i]
        return v0, v1, (volt - v0) / (v1 - v0)

    def force(self, pwm_us, volt):
        v0, v1, w = self._bracket(volt)
        f0 = self._interp(self.pwm[v0], self.f[v0], pwm_us)
        f1 = self._interp(self.pwm[v1], self.f[v1], pwm_us)
        return f0 + w * (f1 - f0)

    def limits(self, volt):
        """(max reverse force (negative), max forward force) at this bus voltage, N."""
        v0, v1, w = self._bracket(volt)
        lo = min(self.f[v0]) + w * (min(self.f[v1]) - min(self.f[v0]))
        hi = max(self.f[v0]) + w * (max(self.f[v1]) - max(self.f[v0]))
        return lo, hi

    def pwm_for_force(self, force_n, volt, stop=1500.0):
        """Pulse width (us) giving `force_n` at `volt`: smallest pulse beyond the vendor dead band that reaches it."""
        if abs(force_n) < 1e-6:
            return stop
        lo, hi = self.limits(volt)
        v0, v1, w = self._bracket(volt)
        grid = sorted(set(self.pwm[v0]) | set(self.pwm[v1]))
        fs = [self.force(p, volt) for p in grid]
        if force_n > 0:
            force_n = min(force_n, hi)
            cand = [(p, f) for p, f in zip(grid, fs) if p > stop]
        else:
            force_n = max(force_n, lo)
            cand = [(p, f) for p, f in zip(grid, fs) if p < stop][::-1]
        prev = None
        for p, f in cand:
            if f == 0.0:
                prev = (p, f)
                continue
            if (force_n > 0 and f >= force_n) or (force_n < 0 and f <= force_n):
                if prev is None or prev[1] == 0.0:
                    return float(p)
                return prev[0] + (force_n - prev[1]) / (f - prev[1]) * (p - prev[0])
            prev = (p, f)
        return float(cand[-1][0])
