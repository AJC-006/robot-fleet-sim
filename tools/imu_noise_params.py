#!/usr/bin/env python3
"""Derive Gazebo IMU noise parameters from datasheet Allan-variance figures.

Model per axis: y = white(sigma_w) + b(t), b = first-order Gauss-Markov bias (correlation time tau).
* sigma_w = N sqrt(f_s)           (N: noise density, white-noise Allan deviation at tau = 1 s equals N)
* gz-sensors GaussianNoiseModel::ApplyImpl (src/GaussianNoiseModel.cc) updates
      b <- exp(-dt/tau) b + N(0, sqrt(-s^2 tau/2 expm1(-2 dt/tau)))
  so its stationary stddev is s*sqrt(tau/2); `dynamic_bias_stddev` s is Kalibr's bias density.
* Allan deviation of white + GM: falls as N/sqrt(T), rises ~ sqrt(T) (bias random walk) up to T ~ tau,
  peaks near 1.9 tau. The local minimum below tau is the datasheet "bias instability" floor; bisection
  on s makes that minimum equal BI. tau (default 1000 s) is an assumption: datasheets do not publish
  the correlation time, only the floor value.
Usage: imu_noise_params.py <noise_density> <bias_instability> <rate_hz> [tau_s]   (SI units).
"""
import sys
import numpy as np


def var_avg(T, V, c):
    """Variance of the T-average of a stationary exponential-correlation process."""
    x = T / c
    return 2.0 * V * (x - 1.0 + np.exp(-x)) / x ** 2


def adev_curve(sigma_w, V, c, fs, taus):
    avar_gm = 2 * var_avg(taus, V, c) - 2 * var_avg(2 * taus, V, c)
    avar_w = sigma_w ** 2 / fs / taus          # white noise: N^2 / tau
    return np.sqrt(avar_gm + avar_w)


def solve(N, BI, fs, tau):
    taus = np.logspace(-1, np.log10(tau), 2000)   # local minimum lies below the GM peak at ~1.9 tau
    sigma_w = N * np.sqrt(fs)
    lo, hi = 0.0, 100 * BI
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        curve = adev_curve(sigma_w, mid ** 2, tau, fs, taus)
        if curve.min() > BI:
            hi = mid
        else:
            lo = mid
    sd = 0.5 * (lo + hi)                      # stationary stddev of the bias
    s = sd / np.sqrt(tau / 2.0)               # gz dynamic_bias_stddev
    curve = adev_curve(sigma_w, sd ** 2, tau, fs, taus)
    return sigma_w, sd, s, curve.min(), taus[curve.argmin()]


if __name__ == '__main__':
    N, BI, fs = (float(v) for v in sys.argv[1:4])
    tau = float(sys.argv[4]) if len(sys.argv) > 4 else 1000.0
    sw, sd, s, m, t = solve(N, BI, fs, tau)
    print(f'white stddev {sw:.4e}  bias stationary stddev {sd:.4e}  gz dynamic_bias_stddev {s:.4e} '
          f'(tau {tau:g} s)  ADEV min {m:.4e} at {t:.1f} s (target {BI:.4e})')
