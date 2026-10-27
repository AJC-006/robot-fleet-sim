#!/usr/bin/env python3
"""Derive Gazebo MulticopterMotorModel constants from a published static thrust table.

usage: fit_motor_prop.py <table.csv> [--mass-prop kg --diameter m --kv rpm_per_volt --resistance ohm]

* motorConstant  kT [N s^2/rad^2]: thrust = kT w^2, least squares over the table (w = rpm 2pi/60).
* momentConstant kQ/kT [m]: torque = kQ w^2 (the table lists torque with 0.01 N m resolution, so kQ is a
  weighted least squares dominated by the high-torque rows), reaction torque = momentConstant x thrust.
  Cross-check with blade-element non-dimensional coefficients CT = T/(rho n^2 D^4), CP = 2 pi Q/(rho n^2 D^5):
  kQ/kT = CP D / (2 pi CT).
* maxRotVelocity: w at the 100 % row.
* Motor time constant (linearisation of  J dw/dt = Kt (V d - Ke w)/R - kQ w^2 ):
      tau(w0) = J / (Kt Ke / R + 2 kQ w0),   Ke = Kt = 60 / (2 pi KV)
  J = propeller (rod bound m D^2 / 12) + motor bell (unpublished, ignored: it is < 5 % of the propeller).
  Spin-down uses the same expression only if the ESC brakes actively (BLHeli_S damped light); a coasting
  propeller has tau = J / (2 kQ w0).
"""
import argparse
import csv
import math

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('table')
    ap.add_argument('--mass-prop', type=float, default=0.013)    # kg, T1045 datasheet
    ap.add_argument('--diameter', type=float, default=0.254)     # m
    ap.add_argument('--kv', type=float, default=920.0)           # rpm/V, AIR2216II datasheet
    ap.add_argument('--resistance', type=float, default=0.115)   # ohm, AIR2216II datasheet
    ap.add_argument('--hover-throttle', type=float, default=0.45)
    a = ap.parse_args()
    rows = [r for r in csv.reader(l for l in open(a.table) if not l.startswith('#'))][1:]
    d = np.array([[float(x) for x in r] for r in rows])
    thr, T, Q, I, rpm = d[:, 0] / 100, d[:, 1] * 9.80665e-3, d[:, 2], d[:, 3], d[:, 4]
    w = rpm * 2 * math.pi / 60
    kT = np.sum(T * w ** 2) / np.sum(w ** 4)
    kQ = np.sum(Q * w ** 2) / np.sum(w ** 4)
    rho, n = 1.225, w / (2 * math.pi)
    CT = T / (rho * n ** 2 * a.diameter ** 4)
    CP = 2 * math.pi * Q / (rho * n ** 2 * a.diameter ** 5)
    ratio_bem = float(np.mean(CP[3:]) * a.diameter / (2 * math.pi * np.mean(CT[3:])))
    print(f'motorConstant kT      = {kT:.4e} N s^2/rad^2  (rms residual {np.sqrt(np.mean((kT*w**2-T)**2)):.3f} N, '
          f'max {np.max(np.abs(kT*w**2-T)/T)*100:.1f} %)')
    print(f'torque constant kQ    = {kQ:.4e} N m s^2/rad^2')
    print(f'momentConstant kQ/kT  = {kQ/kT:.5f} m   (from CP/CT: {ratio_bem:.5f} m)')
    print(f'maxRotVelocity        = {w[-1]:.1f} rad/s ({rpm[-1]:.0f} rpm), max thrust {kT*w[-1]**2:.2f} N')
    print(f'CT {CT.min():.4f}..{CT.max():.4f}  CP {CP.min():.4f}..{CP.max():.4f}')
    Ke = 60 / (2 * math.pi * a.kv)
    J = a.mass_prop * a.diameter ** 2 / 12
    for name, thr0 in (('hover', a.hover_throttle),):
        w0 = np.interp(thr0, thr, w)
        tau = J / (Ke * Ke / a.resistance + 2 * kQ * w0)
        tau_coast = J / (2 * kQ * w0)
        print(f'J_prop (rod bound)    = {J:.3e} kg m^2; at {name} w0={w0:.0f} rad/s: tau_driven {tau*1e3:.1f} ms, '
              f'tau_coast {tau_coast*1e3:.0f} ms')
    # electrical sanity: efficiency of the motor+ESC at the table points
    print('shaft/electrical power ' + ' '.join(f'{x:.2f}' for x in (Q * w / (16.0 * I))[::3]))


if __name__ == '__main__':
    main()
