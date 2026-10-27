# air lane: aircraft realism, catalog and notes (package fleet_aircraft, fleet_drone, catalog categories fc_imu...)

## Status / next
Slice A1 (wyvern realism) done, keep-world result in the Results section.
- Sensors the wyvern gives PX4 now come from catalog entries, not x500 placeholders: ICM-42688-P IMU, MS5611 baro,
  IST8310 mag, MAX-M10S GNSS (src/fleet_components/components/{fc_imu,barometer,magnetometer,gnss}).
  make_sdf.py reads the catalog and writes models/wyvern/model.sdf (no hand-edited numbers).
- Motor/prop: T-Motor AIR2216II KV920 + T1045 16 V thrust table (propeller/*.csv) fitted by tools/fit_motor_prop.py:
  motorConstant 1.2295e-5, momentConstant 0.01404, maxRotVelocity 1032.2 rad/s, time constants 0.062 s up and down,
  rotor link 13 g with J = 6.989e-5 kg m^2. Launch sets SIM_GZ_EC_MAX1..4=1032. Max thrust 13.1 N per motor.
- New plugin fleet_aircraft::GnssError (gz-sim system, lib fleet_gnss_error): Gauss-Markov N/E/U position error
  in metres via WGS84 radii + white velocity noise. The truth navsat sensor is "navsat_truth"; the plugin publishes
  "navsat_sensor", the topic PX4's gz bridge reads. Unit test: test_gnss_core (stationary sigma, autocorrelation, radii).
- Launch arg gnss_errors:=auto|true|false (auto = on in keep only; the indoor worlds have no GNSS, errors off).
- PX4 tuning: default PX4 4001 gains kept, MPC_THR_HOVER left at the airframe's 0.6 (PX4 re-learns it, MPC_USE_HTE; see Results: hover/mission pass without
  retune). Only SIM_GZ_EC_MAX* is changed.
- Tests: python3 -m pytest src/fleet_drone/test/test_air_catalog.py (catalog vs sdf, noise formulas, kT refit).
- Decisions with numbers: white noise = density x sqrt(250 Hz); turn-on bias stddev = datasheet offset bound / 3;
  GNSS sigma_h = CEP / 1.1774 = 1.274 m, sigma_v = 1.5 sigma_h, tau 300 s; baro 2.7 Pa + 50 Pa offset, 1 Pa LSB;
  mag 0.3 uT noise (estimate) + 0.3 uT offset, 7.576e-8 T LSB.
- Files: blender/drone/make_sdf.py, src/fleet_aircraft/**, src/fleet_drone/launch/drone.launch.py,
  src/fleet_drone/test/test_air_catalog.py, tools/fit_motor_prop.py, catalog entries listed above plus
  bldc_motor/tmotor_air2216ii_kv920, propeller/tmotor_t1045, esc_air/tmotor_air_20a, bosch_bmi055.
- Open issues: ICM-42688-P bias instability, IST8310 noise, GNSS correlation time/vertical accuracy are unpublished
  (estimates flagged in the entries). Holybro 2216 KV920 has no thrust table (T-Motor stand-in, not verified the same
  motor). Base mass 2.0 kg and inertia, rotorDragCoefficient, rollingMomentCoefficient remain x500 values (unsourced).
  PX4 gz bridge: single IMU, fixed eph/epv/hdop, own 2 cm GPS noise; no GNSS latency model. Mag/baro/GNSS dynamic
  bias is impossible in gz (no dt passed). Vibration, motor interference, supply voltage sag are not modelled.
- Next slice (A2 VTOL design) needs: reuse the catalog (airspeed MS4525 and aero data are still to add: categories
  airspeed, servo_air), the GnssError plugin as is, make_sdf.py as the pattern for catalog-driven sensors, the
  fit_motor_prop.py procedure for any new motor/prop pair (needs a published thrust table), and note that
  sensors in PX4 gz bridge are looked up by name under link base_link (imu_sensor, magnetometer_sensor,
  air_pressure_sensor, navsat_sensor). A fixed-wing airframe also needs PX4's airspeed bridge topic (check GZBridge).

## Audit of suspect values (before -> after)
| value | was (x500 file) | source before | now | source now |
|---|---|---|---|---|
| gyro noise | 8.73e-4 rad/s, no bias | PX4 default, unsourced | 7.727e-4 + bias sigma 2.9e-3 + GM 1.04e-5 | ICM-42688-P datasheet noise density x sqrt(250), ZRO/3; BI estimate flagged |
| accel noise | 6.37e-3 / 6.86e-3 | unsourced | 1.008e-2 xy, 1.085e-2 z, bias sigma 6.5e-2 | ICM-42688-P 65/70 ug/rtHz, zero-g 20 mg/3 |
| baro | 3 Pa white | unsourced | 2.7 Pa, 1 Pa LSB, 50 Pa offset | MS5611 OSR1024 RMS, LSB, accuracy 1.5 mbar/3 |
| mag | 3e-7 T white | unsourced | 3e-7 T noise (estimate), 7.576e-8 T LSB, 3e-7 T offset | IST8310 datasheet ZGD, PX4 16-bit mode |
| GNSS | no noise in gz (PX4 bridge adds 2 cm) | PX4 F9P model | 1.274 m h / 1.911 m v, tau 300 s, 0.05 m/s | MAX-M10S CEP 1.5 m, v 0.05 m/s; vertical, tau estimated |
| GNSS rate | 30 Hz | unsourced | 10 Hz | MAX-M10S default nav rate |
| motorConstant | 8.54858e-6 | x500 (unsourced) | 1.2295e-5 | T-Motor T1045 table fit |
| momentConstant | 0.016 | x500 | 0.01404 | table kQ/kT, CP/CT cross-check 0.01405 |
| maxRotVelocity | 1000 | x500 | 1032.2 | 100 % table row, 9857 rpm |
| time constants | 0.0125 up, 0.025 down | x500 | 0.062 both | J/(KtKe/R + 2 kQ w0), assumption: active braking |
| rotor link | 16.08 g, iyy 2.6e-5 | x500 | 13 g, iyy = izz = 6.989e-5 | T1045 mass; J = m D^2/12 upper bound |
| base mass/inertia | 2.0 kg, 0.0217/0.0217/0.04 | x500 | unchanged | Holybro publishes only frame weight 610 g: OPEN |
| rotorDrag, rollingMoment | 8.06e-5, 1e-6 | x500 | unchanged | no source: OPEN |
| SIM_GZ_EC_MIN | 150 rad/s | PX4 default | unchanged | PX4 default, flagged |

## Derivations
Motor/prop (tools/fit_motor_prop.py <csv>): thrust T = kT w^2, w = rpm 2 pi/60, least squares over the 15 table rows
(30-100 % throttle, 16 V): kT = 1.2295e-5 N s^2/rad^2, rms residual 0.097 N, max 7 % at low thrust because CT varies
0.089-0.096 with Reynolds number (the hover region sits about 3 % below the fit). kQ = 1.7258e-7, momentConstant =
kQ/kT = 0.01404 m. Max: w = 1032.2 rad/s, T = 13.10 N. Hover of 2.0 kg: w = sqrt(m g/4/kT) = 632 rad/s, command
(632-150)/(1032-150) = 0.546 in PX4's linear mapping (airframe 4001 sets MPC_THR_HOVER 0.6; the 0.05 difference is left to PX4's
hover-thrust estimator).
Time constant from J dw/dt = Kt (V d - Ke w)/R - kQ w^2 linearised at w0: tau = J/(Kt Ke/R + 2 kQ w0), Ke = Kt = 60/(2 pi 920)
= 0.01038, R = 0.115 ohm, J = 13 g x 0.254^2/12 = 6.989e-5 (rod upper bound; motor bell ignored). At the 45 % table row
(w0 555) tau = 62 ms (60.5 ms at hover). A coasting prop would have 365 ms: spin-down uses 62 ms only if the ESC brakes
actively (BLHeli_S damped light, unverified for the kit ESC). 62 ms is conservative (rod bound).
IMU: gz white stddev = density x sqrt(rate) (Kalibr convention). The alternative sqrt(rate/2) (one-sided bandwidth) would
be 1/sqrt 2 lower; the Kalibr form is kept because it matches the Allan deviation at 1 s. Dynamic bias from
tools/imu_noise_params.py (tau 1000 s assumed): gyro 1.0445e-5, accel 3.27e-4 / 3.04e-4 for the estimated floors.
Baro/mag: gz applies noise without dt, so only white noise, turn-on offset and quantisation exist.
GNSS: circular normal CEP = 1.1774 sigma per axis, so 1.5 m CEP gives sigma_h 1.274 m.

## Results
All runs: ROS_DOMAIN_ID=93 GZ_PARTITION=air, gui/rviz off, PX4 default 4001 gains, wyvern with the new sensors and motors.
Baselines are from docs/notes/drone.md (x500 sensors/motors, GNSS ideal).

| run | result | est err max | ground-truth err max | other |
|---|---|---|---|---|
| lab hover 2.0 m, 30 s (gnss_errors off) | success, landed, disarmed | 0.042 m | 0.135 m | hover est std [0.018, 0.021, 0.006] m, gz truth std [0.039, 0.021, 0.025] m, mean z 1.904; rtf 0.96 |
| lab mission (6 cage waypoints, 1.0 m/s) | 6/6, landed, disarmed | 0.147 m | 0.182 m | flight 61.3 s; rtf 0.94 |
| keep mission (8 waypoints, 5 m/s, gnss_errors auto = on) | 8/8, landed, disarmed | 0.212 m | 2.019 m | flight 356.7 s; rtf 0.82 |

Baselines (drone.md): hover est std [0.015, 0.016, 0.010] m; lab6 6/6 truth err <= 0.10 m; keep4 8/8 truth err <= 0.30 m.
Reading: with default PX4 gains and a 62 ms motor lag nothing oscillates or saturates, so no retune was needed (no
PX4 tuning procedure was run; none of the checks needed one). Hover xy est std rose slightly (0.015/0.016 -> 0.018/0.021 m)
because the IMU now has turn-on and in-run bias and about 10 % higher white noise; z is better (0.010 -> 0.006) with the
lower baro noise. Lab truth error grew 0.10 -> 0.18 m for the same reason (bias is observed by EKF2 only partly, and the
lab has no GNSS: the ideal navsat truth feeds PX4). Keep: PX4's estimate follows its own GNSS, so the estimate error
stays 0.21 m while the real position error against gz truth is up to 2.0 m. That is the intended effect of a realistic
GNSS error (sigma_h 1.27 m, tau 300 s): a vehicle flying waypoints from an M10-class receiver is only metre-accurate,
the old 0.30 m figure was the ideal-GNSS result. Not repeated: warehouse, apartment (no GNSS exists there either).
ULogs: .px4/<world>/fs/log/ (newest file per world). Single run per case: no
statistics over seeds yet (the GnssError seed and gz bias draws differ per run unless seed is set).

## Gotchas
- gz-sensors applies Mag/Baro/Navsat noise without dt: no Gauss-Markov bias on those sensors, so GNSS drift is a plugin.
- PX4's gz bridge reads navsat from sensor name navsat_sensor; the truth sensor is therefore navsat_truth and the
  plugin owns the navsat_sensor topic. A model without the plugin and a sensor named navsat_sensor still works.
- gz navsat position noise (stock) is in degrees with one shared instance for lat and lon: unusable, not used.
- gnss_errors:=false works by writing zero stddevs into a temp copy of model.sdf (the plugin then forwards the truth).
- The launch adds fleet_aircraft's lib dir to GZ_SIM_SYSTEM_PLUGIN_PATH of gz; the package env hook does the same
  for shells that sourced install/setup.bash.
- Sim-slot lock: a launch can wait for other lanes' sims; judge progress by the log, not the wall clock.

## Limitations (what is NOT modelled)
- PX4 voting between two IMUs, magnetometer or baro redundancy (bridge takes one IMU, one mag, one baro).
- IMU vibration and g-sensitivity, magnetometer motor interference, hard/soft iron, baro propwash, temperature drift.
- GNSS latency (EKF2_GPS_DELAY 110 ms default), multipath, dropouts; PX4's bridge reports fixed eph/epv/hdop/vdop.
- Battery sag and supply-voltage dependence of thrust (table is 16 V; kit pack is 4S: 14.8 V nominal, 16.8 V full).
- Motor/ESC thermal limits, ESC non-linear throttle curve, prop efficiency loss with advance ratio, ground effect.
- Time constant is a rod-inertia upper bound with assumed active braking; a measured spin-up test would replace it.

## What the usability phase needs
- One YAML per vehicle (sensor and motor catalog ids) that make_sdf.py consumes, instead of hardcoded catalog paths.
- Generalise make_sdf.py from the wyvern to any airframe, add airspeed/servo categories for fixed-wing and VTOL.
- Document ros topics (/fmu/out/sensor_combined, /fmu/out/vehicle_gnss) in the launch docstring per vehicle.
