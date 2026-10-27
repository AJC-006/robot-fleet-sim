# Sea lane notes (knucker AUV: physics, components, plugins)

## Status / next

**S2 (implement) finished 2026-10-25. Builds clean, `colcon test` passes (fleet_auv: 4 pytest; fleet_marine_plugins:
ctest 4/4 incl. cpplint, lint_cmake, xmllint and the thruster-core numeric test). Both launch headless (tank and keep).**
Done (files under `src/fleet_auv`, `src/fleet_marine_plugins`, `blender/auv/build_auv.py`)
- `model.py` generates SDF+URDF from `config/knucker.yaml` (layout, component stems) + `config/knucker_physics.yaml`
  (exported from `tools/derive.py` by `tools/export_physics.py`; derived numbers in `config/knucker_derived.json`).
  Inertial 25.464 kg, CG, I about CG, `<fluid_added_mass>` 10 terms, Hydrodynamics 11 damping + 5 fin-lift terms, all S1
  values; buoyancy collision primitives total the derived volume with CB x matched (pytest checks 0.1 mm).
  `max_am_ratio` and the +-30 N thrusters are gone (remain only in `tools/ref/knucker_s1.yaml`, the S1 derivation input).
- Plugins (`fleet_marine_plugins`): `T200Thruster` (PWM us in, vendor table bilinear in PWM and bus V, von Benzon 3rd-order
  dynamics, deadband from the ESC entry, force applied at the pod incl. offset moment, current out, propeller spin),
  `LiIonBattery` (4S6P, OCV(SoC), R 10.7 mohm, hotel load, over-current report, publishes BatteryState).
  Core in `include/.../t200_core.hh` (no gz dependency) with `test/test_thruster_core.cpp`.
- Driver-compatible emulators (`sensor_drivers.py`): `waterlinked_dvl_driver`, `microstrain_imu`
  (`imu/data`, `imu/data_raw`, `imu/mag`), `bar30_driver` (FluidPressure + temperature). Params from catalog entries.
- `auv.launch.py`: args `world` (keep|tank|any fleet_worlds world) `namespace` `gui` `rviz` `camera` `localization`
  `controller` `mission` `cruise` `log_csv` ... all declared with descriptions. REP-105: static map->odom + robot_localization
  EKF (`config/ekf.yaml`) gives odom->base_link (no ground truth in TF). Thruster command topic
  `<ns>/thrusters/<port|stbd|vfore|vaft>/pwm_us` (std_msgs/Float64, us). `odometry_gt` is for tests only.
- `worlds/tank.sdf` (20 m deep test tank, DART, graded buoyancy at z 0) ; keep lake already has DART + graded buoyancy
  (water -0.8 m, air 1.225 / water 999.103) and the DVL system. Mass/inertia checks: `test/test_model.py`.
- Trim check `test/settle_check.py` (live sim): 80 s at the surface, mean pitch -0.27 deg (limit 0.4), roll 0.05, mean vz
  -0.0001 m/s, bus 16.72 V: PASS, matches the derived BG 15.1 mm / net +2.5 N equilibrium.
- This pass: added `test_depend python3-pytest` + `tests_require` so colcon runs the pytest; fixed 30 cpplint findings.

NOT working well (S3 must fix; I did not retune, as instructed)
1. **The pre-realism depth/heading/pitch controller is unstable with the realistic model.** Tank, depth ref = spawn depth 3 m:
   holds 3.0 m +-0.05 m for ~40 s, then a pitch/surge runaway starts (t46: pitch 0.12 rad, u 0.6 m/s with ~0 N horizontal
   thrust; t63 pitch 1.0 rad; the vehicle later ends nose-up near vertical at the surface with vfore saturated at -39 N).
   Keep-lake mission launches and runs (8 waypoints) but depth diverges (z -12 m vs ref -4.8 m, yaw spinning) after ~15 s.
   Likely causes to test first: (a) Munk moment from the A22-A11 difference (~30 kg) is destabilising in pitch while pitch
   kp 28 N m/rad and kd 12 are the old values; (b) a yaw/pitch/roll coupling via ixz and the offset fin-lift terms;
   (c) controller feed-forward/allocation assumes the old symmetric limits. Test open loop (constant PWM) first to separate
   plant from controller; `settle_check` passes with thrusters at 1500 us so the plant is stable at rest.
2. Surge thrust is bollard (upper bound): no KT(J) (S1 open issue 1 stands). Speeds in tests are optimistic.
3. EKF x/y drifts when the vehicle is not level (DVL looks down, no GPS under water): expected, needs a mission-level
   reset or a USBL-like source in S3; ekf.yaml fuses DVL velocity + Bar30 depth + AHRS.
4. Added mass/damping stay at submerged values at the surface (S1 open issue 4). OCV(SoC) curve and DVL/CV7 numbers
   unpublished (S1 open issues 2-3). Plots in docs/img/auv_*.png are from the pre-realism model: stale.
5. Pod drag bracket (S1 issue 5) still decides surge terminal speed.
S3 needs: open-loop plant tests against `Validation targets`; controller retune from the derived 6x6 (use the S1
numbers for pitch/yaw plant), then rerun `ros2 run fleet_auv auv_tests` and the lake mission; regenerate plots; write
the 'add a robot / swap a sensor' guide (config/knucker.yaml component stems + catalog) in the usability phase.

---

## S1 hand-off (archived)

**S1 (audit, bill of materials, derivations) done 2026-10-25. No simulation code changed.**
Done
- Audit (table below): 22 items replaced or re-sourced: made-up mass/CoG, `max_am_ratio` cap, drag factors without
  sources, symmetric ±30 N thrusters, made-up sensor noise, TF world->base_link taken from ground truth.
- Bill of materials with sources, written as catalog entries (`src/fleet_components/components/`):
  `thruster/bluerobotics_t200.yaml` (+ vendor table `bluerobotics_t200_performance.csv`, 1206 rows, 10-20 V),
  `esc/bluerobotics_basic_esc.yaml`, `battery/bluerobotics_li_ion_4s_18ah.yaml`, `dvl/waterlinked_dvl_a50.yaml`,
  `pressure/bluerobotics_bar30.yaml`, `ahrs/microstrain_3dm_cv7_ahrs.yaml`.
- Rigid body from mesh + component masses (free-flooding 3 mm PETG fairing, BR 4" Al housing; lake 15 °C,
  ρ 999.103, Tanaka 2001). Added mass: panel method (BEM) strips + Lamb factors, 6x6. Damping: ITTC-1957,
  Hoerner, crossflow drag, fin lift. All numbers are in "Derivations".
- `<fluid_added_mass>` with DART in gz-sim 8.15 checked in the source and by a headless test: m 1 kg with A33 1 kg
  falls at exactly g/2; xx-only leaves heave unchanged, so the matrix ordering is right.

Decisions (numbers for S2)
- Housing centre x +0.10 m; 23 BR 200 g SS ballast (4.39 kg) at z -0.072; R-3318 foam 1.205/2.911/5.225 L
  (saddle/nose/tail) → m 25.464 kg, CG (0.0162, 0, -0.0142), CB (0.0162, 0, +0.0009), BG 15.1 mm, net +2.497 N.
  Housing at x 0 reaches only BG 11.4 mm (tail foam bay full at 14 weights).
- Inertia about CG (ideal-fluid roll: entrained water does not spin up): ixx 0.1128, iyy 2.6289, izz 2.6515,
  ixy -0.00113, ixz 0.03144, iyz 0.00003 kg m².
- `<fluid_added_mass>` about the link origin (positive physical values): xx 4.2827, yy 34.7355, zz 35.3626,
  pp 0.2653, qq 5.0906, rr 2.9505, yr -1.375, zq 3.482, yp -0.685, pr 0.153. Note A22/m ≈ 1.36: explicit added
  mass (Hydrodynamics plugin) would go unstable, so DART is required (only dartsim implements AddedMassFeatures).
- Damping with gz Hydrodynamics 8.15, added-mass terms left at zero (it handles the current Coriolis correction
  for `<fluid_added_mass>` itself): xU -0.218, xUabsU -4.455, yVabsV -154.78, zWabsW -183.98, kPabsP -0.574,
  mQabsQ -11.05, nRabsR -9.304, yRabsR 5.285, nVabsV 11.246, zQabsQ -2.851, mWabsW 2.061;
  fin lift yVabsU -36.77, nVabsU 8.278, kVabsU 4.618, zWabsU -127.63, mWabsU 28.93.
- Buoyancy: graded (water 999.103 / air 1.225 kg/m³), because the vehicle surfaces and floats. Collision primitives
  (box/sphere/cylinder/capsule/ellipsoid/cone; graded mode cannot use meshes) must total 25.742 L with their
  centroid at CB. **CB x must match to 0.1 mm: 1 mm of error tilts the vehicle 3.8°.** gravity 9.80665.
- Thrusters: a custom plugin is needed. gz Thruster has one symmetric coefficient and no PWM, voltage, current or
  dynamics. Spec in "Thruster plugin": PWM µs in; vendor table interpolated in (PWM, bus V); von Benzon 2022
  third-order dynamics; current into the battery model.
- Sensors copy the real drivers: WL `waterlinked_dvl_driver` topics (FRD `dvl_link`); microstrain `imu/data`,
  `imu/data_raw`, `imu/mag` (`imu_link`, ENU); Bar30 (no published ROS 2 driver, S2 writes one): `pressure`
  (FluidPressure, Pa abs, 20 Pa RMS, 10 Pa LSB, 20 Hz) + `temperature`.

Files: this note, the 6 catalog YAMLs and the CSV. Derivation script: `src/fleet_auv/tools/derive.py`
(+ `meshlib.py`; `python3 -I derive.py hull.obj knucker.yaml t200.json 0.10`, 66 s → `derived.json`).

Open issues
1. **No T200 data at forward speed.** BR states all data are bollard and no open-water tests were done. At cruise
   the bollard-equivalent advance ratio J ≈ 1.0, so a bollard-only model overstates thrust at speed. S2: build
   KT(J) from the T200 propeller geometry (BR publishes CAD) using blade-element momentum theory, or use the
   Bachmayer/Whitcomb/Grosenbaugh 2000 four-quadrant model. Expose it as a table. Until then, label the result
   "bollard (upper bound)".
2. The 4S6P battery OCV(SoC) curve is not published (cell sheet: 3.6 V nominal, 2.5-4.2 V). S2 needs a measured
   Samsung 30Q curve. Pack R 10.7 mΩ comes from a third-party measurement of the cell's DC resistance.
3. Unpublished: DVL per-ping noise and rate-vs-altitude law; CV7 mag noise, bias correlation time, filter dynamics.
4. Added mass/damping stay at submerged values in air (no runtime `fluid_added_mass` change in 8.15): surfacing only.
5. Pod drag dominates surge uncertainty: xUabsU between -2.17 and -6.73 (DNV/Hoerner bracket).
6. If G2 picks bullet-featherstone/TPE: no added mass at all, explicit fallback unstable (Ma/m > 1). Sea needs DART.

S2 needs: the inertial, `fluid_added_mass`, Hydrodynamics and buoyancy values above; collision primitives that
match V and CB; the plugins `T200Thruster`, `LiIonBattery`, Bar30/DVL/AHRS driver emulators, all with params in
YAML (see "Specs for S2"); and REP-105 TF from a localisation node (not ground truth), with a `namespace` arg.

---

## Audit of the current knucker (before S1)

| # | Item (file) | Current | Problem | Replacement (S1) |
|---|---|---|---|---|
| 1 | mass (`params.py`) | ρV/(1+excess), uniform over primitives | assumed density | component BOM + mesh: 25.464 kg |
| 2 | CoG (`params.py`) | CoB − 25 mm | assumption | trim solve: BG 15.1 mm |
| 3 | fluid density / g (`knucker.yaml`) | 1000 / 9.8 | rounded | 999.103 (15 °C, Tanaka 2001) / 9.80665 |
| 4 | added mass (`params.py`) | Lamb × appendage factors 1.15/1.30 | factors without sources | BEM strip + Lamb, 6x6 |
| 5 | `max_am_ratio` 0.7 | caps added mass for the explicit plugin | jugaad | `<fluid_added_mass>` (implicit, DART) |
| 6 | Kpd −0.06 | hard-coded | no source | removed; full 6x6 matrix |
| 7 | drag cd_x/y/z, A_side×1.25, A_plan×1.15 | unsourced factors | jugaad | ITTC/Hoerner/crossflow/fin-lift derivation |
| 8 | lin damping {xU −4 … nR −4}, rot_quad | unsourced | made up | xU −0.218 (ITTC fit), rest from crossflow |
| 9 | thrusters (`model.py`) | gz Thruster, kt 0.0072, D 0.09, ±30 N | not a T200 (D 76 mm, +51.4/−39.9 N @16 V) | custom T200 plugin + vendor table |
| 10 | lag_tau 0.15 | in the allocator, not the plant | wrong place, no source | von Benzon TF in the plant plugin |
| 11 | IMU | generic σ, no bias, 100 Hz, `/model/knucker/imu` | not a real IMU, absolute topic | CV7 densities, microstrain topics |
| 12 | DVL | gz DVL, 1 cm resolution, 10 Hz, 0.1-60 m, σ 0.004 | not A50 (0.1 mm/s, 4-15 Hz, 0.05-50 m) | A50 spec + WL driver messages |
| 13 | DVL frame | base_link-aligned | WL driver publishes FRD `dvl_link` | URDF joint roll π, FRD output |
| 14 | depth (`sensors.py`) | Float64, 1 cm Gaussian | not a sensor output | Bar30: FluidPressure 20 Pa RMS, 10 Pa LSB |
| 15 | pressure | 101325 + ρ g d, frame base_link | no noise or quantisation, wrong frame | `pressure_link`, MS5837 noise/LSB/rate |
| 16 | TF world->base_link | from ground truth | breaks REP-105, cheats localisation | map->odom->base_link from an estimator |
| 17 | topics | absolute `/knucker/...` | no namespace arg | relative topics + `namespace` |
| 18 | `<physics type="ode">` (tank.sdf) | engine label | gz-sim picks the engine in the Physics system (`<engine>`, default DART); attribute is misleading | remove label; DART explicit |
| 19 | buoyancy (keep.sdf) | graded 1000/1.2 at z −0.8 | rounded densities | 999.103/1.225 |
| 20 | hull collision = 13 cylinders | volume from ρV fit | must match V_disp and CB | primitives to 25.742 L, CB ±0.1 mm |
| 21 | sonar | gpu_lidar proxy | no product | open (not in S1 scope; needs a real product) |
| 22 | battery | none | no voltage sag, no current limit | 4S6P model, 60 A continuous / 132 A burst |

## Bill of materials

| Part | Product | Mass air / water | Position (link frame) | Source |
|---|---|---|---|---|
| Thrusters ×4 | BR T200 R2 | 0.427 / 0.239 kg | port/stbd (−0.585, ±0.15), vfore x 0.33, vaft x −0.30 (`knucker.yaml`) | BR product page + table |
| ESC ×4 | BR Basic ESC BESC30-R3 | 16.3 g each | in housing | BR product page |
| Housing | BR 4" locking, 300 mm 6061-T6 tube (OD 112.03, ID 104.16 mm) | 1066 g | centre x = 0.10 | robotshop.com BR listing |
| Flanges ×2 | BR 4" O-ring flange | 107 g each | tube ends | robotshop.com BR listing |
| End caps ×2 | BR 4" blank Al end cap (10 mm) | 234 g each (upper bound vs penetrator caps 196 g) | tube ends | robotshop.com BR listing |
| Battery | BR Li-ion 14.8 V 18 Ah (4S6P 30Q) | 1152 g | (X_H − 0.06, 0, −0.0144) | BR store |
| DVL | Water Linked A50 | 0.170 / 0.105 kg | (−0.10, 0, −0.0925) | WL docs |
| Depth | BR Bar30 (MS5837-30BA) | 14 g | aft cap | Bar30 listing, TE datasheet |
| AHRS | MicroStrain 3DM-CV7-AHRS | 8.3 g | in housing | datasheet 8400-0141 Rev E |
| Electronics tray, compute, wiring | — | 0.35 kg **estimate (unpublished)** | in housing | — |
| Clamps/saddles | PETG | 0.15 kg **estimate** | housing saddles | — |
| Fairing | Prusament PETG, 3 mm skin, ρ 1270 | skin area 0.5888 m² | mesh | Prusament PETG TDS |
| Decor shells (existing visuals) | PETG solid, 312 shells | 3.810 kg | mesh | mesh volume 3.820 L (3.000 L outside envelope) |
| Foam | BR R-3318 (LAST-A-FOAM R-3300, 288 kg/m³) | 9.341 L = 2.690 kg | saddle/nose/tail bays | bluerobotics.com/?p=20047 |
| Ballast ×23 | BR stainless (AISI 316) 200 g | 191 / 165 g each | (0.10, 0, −0.072) | bluerobotics.com/?p=28583 |

URLs: T200 https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/ ;
ESC https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/ ;
battery https://bluerobotics.com/store/comm-control-power/batteries/battery-li-4s-18ah-r3 ;
tube https://www.robotshop.com/products/bluerobotics-300-watertight-aluminum-enclosure-tube-4 ;
cap https://www.robotshop.com/products/bluerobotics-wte-4-end-cap-packed-assembly-6061-t6-blank ;
flange https://www.robotshop.com/products/bluerobotics-4-inch-o-ring-flange-100-mm-4 ;
A50 https://docs.waterlinked.com/dvl/dvl-a50/ ; WL driver https://github.com/waterlinked/waterlinked_dvl ;
Bar30 https://www.robotshop.com/products/bar30-high-resolution-300m-depth-pressure-sensor-r2 ;
MS5837 https://www.te.com/en/product-CAT-BLPS0017.html ;
CV7 https://www.microstrain.com/sites/default/files/8400-0141_3DM_CV7_Datasheet.pdf ;
microstrain driver https://github.com/LORD-MicroStrain/microstrain_inertial ;
30Q cell https://old.nkon.nl/sk/k/30q-specs.pdf ; 30Q DC IR https://endless-sphere.com/forums/viewtopic.php?p=1471788 ;
T200 dynamics https://doi.org/10.3390/jmse10121898 ; T200 bollard-only statement
https://discuss.bluerobotics.com/t/t200-performance-with-relative-speed/11373 .

Current budget: 4 T200 at full throttle and 16 V draw 4 × 24.3 = 97 A. The battery is rated 60 A continuous and
132 A for 10 s, so the allocator (S3) must cap total current. The plugin reports current; it does not clip.

## Derivations

### Water and constants
ρ(15 °C) = 999.103 kg/m³ (Tanaka et al. 2001, Metrologia 38:301); ρ(4 °C) 999.975, ρ(25 °C) 997.047.
μ = 1.1375e-3 Pa s (IAPWS/CRC), ν = 1.1385e-6 m²/s. g = 9.80665. Air 1.225 kg/m³ (ISA, 15 °C, 101325 Pa).

### Envelope and mass (derive.py, Eberly 2002 polyhedral integrals)
- Mesh hull scaled 0.985. Revolved envelope 24.6516 L. With the old Ø94 mm bores: 21.4945 L (mesh check:
  revolved minus 2 bores = 22.3835 L, mesh 22.3309 L, within 0.24 %). New tunnel bores have r_out 0.058 / r_in 0.052:
  T200 OD 100 mm + 2 mm clearance, 6 mm PETG wall. That gives an envelope of 21.2819 L. Skin area 0.5888 m².
- Decor: 312 shells, 3.8204 L, of which 3.0002 L lies outside the envelope. Decor is counted as solid PETG.
- Total displaced volume 25.742 L (envelope + decor outside + thrusters, DVL, sensors).
  The free-flooding interior water (6.064 kg) is counted both as mass and as displaced volume, so it is neutral.
- Trim solve: for each ballast count, solve nose and tail foam volumes so that net buoyancy is +1.0 % and
  x_G = x_B. Then pick the smallest count with BG ≥ 15 mm (design target, same order as REMUS 100's 19.6 mm,
  Prestero 2001). Housing positions scanned: X_H −0.07 → BG ≤ 7.6 mm (5 weights, tail bay 2.19 L full);
  0.00 → 11.4 mm (13); 0.05 → 14.2 mm (21); **0.10 → 15.08 mm (23)**. X_H range allowed by the tunnels: [−0.074, 0.104].
- Result at X_H 0.10: solids 19.401 kg + water 6.064 kg = 25.464 kg; W 249.719 N; B 252.216 N.
- Inertia: about CG, "ideal" counts the entrained water at full mass in pitch and yaw, but not in roll. Reason:
  the viscous spin-up time R²/ν = 0.09²/1.14e-6 ≈ 7100 s is far longer than any roll motion. Rigid-water Ixx would
  be 0.1715 (ixz 0.02654). Use ideal.

### Added mass (about link origin, FLU, positive-definite, min eigenvalue 0.246)
- Body: Lamb (1932 §373) prolate spheroid a 0.65, b 0.0952 → k1 0.0372, k2 0.9308, k' 0.8002, applied as 3-D factors
  to 2-D strip values. 2-D section values come from a constant-source panel method (Katz & Plotkin 2001 §10.2.1)
  on the real cross-sections.
- Panel-method validation: circle +0.34 %, flat plate heave +4.62 %, circle + 2 fins vs Blevins −1.96 %.
  Convergence at 1 vs 0.5 mm panel size: −1.12 %.
- Appendages: dorsal fin elliptic-plate factor 1/E(e) = 0.658 (S_exp 0.0401 m², span 0.154); pectoral fins factor
  0.949 (A33 3.975, bracket 3.54-5.95); T200 pods as finite cylinders L/D 1.13 → Ca factor 0.611 (DNV-RP-C205
  Table A-2, Patton 1965). Water in the open tunnels is counted in x, y and yaw only.
- Parts (diag xx yy zz pp qq rr): body 0.916 22.925 22.925 0 1.953 1.953; dorsal 0 7.934 −0.081 0.063 −0.005 0.486;
  pylons 0 −0.103 2.462 0.004 0.851 −0.035; pods 0 0.613 6.081 0.036 2.091 0.212; pectorals 0 0 3.975 0.154 0.201 0;
  tunnel water 3.367 3.367 0 0.008 0 0.336.

### Damping (U_ref 1 m/s, Re_L 1.14e6)
- Surge: ITTC-1957 Cf 0.00456. Hoerner form factor → Cd_wet 0.00498. Total CdA = body 0.00293 + fins 0.00115 +
  pylons 0.00005 + pods 0.00065-0.00972 m². The pod value is bracketed between a streamlined pod and a bluff
  duct in axial flow.
- Friction scales with Cf(Re), so drag is not a pure U² law. Least-squares fit (relative weights) of
  F = −(xU·U + xUabsU·U|U|) to ½ρ[CdA_fric·Cf(Re)/Cf(Re_ref) + CdA_pod]U² over 0.1-1.5 m/s:
  mid xU −0.218, xUabsU −4.455 (max error 6.6 %); lo −0.225/−2.174; hi −0.215/−6.727.
- Crossflow (sway, heave, pitch, yaw): Cd cylinder 1.2, plate 1.17 integrated over the real sections. This gives
  yVabsV −154.78, zWabsW −183.98, nRabsR −9.304, mQabsQ −11.048, cross terms yRabsR 5.285, nVabsV 11.246,
  zQabsQ −2.851, mWabsW 2.061, kPabsP −0.574.
  Section Reynolds number at 0.05 m/s is 7.9e3, inside the subcritical range where Cd ≈ 1.2 holds. No linear
  crossflow term is used, because none has a source in this Re range.
- Fin lift: Helmbold low-aspect-ratio slope. Dorsal S 0.0575 m², AR 0.85; pectorals S 0.0328 m² each, AR 4.03;
  chord Re at 0.1 m/s is 1.3e4. Terms: yVabsU −36.77, nVabsU 8.278, kVabsU 4.618, zWabsU −127.63, mWabsU 28.93.
- Signs follow the gz Hydrodynamics form: force_i += c·v_j·|v_k|, in the link frame (FLU), about the link origin.
  The Munk moment and other added-mass Coriolis terms come from DART's spatial inertia; do not add them again.

### T200 steady state (vendor table)
| V | F_max N | F_min N | I_max A | rpm_max | zero-thrust band µs | KT0 fwd/rev |
|---|---|---|---|---|---|---|
| 12 | 36.42 | −28.60 | 17.08 | 2995 | 1464-1536 | 0.431 / 0.340 |
| 14 | 44.39 | −34.49 | 20.71 | 3288 | 1468-1532 | 0.438 / 0.346 |
| 16 | 51.44 | −39.91 | 24.30 | 3533 | 1472-1528 | 0.439 / 0.350 |
F = c·n² fits leave about 1 N of residual, so the plugin interpolates the table instead of using a curve fit.
Reverse/forward thrust ratio at 16 V is 0.776.

### T200 dynamics
von Benzon et al. 2022 (JMSE 10:1898), identified from a T200 data set:
K(s) = (6136 s + 108700)/(s³ + 89 s² + 9258 s + 108700). Poles −13.16 and −37.92 ± 82.60j rad/s, zero −17.7, unity DC
gain. Step response: t10 6.3 ms, t63 20.0 ms, t90 29.3 ms, no overshoot, 2 % settling 0.20 s. The slow pole is
nearly cancelled by the zero, which gives the slow tail.
Motor parameters from a BR forum user's measurement (Rs 0.1 Ω, Ls 0.03 mH, Ke 1.687 Vrms/krpm, J 5.85e-6 kg m², 14
poles) are unpublished by the vendor. They are informative only and are not used.

## Verification: gz-sim 8.15 / gz-physics 7.8 / DART 6.13.2
- `gz-physics` dartsim `SDFFeatures.cc:716`: when the inertial has `FluidAddedMass`, it calls
  `AddedMassFeatures::SetLinkAddedMass`. That function reorders the 6x6 into Featherstone order, adds it to the
  body's spatial tensor (implicit) and sets `setGravityMode(false)`.
- `SimulationFeatures.cc`: every step it applies m·g at the COM, using the rigid mass only, for links with added
  mass. Engine plugins checked with `strings`: only dartsim contains AddedMass. bullet-featherstone, bullet and TPE
  do not; ODE is not a gz-physics engine.
- sdformat 1.11 spec: "fluid added mass matrix about the link's origin".
- gz Hydrodynamics 8.15 parses full 6x6 linear terms (`xU`…), 6x6x6 `xUabsV`-style quadratic terms and
  `default_current`. With `<fluid_added_mass>` present it adds only the current-velocity Coriolis correction; setting
  both is an error.
- Test (a throwaway gz TestFixture program, not kept)
  (TestFixture, dartsim, 200 steps of 1 ms, m 1 kg):
  zz = 1 → dz −0.09855 m; xx = 1 → −0.19710; none → −0.19710. Ratio 0.500 = m/(m+A33). Pass.
- Buoyancy 8.15 graded mode supports box, sphere, cylinder, capsule, ellipsoid and cone, but not mesh
  (`Buoyancy.cc:622-663`). Uniform mode also accepts meshes.
- gz Thruster 8.15: thrust follows the command instantly, uses one symmetric `thrust_coefficient`
  (T = ρ kt D⁴ ω|ω|), and its deadband acts on thrust magnitude only. There is no PWM mapping, no voltage
  dependence and no fwd/rev asymmetry, and battery use is a fixed power load. So a custom plugin is needed.

## Specs for S2

### Thruster plugin `T200Thruster` (gz-sim system, in `src/fleet_marine_plugins`)
- Params from YAML (catalog file stem `bluerobotics_t200` + `bluerobotics_basic_esc`): joint, link, axis,
  table CSV path, deadband_us 25, pwm range 1100-1900, max update 400 Hz, TF coefficients, battery link/entity.
- Input: PWM pulse width (µs), the signal a real ESC receives, as `std_msgs/Float64` on
  `thrusters/<name>/pwm_us` (relative, in the robot namespace). Hold the last value; clamp to the range;
  |pwm−1500| ≤ 25 → 0.
- Steady state: F_ss and I_ss by bilinear interpolation of the CSV in (pwm_us, V_bus). V_bus comes from the battery
  model. Dynamics: apply K(s) to F_ss (discretise with Tustin at the physics step). Use the same normalised
  response for current.
- Output: force along the thruster axis at the thruster link (no reaction torque: none published) and propeller
  joint speed from the table rpm (visual only). Publish `thrusters/<name>/thrust` (N, ground truth for tests)
  and current. Add V·I to the battery each step.
- Hook for open issue 1: optional KT(J) table, J = U_axial/(n D), default = bollard (none).
- A separate allocator node (S3) does N → µs: invert the table at the measured V_bus, as ArduSub's thrust-curve
  linearisation does.

### Battery `LiIonBattery`
4S6P, 18 Ah. V = OCV(SoC) − 0.0107 Ω · I_total; SoC integrates I_total. Warn when I_total > 60 A, or > 132 A for
more than 10 s. Publish `battery` (`sensor_msgs/BatteryState`, 1 Hz, frame `battery_link`). The OCV table is open
issue 2; LinearBatteryPlugin's linear OCV is not acceptable for a Li-ion curve.

### DVL emulator (WL A50 + `waterlinked_dvl_driver` interface)
`dvl_link` with joint roll π from base_link (FRD), at (−0.10, 0, −0.105). Bottom-track velocity of dvl_link,
expressed in FRD, quantised to 1e-4 m/s, with a per-run scale error within ±1.01 % (default 0).
Invalid outside altitude 0.05-50 m or above 3.75 m/s. Rate: 4-15 Hz; the law vs altitude is unpublished, so
make it a parameter (default 15 Hz).
Topics under `waterlinked_dvl_driver/`: `velocity_report` (Dvl, BOTTOM), `velocity_water_report` (WATER,
altitude NaN), `odom` (frame and child `dvl_link`), `dead_reckoning_report`. QoS system default. Fill the fields as
the driver does: DVL_TYPE_PISTON, sound_speed 1481, course_gnd = atan2(vy, vx), speed_gnd, num_good_beams,
beam_unit_vec (±0.6533, ±0.6533, 0.3827) as the driver publishes them. Gotcha: those vectors are 67.5° from z,
while the datasheet beam angle is 22.5°. Range casting should use the datasheet geometry. Altitude comes from
4-beam range casting (the gz DopplerVelocityLog sensor can do the casting).

### Bar30 driver emulator
`pressure_link` at the aft cap. `pressure` (FluidPressure): p_atm + ρ g depth + N(0, 20 Pa), quantised to 10 Pa,
variance 400 Pa². `temperature` (Temperature, 0.0022 K resolution; water temperature parameter 15 °C).
20 Hz (limit 27.6 Hz at OSR 8192). QoS sensor_data. Depth for estimators comes from a separate node using the
ρ, g and surface-zero parameters, as ArduSub zeroes at the surface. Output it as a PoseWithCovarianceStamped z
for robot_localization.

### AHRS (CV7 + microstrain driver interface)
`imu_link`, ENU. `imu/data` 100 Hz with orientation + rates + accel. `imu/data_raw` (driver default 1 Hz; set as
in the driver). `imu/mag` (off by default). Gyro σ 4.121e-4 rad/s and accel σ 2.942e-3 m/s² at 100 Hz
(σ = N·√rate, Kalibr). Turn-on bias drawn once per run: 6.98e-5 rad/s and 3.92e-4 m/s². Bias instability 7.27e-6
rad/s and 1.765e-4 m/s² (correlation time unpublished). Orientation error kept within 0.5° roll/pitch and 2°
heading (dynamic spec). Covariances as the driver publishes them (static diag 0.01 by default).

## Validation targets for S3
| Test | Target | Derived from |
|---|---|---|
| Float at rest at the surface | emerged volume 0.255 L (V − W/(ρg)) | mass, V_disp |
| Static trim submerged | pitch 0 ± 0.4° (CB x matched to 0.1 mm), roll 0 | x_G = x_B |
| Free ascent, level, thrusters off | terminal w 0.117 m/s | (B−W)/zWabsW |
| Roll free decay (small) | period 1.99 s | (Ixx + A44)/(m g BG) |
| Pitch free decay (small) | period 9.0 s | (Iyy + A55 + A33 x_G²)/(m g BG) |
| Surge acceleration from rest | 0.336 m/s² per 10 N | 1/(m + A11) |
| Thruster step 1500 → 1900 µs, 16 V | 63 % at 20 ms, 2 % at 0.20 s, F_ss 51.4 N, I 24.3 A | table, von Benzon TF |
| Thruster deadband | zero thrust for 1475-1525 µs (ESC); table zero 1472-1528 µs @16 V | ESC, table |
| Surge drag | 1.22 N @0.5, 4.67 N @1.0, 10.35 N @1.5 m/s (mid; lo/hi bracket) | ITTC fit |
| Gyro Allan deviation at τ = 1 s | 4.12e-5 rad/s | CV7 ARW |
| Pressure noise | σ 20 Pa, steps of 10 Pa | MS5837 OSR 8192 |
| DVL output | values on a 1e-4 m/s grid, invalid when altitude < 0.05 m | A50 |

## Gotchas
- `cd` does not persist between tool calls; use absolute paths.
- The gz-sim python `Link.world_pose` needs `import gz.math7` first, otherwise pybind cannot convert the return value.
- Graded buoyancy ignores mesh collisions (warning only), so a mesh hull floats wrong without an error.
- The tail foam bay limits BG when the housing sits aft. Keep the housing forward (x 0.10).
- BR product pages saved by an earlier attempt returned "Page not found". Retailer listings and `?p=` links were
  used instead.

## Limitations (honest list)
Bollard-only thrust (open issue 1); battery OCV pending; DVL per-ping noise, CV7 magnetometer noise and filter
dynamics unpublished; added mass and damping do not change during surface crossing; electronics tray and
clamp masses (0.50 kg total) are estimates; no thruster reaction torque (not published); no hull-thruster
interaction (thrust deduction) because no data is published for it.

## Notes for the usability guides
- To swap a component, change the catalog stem in the robot YAML. Physics numbers are regenerated by the derivation
  script from the mesh, BOM and catalog. Document the derive.py inputs: hull mesh, knucker.yaml, catalog tables,
  housing x.
- Every plugin parameter lives in YAML with units and a source comment; launch args take `namespace`,
  `use_sim_time`, and the `world` and `water_temperature_c` parameters (ρ from Tanaka).
