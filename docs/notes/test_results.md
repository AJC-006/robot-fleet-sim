# Test results

Written by `scripts/run_all_tests.sh` (rows of earlier runs are kept; each row has the date of its last run).

Summary: 17 PASS, 1 XFAIL

| test | status | seconds | mode | date | note |
|---|---|---|---|---|---|
| nav2_box_apartment | PASS | 121 | full | 2026-10-25 10:33 | rc=0 rtf=[ 1.00 1.00 0.56] [patrol] kids_room: OK SUCCEEDED 49.3 s;[patrol] ensuite: OK SUCCEEDED 58.9 s;[patrol] apartment: 2/2 goals reached; |
| nav2_wyrm_apartment | PASS | 241 | full | 2026-10-25 10:37 | rc=0 rtf=[ 0.78 0.34 0.89] [patrol] kids_room: OK SUCCEEDED 86.3 s;[patrol] ensuite: OK SUCCEEDED 109.6 s;[patrol] apartment: 2/2 goals reached; |
| nav2_truck_lab | PASS | 121 | full | 2026-10-25 10:39 | rc=0 rtf=[ 0.83 1.00 0.93] [patrol] workshop: OK SUCCEEDED 55.7 s;[patrol] robot_arm: OK SUCCEEDED 24.6 s;[patrol] lab: 2/2 goals reached; |
| nav2_hound_lab | PASS | 126 | full | 2026-10-25 10:41 | rc=0 rtf=[ 0.95 0.39 0.90] [patrol] workshop: OK SUCCEEDED 66.3 s;[patrol] robot_arm: OK SUCCEEDED 30.0 s;[patrol] lab: 2/2 goals reached; |
| nav2_rover_warehouse | PASS | 192 | full | 2026-10-25 10:45 | rc=0 rtf=[ 0.55 0.50 0.76] [patrol] staging: OK SUCCEEDED 85.7 s;[patrol] dock_south: OK SUCCEEDED 56.1 s;[patrol] warehouse: 2/2 goals reached; |
| nav2_spider_warehouse | PASS | 238 | full | 2026-10-25 10:49 | rc=0 rtf=[ 0.40 0.85 0.37] [patrol] staging: OK SUCCEEDED 105.9 s;[patrol] dock_south: OK SUCCEEDED 63.8 s;[patrol] warehouse: 2/2 goals reached; |
| nav2_round_keep | PASS | 241 | full | 2026-10-25 10:53 | rc=0 rtf=[ 0.55 0.97 0.64] [patrol] hall_carpet: OK SUCCEEDED 17.3 s;[patrol] hall_statue: OK SUCCEEDED 153.3 s;[patrol] keep: 2/2 goals reached; |
| auv_lake_mission | XFAIL | 1349 | full | 2026-10-25 11:16 | mission node failed:  [known: depth/heading controller unstable on the realistic model (docs/notes/sea.md, S3)] |
| wyvern_mission_lab | PASS | 105 | full | 2026-10-25 11:17 | {'reason': ''} |
| colcon_test | PASS | 37 | quick | 2026-10-25 13:15 | Summary: 88 tests, 0 errors, 0 failures, 0 skipped |
| world_check_apartment | PASS | 0 | quick | 2026-10-25 13:15 | free area reachable from spawn: 217.3 m^2 of 296.2 m^2 free |
| world_check_lab | PASS | 0 | quick | 2026-10-25 13:15 | free area reachable from spawn: 396.6 m^2 of 516.4 m^2 free |
| world_check_warehouse | PASS | 1 | quick | 2026-10-25 13:15 | free area reachable from spawn: 1006.5 m^2 of 1217.2 m^2 free |
| world_check_keep | PASS | 1 | quick | 2026-10-25 13:15 | free area reachable from spawn: 28057.2 m^2 of 29499.5 m^2 free |
| ready_wheeled_round_apartment | PASS | 18 | quick | 2026-10-26 10:21 | ready: /Managed nodes are active/ and a message on /odometry/filtered |
| ready_legged_hound_lab | PASS | 17 | quick | 2026-10-26 10:21 | ready: /Managed nodes are active/ and a message on /odometry/filtered |
| ready_auv_tank | PASS | 13 | quick | 2026-10-26 10:22 | ready: /process started/ and a message on /knucker/odometry/filtered |
| ready_drone_wyvern_lab | PASS | 23 | quick | 2026-10-26 10:22 | ready: /Ready for takeoff/ and a message on /fmu/out/vehicle_odometry |
