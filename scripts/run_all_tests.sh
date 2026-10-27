#!/usr/bin/env bash
# One entry point for every test of the workspace.
#
#   scripts/run_all_tests.sh --quick        # ~15-25 min: colcon test (all packages), world map checks,
#                                           # one headless launch to "ready" per robot kind
#   scripts/run_all_tests.sh --full         # --quick plus Nav2 2-goal runs of every ground robot in every
#                                           # world, lidar coverage checks, AUV lake mission, wyvern missions,
#                                           # SLAM run (about 3-4 h on an 8-thread laptop; see docs/notes/test_results.md)
#   scripts/run_all_tests.sh --only REGEX   # run just the tests whose name matches (works with either mode;
#                                           # the table keeps the rows of earlier runs)
#   scripts/run_all_tests.sh --list         # print the test names of the chosen mode and exit
#
# Needs the workspace built (colcon build --symlink-install) and `source /opt/ros/jazzy/setup.bash`.
# Never opens a window (gui:=false rviz:=false). Exit code is non-zero when any test FAILs.
#
# Environment (all optional)
#   ROS_DOMAIN_ID, GZ_PARTITION  isolate the test run from other ROS/Gazebo users (defaults 90, fleet_tests)
#   SIM_WRAPPER                  command that starts each simulation, e.g. a lock that limits parallel sims
#   LOG_DIR                      run logs (default <workspace>/log/run_all_tests)
#   RESULTS_MD                   markdown table (default <workspace>/docs/notes/test_results.md)
#
# Status values: PASS, FAIL, SKIP (cannot run here, reason given) and XFAIL (a known failure that is
# documented in docs/notes; it does not fail the run, and turns into PASS when fixed).

set -uo pipefail

WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MODE=""; ONLY=""; LIST=0
while [ $# -gt 0 ]; do
  case $1 in
    --quick) MODE=quick;;
    --full) MODE=full;;
    --only) ONLY=$2; shift;;
    --list) LIST=1;;
    -h|--help) sed -n 2,26p "$0"; exit 0;;
    *) echo "unknown argument $1 (try --help)"; exit 2;;
  esac
  shift
done
[ -n "$MODE" ] || { echo "give --quick or --full (try --help)"; exit 2; }

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-90}
export GZ_PARTITION=${GZ_PARTITION:-fleet_tests}
LOG_DIR=${LOG_DIR:-$WS/log/run_all_tests}
RESULTS_MD=${RESULTS_MD:-$WS/docs/notes/test_results.md}
RESULTS_TSV=$LOG_DIR/results.tsv
RUN_DIR=$LOG_DIR/$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR"; touch "$RESULTS_TSV"

set +u
# shellcheck disable=SC1091
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
if [ -f "$WS/install/setup.bash" ]; then source "$WS/install/setup.bash"
else echo "no $WS/install/setup.bash: run colcon build --symlink-install first"; exit 2; fi
set -u

# ---------------------------------------------------------------- test definitions
WHEELED="round box rover truck"
LEGGED="wyrm hound spider"
WORLDS="apartment lab warehouse keep"
declare -A SPAWN GOALS
SPAWN[apartment]=master_bedroom; GOALS[apartment]=kids_room,ensuite
SPAWN[lab]=arena;                GOALS[lab]=workshop,robot_arm
SPAWN[warehouse]=dock_north;     GOALS[warehouse]=staging,dock_south
SPAWN[keep]=hall;                GOALS[keep]=hall_carpet,hall_statue

# Known failures: name -> reason (see the lane notes). They are reported XFAIL, not FAIL.
declare -A XFAIL_WHY
XFAIL_WHY[auv_lake_mission]="depth/heading controller unstable on the realistic model (docs/notes/sea.md, S3)"
XFAIL_WHY[world_lidar_keep_slamtec_rplidar_c1]="12 m lidar cannot reach 30% beam hits in the 20 m wide keep lanes (ROADMAP.md)"

TESTS=()
add() { TESTS+=("$1"); }
add colcon_test
for w in $WORLDS; do add "world_check_$w"; done
add ready_wheeled_round_apartment
add ready_legged_hound_lab
add ready_auv_tank
add ready_drone_wyvern_lab
if [ "$MODE" = full ]; then
  for w in $WORLDS; do for r in $WHEELED $LEGGED; do add "nav2_${r}_$w"; done; done
  for w in $WORLDS; do add "world_lidar_${w}"; done
  add auv_lake_mission
  for w in lab apartment warehouse keep; do add "wyvern_mission_$w"; done
  add slam_round_apartment
  add two_robot_namespaced
fi

if [ "$LIST" = 1 ]; then printf '%s\n' "${TESTS[@]}"; exit 0; fi

# ---------------------------------------------------------------- helpers
record() {  # name status seconds note
  local name=$1 st=$2 secs=$3 note=${4//$'\t'/ }
  note=${note//$'\n'/ }
  if [ "$st" = FAIL ] && [ -n "${XFAIL_WHY[$name]:-}" ]; then st=XFAIL; note="$note [known: ${XFAIL_WHY[$name]}]"; fi
  grep -v -P "^\Q$name\E\t" "$RESULTS_TSV" > "$RESULTS_TSV.tmp" || true
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$name" "$st" "$secs" "$MODE" "$(date '+%F %H:%M')" "$note" >> "$RESULTS_TSV.tmp"
  mv "$RESULTS_TSV.tmp" "$RESULTS_TSV"
  printf '[%s] %-44s %-5s %5ss  %s\n' "$(date +%H:%M:%S)" "$name" "$st" "$secs" "$note"
}

render_md() {
  python3 - "$RESULTS_TSV" "$RESULTS_MD" "$ROS_DOMAIN_ID" "$GZ_PARTITION" <<'PY'
import sys, collections
tsv, md, dom, part = sys.argv[1:5]
rows = [l.rstrip("\n").split("\t") for l in open(tsv) if l.strip()]
cnt = collections.Counter(r[1] for r in rows)
out = ["# Test results", "",
       "Written by `scripts/run_all_tests.sh` (rows of earlier runs are kept; each row has the date of its last run).",
       "", "Summary: " + ", ".join("%d %s" % (v, k) for k, v in sorted(cnt.items())), "",
       "| test | status | seconds | mode | date | note |", "|---|---|---|---|---|---|"]
for r in rows:
    r += [""] * (6 - len(r))
    out.append("| %s | %s | %s | %s | %s | %s |" % tuple(x.replace("|", "/") for x in r[:6]))
open(md, "w").write("\n".join(out) + "\n")
PY
}

SIM_PGID=""
stop_sim() {
  [ -n "$SIM_PGID" ] || return 0
  kill -INT -- "-$SIM_PGID" 2>/dev/null
  for _ in $(seq 12); do kill -0 -- "-$SIM_PGID" 2>/dev/null || break; sleep 1; done
  kill -9 -- "-$SIM_PGID" 2>/dev/null
  SIM_PGID=""
  sleep 2
}
trap 'stop_sim; render_md' EXIT
trap 'echo "interrupted"; exit 130' INT TERM

# start_sim LOG cmd... : starts cmd as its own process group (through $SIM_WRAPPER), sets SIM_PGID
start_sim() {
  local log=$1; shift
  # shellcheck disable=SC2086
  setsid bash -c 'echo $$ > "$0"; exec "$@"' "$RUN_DIR/pid" ${SIM_WRAPPER:-} "$@" > "$log" 2>&1 &
  disown $! 2>/dev/null
  for _ in $(seq 20); do [ -s "$RUN_DIR/pid" ] && break; sleep 0.2; done
  SIM_PGID=$(ps -o pgid= -p "$(cat "$RUN_DIR/pid")" | tr -d ' ')
  rm -f "$RUN_DIR/pid"
}

# wait_for LOG REGEX SECONDS : returns 0 when REGEX shows up in LOG, 1 on timeout or when the sim died
wait_for() {
  local log=$1 re=$2 t=$3 i=0
  while [ $i -lt "$t" ]; do
    grep -q -E "$re" "$log" 2>/dev/null && return 0
    kill -0 -- "-$SIM_PGID" 2>/dev/null || return 1
    sleep 2; i=$((i + 2))
  done
  return 1
}

# wait_topic TOPIC SECONDS : first message of TOPIC arrived (retried until the deadline)
wait_topic() {
  local end=$((SECONDS + $2))
  while [ $SECONDS -lt $end ]; do
    timeout 15 ros2 topic echo --once "$1" >> "$RUN_DIR/wait_topic.log" 2>&1 && return 0
    kill -0 -- "-$SIM_PGID" 2>/dev/null || return 1
    sleep 2
  done
  return 1
}

rtf_sample() {  # three gz /stats readings, 20 s apart, after the sim has settled
  local out=""
  for _ in 1 2 3; do
    out="$out $(timeout 10 gz topic -e -t /stats -n 1 2>/dev/null | grep -m1 real_time_factor | awk '{printf "%.2f", $2}')"
    sleep 20
  done
  echo "$out"
}

selected() { [ -z "$ONLY" ] || [[ $1 =~ $ONLY ]]; }

# ---------------------------------------------------------------- tests
t_colcon_test() {
  local t0=$SECONDS
  ( cd "$WS" && timeout 1500 colcon test --event-handlers console_cohesion- > "$RUN_DIR/colcon_test.log" 2>&1 )
  local rc=$?
  ( cd "$WS" && colcon test-result > "$RUN_DIR/colcon_test_result.log" 2>&1 )
  local sum; sum=$(tail -n 1 "$RUN_DIR/colcon_test_result.log")
  local fails; fails=$( (cd "$WS" && colcon test-result 2>/dev/null | grep -E ", ([1-9][0-9]* (errors|failures?)|0 errors, [1-9][0-9]* failures?)," | sed 's|^build/\([^/]*\)/.*|\1|' | sort | uniq -c | tr -s ' ' | tr '\n' ';') )
  if [ $rc -eq 0 ] && ! grep -q -E "[1-9][0-9]* (failures|errors)" <<<"$sum"; then
    record colcon_test PASS $((SECONDS - t0)) "$sum"
  else
    record colcon_test FAIL $((SECONDS - t0)) "rc=$rc $sum $fails (log $RUN_DIR/colcon_test.log)"
  fi
}

t_world_check() {  # world
  local w=$1 t0=$SECONDS out
  out=$(cd "$WS" && timeout 300 python3 tools/check_world.py "$w" 2>&1); local rc=$?
  echo "$out" > "$RUN_DIR/world_check_$w.log"
  if [ $rc -eq 0 ]; then
    record "world_check_$w" PASS $((SECONDS - t0)) "$(grep -m1 'free area' <<<"$out" | cut -c1-90)"
  else
    record "world_check_$w" FAIL $((SECONDS - t0)) "$(grep -m3 'BAD' <<<"$out" | tr -s ' ' | tr '\n' ';' | cut -c1-200)"
  fi
}

t_world_lidar() {  # world: one row per distinct lidar of robots.yaml, names world_lidar_<w>_<stem>
  local w=$1 t0=$SECONDS out stem rc
  for stem in $(python3 - <<PY
import sys
sys.path.insert(0, "$WS/tools")
import check_world
print(" ".join(check_world.catalog_lidars()))
PY
  ); do
    selected "world_lidar_${w}_$stem" || continue
    t0=$SECONDS
    out=$(cd "$WS" && timeout 1800 python3 tools/check_world.py "$w" --lidar --lidar-step 2 --lidar-model "$stem" 2>&1); rc=$?
    echo "$out" > "$RUN_DIR/world_lidar_${w}_$stem.log"
    record "world_lidar_${w}_$stem" "$([ $rc -eq 0 ] && echo PASS || echo FAIL)" $((SECONDS - t0)) "$(grep 'hit fraction' <<<"$out" | tail -n 1 | cut -c1-140)"
  done
}

t_ready() {  # name ready_regex ready_timeout topic -- launch command...
  local name=$1 re=$2 tmo=$3 topic=$4; shift 5
  local t0=$SECONDS log="$RUN_DIR/$name.log"
  start_sim "$log" "$@"
  if wait_for "$log" "$re" "$tmo" && { [ -z "$topic" ] || wait_topic "$topic" 90; }; then
    record "$name" PASS $((SECONDS - t0)) "ready: /$re/ ${topic:+and a message on $topic}"
  else
    record "$name" FAIL $((SECONDS - t0)) "not ready within ${tmo}s (log $log): $(grep -m1 -E 'ERROR|Traceback|died' "$log" | cut -c1-120)"
  fi
  stop_sim
}

t_nav2() {  # robot world [extra launch args...]; name from $NAME
  local r=$1 w=$2; shift 2
  local name=${NAME:-nav2_${r}_$w} t0=$SECONDS log="$RUN_DIR/${NAME:-nav2_${r}_$w}.log"
  local tmo=240; case $r in wyrm|hound|spider) tmo=360;; esac
  start_sim "$log" ros2 launch fleet_bringup sim.launch.py robot:="$r" world:="$w" spawn:="${SPAWN[$w]}" nav:=true gui:=false rviz:=false "$@"
  if ! wait_for "$log" "Managed nodes are active" 240; then
    record "$name" FAIL $((SECONDS - t0)) "Nav2 never became active (log $log): $(grep -m1 -E 'ERROR|Traceback|died' "$log" | cut -c1-120)"
    stop_sim; return
  fi
  sleep 5
  ( sleep 40; rtf_sample > "$RUN_DIR/$name.rtf" ) &
  local rtfpid=$!
  timeout $((2 * tmo + 120)) ros2 run fleet_bringup patrol.py --world "$w" --goals "${GOALS[$w]}" --timeout "$tmo" > "$RUN_DIR/$name.patrol.log" 2>&1
  local rc=$?
  wait $rtfpid 2>/dev/null
  local summ; summ=$(grep '\[patrol\]' "$RUN_DIR/$name.patrol.log" | tr -s ' ' | tr '\n' ';' | cut -c1-220)
  record "$name" "$([ $rc -eq 0 ] && echo PASS || echo FAIL)" $((SECONDS - t0)) "rc=$rc rtf=[$(cat "$RUN_DIR/$name.rtf" 2>/dev/null)] $summ"
  stop_sim
}

t_auv_mission() {
  local name=auv_lake_mission t0=$SECONDS log="$RUN_DIR/auv_lake_mission.log"
  start_sim "$log" ros2 launch fleet_auv auv.launch.py world:=keep controller:=true mission:=true gui:=false rviz:=false
  # the mission node exits 0 on success, 1 on failure; launch reports either in its log
  if wait_for "$log" "auv_mission-[0-9]+\]: process (has finished cleanly|has died)" 1500; then
    if grep -q -E "auv_mission-[0-9]+\]: process has finished cleanly" "$log"; then
      record "$name" PASS $((SECONDS - t0)) "mission completed (log $log)"
    else
      record "$name" FAIL $((SECONDS - t0)) "mission node failed: $(grep -m1 -E 'RESULT|reason' "$log" | cut -c1-140)"
    fi
  else
    record "$name" FAIL $((SECONDS - t0)) "mission did not finish in 1500 s"
  fi
  stop_sim
}

t_wyvern() {  # world
  local w=$1 name="wyvern_mission_$1" t0=$SECONDS log="$RUN_DIR/wyvern_mission_$1.log"
  local res="$RUN_DIR/wyvern_$w.json"
  start_sim "$log" ros2 launch fleet_drone drone.launch.py world:="$w" mission:=true \
      shutdown_on_done:=true result_file:="$res" gui:=false rviz:=false
  local i=0
  while [ $i -lt 1800 ] && kill -0 -- "-$SIM_PGID" 2>/dev/null; do sleep 5; i=$((i + 5)); done
  if [ -f "$res" ] && python3 -c "import json,sys; sys.exit(0 if json.load(open('$res'))['success'] else 1)"; then
    record "$name" PASS $((SECONDS - t0)) "$(python3 -c "import json; r=json.load(open('$res')); print({k: r[k] for k in r if k in ('reason','duration','max_err')})" 2>/dev/null | cut -c1-140)"
  else
    record "$name" FAIL $((SECONDS - t0)) "$( [ -f "$res" ] && python3 -c "import json; print(json.load(open('$res')).get('reason'))" || echo "no result file, mission did not finish in 1800 s")"
  fi
  stop_sim
}

t_slam() {
  local name=slam_round_apartment t0=$SECONDS log="$RUN_DIR/slam_round_apartment.log"
  start_sim "$log" ros2 launch fleet_bringup sim.launch.py robot:=round world:=apartment spawn:="${SPAWN[apartment]}" nav:=true slam:=true gui:=false rviz:=false
  if ! wait_for "$log" "Managed nodes are active" 240; then
    record "$name" FAIL $((SECONDS - t0)) "Nav2 never became active (log $log)"; stop_sim; return
  fi
  sleep 5
  timeout 900 ros2 run fleet_bringup patrol.py --world apartment --goals "${GOALS[apartment]}" --timeout 240 > "$RUN_DIR/$name.patrol.log" 2>&1
  local rc=$?
  timeout 60 ros2 run nav2_map_server map_saver_cli -f "$RUN_DIR/slam_map" --ros-args -p save_map_timeout:=20.0 > "$RUN_DIR/$name.save.log" 2>&1
  local rc2=$?
  if [ $rc -eq 0 ] && [ $rc2 -eq 0 ] && [ -s "$RUN_DIR/slam_map.pgm" ]; then
    record "$name" PASS $((SECONDS - t0)) "2/2 goals with slam_toolbox, map saved"
  else
    record "$name" FAIL $((SECONDS - t0)) "patrol rc=$rc, map_saver rc=$rc2"
  fi
  stop_sim
}

# ---------------------------------------------------------------- run
for name in "${TESTS[@]}"; do
  case $name in world_lidar_*) selected "$name" || [[ $ONLY == world_lidar* ]] || continue;; *) selected "$name" || continue;; esac
  case $name in
    colcon_test) t_colcon_test;;
    world_check_*) t_world_check "${name#world_check_}";;
    world_lidar_*) t_world_lidar "${name#world_lidar_}";;
    ready_wheeled_round_apartment)
      t_ready "$name" "Managed nodes are active" 240 /odometry/filtered -- ros2 launch fleet_bringup sim.launch.py robot:=round world:=apartment spawn:="${SPAWN[apartment]}" nav:=true gui:=false rviz:=false;;
    ready_legged_hound_lab)
      t_ready "$name" "Managed nodes are active" 240 /odometry/filtered -- ros2 launch fleet_bringup sim.launch.py robot:=hound world:=lab spawn:="${SPAWN[lab]}" nav:=true gui:=false rviz:=false;;
    ready_auv_tank)
      t_ready "$name" "process started" 120 /knucker/odometry/filtered -- ros2 launch fleet_auv auv.launch.py world:=tank gui:=false rviz:=false;;
    ready_drone_wyvern_lab)
      t_ready "$name" "Ready for takeoff" 240 /fmu/out/vehicle_odometry -- ros2 launch fleet_drone drone.launch.py world:=lab mission:=false gui:=false rviz:=false;;
    nav2_*) rest=${name#nav2_}; t_nav2 "${rest%%_*}" "${rest#*_}";;
    auv_lake_mission) t_auv_mission;;
    wyvern_mission_*) t_wyvern "${name#wyvern_mission_}";;
    slam_round_apartment) t_slam;;
    two_robot_namespaced)
      if grep -q "'namespace'" "$WS/src/fleet_bringup/launch/sim.launch.py"; then
        record "$name" SKIP 0 "namespace argument exists but no two-robot test is written yet"
      else
        record "$name" SKIP 0 "sim.launch.py has no namespace argument yet (only fleet_auv has one)"
      fi;;
  esac
done

render_md
echo
echo "results: $RESULTS_MD   logs: $RUN_DIR"
if awk -F'\t' '$2=="FAIL"{f=1} END{exit !f}' "$RESULTS_TSV"; then
  echo "FAILURES:"; awk -F'\t' '$2=="FAIL"{print "  " $1 ": " $6}' "$RESULTS_TSV"; exit 1
fi
exit 0
