#!/bin/bash
# Run one robot x world patrol headless and print a summary line.
#   tools/run_combo.sh ROBOT WORLD SPAWN GOALS GOAL_TIMEOUT [extra launch args...]
# Env: ROS_DOMAIN_ID (default 82), GZ_PARTITION (default matrix), OUT (log dir), NAV=true, WAIT=150
ROBOT=$1; WORLD=$2; SPAWN=$3; GOALS=$4; TMO=${5:-240}; shift 5
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-82} GZ_PARTITION=${GZ_PARTITION:-matrix}
WS=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-$WS/log/matrix}; mkdir -p "$OUT"
TAG=${TAG:-${ROBOT}_${WORLD}}
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
LOG=$OUT/$TAG.launch.log; PLOG=$OUT/$TAG.patrol.log
setsid ros2 launch fleet_bringup sim.launch.py robot:=$ROBOT world:=$WORLD spawn:=$SPAWN nav:=${NAV:-true} gui:=false rviz:=false "$@" >$LOG 2>&1 &
PG=$!
echo $PG > $OUT/$TAG.pgid
cleanup(){ kill -INT -- -$PG 2>/dev/null; for i in $(seq 10); do kill -0 -- -$PG 2>/dev/null || break; sleep 1; done; kill -9 -- -$PG 2>/dev/null; }
trap cleanup EXIT
for i in $(seq ${WAIT:-150}); do grep -q "Managed nodes are active" $LOG && break; sleep 1; done
if ! grep -q "Managed nodes are active" $LOG; then echo "$TAG: LAUNCH FAILED (no active nodes in ${WAIT:-150}s)"; exit 2; fi
sleep 5
if [ -n "$GOALS" ]; then
  T0=$(date +%s)
  ( sleep 40; for k in 1 2 3; do gz topic -e -t /stats -n 1 2>/dev/null | grep -E "real_time_factor" ; sleep 20; done ) > $OUT/$TAG.rtf 2>&1 &
  ( sleep 55; uptime; ps -eo pcpu,rss,args --sort=-pcpu | head -8 | cut -c1-140 ) > $OUT/$TAG.top 2>&1 &
  python3 "$WS/tools/gt_monitor.py" --world $WORLD --robot $ROBOT --out $OUT/$TAG.gt.csv > $OUT/$TAG.gt.log 2>&1 &
  GTP=$!
  timeout ${TOTAL:-1500} ros2 run fleet_bringup patrol.py --world $WORLD --goals $GOALS --timeout $TMO > $PLOG 2>&1
  RC=$?
  kill -INT $GTP 2>/dev/null; sleep 2
  echo "$TAG: $(cat $OUT/$TAG.gt.log | tail -1)"
  echo "$TAG: rc=$RC wall=$(( $(date +%s)-T0 ))s rtf=$(grep -o '[0-9.]*' $OUT/$TAG.rtf | tr '\n' ' ')"
  grep "\[patrol\]" $PLOG
fi
