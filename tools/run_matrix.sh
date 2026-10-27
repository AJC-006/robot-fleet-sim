#!/bin/bash
# tools/run_matrix.sh [robots...] -- worlds via $WORLDS : sequential 2-goal patrol per robot x world.
WS=$(cd "$(dirname "$0")/.." && pwd)
M=${M:-$WS/log/matrix}; mkdir -p "$M/runs"
export OUT=$M/runs ROS_DOMAIN_ID=82 GZ_PARTITION=matrix
ROBOTS=${@:-round box rover truck wyrm hound spider}
declare -A SP GL
SP[apartment]=master_bedroom; GL[apartment]=kids_room,ensuite
SP[lab]=arena;                GL[lab]=workshop,robot_arm
SP[warehouse]=dock_north;     GL[warehouse]=staging,dock_south
SP[keep]=hall;                GL[keep]=hall_carpet,hall_statue
for w in ${WORLDS:-apartment lab warehouse keep}; do for r in $ROBOTS; do
  TMO=240; case $r in wyrm|spider|hound) TMO=360;; esac
  [ -f $OUT/${r}_$w.done ] && continue
  "$WS/tools/run_combo.sh" $r $w ${SP[$w]} ${GL[$w]} $TMO 2>&1 | grep -v "Aborted\|terminate called" | tee -a $M/matrix_results.txt
  touch $OUT/${r}_$w.done; sleep 3
done; done
