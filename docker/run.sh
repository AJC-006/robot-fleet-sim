#!/usr/bin/env bash
# Run a command in the robot-fleet-sim container, with this workspace mounted at the same path as on the host.
#
#   docker/run.sh                    # interactive shell (ROS and, once built, the workspace are sourced)
#   docker/run.sh ./setup.sh         # first time: rosdep check, uXRCE-DDS agent, colcon build
#   docker/run.sh ros2 launch fleet_bringup sim.launch.py robot:=round world:=apartment gui:=false rviz:=false
#   docker/run.sh --gui rviz2        # with the host's X11 display (Linux; run `xhost +local:` first)
#
# The image is built on first use. Environment: IMAGE (default robot-fleet-sim), REBUILD=1 rebuilds it,
# WITH_PX4=1 builds it with PX4 SITL. ROS_DOMAIN_ID and GZ_PARTITION are passed through.

set -o pipefail
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE=${IMAGE:-robot-fleet-sim}
GUI=0
if [ "${1:-}" = "--gui" ]; then GUI=1; shift; fi

command -v docker >/dev/null || { echo "docker is not installed: https://docs.docker.com/get-docker/"; exit 1; }
if [ "${REBUILD:-0}" = 1 ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "building $IMAGE (once; 10-20 min, about 6 GB)"
  docker build -f "$WS/docker/Dockerfile" -t "$IMAGE" ${WITH_PX4:+--build-arg WITH_PX4=1} "$WS" || exit 1
fi

# HOME inside the workspace keeps the rosdep cache and shell history between runs (git-ignored)
mkdir -p "$WS/.docker-home"
args=(--rm --net=host --shm-size=2g -v "$WS:$WS" -w "$WS" -e HOME="$WS/.docker-home"
      -e ROS_DOMAIN_ID -e GZ_PARTITION)
# on Linux, run as you so files written to the workspace stay yours
[ "$(uname -s)" = Linux ] && args+=(--user "$(id -u):$(id -g)")
# GPU rendering for the camera and lidar sensors when the host has a DRI device; software rendering otherwise
if [ -d /dev/dri ]; then
  args+=(--device /dev/dri)
  for d in /dev/dri/renderD* /dev/dri/card*; do [ -e "$d" ] && args+=(--group-add "$(stat -c %g "$d")"); done
fi
[ $GUI = 1 ] && args+=(-e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix)
[ -t 0 ] && [ -t 1 ] && args+=(-it)
[ $# -eq 0 ] && set -- bash

exec docker run "${args[@]}" "$IMAGE" bash -c \
  'source /opt/ros/jazzy/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; exec "$@"' bash "$@"
