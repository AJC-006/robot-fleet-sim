#!/usr/bin/env bash
# System packages for robot-fleet-sim (ROS 2 Jazzy + Gazebo Harmonic + Nav2 + gz_ros2_control).
#
#   sudo bash scripts/install_deps.sh      # from the workspace root
#   bash scripts/install_deps.sh --list    # print the package list and exit (no sudo; setup.sh uses it)
#
# ./setup.sh runs this when packages are missing, after adding the ROS 2 and Gazebo apt sources. Run alone, it
# assumes both sources are configured (ROS 2 Jazzy install guide; packages.osrfoundation.org for python3-gz-*).
# Environment: LOG=<file> (default scripts/install_deps.log), SKIP_ROSDEP=1 (skip the rosdep install step).
#
# Not installed here (optional): Blender 5.x for rebuilding models (blender/), PX4-Autopilot and
# Micro-XRCE-DDS-Agent for the wyvern (see scripts/install_xrce_agent.sh and docs/notes/drone.md).
#
# Never stops on a failure: tries everything in one apt transaction first, then package by package
# if that fails, and prints a summary of what is missing. Safe to re-run.

set -uo pipefail
export DEBIAN_FRONTEND=noninteractive
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Everything the packages in src/ declare in package.xml (checked with
#   rosdep check --from-paths src --ignore-src -r)
# plus what the tools/ scripts and the test runner import. rosdep install (below) is a second net.
PKGS=(
  # navigation and localisation
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander
  ros-jazzy-slam-toolbox ros-jazzy-robot-localization
  # Gazebo Harmonic (gz-sim 8) and the ROS bridge
  ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
  ros-jazzy-gz-sim-vendor ros-jazzy-gz-math-vendor ros-jazzy-gz-msgs-vendor
  ros-jazzy-gz-transport-vendor ros-jazzy-gz-common-vendor ros-jazzy-gz-plugin-vendor
  ros-jazzy-gz-physics-vendor
  # ros2_control in Gazebo
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control
  ros-jazzy-controller-manager ros-jazzy-diff-drive-controller ros-jazzy-joint-state-broadcaster
  ros-jazzy-position-controllers
  # model description, messages
  ros-jazzy-xacro ros-jazzy-urdf ros-jazzy-robot-state-publisher
  ros-jazzy-marine-acoustic-msgs ros-jazzy-visualization-msgs ros-jazzy-nav-msgs
  ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-lifecycle-msgs
  ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime
  # tools, lint and test
  ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard ros-jazzy-rviz2
  ros-jazzy-rqt-robot-steering ros-jazzy-topic-tools liburdfdom-tools
  ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common ros-jazzy-ament-cmake-pytest
  ros-jazzy-ament-cmake-cpplint ros-jazzy-ament-cmake-flake8 ros-jazzy-ament-cmake-pep257
  ros-jazzy-ament-cmake-lint-cmake ros-jazzy-ament-cmake-xmllint ros-jazzy-ament-cmake-python
  python3-colcon-common-extensions python3-rosdep
  python3-numpy python3-scipy python3-matplotlib python3-yaml python3-pytest
  # Gazebo Python bindings (packages.osrfoundation.org): tests, walk_test.py, gz_capture.py
  python3-gz-sim8 python3-gz-transport13 python3-gz-msgs10 python3-gz-math7
)

if [ "${1:-}" = "--list" ]; then printf '%s\n' "${PKGS[@]}"; exit 0; fi
[ "$EUID" -eq 0 ] || { echo "run with sudo"; exit 1; }

LOG=${LOG:-$HERE/install_deps.log}
exec > >(tee -a "$LOG") 2>&1
echo "=================== install_deps $(date '+%F %T') ==================="

FAILED=()
step() {
  local desc=$1; shift
  echo ">>> $desc"
  "$@" || { echo ">>> FAILED: $desc"; FAILED+=("$desc"); return 1; }
}

step "apt update" apt-get update
if ! step "install all packages" apt-get install -y "${PKGS[@]}"; then
  echo ">>> retrying one package at a time"
  for p in "${PKGS[@]}"; do step "install $p" apt-get install -y "$p"; done
fi

# second net: whatever package.xml lists that the explicit list above missed
SRC=$(cd "$HERE/../src" && pwd)
if [ "${SKIP_ROSDEP:-0}" != 1 ] && command -v rosdep >/dev/null; then
  step "rosdep install (package.xml keys)" rosdep install --from-paths "$SRC" --ignore-src -y -r
fi

# ---------------------------------------------------------------- check
for f in nav2_bringup slam_toolbox gz_ros2_control diff_drive_controller robot_localization; do
  step "$f present" test -d "/opt/ros/jazzy/share/$f"
done

echo
echo "=================== summary ==================="
if [ ${#FAILED[@]} -eq 0 ]; then
  echo "all steps OK"
else
  echo "${#FAILED[@]} step(s) failed:"; printf '  - %s\n' "${FAILED[@]}"
fi
echo "log: $LOG"
