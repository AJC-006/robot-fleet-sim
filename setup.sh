#!/usr/bin/env bash
# One-step setup for robot-fleet-sim on Ubuntu 24.04 (noble) or a derivative (Pop!_OS, Mint 22, ...).
# Other systems (other Linux, macOS, Windows/WSL): use the container, see docker/README.md.
#
#   ./setup.sh               # ROS 2 Jazzy if missing, apt dependencies, rosdep, uXRCE-DDS agent, colcon build
#   ./setup.sh --px4         # also clone and build PX4-Autopilot SITL for the wyvern drone
#   ./setup.sh --help
#
# Every step is skipped when its result is already there, so re-running is cheap. Never stops on a failure: it
# runs every step it can and ends with a summary of what failed. Output is also written to setup.log.
# sudo is used only for apt (and asked for once, up front); as root it is not needed.

set -o pipefail
export DEBIAN_FRONTEND=noninteractive

WS=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$WS" || exit 1
LOG="$WS/setup.log"
# PX4 commit that src/px4_msgs was generated from (see README "Install")
PX4_COMMIT=e146fe4cc69459493196b172d9b4576dc4bce127
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}

WITH_PX4=0 WITH_DRONE=1 WITH_BUILD=1 UPGRADE=0
usage() {
  cat <<EOF
usage: ./setup.sh [--px4] [--no-drone] [--no-build] [--upgrade]

  --px4        also clone PX4-Autopilot into \$PX4_DIR (now: $PX4_DIR), check out the pinned commit and build
               px4_sitl_default (about 20 min and 10 GB; runs PX4's own Tools/setup/ubuntu.sh, which uses sudo)
  --no-drone   skip the Micro-XRCE-DDS agent build (only the wyvern drone needs it)
  --no-build   install dependencies only, do not run colcon build
  --upgrade    bring every installed ROS 2 Jazzy and Gazebo package up to the current release (sudo); use it when
               ROS 2 was installed a while ago and not updated since. Done automatically whenever apt installs
               something, because mixing old and new ROS packages can fail at run time with undefined symbols.
EOF
}
for a in "$@"; do
  case $a in
    --px4) WITH_PX4=1 ;;
    --no-drone) WITH_DRONE=0 ;;
    --no-build) WITH_BUILD=0 ;;
    --upgrade) UPGRADE=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $a"; usage; exit 2 ;;
  esac
done

exec > >(tee -a "$LOG") 2>&1
echo "=================== setup $(date '+%F %T') ==================="

FAILED=()
step() {
  local desc=$1; shift
  echo ">>> $desc"
  "$@" || { echo ">>> FAILED: $desc"; FAILED+=("$desc"); return 1; }
}
installed() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"; }

# ---------------------------------------------------------------- system check
. /etc/os-release 2>/dev/null
CODENAME=${UBUNTU_CODENAME:-$VERSION_CODENAME}
if [ "$CODENAME" != noble ] || ! command -v apt-get >/dev/null; then
  echo "This system is ${PRETTY_NAME:-unknown}. ROS 2 Jazzy binaries need Ubuntu 24.04 (noble) or a derivative."
  echo "Use the container instead: docker/README.md (docker/run.sh ./setup.sh)."
  exit 1
fi
ARCH=$(dpkg --print-architecture)
case $ARCH in amd64|arm64) ;; *) echo "ROS 2 Jazzy has no binaries for $ARCH"; exit 1 ;; esac
echo "system: $PRETTY_NAME ($ARCH), workspace: $WS"

# Start from a clean ROS environment: another distribution or workspace already sourced in this shell (say from
# ~/.bashrc) would leak into rosdep and the build. Only this script's environment changes, not your shell's.
if [ -n "${ROS_DISTRO:-}${AMENT_PREFIX_PATH:-}${COLCON_PREFIX_PATH:-}" ]; then
  echo "note: this shell has ROS sourced (${ROS_DISTRO:-?}: ${AMENT_PREFIX_PATH:-}${COLCON_PREFIX_PATH:+ $COLCON_PREFIX_PATH}); setup.sh uses only /opt/ros/jazzy"
  clean_path=
  IFS=: read -ra parts <<< "$PATH"
  for d in "${parts[@]}"; do
    case $d in /opt/ros/*) continue ;; esac
    for pre in ${AMENT_PREFIX_PATH//:/ } ${COLCON_PREFIX_PATH//:/ }; do [ "$d" = "$pre/bin" ] && continue 2; done
    clean_path=${clean_path:+$clean_path:}$d
  done
  PATH=$clean_path
  unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ETC_DIR AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX \
        CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH GZ_SIM_RESOURCE_PATH \
        GZ_SIM_SYSTEM_PLUGIN_PATH GZ_CONFIG_PATH GZ_VERSION
fi

# sudo only when not root; ask for the password once, here, rather than in the middle of a long step
if [ "$EUID" -eq 0 ]; then SUDO=(); else SUDO=(sudo); fi
need_root() {
  [ "$EUID" -eq 0 ] && return 0
  sudo -n true 2>/dev/null && return 0
  if ! command -v sudo >/dev/null; then echo "sudo is not installed; run this step as root"; return 1; fi
  if [ -t 0 ]; then sudo -v; else echo "sudo needs a password and there is no terminal; run ./setup.sh in a terminal"; return 1; fi
}

# ---------------------------------------------------------------- 1. apt sources: ROS 2 and Gazebo (OSRF)
# ROS: the ros2-apt-source package, as in the official Jazzy install guide.
# Gazebo: packages.osrfoundation.org, for the Gazebo Python bindings (python3-gz-*) the tests and tools import.
add_ros_source() {
  "${SUDO[@]}" apt-get update && "${SUDO[@]}" apt-get install -y curl ca-certificates software-properties-common || return 1
  "${SUDO[@]}" add-apt-repository -y universe || return 1
  local ver deb
  ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        grep -F '"tag_name"' | awk -F'"' '{print $4}')
  [ -n "$ver" ] || { echo "could not read the ros-apt-source release from GitHub"; return 1; }
  deb=$(mktemp --suffix=.deb)
  curl -fsSL -o "$deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$ver/ros2-apt-source_${ver}.${CODENAME}_all.deb" &&
    "${SUDO[@]}" apt-get install -y "$deb"
  local rc=$?; rm -f "$deb"; return $rc
}
add_gazebo_source() {
  "${SUDO[@]}" curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg || return 1
  echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $CODENAME main" |
    "${SUDO[@]}" tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
}
has_source() { grep -RqsE "$1" /etc/apt/sources.list /etc/apt/sources.list.d/; }

ROOT_OK=1
if ! has_source 'packages.ros.org/ros2' || ! has_source 'packages.osrfoundation.org/gazebo' || [ ! -f /opt/ros/jazzy/setup.bash ]; then
  step "root access for apt" need_root || ROOT_OK=0
fi
if [ $ROOT_OK = 1 ]; then
  has_source 'packages.ros.org/ros2' && echo ">>> ROS 2 apt source: present" || step "add the ROS 2 apt source" add_ros_source
  has_source 'packages.osrfoundation.org/gazebo' && echo ">>> Gazebo apt source: present" || step "add the Gazebo (OSRF) apt source" add_gazebo_source
fi

# ---------------------------------------------------------------- 2. ROS 2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
  echo ">>> ROS 2 Jazzy: present (/opt/ros/jazzy)"
elif [ $ROOT_OK = 1 ]; then
  step "apt update" "${SUDO[@]}" apt-get update
  step "install ROS 2 Jazzy (ros-jazzy-desktop, ros-dev-tools)" \
    "${SUDO[@]}" apt-get install -y ros-jazzy-desktop ros-dev-tools git cmake build-essential
fi

# ---------------------------------------------------------------- 3. project apt packages
# Installed ROS 2 Jazzy and Gazebo packages. ROS debs do not pin each other's versions, so installing new packages
# next to an old installation can mix releases; they are upgraded together whenever apt runs.
ros_installed() {
  dpkg-query -W -f='${Package} ${Status}\n' 'ros-jazzy-*' 'gz-*' 'libgz-*' 'python3-gz-*' 'libsdformat*' 2>/dev/null |
    awk '/ install ok installed$/ {print $1}'
}
upgrade_ros() {
  local pk; mapfile -t pk < <(ros_installed)
  [ ${#pk[@]} -gt 0 ] || return 0
  "${SUDO[@]}" apt-get update && "${SUDO[@]}" apt-get install -y --only-upgrade "${pk[@]}"
}
mapfile -t PKGS < <(bash "$WS/scripts/install_deps.sh" --list)
MISSING=()
for p in "${PKGS[@]}"; do installed "$p" || MISSING+=("$p"); done
if [ ${#MISSING[@]} -eq 0 ]; then
  echo ">>> project apt packages: all ${#PKGS[@]} present"
else
  echo ">>> ${#MISSING[@]} of ${#PKGS[@]} project apt packages missing: ${MISSING[*]}"
fi
if [ ${#MISSING[@]} -gt 0 ] || [ $UPGRADE = 1 ]; then
  if step "root access for apt" need_root; then
    step "upgrade installed ROS 2 Jazzy and Gazebo packages to the current release" upgrade_ros
    [ ${#MISSING[@]} -gt 0 ] && step "install project apt packages (scripts/install_deps.sh)" \
      "${SUDO[@]}" env LOG="$WS/scripts/install_deps.log" SKIP_ROSDEP=1 bash "$WS/scripts/install_deps.sh"
  fi
else
  # no root needed: compare with the package lists apt last downloaded
  n=$(apt list --upgradable 2>/dev/null | grep -cE '^(ros-jazzy-|gz-|libgz-|python3-gz-|libsdformat)')
  lists_age=$(( ($(date +%s) - $(stat -c %Y /var/cache/apt/pkgcache.bin 2>/dev/null || date +%s)) / 86400 ))
  if [ "$n" -gt 0 ]; then
    echo ">>> note: $n installed ROS 2 / Gazebo package(s) have updates; ./setup.sh --upgrade installs them"
  elif [ "$lists_age" -gt 30 ]; then
    echo ">>> note: apt last updated its package lists $lists_age days ago, so updates may be missing; ./setup.sh --upgrade checks"
  fi
fi

# ROS setup files read unset variables, so no `set -u` in this script
if [ -f /opt/ros/jazzy/setup.bash ]; then
  . /opt/ros/jazzy/setup.bash
else
  FAILED+=("ROS 2 Jazzy is not installed; the steps below were skipped")
  WITH_DRONE=0 WITH_BUILD=0
fi

# ---------------------------------------------------------------- 4. rosdep (the package.xml keys)
if command -v rosdep >/dev/null; then
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    step "root access for rosdep init" need_root && step "rosdep init" "${SUDO[@]}" rosdep init
  fi
  step "rosdep update" rosdep update --rosdistro jazzy
  if rosdep check --from-paths "$WS/src" --ignore-src -r --rosdistro jazzy >/dev/null 2>&1; then
    echo ">>> rosdep: all package.xml dependencies satisfied"
  elif step "root access for rosdep install" need_root; then
    step "rosdep install" rosdep install --from-paths "$WS/src" --ignore-src -r -y --rosdistro jazzy
  fi
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  FAILED+=("rosdep not found (python3-rosdep)")
fi

# ---------------------------------------------------------------- 5. Micro-XRCE-DDS agent (wyvern drone)
if [ $WITH_DRONE = 1 ]; then
  step "Micro-XRCE-DDS agent (scripts/install_xrce_agent.sh)" bash "$WS/scripts/install_xrce_agent.sh"
  [ -x "$WS/.deps/xrce_agent/bin/MicroXRCEAgent" ] || FAILED+=("Micro-XRCE-DDS agent not built; see scripts/install_xrce_agent.log")
fi

# ---------------------------------------------------------------- 6. PX4 SITL (optional)
px4_setup() {
  if [ ! -d "$PX4_DIR/.git" ]; then
    git clone https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" || return 1
  fi
  local head; head=$(git -C "$PX4_DIR" rev-parse HEAD 2>/dev/null)
  if [ "$head" != "$PX4_COMMIT" ]; then
    if [ -n "$(git -C "$PX4_DIR" status --porcelain --untracked-files=no)" ]; then
      echo "$PX4_DIR has local changes and is at ${head:0:10}, not ${PX4_COMMIT:0:10}; leaving it alone"
      echo "(another PX4 version needs src/px4_msgs regenerated from its msg/ directory)"
      return 1
    fi
    git -C "$PX4_DIR" fetch origin "$PX4_COMMIT" && git -C "$PX4_DIR" checkout "$PX4_COMMIT" || return 1
  fi
  git -C "$PX4_DIR" submodule update --init --recursive || return 1
  (cd "$PX4_DIR" && bash Tools/setup/ubuntu.sh --no-nuttx) || return 1
  make -C "$PX4_DIR" px4_sitl_default
}
if [ $WITH_PX4 = 1 ]; then
  if [ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    echo ">>> PX4 SITL: present ($PX4_DIR/build/px4_sitl_default)"
  else
    step "root access for PX4's setup script" need_root && step "PX4 SITL in $PX4_DIR" px4_setup
  fi
fi

# ---------------------------------------------------------------- 7. build
if [ $WITH_BUILD = 1 ]; then
  step "colcon build" colcon build --symlink-install
fi

# ---------------------------------------------------------------- summary
echo
echo "=================== summary ==================="
if [ ${#FAILED[@]} -eq 0 ]; then
  echo "all steps OK"
else
  echo "${#FAILED[@]} step(s) failed:"; printf '  - %s\n' "${FAILED[@]}"
fi
if [ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
  echo "PX4 SITL: $PX4_DIR"
elif [ $WITH_DRONE = 1 ]; then
  echo "PX4 SITL: not found at $PX4_DIR (only the wyvern drone needs it: ./setup.sh --px4, or set PX4_DIR)"
fi
cat <<EOF
log: $LOG

next:
  source $WS/install/setup.bash
  ros2 launch fleet_bringup sim.launch.py robot:=round world:=apartment gui:=false rviz:=false
  scripts/run_all_tests.sh --quick        # about 2 min of test time
EOF
[ ${#FAILED[@]} -eq 0 ]
