#!/usr/bin/env bash
# Micro-XRCE-DDS Agent v2.4.3 (eProsima, Apache-2.0) for the wyvern drone (fleet_drone), built from source
# against ROS 2 Jazzy's Fast-DDS / Fast-CDR. No sudo: everything goes to <workspace>/.deps.
#
#   bash scripts/install_xrce_agent.sh      # from the workspace root
#
# Result: <workspace>/.deps/xrce_agent/bin/MicroXRCEAgent (drone.launch.py finds it there; agent:= overrides).
# Needs network (GitHub tarball; the agent's superbuild also fetches Micro-CDR). Never stops on a failure; prints a
# summary and logs to scripts/install_xrce_agent.log. Safe to re-run (skips the build if the binary exists, unless
# FORCE=1).

set -uo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(dirname "$HERE")
LOG="$HERE/install_xrce_agent.log"
exec > >(tee -a "$LOG") 2>&1
echo "=================== install_xrce_agent $(date '+%F %T') ==================="

VER=2.4.3
DEPS="$WS/.deps"
SRC="$DEPS/src/Micro-XRCE-DDS-Agent-$VER"
BUILD="$DEPS/build/xrce_agent"
PREFIX="$DEPS/xrce_agent"
BIN="$PREFIX/bin/MicroXRCEAgent"

if [ -x "$BIN" ] && [ "${FORCE:-0}" != "1" ]; then
  echo "already installed: $BIN (FORCE=1 to rebuild)"
  exit 0
fi
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy not found in /opt/ros/jazzy: run scripts/install_deps.sh first"
  exit 0
fi
# shellcheck disable=SC1091
set +u; source /opt/ros/jazzy/setup.bash; set -u   # ROS setup files read unset variables
mkdir -p "$DEPS/src" "$BUILD"

ok=1
if [ ! -d "$SRC" ]; then
  echo "--- download v$VER"
  curl -fsSL "https://github.com/eProsima/Micro-XRCE-DDS-Agent/archive/refs/tags/v$VER.tar.gz" -o "$DEPS/src/agent.tar.gz" \
    && tar -xzf "$DEPS/src/agent.tar.gz" -C "$DEPS/src" || { echo "DOWNLOAD FAILED"; ok=0; }
fi
if [ $ok = 1 ]; then
  echo "--- configure"
  cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_P2P_PROFILE=OFF \
    -DUAGENT_SOCKETCAN_PROFILE=OFF -DUAGENT_BUILD_TESTS=OFF -DCMAKE_PREFIX_PATH=/opt/ros/jazzy \
    -DCMAKE_INSTALL_RPATH="$PREFIX/lib;/opt/ros/jazzy/lib" || { echo "CMAKE FAILED"; ok=0; }
fi
if [ $ok = 1 ]; then
  echo "--- build"
  cmake --build "$BUILD" -j"$(( $(nproc) > 2 ? 2 : 1 ))" || { echo "BUILD FAILED"; ok=0; }
fi
if [ $ok = 1 ]; then
  cmake --install "$BUILD" || { echo "INSTALL FAILED"; ok=0; }
fi
if [ $ok = 1 ] && [ -x "$BIN" ]; then
  echo "OK: $BIN"
else
  echo "MicroXRCEAgent NOT installed; see $LOG. drone.launch.py also accepts agent:=<binary> or XRCE_AGENT."
fi
exit 0
