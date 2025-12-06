#!/bin/bash
set -euo pipefail

source_if_exists() {
  local file="$1"
  if [ -f "$file" ]; then
    # Some ROS setup files expect nounset to be disabled.
    set +u
    # shellcheck disable=SC1090
    source "$file"
    set -u
  fi
}

# Source the base ROS 2 installation so ros2 CLI is available.
source_if_exists "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

# Pick up overlays from the colcon workspace if it exists.
source_if_exists /ws/install/setup.bash

LOG_DIR=${ROS_LOG_DIR:-/root/.ros}
mkdir -p "${LOG_DIR}"
FOXGLOVE_LOG="${LOG_DIR}/foxglove_bridge.log"

echo "Launching foxglove_bridge (logs: ${FOXGLOVE_LOG})"
ros2 launch foxglove_bridge foxglove_bridge_launch.xml >"${FOXGLOVE_LOG}" 2>&1 &
FOXGLOVE_PID=$!

cleanup() {
  if kill -0 "${FOXGLOVE_PID}" >/dev/null 2>&1; then
    kill "${FOXGLOVE_PID}" >/dev/null 2>&1 || true
    wait "${FOXGLOVE_PID}" || true
  fi
}
trap cleanup EXIT SIGINT SIGTERM

if [ "$#" -gt 0 ]; then
  "$@"
else
  tail -f /dev/null
fi
