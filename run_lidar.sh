#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/ws"
UGV_SETUP_HELPER="${SCRIPT_DIR}/scripts/lib/source_setup.bash"
UGV_ROS_DISTRO="humble"
UGV_ROS_SETUP="/opt/ros/${UGV_ROS_DISTRO}/setup.bash"

if [[ ! -f "${UGV_SETUP_HELPER}" ]]; then
    echo "ERROR: Setup-source helper not found: ${UGV_SETUP_HELPER}" >&2
    exit 1
fi
source "${UGV_SETUP_HELPER}"

if [[ ! -f "${UGV_ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 Humble was not found at ${UGV_ROS_SETUP}" >&2
    exit 1
fi
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    echo "ERROR: Built workspace setup not found: ${WORKSPACE_DIR}/install/setup.bash" >&2
    echo "Run ./build_ws.sh first." >&2
    exit 1
fi
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${UGV_ROS_DISTRO}" ]]; then
    echo "ERROR: ROS_DISTRO is already '${ROS_DISTRO}', but this script requires Humble." >&2
    exit 1
fi

source_setup_file "${UGV_ROS_SETUP}"
source_setup_file "${WORKSPACE_DIR}/install/setup.bash"

ros2 launch unitree_lidar_ros2 launch.py "$@"
