#!/usr/bin/env bash
set -euo pipefail

echo "Building Sensor Fusion UGV ROS 2 workspace..."

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 Jazzy was not found at ${ROS_SETUP}" >&2
    echo "Install ROS 2 Jazzy on Ubuntu 24.04 before building." >&2
    exit 1
fi
source "${ROS_SETUP}"

if ! command -v colcon >/dev/null 2>&1; then
    echo "ERROR: colcon was not found after sourcing ROS 2 Jazzy." >&2
    echo "Install python3-colcon-common-extensions before building." >&2
    exit 1
fi

cd "${WORKSPACE_DIR}"
colcon build --symlink-install "$@"

echo ""
echo "Build complete."
echo "To use the workspace, run:"
printf 'source "%s/install/setup.bash"\n' "${WORKSPACE_DIR}"
