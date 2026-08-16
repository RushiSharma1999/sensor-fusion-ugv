#!/usr/bin/env bash
set -euo pipefail

echo "Building Sensor Fusion UGV ROS 2 workspace..."

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/ws"
UGV_ROS_DISTRO="humble"
UGV_ROS_SETUP="/opt/ros/${UGV_ROS_DISTRO}/setup.bash"
UGV_CLEAN_BUILD=false

if [[ "${1:-}" == "--clean" ]]; then
    UGV_CLEAN_BUILD=true
    shift
fi

if [[ ! -d "${WORKSPACE_DIR}/src" ]]; then
    echo "ERROR: ROS workspace source directory not found: ${WORKSPACE_DIR}/src" >&2
    exit 1
fi

if [[ ! -f "${UGV_ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 Humble was not found at ${UGV_ROS_SETUP}" >&2
    echo "Install ROS 2 Humble on Ubuntu 22.04 Jammy before building." >&2
    exit 1
fi

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${UGV_ROS_DISTRO}" ]]; then
    echo "ERROR: ROS_DISTRO is already '${ROS_DISTRO}', but Phase 1 supports '${UGV_ROS_DISTRO}'." >&2
    echo "Open a clean shell before running this script; do not mix ROS distributions." >&2
    exit 1
fi
source "${UGV_ROS_SETUP}"

if [[ "${ROS_DISTRO:-}" != "${UGV_ROS_DISTRO}" ]]; then
    echo "ERROR: Sourcing ${UGV_ROS_SETUP} did not select ROS 2 Humble." >&2
    exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
    echo "ERROR: colcon was not found after sourcing ROS 2 Humble." >&2
    echo "Install python3-colcon-common-extensions before building." >&2
    exit 1
fi

if [[ "${UGV_CLEAN_BUILD}" == true ]]; then
    echo "Removing generated colcon directories for a clean build..."
    for UGV_GENERATED_DIR in build install log; do
        rm -rf -- "${WORKSPACE_DIR}/${UGV_GENERATED_DIR}"
    done
fi

cd "${WORKSPACE_DIR}"
colcon build --symlink-install "$@"

echo ""
echo "Build complete."
echo "To use the workspace, run:"
printf 'source "%s/install/setup.bash"\n' "${WORKSPACE_DIR}"
