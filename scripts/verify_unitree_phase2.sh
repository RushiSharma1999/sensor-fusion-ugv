#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${REPOSITORY_ROOT}/ws"
UGV_ROS_DISTRO="humble"
UGV_ROS_SETUP="/opt/ros/${UGV_ROS_DISTRO}/setup.bash"
UGV_SETUP_HELPER="${SCRIPT_DIR}/lib/source_setup.bash"
UGV_UNITREE_PACKAGE="unitree_lidar_ros2"
UGV_PHASE2_OUTPUT_DIR="${WORKSPACE_DIR}/.phase2-verification"
UGV_PHASE2_BUILD_DIR="${UGV_PHASE2_OUTPUT_DIR}/build"
UGV_PHASE2_INSTALL_DIR="${UGV_PHASE2_OUTPUT_DIR}/install"
UGV_PHASE2_LOG_DIR="${UGV_PHASE2_OUTPUT_DIR}/log"

if [[ ! -d "${WORKSPACE_DIR}/src/${UGV_UNITREE_PACKAGE}" ]]; then
    echo "ERROR: Unitree package source directory not found: ${WORKSPACE_DIR}/src/${UGV_UNITREE_PACKAGE}" >&2
    exit 1
fi
if [[ ! -f "${UGV_SETUP_HELPER}" ]]; then
    echo "ERROR: Setup-source helper not found: ${UGV_SETUP_HELPER}" >&2
    exit 1
fi
source "${UGV_SETUP_HELPER}"
"${SCRIPT_DIR}/test_source_setup.sh"

if [[ ! -f "${UGV_ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 Humble was not found at ${UGV_ROS_SETUP}" >&2
    echo "The Phase 2 verifier requires Ubuntu 22.04 Jammy with ROS 2 Humble." >&2
    exit 1
fi

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${UGV_ROS_DISTRO}" ]]; then
    echo "ERROR: ROS_DISTRO is already '${ROS_DISTRO}', but Phase 2 requires '${UGV_ROS_DISTRO}'." >&2
    echo "Open a clean shell before running this script; do not mix ROS distributions." >&2
    exit 1
fi
source_setup_file "${UGV_ROS_SETUP}"

if [[ "${ROS_DISTRO:-}" != "${UGV_ROS_DISTRO}" ]]; then
    echo "ERROR: Sourcing ${UGV_ROS_SETUP} did not select ROS 2 Humble." >&2
    exit 1
fi

for UGV_REQUIRED_COMMAND in python3 colcon ros2 timeout; do
    if ! command -v "${UGV_REQUIRED_COMMAND}" >/dev/null 2>&1; then
        echo "ERROR: Required command not found: ${UGV_REQUIRED_COMMAND}" >&2
        exit 1
    fi
done

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "aarch64" ]]; then
    echo "ERROR: Phase 2 build verification requires Linux aarch64." >&2
    echo "Detected: $(uname -s) $(uname -m)" >&2
    exit 1
fi

if [[ ! -r /etc/os-release ]] \
    || ! grep -Fqx 'ID=ubuntu' /etc/os-release \
    || ! grep -Fqx 'VERSION_ID="22.04"' /etc/os-release; then
    echo "ERROR: Phase 2 build verification requires Ubuntu 22.04 LTS Jammy." >&2
    echo "The current /etc/os-release does not identify that supported baseline." >&2
    exit 1
fi

UGV_PYTHON_VERSION="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ "${UGV_PYTHON_VERSION}" != "3.10" ]]; then
    echo "ERROR: Phase 2 build verification requires Python 3.10; detected ${UGV_PYTHON_VERSION}." >&2
    exit 1
fi

python3 "${SCRIPT_DIR}/validate_ros_packages.py"
python3 "${SCRIPT_DIR}/validate_unitree_driver.py"

UGV_DISCOVERED_PACKAGES="$(colcon list --base-paths "${WORKSPACE_DIR}/src" --names-only)"
if ! grep -Fqx "${UGV_UNITREE_PACKAGE}" <<< "${UGV_DISCOVERED_PACKAGES}"; then
    echo "ERROR: ${UGV_UNITREE_PACKAGE} is not discoverable by colcon." >&2
    exit 1
fi

echo "Resetting the isolated Phase 2 output directory for a clean Unitree build..."
rm -rf -- "${UGV_PHASE2_OUTPUT_DIR}"
mkdir -p "${UGV_PHASE2_OUTPUT_DIR}"

cd "${WORKSPACE_DIR}"
colcon --log-base "${UGV_PHASE2_LOG_DIR}" build \
    --base-paths "${WORKSPACE_DIR}/src" \
    --build-base "${UGV_PHASE2_BUILD_DIR}" \
    --install-base "${UGV_PHASE2_INSTALL_DIR}" \
    --symlink-install \
    --packages-select "${UGV_UNITREE_PACKAGE}"

UGV_UNITREE_ARCHIVE="${WORKSPACE_DIR}/src/unitree_lidar_sdk/lib/aarch64/libunilidar_sdk2.a"
UGV_UNITREE_SELECTION="${UGV_PHASE2_BUILD_DIR}/${UGV_UNITREE_PACKAGE}/unitree_sdk_library.txt"
UGV_UNITREE_LINK_COMMAND="${UGV_PHASE2_BUILD_DIR}/${UGV_UNITREE_PACKAGE}/CMakeFiles/unitree_lidar_ros2_node.dir/link.txt"
if [[ ! -f "${UGV_UNITREE_SELECTION}" ]] \
    || ! grep -Fqx "${UGV_UNITREE_ARCHIVE}" "${UGV_UNITREE_SELECTION}"; then
    echo "ERROR: Unitree wrapper did not select the checked-in aarch64 SDK archive." >&2
    echo "Expected archive: ${UGV_UNITREE_ARCHIVE}" >&2
    exit 1
fi
if [[ ! -f "${UGV_UNITREE_LINK_COMMAND}" ]] \
    || ! grep -Fq "${UGV_UNITREE_ARCHIVE}" "${UGV_UNITREE_LINK_COMMAND}"; then
    echo "ERROR: Unitree link command does not reference the selected aarch64 SDK archive." >&2
    exit 1
fi

source_setup_file "${UGV_PHASE2_INSTALL_DIR}/setup.bash"

UGV_UNITREE_SHARE="$(ros2 pkg prefix --share "${UGV_UNITREE_PACKAGE}")"
for UGV_UNITREE_RESOURCE in \
    launch/launch.py \
    config/driver.defaults.yaml \
    rviz/view.rviz; do
    if [[ ! -f "${UGV_UNITREE_SHARE}/${UGV_UNITREE_RESOURCE}" ]]; then
        echo "ERROR: Installed Unitree resource is missing: ${UGV_UNITREE_SHARE}/${UGV_UNITREE_RESOURCE}" >&2
        exit 1
    fi
done
ros2 launch "${UGV_UNITREE_PACKAGE}" launch.py --show-args >/dev/null

colcon --log-base "${UGV_PHASE2_LOG_DIR}" test \
    --build-base "${UGV_PHASE2_BUILD_DIR}" \
    --install-base "${UGV_PHASE2_INSTALL_DIR}" \
    --packages-select "${UGV_UNITREE_PACKAGE}"
colcon test-result \
    --test-result-base "${UGV_PHASE2_BUILD_DIR}/${UGV_UNITREE_PACKAGE}" \
    --verbose

if ! ros2 pkg executables "${UGV_UNITREE_PACKAGE}" \
    | grep -Fqx "${UGV_UNITREE_PACKAGE} unitree_lidar_ros2_node"; then
    echo "ERROR: Installed Unitree executable was not discovered." >&2
    exit 1
fi

UGV_INVALID_LOG="$(mktemp "${TMPDIR:-/tmp}/ugv-unitree-invalid-config.XXXXXX")"
cleanup_unitree_phase2() {
    rm -f -- "${UGV_INVALID_LOG}"
}
trap cleanup_unitree_phase2 EXIT

UGV_INVALID_STATUS=0
if timeout 10s ros2 run "${UGV_UNITREE_PACKAGE}" unitree_lidar_ros2_node \
    --ros-args -p initialize_type:=0 > "${UGV_INVALID_LOG}" 2>&1; then
    echo "ERROR: Invalid initialize_type unexpectedly succeeded." >&2
    cat "${UGV_INVALID_LOG}" >&2
    exit 1
else
    UGV_INVALID_STATUS=$?
fi
if [[ "${UGV_INVALID_STATUS}" -eq 124 ]]; then
    echo "ERROR: Invalid initialize_type did not fail before the timeout." >&2
    cat "${UGV_INVALID_LOG}" >&2
    exit 1
fi
if ! grep -Fq 'initialize_type must be 1 (serial) or 2 (UDP)' "${UGV_INVALID_LOG}"; then
    echo "ERROR: Invalid initialize_type did not produce the expected validation diagnostic." >&2
    cat "${UGV_INVALID_LOG}" >&2
    exit 1
fi
if grep -Fq 'Unitree driver initialized' "${UGV_INVALID_LOG}"; then
    echo "ERROR: Driver reported initialization for an invalid initialize_type." >&2
    cat "${UGV_INVALID_LOG}" >&2
    exit 1
fi

echo "PASS: Phase 2 Unitree clean build, ARM64 SDK link, installed resources, package tests, executable discovery, and pre-SDK invalid-configuration rejection completed without starting hardware."
