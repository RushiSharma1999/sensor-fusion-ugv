#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${REPOSITORY_ROOT}/ws"
UGV_ROS_DISTRO="humble"
UGV_ROS_SETUP="/opt/ros/${UGV_ROS_DISTRO}/setup.bash"
UGV_SETUP_HELPER="${SCRIPT_DIR}/lib/source_setup.bash"

if [[ ! -f "${UGV_SETUP_HELPER}" ]]; then
    echo "ERROR: Setup-source helper not found: ${UGV_SETUP_HELPER}" >&2
    exit 1
fi
source "${UGV_SETUP_HELPER}"
"${SCRIPT_DIR}/test_source_setup.sh"

if [[ ! -f "${UGV_ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 Humble was not found at ${UGV_ROS_SETUP}" >&2
    echo "This verification script requires Ubuntu 22.04 Jammy with ROS 2 Humble." >&2
    exit 1
fi

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${UGV_ROS_DISTRO}" ]]; then
    echo "ERROR: ROS_DISTRO is already '${ROS_DISTRO}', but Phase 1 supports '${UGV_ROS_DISTRO}'." >&2
    echo "Open a clean shell before running this script; do not mix ROS distributions." >&2
    exit 1
fi
source_setup_file "${UGV_ROS_SETUP}"

if [[ "${ROS_DISTRO:-}" != "${UGV_ROS_DISTRO}" ]]; then
    echo "ERROR: Sourcing ${UGV_ROS_SETUP} did not select ROS 2 Humble." >&2
    exit 1
fi

for required_command in python3 colcon ros2; do
    if ! command -v "${required_command}" >/dev/null 2>&1; then
        echo "ERROR: Required command not found: ${required_command}" >&2
        exit 1
    fi
done

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "aarch64" ]]; then
    echo "ERROR: Target verification requires Linux aarch64." >&2
    echo "Detected: $(uname -s) $(uname -m)" >&2
    exit 1
fi

if [[ ! -r /etc/os-release ]] \
    || ! grep -Fqx 'ID=ubuntu' /etc/os-release \
    || ! grep -Fqx 'VERSION_ID="22.04"' /etc/os-release; then
    echo "ERROR: Target verification requires Ubuntu 22.04 LTS Jammy." >&2
    echo "The current /etc/os-release does not identify that supported baseline." >&2
    exit 1
fi

UGV_PYTHON_VERSION="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ "${UGV_PYTHON_VERSION}" != "3.10" ]]; then
    echo "ERROR: Target verification requires Python 3.10; detected ${UGV_PYTHON_VERSION}." >&2
    exit 1
fi

python3 "${SCRIPT_DIR}/validate_ros_packages.py"
python3 "${SCRIPT_DIR}/validate_unitree_driver.py"

UGV_DISCOVERED_PACKAGES="$(colcon list --base-paths "${WORKSPACE_DIR}/src" --names-only)"
while read -r UGV_EXPECTED_PACKAGE; do
    if ! grep -Fqx "${UGV_EXPECTED_PACKAGE}" <<< "${UGV_DISCOVERED_PACKAGES}"; then
        echo "ERROR: Expected source package is not discoverable by colcon: ${UGV_EXPECTED_PACKAGE}" >&2
        exit 1
    fi
done <<'EOF'
esp_bridge
esp_fusion_pkg
fusion_pkg
gap_follow
lidar_logger_pkg
tof_processor_pkg
ultrasonic_pkg
unitree_lidar_ros2
unitree_lidar_sdk
vfh_pkg
vision_uart_pkg
EOF

"${REPOSITORY_ROOT}/build_ws.sh" --clean

UGV_UNITREE_ARCHIVE="${WORKSPACE_DIR}/src/unitree_lidar_sdk/lib/aarch64/libunilidar_sdk2.a"
UGV_UNITREE_SELECTION="${WORKSPACE_DIR}/build/unitree_lidar_ros2/unitree_sdk_library.txt"
UGV_UNITREE_LINK_COMMAND="${WORKSPACE_DIR}/build/unitree_lidar_ros2/CMakeFiles/unitree_lidar_ros2_node.dir/link.txt"
if [[ ! -f "${UGV_UNITREE_SELECTION}" ]] \
    || ! grep -Fqx "${UGV_UNITREE_ARCHIVE}" "${UGV_UNITREE_SELECTION}"; then
    echo "ERROR: Unitree wrapper did not configure the checked-in aarch64 SDK archive." >&2
    echo "Expected archive: ${UGV_UNITREE_ARCHIVE}" >&2
    exit 1
fi
if [[ -f "${UGV_UNITREE_LINK_COMMAND}" ]] \
    && ! grep -Fq "${UGV_UNITREE_ARCHIVE}" "${UGV_UNITREE_LINK_COMMAND}"; then
    echo "ERROR: Available Unitree link command does not reference the configured aarch64 SDK archive." >&2
    exit 1
fi

source_setup_file "${WORKSPACE_DIR}/install/setup.bash"

UGV_UNITREE_SHARE="$(ros2 pkg prefix --share unitree_lidar_ros2)"
for UGV_UNITREE_RESOURCE in \
    launch/launch.py \
    config/driver.defaults.yaml \
    rviz/view.rviz; do
    if [[ ! -f "${UGV_UNITREE_SHARE}/${UGV_UNITREE_RESOURCE}" ]]; then
        echo "ERROR: Installed Unitree resource is missing: ${UGV_UNITREE_SHARE}/${UGV_UNITREE_RESOURCE}" >&2
        exit 1
    fi
done
ros2 launch unitree_lidar_ros2 launch.py --show-args >/dev/null

cd "${WORKSPACE_DIR}"
if ! colcon test; then
    echo "ERROR: colcon test did not complete successfully." >&2
    exit 1
fi
if ! colcon test-result --verbose; then
    echo "ERROR: One or more workspace tests failed." >&2
    exit 1
fi

while read -r package executable; do
    if ! ros2 pkg executables "${package}" | grep -Fqx "${package} ${executable}"; then
        echo "ERROR: Expected ROS executable is not installed: ${package} ${executable}" >&2
        exit 1
    fi
done <<'EOF'
esp_bridge cmd_vel_to_serial
esp_fusion_pkg esp_lidar_fusion_driver
fusion_pkg fusion_node
gap_follow gap_follow
lidar_logger_pkg lidar_logger
tof_processor_pkg tof_processor_node
ultrasonic_pkg ultrasonic_processor
unitree_lidar_ros2 unitree_lidar_ros2_node
vfh_pkg vfh_node
vision_uart_pkg vision_uart_receiver_node
vision_uart_pkg camera_follow_node
vision_uart_pkg safety_mux_node
EOF

echo "PASS: workspace build, tests, and ROS executable discovery completed."
