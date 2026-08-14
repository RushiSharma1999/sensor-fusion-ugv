#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${REPOSITORY_ROOT}/ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 Jazzy was not found at ${ROS_SETUP}" >&2
    echo "This verification script requires Ubuntu 24.04 with ROS 2 Jazzy." >&2
    exit 1
fi
source "${ROS_SETUP}"

for required_command in python3 colcon ros2; do
    if ! command -v "${required_command}" >/dev/null 2>&1; then
        echo "ERROR: Required command not found: ${required_command}" >&2
        exit 1
    fi
done

python3 "${SCRIPT_DIR}/validate_ros_packages.py"
"${REPOSITORY_ROOT}/build_ws.sh"
source "${WORKSPACE_DIR}/install/setup.bash"

cd "${WORKSPACE_DIR}"
colcon test
colcon test-result --verbose

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
