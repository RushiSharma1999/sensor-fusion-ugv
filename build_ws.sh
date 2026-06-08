#!/bin/bash
set -e

echo "Building Sensor Fusion UGV ROS 2 workspace..."

# Move to the workspace directory relative to this script
cd "$(dirname "$0")/ws"

# Source ROS 2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: ROS 2 Jazzy was not found at /opt/ros/jazzy/setup.bash"
    echo "Install ROS 2 Jazzy on Ubuntu 24.04 first."
    exit 1
fi

# Install package dependencies if rosdep is available
if command -v rosdep >/dev/null 2>&1; then
    rosdep update || true
    rosdep install --from-paths src --ignore-src -r -y
else
    echo "WARNING: rosdep not found. Skipping dependency install."
fi

# Build workspace
colcon build --symlink-install

echo ""
echo "Build complete."
echo "To use the workspace, run:"
echo "source ws/install/setup.bash"
