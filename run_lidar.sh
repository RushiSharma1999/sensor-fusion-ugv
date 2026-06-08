#!/bin/bash
set -e

cd "$(dirname "$0")/ws"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch unitree_lidar_ros2 launch.py
