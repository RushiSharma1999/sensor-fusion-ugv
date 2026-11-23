#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$root/scripts/lib/source_setup.bash"
source_setup_file /opt/ros/humble/setup.bash
source_setup_file "$root/ws/install/local_setup.bash"

exec ros2 launch fusion_pkg fused_stack.launch.py "$@"
