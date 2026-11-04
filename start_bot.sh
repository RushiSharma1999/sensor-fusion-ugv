#!/bin/bash
set -e

SESSION="bot_stack_old_migration"
WS="$HOME/sensor-fusion-ugv-old-migration/ws"
TMUX=(tmux -L bot_old_migration)

if [[ ! -f "$WS/install/local_setup.bash" ]]; then
    echo "Build the new workspace first: $WS"
    exit 1
fi

if "${TMUX[@]}" has-session -t "$SESSION" 2>/dev/null; then
    echo "This stack already has a session. Attach with:"
    echo "tmux -L bot_old_migration attach-session -t $SESSION"
    exit 1
fi

# Keep ROS package paths from other workspaces out of each pane.
PANE_ENV=(
    "HOME=$HOME"
    "USER=${USER:-sdp2026}"
    "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    "TERM=${TERM:-xterm-256color}"
)
for name in DISPLAY XAUTHORITY XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS \
            ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION \
            ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE \
            FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI; do
    if [[ -v "$name" ]]; then
        PANE_ENV+=("$name=${!name}")
    fi
done

printf -v SETUP 'cd %q && source /opt/ros/humble/setup.bash && source install/local_setup.bash' "$WS"

start_pane() {
    local command="$1"
    local pane_command
    printf -v pane_command '%q ' env -i "${PANE_ENV[@]}" \
        /bin/bash --noprofile --norc -c \
        "$SETUP && $command; exec /bin/bash --noprofile --norc"

    if ! "${TMUX[@]}" has-session -t "$SESSION" 2>/dev/null; then
        "${TMUX[@]}" new-session -d -s "$SESSION" -x 240 -y 80 "$pane_command"
    else
        "${TMUX[@]}" split-window -h -t "$SESSION:0" "$pane_command"
    fi
    "${TMUX[@]}" select-layout -t "$SESSION:0" tiled >/dev/null
}

# Keep the current Ethernet configuration; do not flush an SSH interface.
echo "Using the existing LiDAR Ethernet configuration:"
ip -br addr show enP8p1s0

# LiDAR
start_pane 'ros2 launch unitree_lidar_ros2 launch.py'

# Pointcloud to LaserScan
start_pane 'ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/unilidar/cloud -r scan:=/scan -p target_frame:=unilidar_lidar -p min_height:=-0.10 -p max_height:=1.0 -p range_min:=0.15 -p range_max:=20.0 -p use_inf:=true'

# VFH obstacle avoidance
start_pane 'ros2 run vfh_pkg vfh_node'

# Camera tracking
start_pane 'ros2 run vision_uart_pkg vision_uart_receiver_node'

# Camera follow
start_pane 'ros2 run vision_uart_pkg camera_follow_node'

# Fusion / safety sensors
start_pane 'ros2 run fusion_pkg fusion_node'

# Safety mux
start_pane 'ros2 run vision_uart_pkg safety_mux_node'

# ESP serial motor bridge
start_pane 'ros2 run esp_bridge cmd_vel_to_serial --ros-args -p port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A84021142-if00 -p baud:=115200'

# Foxglove Bridge (optional)
# start_pane 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'

# RViz on the small display, using your existing helper.
if [[ -x "$HOME/start_rviz.sh" ]]; then
    start_pane '"$HOME/start_rviz.sh"'
else
    echo "Skipping RViz: $HOME/start_rviz.sh is missing or not executable."
fi

"${TMUX[@]}" attach-session -t "$SESSION"

