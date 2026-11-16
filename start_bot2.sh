#!/bin/bash

SESSION=bot_stack
WS="$HOME/sensor-fusion-ugv-migration/ws"
SRC="cd $WS && source install/setup.bash"

tmux kill-session -t $SESSION 2>/dev/null
pkill -f rviz2 2>/dev/null

echo "Setting up LiDAR ethernet..."
sudo ip addr flush dev enP8p1s0 || true
sudo ip addr add 192.168.1.2/24 dev enP8p1s0
sudo ip link set enP8p1s0 up
echo "Ethernet configured"
sleep 2

# first <window> <cmd>: run in the window's existing (first) pane
first() {
  tmux send-keys -t "$SESSION:$1" "$SRC && $2" C-m
}

# pane <window> <cmd>: add a new pane to that window and run cmd in it
pane() {
  tmux split-window -t "$SESSION:$1"
  tmux select-layout -t "$SESSION:$1" tiled
  tmux send-keys -t "$SESSION:$1" "$SRC && $2" C-m
}

# ---------- Window 1: setup (everything except VFH) ----------
tmux new-session -d -s $SESSION -n setup

# LiDAR
first setup "ros2 launch unitree_lidar_ros2 launch.py"

# Pointcloud to LaserScan
pane setup "ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/unilidar/cloud -r scan:=/scan -p target_frame:=unilidar_lidar -p min_height:=-0.10 -p max_height:=1.0 -p range_min:=0.15 -p range_max:=20.0 -p use_inf:=true"

# Camera tracking
pane setup "ros2 run vision_uart_pkg vision_uart_receiver_node"

# Camera follow
pane setup "ros2 run vision_uart_pkg camera_follow_node"

# Fusion / safety sensors
pane setup "ros2 run fusion_pkg fusion_node"

# Safety mux
pane setup "ros2 run vision_uart_pkg safety_mux_node"

# ---------- Window 2: active ----------
tmux new-window -d -t $SESSION -n active

# ESP serial motor bridge
#first active "ros2 run esp_bridge cmd_vel_to_serial --ros-args -p port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A84021142-if00 -p baud:=115200"

# VFH Obstacel Avoidance
first active "ros2 run vfh_pkg vfh_node"

# ---------- RViz: background, no pane ----------
(
  export DISPLAY=:0
  cd $WS && source install/setup.bash
  nohup ~/start_rviz.sh > /tmp/rviz.log 2>&1 &
)

tmux select-window -t $SESSION:active
tmux attach-session -t $SESSION
