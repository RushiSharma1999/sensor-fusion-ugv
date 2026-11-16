#!/bin/bash

SESSION=bot_stack
WS="$HOME/sensor-fusion-ugv-migration/ws"
tmux kill-session -t $SESSION 2>/dev/null
echo "Setting up LiDAR ethernet..."
sudo ip addr flush dev enP8p1s0 || true
sudo ip addr add 192.168.1.2/24 dev enP8p1s0
sudo ip link set enP8p1s0 up
echo "Ethernet configured"
sleep 2
tmux new-session -d -s $SESSION

# LiDAR
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch unitree_lidar_ros2 launch.py" C-m

# Pointcloud to LaserScan
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/unilidar/cloud -r scan:=/scan -p target_frame:=unilidar_lidar -p min_height:=-0.10 -p max_height:=1.0 -p range_min:=0.15 -p range_max:=20.0 -p use_inf:=true" C-m

# VFH obstacle avoidance
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 run vfh_pkg vfh_node" C-m

# Camera tracking
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 run vision_uart_pkg vision_uart_receiver_node" C-m

# Camera Follow
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 run vision_uart_pkg camera_follow_node" C-m

# Fusion / safety sensors
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 run fusion_pkg fusion_node" C-m

# Safety mux
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 run vision_uart_pkg safety_mux_node" C-m

# ESP serial motor bridge
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 run esp_bridge cmd_vel_to_serial --ros-args -p port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A84021142-if00 -p baud:=115200" C-m

# Launch Rviz separately
tmux new-window -d -t $SESSION -n rviz "cd $WS && source install/setup.bash && DISPLAY=:0 ~/start_rviz.sh"

tmux select-layout -t $SESSION tiled
tmux attach-session -t $SESSION 
