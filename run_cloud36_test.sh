#!/bin/bash
# Low-speed reactive integration mode. Camera/fusion/mux are not launched.
# No overall test timer. Ctrl+C or a node exit shuts down the process groups.
set -eo pipefail
stop_stack() {
    tmux -L bot_old_migration kill-session -t bot_stack_old_migration 2>/dev/null || true
    tmux kill-session -t bot_stack 2>/dev/null || true
    pkill -INT -f 'sensor-fusion-ugv(-old)?-migration/ws/(install|build)/|jetson_yolo_ros.py|live_yolo_headless_movement_command.py|ugv_motor_check.py|ugv_scan_diagnostic.py|ros2 (run|launch) (unitree_lidar_ros2|pointcloud_to_laserscan|vfh_pkg|esp_bridge|fusion_pkg|vision_uart_pkg)|pointcloud_to_laserscan_node|rviz2|foxglove_bridge' || true
}
if [[ ${1:-} == --stop ]]; then
    stop_stack
    exit 0
fi
if [[ ${1:-} != --clean ]]; then
    case ${1:---dry-run} in --dry-run|--motors) ;; *) echo 'Usage: bash run_continuous_bot.sh [--dry-run|--motors|--stop]'; exit 2;; esac
    exec env -i HOME="$HOME" USER="${USER:-sdp2026}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin TERM="${TERM:-xterm-256color}" /bin/bash --noprofile --norc "$0" --clean "${1:---dry-run}"
fi
shift
mode=${1:---dry-run}
case $mode in --dry-run) dry=true;; --motors) dry=false;; *) exit 2;; esac
root="$HOME/sensor-fusion-ugv-old-migration"
if [[ ! -f "$root/ws/install/local_setup.bash" ]]; then
    echo "Missing build: $root/ws/install/local_setup.bash"; exit 1
fi
source /opt/ros/humble/setup.bash
source "$root/ws/install/local_setup.bash"
if ! ip -o -4 addr show dev enP8p1s0 | awk '$4 ~ /^192[.]168[.]1[.]2\// {found=1} END {exit !found}'; then
    echo 'LiDAR interface must already have local IP 192.168.1.2; no network settings were changed.'; exit 1
fi
pattern='sensor-fusion-ugv(-old)?-migration/ws/(install|build)/|ros2 (run|launch) (unitree_lidar_ros2|pointcloud_to_laserscan|vfh_pkg|esp_bridge|fusion_pkg|vision_uart_pkg)|pointcloud_to_laserscan_node|jetson_yolo_ros.py|live_yolo_headless_movement_command.py|ugv_motor_check.py'
if pgrep -af "$pattern"; then
    echo 'Existing robot/camera processes found. Stop them first with this script --stop.'; exit 1
fi
# Confirm the rebuilt node exposes the new mode. This helper does not spin nodes.
python3 - <<'PY'
import inspect
from vfh_pkg.vfh_node import VFHObstacleAvoidance
if not hasattr(VFHObstacleAvoidance, 'run_continuous_avoidance'):
    raise SystemExit('Continuous controller not installed. Run installer and rebuild vfh_pkg first.')
print('VFH source:', inspect.getfile(VFHObstacleAvoidance))
PY
logs="$root/continuous-$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$logs"
pids=()
monitor_pid=''
cleanup() {
    rc=$?
    trap - EXIT INT TERM
    if [[ -n $monitor_pid ]]; then kill -TERM "$monitor_pid" 2>/dev/null || true; fi
    for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done
    sleep 1
    for p in "${pids[@]}"; do kill -TERM -- "-$p" 2>/dev/null || true; done
    sleep 1
    for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || true; done
    for p in "${pids[@]}"; do wait "$p" 2>/dev/null || true; done
    if [[ -n $monitor_pid ]]; then wait "$monitor_pid" 2>/dev/null || true; fi
    echo "Stopped child groups. Logs: $logs"
    if [[ -f $logs/bridge.log ]]; then tail -n 12 "$logs/bridge.log"; fi
    return "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() {
    name=$1; shift
    setsid "$@" >"$logs/$name.log" 2>&1 &
    pids+=("$!")
}
start lidar ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args -p initialize_type:=2 -p work_mode:=0 -p lidar_ip:=192.168.1.62 -p local_ip:=192.168.1.2 -p lidar_port:=6101 -p local_port:=6201 -p cloud_scan_num:=36
start scan ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/unilidar/cloud -r scan:=/scan -p target_frame:=unilidar_lidar -p min_height:=-0.10 -p max_height:=1.0 -p range_min:=0.15 -p range_max:=20.0 -p use_inf:=true
start vfh ros2 run vfh_pkg vfh_node --ros-args -r /cmd_vel_vfh:=/diagnostic/cmd_vel_vfh -r /cmd_vel:=/diagnostic/cmd_vel_vfh -p continuous_avoidance:=true -p forward_speed:=0.05 -p min_speed:=0.02 -p max_yaw_rate:=0.25 -p escape_turn_rate:=0.25 -p hazard_escape_turn_rate:=0.25 -p front_stop_dist:=0.65 -p hard_turn_dist:=0.85 -p front_slowdown_dist:=1.2
start bridge ros2 run esp_bridge cmd_vel_to_serial --ros-args -p port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A84021142-if00 -p baud:=115200 -p cmd_vel_topic:=/diagnostic/cmd_vel_vfh -r /cmd_vel:=/diagnostic/cmd_vel_vfh -p dry_run:="$dry" -p firmware_debug:=true -p max_linear_speed:=0.05 -p max_angular_speed:=0.25 -p allow_reverse:=false
printf 'Mode: %s. No overall timer. Ctrl+C stops the stack.\nLogs: %s\n' "$mode" "$logs"
tail -n 0 -F "$logs/vfh.log" "$logs/bridge.log" &
monitor_pid=$!
if wait -n "${pids[@]}"; then
    echo 'A robot node exited. Stopping the other nodes.'
    exit 1
else
    rc=$?
    echo "A robot node failed (status $rc). Stopping the other nodes."
    exit "$rc"
fi
