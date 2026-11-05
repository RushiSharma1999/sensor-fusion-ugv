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
    # Parse after re-exec so all options survive the clean environment.
    exec env -i HOME="$HOME" USER="${USER:-sdp2026}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin TERM="${TERM:-xterm-256color}" /bin/bash --noprofile --norc "$0" --clean "$@"
fi
shift
mode=--dry-run
behavior=baseline
speed_profile=conservative
params=''
while (($#)); do
    case $1 in
        --dry-run|--motors) mode=$1; shift;;
        --behavior) behavior=${2:?Missing behavior}; shift 2;;
        --speed-profile) speed_profile=${2:?Missing speed profile}; shift 2;;
        --params) params=${2:?Missing absolute ROS parameter file}; shift 2;;
        *) echo 'Usage: bash run_cloud36_test.sh [--dry-run|--motors|--stop] [--behavior baseline|enhanced] [--speed-profile conservative|open_space] [--params /absolute/geometry.yaml]'; exit 2;;
    esac
done
case $behavior in baseline|enhanced) ;; *) echo 'Invalid behavior'; exit 2;; esac
case $speed_profile in conservative|open_space) ;; *) echo 'Invalid speed profile'; exit 2;; esac
if [[ $behavior == baseline && $speed_profile != conservative ]]; then
    echo 'The preserved baseline uses conservative speed. Select enhanced for open_space.'; exit 2
fi
case $mode in --dry-run) dry=true;; --motors) dry=false;; esac
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
controller_params=()
if [[ -n $params ]]; then
    if [[ $behavior != enhanced || $params != /* || ! -f $params ]]; then
        echo '--params requires enhanced behavior and an existing absolute parameter file'; exit 2
    fi
    controller_params=(--params-file "$params")
fi
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
# Imports only: never instantiate the bridge during a launcher preflight.
python3 - "$root" <<'PY_CHECK'
import inspect
from pathlib import Path
import sys
from vfh_pkg import vfh_node, motion_core
from esp_bridge import cmd_vel_to_serial, speed_profiles
root = Path(sys.argv[1]).resolve() / 'ws'
for module in (vfh_node, motion_core, cmd_vel_to_serial, speed_profiles):
    source = Path(inspect.getfile(module)).resolve()
    if root not in source.parents:
        raise SystemExit(f'Wrong overlay: {source}; rebuild/source this checkout')
    print(f'{module.__name__}: {source}')
PY_CHECK
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
if [[ $behavior == baseline ]]; then
    controller_params+=(-p continuous_avoidance:=true -p min_speed:=0.02 -p escape_turn_rate:=0.25 -p hazard_escape_turn_rate:=0.25 -p front_stop_dist:=0.65 -p hard_turn_dist:=0.85 -p front_slowdown_dist:=1.2)
fi
start vfh ros2 run vfh_pkg vfh_node --ros-args "${controller_params[@]}" -r /cmd_vel_vfh:=/diagnostic/cmd_vel_vfh -r /cmd_vel:=/diagnostic/cmd_vel_vfh -p behavior:="$behavior" -p speed_profile:="$speed_profile"
start bridge ros2 run esp_bridge cmd_vel_to_serial --ros-args -p port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A84021142-if00 -p baud:=115200 -p cmd_vel_topic:=/diagnostic/cmd_vel_vfh -r /cmd_vel:=/diagnostic/cmd_vel_vfh -p dry_run:="$dry" -p firmware_debug:=true -p speed_profile:="$speed_profile" -p allow_reverse:=false
printf 'Behavior: %s; requested speed profile: %s\n' "$behavior" "$speed_profile"
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
