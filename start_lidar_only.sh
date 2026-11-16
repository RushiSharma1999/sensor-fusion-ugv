#!/usr/bin/env bash
# LiDAR-only Jetson bring-up. Invocation explicitly enables the motor bridge.
set -Eeuo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS="$ROOT/ws"
PROFILE="$ROOT/config/lidar_only.yaml"
PROBE="$ROOT/scripts/scan_probe.py"
SESSION=lidar_only
PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A84021142-if00
INTERFACE=enP8p1s0
ADDRESS=192.168.1.2/24
DRIVE=1 REPLACE=1 DETACH=0 DRY=0 STOP=0
usage() {
  cat <<'EOF'
Usage: ./start_lidar_only.sh
       ./stop_lidar_only.sh
       ./start_lidar_only.sh [--stationary] [--detach] [--port PATH]
       ./start_lidar_only.sh --stop
       ./start_lidar_only.sh --dry-run [--drive]

Default: replace the old managed session, open all six panes and start the bridge.
Running this launcher enables the bridge after verifying original v4 firmware
is stopped with onboard AUTO disabled. Powered motors may move immediately.
Upload the unchanged bot_firmware_v4.ino first. Its local sonar veto remains
active; its STOP is not latched. No ARM, MODE AUTO or raw PWM is sent.
--stationary  Open diagnostic panes with the motor bridge disabled.
--drive/--replace  Accepted for compatibility; already on by default.
--detach   Create panes without attaching (otherwise opens the tmux view).
--port     Override the verified ESP32 serial by-id path.
--stop     Stop bridge first, then other panes in lidar_only; close that session.
--dry-run  Print all pane commands without ROS, network, tmux or serial changes.

In tmux: Ctrl+B then D detaches; ./start_lidar_only.sh --stop stops the stack.
Stop without flags: ./stop_lidar_only.sh
Hardware motor disable remains the immediate physical stop.
EOF
}
while (($#)); do
  case "$1" in
    --drive) DRIVE=1;; --arm) echo "v4 has no ARM command; run without --arm" >&2; exit 2;; --replace) REPLACE=1;; --detach) DETACH=1;;
    --stationary) DRIVE=0;;
    --dry-run) DRY=1;; --stop) STOP=1;;
    --port) (($#>=2)) || { echo 'Missing --port value' >&2; exit 2; }; PORT=$2; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2;;
  esac
  shift
done
fail() { echo "ERROR: $*" >&2; exit 1; }
stop_session() {
  local name=$1 pane title
  tmux has-session -t "=$name" 2>/dev/null || return 0
  # Let a running bridge execute its zero/shutdown path before other nodes exit.
  while IFS='|' read -r pane title; do
    if [[ "$title" == '6 MOTOR BRIDGE'* ]]; then tmux send-keys -t "$pane" C-c || true; fi
  done < <(tmux list-panes -s -t "=$name" -F '#{pane_id}|#{pane_title}')
  sleep 0.4
  while read -r pane; do tmux send-keys -t "$pane" C-c || true; done \
    < <(tmux list-panes -s -t "=$name" -F '#{pane_id}')
  sleep 0.6
  tmux kill-session -t "=$name" 2>/dev/null || true
}
# A command is shell-quoted only once; never inject commands into idle shell text.
node_shell() {
  printf 'cd %q && source /opt/ros/humble/setup.bash && source %q && exec ' "$WS" "$WS/install/setup.bash"
  printf '%q ' "$@"
}
LIDAR=$(node_shell ros2 launch unitree_lidar_ros2 launch.py)
SCAN=$(node_shell ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/unilidar/cloud -r scan:=/scan -p target_frame:=unilidar_lidar \
  -p min_height:=-0.10 -p max_height:=1.0 -p range_min:=0.15 -p range_max:=20.0 -p use_inf:=true)
VFH=$(node_shell ros2 run vfh_pkg vfh_node --ros-args --params-file "$PROFILE")
MUX=$(node_shell ros2 run vision_uart_pkg safety_mux_node --ros-args --params-file "$PROFILE")
DIAG=$(node_shell python3 "$PROBE")
PREPARE=(python3 "$ROOT/scripts/esp32_prepare_lidar.py" --firmware v4 --port "$PORT")
BRIDGE=$(node_shell ros2 run esp_bridge cmd_vel_to_serial --ros-args --params-file "$PROFILE" -p "port:=$PORT" -p baud:=115200 -p debug_serial:=true)
if ((DRY)); then
  printf '1 LIDAR: %s\n2 SCAN: %s\n3 VFH: %s\n4 MUX: %s\n5 PROBE: %s\n' "$LIDAR" "$SCAN" "$VFH" "$MUX" "$DIAG"
  if ((DRIVE)); then printf '6 MOTOR BRIDGE: '; printf '%q ' "${PREPARE[@]}"; printf '&& %s\n' "$BRIDGE"; else echo '6 MOTOR BRIDGE: DISABLED (stationary mode)'; fi
  exit 0
fi
command -v tmux >/dev/null || fail 'tmux is missing.'
if ((STOP)); then stop_session "$SESSION"; echo 'LiDAR-only session stopped. Verify physical motor stop.'; exit 0; fi
for path in /opt/ros/humble/setup.bash "$WS/install/setup.bash" "$PROFILE" "$PROBE"; do
  [[ -r "$path" ]] || fail "Missing $path"
done
# ROS setup scripts are not nounset-safe.
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
# Validate mode/config before stopping any existing session or starting a node.
python3 - "$PROFILE" <<'PY'
import sys, yaml
from pathlib import Path
from ament_index_python.packages import get_package_prefix
p=yaml.safe_load(Path(sys.argv[1]).read_text())
v=p['vfh_obstacle_avoidance']['ros__parameters']
m=p['safety_mux_node']['ros__parameters']
b=p['cmd_vel_to_serial']['ros__parameters']
assert v['use_fusion'] is False and m['lidar_only_mode'] is True
assert m['prefer_camera_when_target_seen'] is False
assert 0 <= v['forward_speed'] <= .08 and 0 < v['max_yaw_rate'] <= .4
for params in (m,b):
    assert 0 < params['max_linear_speed'] <= .08
    assert 0 < params['max_angular_speed'] <= .4
for pkg,exe in [('unitree_lidar_ros2','unitree_lidar_ros2_node'),
                ('pointcloud_to_laserscan','pointcloud_to_laserscan_node'),
                ('vfh_pkg','vfh_node'),('vision_uart_pkg','safety_mux_node'),
                ('esp_bridge','cmd_vel_to_serial')]:
    path=Path(get_package_prefix(pkg))/'lib'/pkg/exe
    assert path.is_file(), f'Missing executable: {path}'
PY
if ((REPLACE)); then stop_session bot_stack; stop_session "$SESSION"; fi
if tmux has-session -t "=$SESSION" 2>/dev/null; then
  fail "Session $SESSION already exists. Use --replace, or: tmux attach -t $SESSION"
fi
if tmux has-session -t '=bot_stack' 2>/dev/null; then
  fail 'Old bot_stack is running. Use --replace to stop it first.'
fi
pattern='(^|/)(vfh_node|safety_mux_node|cmd_vel_to_serial|unitree_lidar_ros2_node|pointcloud_to_laserscan_node|gap_follow|gap_follow_node|vfh_with_esp_fusion|esp_lidar_fusion_driver|camera_follow_node|fusion_node)([[:space:]]|$)'
if existing=$(pgrep -af "$pattern"); then
  printf '%s\n' "$existing" >&2
  fail 'Robot nodes remain outside the managed sessions. Stop those before launching duplicates.'
fi
if ((DRIVE)); then
  [[ -r "$ROOT/scripts/esp32_prepare_lidar.py" ]] || fail "Missing ESP32 preparation helper"
  [[ -e "$PORT" ]] || fail "ESP32 port missing: $PORT"
  [[ -r "$PORT" && -w "$PORT" ]] || fail "No read/write access to $PORT"
  command -v fuser >/dev/null || fail 'fuser is required for serial ownership checks.'
  if fuser "$PORT" >/dev/null 2>&1; then fail 'ESP32 port is busy; close Arduino Serial Monitor/other serial owners.'; fi
fi
command -v ip >/dev/null || fail 'ip command is missing.'
ip link show dev "$INTERFACE" >/dev/null || fail "LiDAR interface missing: $INTERFACE"
# Preserve other addresses/routes. No address flush.
if ! ip -4 addr show dev "$INTERFACE" | grep -Fq "inet $ADDRESS "; then
  sudo ip addr replace "$ADDRESS" dev "$INTERFACE"
fi
if ! ip link show dev "$INTERFACE" | grep -Eq '<[^>]*\bUP\b'; then
  sudo ip link set dev "$INTERFACE" up
fi
LOGDIR="$WS/log/lidar_only/$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$LOGDIR"
CREATED=0
cleanup() { local code=$?; trap - EXIT INT TERM; if ((CREATED)); then stop_session "$SESSION"; fi; exit "$code"; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Write pane runner files so initial logs are captured and all quoting is stable.
make_runner() {
  local label=$1 cmd=$2 path="$LOGDIR/$1.sh"
  { printf '#!/usr/bin/env bash\nset -o pipefail\n'; printf '%s 2>&1 | tee %q\n' "$cmd" "$LOGDIR/$label.log"; } > "$path"
  chmod +x "$path"
  printf '%s' "$path"
}
runner=$(make_runner 1-lidar "$LIDAR")
pane=$(tmux new-session -d -P -F '#{pane_id}' -s "$SESSION" -n lidar -x 180 -y 54 bash "$runner")
CREATED=1
tmux set-option -w -t "$SESSION:lidar" remain-on-exit on
tmux set-option -w -t "$SESSION:lidar" pane-border-status top
tmux set-option -w -t "$SESSION:lidar" pane-border-format '#{pane_title}'
tmux select-pane -t "$pane" -T '1 LIDAR'
LAST_PANE=$pane
add_pane() {
  local label=$1 title=$2 cmd=$3 runner pane
  runner=$(make_runner "$label" "$cmd")
  pane=$(tmux split-window -h -d -P -F '#{pane_id}' -t "$LAST_PANE" bash "$runner")
  LAST_PANE=$pane
  tmux select-pane -t "$pane" -T "$title"
  tmux select-layout -t "$SESSION:lidar" tiled >/dev/null
}
add_pane 2-scan '2 SCAN CONVERTER' "$SCAN"
add_pane 3-vfh '3 VFH' "$VFH"
add_pane 4-mux '4 SAFETY MUX' "$MUX"
add_pane 5-probe '5 SCAN PROBE' "$DIAG"
if ((DRIVE)); then
  echo 'START requested: original v4 must confirm stopped/AUTO disabled before LiDAR drives.'
  echo 'Checking one final command owner, scan input and bounded final requests (up to 12 seconds)...'
  python3 - <<'PY'
import math,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from vfh_pkg.vfh_node import inspect_scan
rclpy.init()
n=rclpy.create_node('lidar_launcher_preflight')
scan=None; cmd=None; reason='waiting for messages'
def on_scan(msg):
    global scan, reason
    ok, reason=inspect_scan(msg,n.get_clock().now().nanoseconds*1e-9,.5,math.radians(80))
    scan=time.monotonic() if ok else None
def on_cmd(msg):
    global cmd
    fields=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
    ok=all(math.isfinite(v) for v in fields) and 0<=msg.linear.x<=.080001 and abs(msg.angular.z)<=.400001
    ok=ok and all(v==0 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y))
    cmd=time.monotonic() if ok else None
n.create_subscription(LaserScan,'/scan',on_scan,qos_profile_sensor_data)
n.create_subscription(Twist,'/cmd_vel',on_cmd,10)
passed=False
try:
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.1)
        owners=n.get_publishers_info_by_topic('/cmd_vel')
        if len(owners)>1: raise RuntimeError('Multiple final /cmd_vel publishers; refusing motor bridge')
        now=time.monotonic()
        if (len(owners)==1 and owners[0].node_name=='safety_mux_node'
            and owners[0].node_namespace=='/' and owners[0].topic_type=='geometry_msgs/msg/Twist'
            and n.count_publishers('/scan')==1 and n.count_publishers('/cmd_vel_vfh')==1
            and scan is not None and now-scan<.25
            and cmd is not None and now-cmd<.25):
            passed=True;break
    if not passed: raise RuntimeError(f'ROS preflight timed out: scan={reason}; check LiDAR/converter/profile')
    print('ROS startup checks passed. Checking stopped ESP32 before connecting MOTOR BRIDGE.')
finally:
    n.destroy_node();rclpy.shutdown()
PY
  # Recheck serial ownership and confirm original v4 is stopped with AUTO disabled.
  if fuser "$PORT" >/dev/null 2>&1; then fail 'ESP32 port became busy during startup.'; fi
  "${PREPARE[@]}"
  add_pane 6-bridge '6 MOTOR BRIDGE — DRIVE ENABLED' "$BRIDGE"
else
  add_pane 6-bridge '6 MOTOR BRIDGE — DISABLED' "printf '%s\n' 'Motor bridge is OFF (stationary mode). Normal launcher invocation enables motion.'"
fi
trap - EXIT INT TERM
printf 'Session: %s\nLogs: %s\nStop: %s\n' "$SESSION" "$LOGDIR" "$ROOT/stop_lidar_only.sh"
echo 'PWM/speed calibration and physical stop behavior remain unverified; startup checks do not certify floor operation.'
if ((DETACH)); then exit 0; fi
if [[ -n "${TMUX:-}" ]]; then exec tmux switch-client -t "$SESSION"; else exec tmux attach-session -t "$SESSION"; fi
