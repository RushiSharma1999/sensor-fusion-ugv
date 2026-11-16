import json, math, subprocess, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

OUT=Path('/home/sdp2026/sensor-fusion-ugv-migration/docs/verification/artifacts')
OUT.mkdir(exist_ok=True)
WS='/home/sdp2026/sensor-fusion-ugv-migration/ws'
PROFILE='/home/sdp2026/ugv-lidar-only-fix/lidar_only.yaml'
CONVERTER='ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/unilidar/cloud -r scan:=/scan -p target_frame:=unilidar_lidar -p min_height:=-0.10 -p max_height:=1.0 -p range_min:=0.15 -p range_max:=20.0 -p use_inf:=true'
VFH=f'ros2 run vfh_pkg vfh_node --ros-args --params-file {PROFILE}'
rclpy.init()
node=Node('ugv_stationary_evidence_observer')
phase='baseline'
started=time.monotonic()
events=[]
scans=[]
cmds=[]
debug=[]
def stamp(): return round(time.monotonic()-started,6)
def scan_cb(msg):
 front=[float(r) for i,r in enumerate(msg.ranges) if abs(msg.angle_min+i*msg.angle_increment)<=math.radians(15)]
 finite=[r for r in front if math.isfinite(r) and r>0 and msg.range_min<=r<=msg.range_max]
 scans.append(dict(t=stamp(),phase=phase,frame=msg.header.frame_id,front_bins=len(front),front_finite=len(finite),front_min=min(finite) if finite else None))
def cmd_cb(msg): cmds.append(dict(t=stamp(),phase=phase,v=msg.linear.x,w=msg.angular.z))
def dbg_cb(msg): debug.append(dict(t=stamp(),phase=phase,data=msg.data))
node.create_subscription(LaserScan,'/scan',scan_cb,qos_profile_sensor_data)
node.create_subscription(Twist,'/cmd_vel',cmd_cb,10)
node.create_subscription(String,'/mux/debug',dbg_cb,10)
def spin(seconds):
 until=time.monotonic()+seconds
 while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=0.02)
def stop(pane):
 subprocess.run(['tmux','send-keys','-t',pane,'C-c'],check=True)
 events.append(dict(t=stamp(),event='stop',pane=pane))
def restart(pane,command):
 current=subprocess.check_output(['tmux','display-message','-p','-t',pane,'#{pane_current_command}'],text=True).strip()
 if current!='bash': raise RuntimeError(f'{pane} still active: {current}')
 subprocess.run(['tmux','send-keys','-t',pane,'C-u'],check=True)
 subprocess.run(['tmux','send-keys','-t',pane,'-l',f'cd {WS} && source install/setup.bash && {command}'],check=True)
 subprocess.run(['tmux','send-keys','-t',pane,'C-m'],check=True)
 events.append(dict(t=stamp(),event='restart',pane=pane))
try:
 spin(10)
 phase='scan_loss_transition'; stop('bot_stack:0.1'); spin(0.7)
 phase='scan_loss_settled'; spin(1)
 restart('bot_stack:0.1',CONVERTER); phase='scan_recovery'; spin(3)
 phase='vfh_loss_transition'; stop('bot_stack:0.2'); spin(0.7)
 phase='vfh_loss_settled'; spin(1)
 restart('bot_stack:0.2',VFH); phase='restored'; spin(3)
finally:
 for pane,command in [('bot_stack:0.1',CONVERTER),('bot_stack:0.2',VFH)]:
  if subprocess.check_output(['tmux','display-message','-p','-t',pane,'#{pane_current_command}'],text=True).strip()=='bash': restart(pane,command)
 node.destroy_node(); rclpy.shutdown()
results={}
for key in ['scan_loss_settled','vfh_loss_settled']:
 samples=[c for c in cmds if c['phase']==key]
 results[key]=dict(samples=len(samples),all_zero=bool(samples) and all(c['v']==0 and c['w']==0 for c in samples))
base=[s for s in scans if s['phase']=='baseline']
results['baseline']=dict(scans=len(base),no_finite_front=sum(s['front_finite']==0 for s in base),frame_ids=sorted({s['frame'] for s in base}))
results['observed_command_maxima']=dict(v=max(abs(c['v']) for c in cmds),w=max(abs(c['w']) for c in cmds))
(OUT/'jetson_lidar_only_runtime_2026-11-01.json').write_text(json.dumps(dict(results=results,events=events,scans=scans,commands=cmds,debug=debug),indent=2)+'\n')
print(json.dumps(results,indent=2))
assert all(results[k]['all_zero'] for k in ['scan_loss_settled','vfh_loss_settled'])
assert results['observed_command_maxima']['v']<=0.08 and results['observed_command_maxima']['w']<=0.4
