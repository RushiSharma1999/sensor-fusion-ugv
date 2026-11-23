# LiDAR and ultrasonic integration

## Why the existing stack can stop

1. **Verified from code** — The old full-stack launcher sourced a fixed workspace under the user's home directory. On a different checkout it could run old installed packages or fail to find them. It launched the fusion node without an ultrasonic or ToF processor, and the port's default VFH and mux modes ignored host-side fusion.
2. **Documentation says** — In the earlier Jetson stationary capture, 20 of 111 baseline scans lacked finite front returns. The port's mux requires fresh, advancing scan timestamps and a finite return within ±15 degrees. It commands zero when either check fails even while the LiDAR node is alive. The converter projects in the LiDAR frame and drops points outside its height and range filters. The capture did not establish why the front returns were missing. [Jetson evidence](artifacts/jetson_lidar_only_runtime_2026-11-01.json)
3. **Potential issue** — An absent or incorrect host address or route can prevent UDP reception. If `/unilidar/cloud` is silent, check the host address, interface and route before investigating VFH. The tested profile uses LiDAR `192.168.1.62:6101` and host `192.168.1.2:6201`. [Port driver transport evidence](../interfaces/unitree_driver_tf_contract.md)

The current robot's cause requires a live topic and mux capture.

## Bring-up

Build this checkout using the [Humble procedure](../../README.md#fresh-clone-build-procedure). Stop other robot stacks before starting this one. The launcher uses this checkout's overlay and leaves network configuration to the host.

```bash
./start_bot.sh
```

That starts LiDAR, projection, VFH, ultrasonic processing, fusion and the mux. The mux publishes zero and the ESP32 serial bridge is closed. Pass the robot's verified serial path as `serial_port:=/dev/serial/by-id/...` to read ultrasonic telemetry while the mux still publishes zero. Set `use_tof:=true` only on a robot with working ToF telemetry. Stop with Ctrl+C.

The driver and VFH are derived from `main` at `09774b3`. The launch uses the 36-packet setting from the main navigation launcher. `cloud_scan_num:=18` selects the main standalone driver's stock setting. The VFH runs main's baseline histogram mode, which consumes sector penalties. Its open sectors require observed LiDAR returns. The port's mux independently checks scan and command freshness. The port's standalone driver, ESP32 telemetry bridge, and LiDAR-only launcher remain available. [Main navigation profile](https://github.com/RushiSharma1999/sensor-fusion-ugv/blob/09774b306ec9fc14a1354215423ec8adb038ac45/docs/lidar_avoidance.md)

The scan is projected in `unilidar_lidar` using the existing −0.10 to 1.0 m height window and 0.15 to 20 m range. Confirm the sensor's physical forward direction before enabling motion. No measured robot-frame transform is supplied. The mux stops for stale or invalid scans, missing required fusion, or a critical stop. Ultrasound must report at least one valid front, left, or right direction. Explicit invalid directions provide no clearance evidence. With ToF enabled its data must also be fresh. The current ESP32 firmware may independently veto commands.

`motion_enabled:=true` with an explicit `serial_port` authorizes the mux to forward nonzero commands once perception guards pass. Use only for an attended wheels-raised check after verifying the physical stop and exclusive serial ownership. Floor operation requires hardware verification of command signs, stopping and communication-loss behavior.

## Localize the failure

From a second terminal with the same ROS underlay, workspace and domain, check the signal path in this order.

| Check | What the result tells you |
|---|---|
| `ros2 topic hz /unilidar/cloud` | Silence places the fault at the UDP/driver boundary. Check driver output, `ip -br -4 addr` and `ip route get 192.168.1.62`. |
| `ros2 topic hz /scan` | A live cloud and silent scan point to the converter, topic remap or frame. A live scan can still contain no usable front ranges. |
| `ros2 topic echo /mux/debug --once` | `scan_guard:*` shows scan validation or timeout, `fusion_stale` means required fusion topics are absent or stale, and `motion_disabled` confirms the stopped launch profile. |
| `ros2 topic echo /fusion/debug_state --once` | Shows a critical stop, speed limit, and which required sensor is unavailable. Check `/esp/ultrasonic_raw` and `/ultrasonic/features` when sonar is unavailable. |
| `ros2 topic info /cmd_vel --verbose` | The fused launch should show one publisher, `safety_mux_node`. If final commands are nonzero but the wheels stay still, inspect serial device access, `/esp/motor_status`, and firmware vetoes. |

## Software evidence

**Verified by automated software test** — The installed launch ran under ROS 2 Jazzy on x86_64 with the upstream `pointcloud_to_laserscan` and a virtual serial device. A synthetic point cloud and v4-style sonar input exercised projection, directional penalties, reduced speed, near obstacles and sensor loss, stationary mode, optional ToF timeout, and a single final command publisher. This used neither a physical LiDAR nor an ESP32. [Evidence log](evidence_log.md)

**Not yet hardware verified** — Humble ARM64 deployment, live UDP acquisition with this launch, physical alignment, wheels-raised motion and floor behavior.
