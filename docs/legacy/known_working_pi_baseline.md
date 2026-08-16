# Recovered Known-Working Raspberry Pi Baseline

## Purpose and evidence boundary

This file records sanitized facts established by inspecting and backing up the recovered Raspberry Pi and its workspace. It contains no credentials, account names, shell history, keys, or private machine files.

**Documentation says** — the previous-team handoff identifies this machine as a successfully functioning autonomous robot deployment. We did not independently run the robot or verify autonomous behavior. The recovery also did not execute the corrected repository's fresh-clone Phase 1 procedure, so the directly inspected machine facts are historical compatibility evidence rather than proof of successful autonomy or current reproducibility.

## Recovered platform

| Item | Recovered value |
|---|---|
| Hardware | Raspberry Pi 4 |
| Operating system | Ubuntu 22.04.5 LTS, Jammy Jellyfish |
| Architecture | Linux `aarch64` |
| Kernel | `5.15.0-1102-raspi` |
| ROS | ROS 2 Humble |
| Python | 3.10.12 |
| ROS domain | `ROS_DOMAIN_ID=10` |
| Legacy workspace | `~/unilidar_sdk2/ws` |

Recovered machine inspection established every value in this table. It also established that the old shell configuration exported `ROS_DOMAIN_ID=10`, sourced `/opt/ros/humble/setup.bash`, and sourced the legacy workspace install overlay. Phase 1 records domain ID 10 as a recovered runtime default but does not require it to compile and does not modify global shell configuration.

Pi 5 compatibility cannot be inferred from this Pi 4 evidence.

## Recovered source packages

Recovered machine inspection of the old Pi's `colcon list` reported:

```text
esp_bridge
esp_fusion_pkg
fusion_pkg
gap_follow
lidar_logger_pkg
tof_processor_pkg
ultrasonic_pkg
unitree_lidar_ros2
unitree_lidar_sdk
vfh_pkg
vision_uart_pkg
```

**Verified from code** those same eleven source-package directories are present under this repository's `ws/src`. `unitree_lidar_sdk` is a plain CMake package rather than a ROS package manifest; the other ten have `package.xml` files.

The recovered `ws/` directory was untracked in the original `unilidar_sdk2` Git repository. **Documentation says** a checksum comparison against the pre-Phase-1 repository found the meaningful source independently preserved in `sensor-fusion-ugv`. The primary structural difference was the flattened `unitree_lidar_ros2` directory, and a direct comparison found no source differences apart from generated Python cache content. The flattened repository layout is therefore retained.

Generated `build/`, `install/`, `log/`, `__pycache__/`, `*.pyc`, and `lidar_data_*.csv` content is not authoritative source.

## Externally installed tools and ROS packages

Recovered machine inspection found these additional ROS packages under `/opt/ros/humble`:

- `pointcloud_to_laserscan` — legacy robot bring-up dependency;
- `teleop_twist_keyboard` — development/manual-test dependency; and
- `foxglove_bridge` — optional visualization/telemetry dependency.

Inspection also found colcon, rosdep, and tmux on the machine. The recovered rosdep version was 0.26.0 and was configured for Jammy. Their historical presence does not by itself prove which tools were required for every successful robot run. The current dependency categories and installation boundaries are maintained in the root [`README.md`](../../README.md#dependency-classification).

## Legacy runtime process list

The recovered untracked legacy `start_bot.sh` was inspected on the machine and contained commands for these Pi-side processes:

1. `unitree_lidar_ros2` launch file;
2. `pointcloud_to_laserscan_node`;
3. `vfh_pkg/vfh_node`;
4. `vision_uart_pkg/vision_uart_receiver_node`;
5. `vision_uart_pkg/camera_follow_node`;
6. `fusion_pkg/fusion_node`;
7. `vision_uart_pkg/safety_mux_node`;
8. `esp_bridge/cmd_vel_to_serial`; and
9. `foxglove_bridge`.

The recovered point-cloud projection command was:

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/unilidar/cloud \
  -r scan:=/scan \
  -p target_frame:=unilidar_lidar \
  -p min_height:=-0.10 \
  -p max_height:=1.0 \
  -p range_min:=0.15 \
  -p range_max:=20.0 \
  -p use_inf:=true
```

The motor bridge was launched with:

```bash
ros2 run esp_bridge cmd_vel_to_serial --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baud:=115200
```

These are historical commands, not approved Phase 1 or production bring-up. The projection frame/limits, serial identity, and hardware behavior still require the later verification defined by the roadmap.

The legacy launcher also flushed and reconfigured `eth0`. That behavior is intentionally not reproduced: it can disconnect an SSH session, and safe configurable LiDAR networking belongs to a later phase.

## Known-working versus reproducible

Recovered machine inspection establishes the platform, workspace inventory, shell configuration, installed packages/tooling, and legacy launcher contents recorded above. It does not independently establish that those processes ran successfully together, that the robot drove autonomously, which undocumented machine state was necessary, that all current source built there, or that the corrected branch works from a fresh image. Successful autonomous operation remains a previous-team handoff/history claim.

**Potential issue** — fresh-clone reproducibility remains unverified until the target procedure records the exact commit, prerequisite installation, rosdep resolution, clean colcon build/test, package/executable discovery, Unitree ARM64 link, new-shell check, reboot, and rerun.
