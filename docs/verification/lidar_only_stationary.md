# Jetson LiDAR-only stationary correction — 2026-11-01

User-authorized installation on existing `fix/02-unitree-driver-and-tf`, working tree atop `775a357`. Existing vision receiver edits and untracked `start_bot.sh` / `start_bot.sh.save` were preserved. This is an exception for stationary validation, not acceptance of later roadmap phases.

## Installed sources and configuration

Archive SHA-256: `5e73f77e1067396cba5e327e4f665a4a47f68d360c9bc70fecf170a935d8517e`. All member paths, package checksums, and original target hashes matched before installation. Archive: `/home/sdp2026/ugv-lidar-only-fix.tar.gz`; extracted original package: `/home/sdp2026/ugv-lidar-only-fix`.

Installer backup: `/home/sdp2026/sensor-fusion-ugv-migration/ws/ugv-lidar-backups/20261101T230850.204824Z`. The three installed sources match the bundle's patched bytes. Installer added `sensor_msgs` to `vision_uart_pkg/package.xml`. Initial build failed because the backed-up manifest was discovered as a duplicate package. Added `ws/ugv-lidar-backups/COLCON_IGNORE` before retrying successfully. Keep this marker when restoring or reinstalling. Backups are locally ignored by Git.

**Verified from code** — VFH rejects unusable/stale scans, treats unknown sectors as closed, retains hazards briefly, removes its positive speed floor and blind escape. Mux independently guards scans and chooses the VFH candidate in explicit LiDAR-only mode. Bridge bounds requests and latches serial faults. Firmware and encoder motor-speed control are unchanged.

Profile: [`../../config/lidar_only.yaml`](../../config/lidar_only.yaml), identical to the active `/home/sdp2026/ugv-lidar-only-fix/lidar_only.yaml`. Requested limits 0.08 m/s and 0.4 rad/s; stop distance 0.50 m, slowdown 1.0 m; receipt/candidate timeouts 0.25 s; maximum source age 0.5 s; hazard hold 0.5 s. These came from the supplied profile and are provisional, not measured safe limits.

## Commands and results

```bash
python3 ~/ugv-lidar-only-fix/install.py
cd ~/sensor-fusion-ugv-migration/ws
source /opt/ros/humble/setup.bash
# Exclude installer backup manifests before building:
touch ugv-lidar-backups/COLCON_IGNORE
colcon build --packages-select vfh_pkg vision_uart_pkg esp_bridge --symlink-install
source install/setup.bash
ros2 run vfh_pkg vfh_node --ros-args --params-file ~/ugv-lidar-only-fix/lidar_only.yaml
# In another sourced terminal:
ros2 run vision_uart_pkg safety_mux_node --ros-args --params-file ~/ugv-lidar-only-fix/lidar_only.yaml
# In a spare sourced terminal:
python3 ~/ugv-lidar-only-fix/scan_probe.py
```

The build completed all three packages in 4.12 s. `python3 -m unittest discover -s tests -v` in the original bundle passed 48 tests. `python3 -m unittest discover -s tests/lidar_only -v` from this repository passed 43 callback regressions against actual deployed sources (including three original-defect reproductions using the historical fixture). Package validator and `git diff --check` passed. No full workspace lint/test claim is made.

Environment: NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super; Linux aarch64; Ubuntu 22.04.5 LTS; ROS 2 Humble; Python 3.10. Motor bridge stayed stopped; `/dev/ttyACM0` had no open owner at checks. Physical disable state, wheel position, target placement, sensor mounting and measured obstacle geometry were not independently verified.

## Observed runtime evidence

The original converter settings were restored without tuning: target frame `unilidar_lidar`, height -0.10 to 1.0 m, range 0.15 to 20.0 m, `use_inf=true`, `/unilidar/cloud` -> `/scan`. A launch initially failed due to unfinished shell input in the idle pane; clearing the line and repeating succeeded. Cloud observation was approximately 11.94 Hz. One `/scan` publisher and one final `/cmd_vel` publisher (`safety_mux_node`) were observed; final `/cmd_vel` had no subscribers during the initial ownership check.

During the observer baseline, **20 of 111 received scans had no finite front returns**. Probe samples alternated between `front_finite=0/30`, `front_inf=30`, and finite readings around 0.62–0.65 m. New VFH output included `[STOP] no_finite_front_returns` and `[STOP] no_observed_open_valley`; an example `observed_front=3.50` still retained `front_min=0.62` through hazard history. This establishes intermittent missing front data at `/scan`; the cause within cloud acquisition/projection/geometry remains unknown. It does not verify a physically held obstacle or physical clearance.

The read-only observer briefly stopped/restored the converter, then VFH. After 0.7 s settling, all 21 sampled final commands during scan loss and all 22 during VFH loss were zero in both axes. These are **verified by automated software test** with live ROS transport. They do not measure a strict 250 ms bound or physical stopping latency. Maximum sampled requests across this run were 0.018369 m/s and 0.290150 rad/s, below configured caps. VFH and converter were restored afterward.

VFH shutdown still raises `RCLError` when it tries publishing after the ROS context closes (seen before and after installation). The candidate-loss test confirms the mux watchdog stop; clean node shutdown is not verified by this work.

## Current panes and remaining checks

`bot_stack:0.0` LiDAR; `0.1` converter; `0.2` patched VFH; `0.3` probe; `0.6` patched mux. Camera/fusion panes stopped; motor bridge pane `0.7` stopped. Do not use the existing `start_bot.sh` for stationary validation: it starts the motor bridge and does not select this profile.

**Not yet hardware verified** — measured target placement, scan orientation/mount transform, stop distance, emergency stop, MCU watchdog, wheels-raised motor behavior, and controlled low-speed floor behavior. No motor commands were sent over serial. This profile's graph can emit bounded candidate/final requests; the bridge must remain stopped for these checks.

Continue stationary target/projection diagnosis before considering motor testing. A true clear area may also stop when projected returns are all infinite. Do not relax guards to bypass missing observations.

## Artifacts

- [Validation/build transcript and hashes](artifacts/jetson_lidar_only_validation_2026-11-01.txt)
- [Live observer events, scans, commands, debug and results](artifacts/jetson_lidar_only_runtime_2026-11-01.json)
- [Exact live observer procedure](artifacts/jetson_lidar_only_runtime_2026-11-01.py) (uses the named tmux panes and briefly interrupts converter/VFH; bridge must already be stopped)
- [VFH pane](artifacts/jetson_lidar_only_vfh_2026-11-01.log), [scan probe pane](artifacts/jetson_lidar_only_probe_2026-11-01.log), [converter pane](artifacts/jetson_lidar_only_converter_2026-11-01.log)

Recorded 2026-11-01T15:12:32-08:00. Uncommitted source/config hashes in the transcript identify this working-tree result; no commit, push or merge was performed.
