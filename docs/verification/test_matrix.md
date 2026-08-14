# Master Verification Matrix

## Use

This matrix is the planned acceptance inventory for the robot-ready roadmap. A requirement remains **Not Run** until its stated procedure is actually executed. Compilation, documentation, or static analysis alone does not pass a runtime or hardware claim. Update the status, commit/tag, and notes together, and add the resulting artifact to [`evidence_log.md`](evidence_log.md).

Allowed initial status is **Not Run**. Later results should use explicit terms such as **Pass**, **Fail**, **Blocked**, or **Not Applicable**, with evidence. “Automated” identifies software-executable checks; “hardware” identifies a physical test and must record whether wheels were raised or the robot was on a controlled floor.

## Phase 1 target-Pi acceptance procedure

Run this procedure on a fresh supported Raspberry Pi image. It verifies build, install, test, and package discovery only; it does not require any hardware node to connect to a device.

```bash
git clone <repository-url>
cd sensor-fusion-ugv

uname -a
uname -m
lsb_release -a || cat /etc/os-release
source /opt/ros/jazzy/setup.bash
printenv ROS_DISTRO

rosdep update
rosdep install --from-paths ws/src --ignore-src -r -y

chmod +x build_ws.sh scripts/verify_build.sh scripts/validate_ros_packages.py
./build_ws.sh
source ws/install/setup.bash

cd ws
colcon test
colcon test-result --verbose
cd ..

ros2 pkg executables esp_bridge
ros2 pkg executables esp_fusion_pkg
ros2 pkg executables fusion_pkg
ros2 pkg executables gap_follow
ros2 pkg executables lidar_logger_pkg
ros2 pkg executables tof_processor_pkg
ros2 pkg executables ultrasonic_pkg
ros2 pkg executables unitree_lidar_ros2
ros2 pkg executables vfh_pkg
ros2 pkg executables vision_uart_pkg
```

Record the clone commit, environment inventory, rosdep output, build/test summaries, executable output, and the Unitree link result. The SDK link must resolve the checked-in `ws/src/unitree_lidar_sdk/lib/aarch64/libunilidar_sdk2.a` without a source edit.

| ID | Subsystem | Requirement / claim | Phase | Verification method | Automated or hardware | Expected evidence | Status | Commit/tag | Notes |
|---|---|---|---:|---|---|---|---|---|---|
| BUILD-001 | Dependency resolution | A fresh target-Pi clone resolves all declared workspace dependencies through the documented rosdep procedure. | 1 | Run `rosdep update` and `rosdep install --from-paths ws/src --ignore-src -r -y` on a clean supported Pi image. | Deployment + automated | Environment inventory, clone commit, and complete rosdep transcript. | Not Run | — | Must not use undeclared developer-machine or global-pip state. |
| BUILD-002 | Clean build | The intended workspace builds on Ubuntu 24.04 / ROS 2 Jazzy / Linux `aarch64`. | 1 | Run `./build_ws.sh` after BUILD-001 on the fresh target Pi. | Automated on target | Colcon build summary and compiler/linker output. | Not Run | — | macOS static checks do not pass this target claim. |
| BUILD-003 | Workspace tests | Workspace tests complete without build, install, or packaging failures. | 1 | Source `ws/install/setup.bash`, run `colcon test`, then `colcon test-result --verbose`. | Automated on target | Colcon test summary and detailed results. | Not Run | — | Hardware devices need not be connected. |
| BUILD-004 | Build tooling | Developer build/verification scripts locate the workspace reliably, pass shell/static validation, and fail clearly when Jazzy is unavailable. | 1 | Run `bash -n` on shell tools, run the static validator, and exercise the missing-Jazzy prerequisite path. | Automated static | Phase 1 static-validation artifact and exact command output. | Pass | Pending (uncommitted; base `20f5563`) | Passed on macOS 26.5.1 ARM64; this does not replace BUILD-001 through BUILD-003. |
| PKG-001 | Installed packages | Every checked-in ROS executable is installed and discoverable through `ros2 pkg executables`. | 1 | Run the executable-discovery commands above or `./scripts/verify_build.sh` after a clean build. | Automated on target | Command output for all expected package/executable pairs. | Not Run | — | No hardware connection is required. |
| UNITREE-BUILD-001 | Unitree SDK linkage | The Unitree ROS wrapper resolves the checked-in sibling SDK include path and links the matching `aarch64` Linux archive. | 1 | Inspect configuration diagnostics and linker input/output during a clean `aarch64` build. | Automated on target | CMake configuration and link log tied to the vendor archive checksum. | Not Run | — | Static inspection confirms paths and archive architecture; Pi linkage remains pending. |
| DEP-001 | Dependency metadata | All current runtime Python imports, C++/launch includes, and external serial/PCL requirements are represented in package metadata. | 1 | Run `python3 scripts/validate_ros_packages.py` and perform a direct manifest/source comparison. | Automated static | Static audit output and reviewed manifest diff. | Pass | Pending (uncommitted; base `20f5563`) | Verified against current source; clean rosdep resolution remains BUILD-001. |
| LIDAR-001 | Unitree driver | Unitree driver starts successfully and reports initialization failure deterministically. | 2 | Automated failure-path test and physical device startup using approved transport/configuration. | Automated + hardware | Test report, driver logs, model/firmware/transport record. | Not Run | — | Exact model must be confirmed, not inferred from older docs. |
| LIDAR-002 | Unitree cloud | `/unilidar/cloud` is fresh and contains valid finite point fields at the required operating rate. | 2 | Inspect topic type/rate/age and record a stationary/motion sample. | Hardware | Topic statistics and rosbag/capture tied to sensor configuration. | Not Run | — | Acceptance bounds must be specified before test. |
| TF-001 | TF | Unitree sensor frames are connected to `base_link` with measured transform and valid orientation/timestamps. | 2 | TF graph inspection, quaternion checks, static placement checks, and measured extrinsic review. | Automated + hardware | TF graph, test output, measurement worksheet, configuration. | Not Run | — | Current repository trees are disconnected. |
| SCAN-001 | Scan projection | A production component publishes `/scan` from the approved Unitree path. | 3 | Launch production sensor path and verify one expected publisher/type/QoS/rate. | Automated + hardware | Launch-test output and ROS graph/topic capture. | Not Run | — | Current source has no `/scan` producer. |
| SCAN-002 | Scan validity | `/scan` contains finite/in-range data under the defined validity policy and flags/stops safely when source data is unusable. | 3 | Synthetic valid/NaN/Inf/empty/cloud-loss fixtures plus live observation. | Automated + hardware | Fixture results, topic capture, fault trace. | Not Run | — | Do not equate “message exists” with valid ranges. |
| SCAN-003 | Scan geometry | Scan zero angle is robot-forward, positive angle convention is confirmed, and measured obstacle ranges/bearings match placement. | 3 | Place targets at measured robot-frame positions and compare projected ranges/bearings. | Hardware | Test layout, measurements, scan capture, error summary. | Not Run | — | Requires confirmed LiDAR mounting transform. |
| CMD-001 | Command ownership | Production bring-up has exactly one final `/cmd_vel` publisher. | 4 | Launch graph test and runtime publisher-count check for every approved production mode. | Automated | ROS graph and assertion output. | Not Run | — | Candidate and experimental topics must remain distinct. |
| CMD-002 | Serial ownership | Production and each test mode have exactly one intended actuator serial owner with no device collision. | 4 | Configuration/launch assertions plus runtime device/process inspection. | Automated + hardware | Mode matrix, process/device trace, startup logs. | Not Run | — | Current defaults can collide on `/dev/ttyACM0`. |
| CMD-003 | Startup enable | Production bring-up starts stopped and motion requires explicit autonomous enable. | 5 | Launch/state-transition test with candidate commands present before and after enable. | Automated + wheels-raised hardware | State and `/cmd_vel` trace; wheels-raised observation. | Not Run | — | Enable/reset authority must be documented. |
| SAFE-001 | Scan freshness | A stale or lost required `/scan` causes defined final safe behavior. | 5 | Stop scan publication during commanded autonomy and measure supervisor/final-command transition. | Automated + wheels-raised hardware | Timestamps, health state, `/cmd_vel`, and motor trace. | Not Run | — | Safe timeout and bound require approval. |
| SAFE-002 | Invalid scan | Empty or invalid-but-nonempty required scans cannot produce unsafe motion. | 5 | Inject empty, all-NaN, all-Inf, zero, negative, and malformed range sets. | Automated | Test assertions and final-command trace. | Not Run | — | Cover candidate and supervisor behavior. |
| SAFE-003 | Fusion freshness | Stale/missing/invalid required fusion input produces the documented safe state. | 5 | Drop or corrupt each required fusion input and stop `fusion_node`; inspect final arbitration. | Automated | Fault-injection logs, health/fault state, `/cmd_vel` trace. | Not Run | — | Missing data must not be accidentally clear. |
| SAFE-004 | Critical stop | Any approved critical-stop source produces immediate zero at final `/cmd_vel` independent of candidate smoothing. | 5 | Inject each critical-stop source while camera/VFH candidates are nonzero. | Automated + wheels-raised hardware | Latency trace, supervisor mode, final command, motor response. | Not Run | — | Physical stop latency is separately measured. |
| SAFE-005 | Final limits | Confirmed safe linear/angular limits are enforced in the final supervisory layer for every candidate. | 5 | Boundary/fuzz tests using over-limit positive and negative commands, then wheels-raised confirmation. | Automated + wheels-raised hardware | Clamp assertions, configuration provenance, command/motor trace. | Not Run | — | Values must be measured/approved, never guessed. |
| ESP-001 | MCU command timeout | Loss of fresh host commands causes the MCU/motors to stop within the approved bound. | 6–7 | Interrupt command stream while a wheels-raised motion command is active; measure response. | Hardware | Host/MCU/motor timestamp trace and timeout configuration. | Not Run | — | Host watchdog alone does not prove MCU behavior. |
| ESP-002 | Serial disconnect | Runtime serial disconnect causes a safe stop and defined latched/recovery behavior. | 7 | Disconnect/reconnect the approved serial link during wheels-raised motion. | Automated + hardware | Fault logs, final command, motor trace, recovery outcome. | Not Run | — | Test exact device identity and ownership. |
| ESP-003 | Motor signs | Positive/negative linear and angular commands produce verified forward/reverse and left/right behavior. | 6–7 | Apply bounded commands with wheels raised and record each wheel/direction response. | Hardware | Signed command table, video/log reference, robot configuration. | Not Run | — | Include motor polarity and ROS angular convention. |
| ESP-004 | Firmware compatibility | Checked-in firmware builds, reports a version, and matches the host command/telemetry contract. | 6 | Reproducible firmware build plus protocol conformance fixtures and connected bench exchange. | Automated + hardware | Firmware hash, build log, version handshake, protocol test report. | Not Run | — | Firmware is absent in the current repository. |
| ESTOP-001 | Emergency stop | Physical emergency stop overrides autonomy and stops motion within the approved measured bound. | 7 | Activate physical E-stop during controlled wheels-raised and low-speed tests. | Hardware | Wiring/config record, timestamped motor trace, stopping distance/time. | Not Run | — | Software stop documentation cannot substitute for this test. |
| BASE-001 | Gap baseline | Mutually exclusive `gap_follow` mode avoids the defined obstacle course without command-owner conflict. | 8 | Run synthetic fixtures, then repeated conservative controlled-course trials. | Automated + hardware | ROS graph, fixtures, course layout, rosbag/CSV/video, trial results. | Not Run | — | Baseline only; not the production target. |
| BASE-002 | Gap baseline faults | Baseline mode stops safely on scan loss/invalidity and bridge/serial faults. | 8 | Fault injection in simulation and wheels-raised baseline launch. | Automated + hardware | Fault matrix and command/motor traces. | Not Run | — | Must use final safety policy or a documented safe exclusive harness. |
| FUSION-001 | ToF calibration | ToF baseline and persistence use unique sensor observations, not repeated timer processing of one cached value. | 9 | Feed repeated timestamp/sequence fixtures and assert sample/count behavior. | Automated | Unit-test output and captured fixture definition. | Not Run | — | Current code counts timer executions. |
| FUSION-002 | ToF drop-off | Confirmed ToF drop-off conditions produce a critical stop at final supervision. | 9–10 | Replay calibrated data and run controlled edge/drop-off surrogate tests without exposing robot to an uncontrolled fall. | Automated + hardware | Dataset provenance, feature/fusion/stop trace, safe test setup. | Not Run | — | Requires confirmed ToF units/orientation/aggregation. |
| FUSION-003 | Ultrasonic stop | A valid ultrasonic critical-distance event produces final zero; invalid/no-echo/stale behavior matches policy. | 9–10 | Inject boundary/invalid/stale fixtures and perform measured obstacle tests. | Automated + hardware | Parser/fusion assertions, sensor placement, distance measurements, stop trace. | Not Run | — | Include firmware validity semantics. |
| FUSION-004 | Fusion determinism | Given timestamped sensor fixtures, sector penalties, speed limit, stop, and health state match the documented deterministic rules. | 9 | Unit/property tests at thresholds, invalid values, and combined hazards. | Automated | Test vectors and exact output report. | Not Run | — | Do not label this as Kalman filtering. |
| VFH-001 | Rich VFH course | Rich VFH plus fusion completes the defined controlled low-speed obstacle-avoidance course. | 10 | Automated scan/fusion fixtures followed by repeated controlled course trials. | Automated + hardware | Configuration, course layout, rosbag/CSV/video, clearance/intervention/trial summary. | Not Run | — | Production candidate uses final supervisor. |
| VFH-002 | Rich VFH safety cases | Rich VFH handles blocked path, corners, invalid scan, fusion stop, and recovery without violating final safety rules. | 10 | Scenario fixtures and controlled physical cases with stop/enable monitoring. | Automated + hardware | Scenario report and synchronized candidate/final/motor traces. | Not Run | — | Open-loop recovery assumptions require measurement. |
| VISION-001 | Vision fallback | Lost, stale, malformed, or low-confidence vision input falls back or stops exactly as specified without bypassing safety. | 11 | UART fixture tests and runtime dropout/corruption injection with VFH available/unavailable. | Automated + hardware | Protocol fixtures, supervisor traces, fallback mode assertions. | Not Run | — | Vision is optional after core autonomy. |
| VISION-002 | Vision convention | Target class, UART angle unit/sign, camera transform, and turn response are confirmed. | 11 | Measured target placements and bounded wheels-raised/controlled-floor response checks. | Hardware | Protocol/config record, measurement table, command/robot trace. | Not Run | — | No convention may be guessed. |
| DEPLOY-001 | Fresh Pi deployment | Fresh target-Pi deployment reaches a passing preflight and starts the approved graph from one command. | 12 | Execute the versioned deployment runbook on a wiped/new supported image. | Deployment + hardware | Full transcript, inventory, preflight, ROS graph, config/firmware hashes. | Not Run | — | Robot must remain stopped after bring-up. |
| DEPLOY-002 | Preflight | Preflight rejects missing/wrong devices, incompatible firmware, bad TF/topics, extra command owners, and unavailable stop paths. | 12 | Automated matrix of valid and intentionally invalid configurations plus live device checks. | Automated + hardware | Case report with expected/actual decision and diagnostic output. | Not Run | — | No autonomous enable on failed preflight. |
| RELEASE-001 | Release candidate | Frozen release candidate passes the full required matrix and repeated autonomous trials. | 13 | Run reviewed release procedure across required software, deployment, wheels-raised, and controlled-floor cases. | Automated + hardware | Completed matrix, release commit/tag/configuration, artifacts, trial summary. | Not Run | — | Failed trials are recorded, not omitted. |
| RELEASE-002 | Evidence traceability | Every release/manuscript result is traceable to commit, configuration, procedure, environment, and artifact. | 13 | Cross-audit release notes, matrix, evidence log, and planned manuscript claims. | Documentation audit | Traceability report with no orphan result claims. | Not Run | — | Planned/proposed statements need no false pass result. |

## First-release gate

The first robot-ready release cannot pass while any applicable required row is **Not Run**, **Fail**, or lacks a traceable artifact. Optional vision rows may be formally marked **Not Applicable** for a release that excludes camera following, with the scope recorded in the release evidence.
