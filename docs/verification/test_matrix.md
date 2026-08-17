# Master Verification Matrix

## Use

This matrix is the planned acceptance inventory for the robot-ready roadmap. A requirement remains **Not Run** until its stated procedure is actually executed. Compilation, documentation, or static analysis alone does not pass a runtime or hardware claim. Update the status, commit/tag, and notes together, and add the resulting artifact to [`evidence_log.md`](evidence_log.md).

Allowed initial status is **Not Run**. Later results should use explicit terms such as **Pass**, **Fail**, **Blocked**, or **Not Applicable**, with evidence. “Automated” identifies software-executable checks; “hardware” identifies a physical test and must record whether wheels were raised or the robot was on a controlled floor.

## Phase 1 target-Pi acceptance procedure

Run this procedure on a freshly imaged Raspberry Pi 4 with Ubuntu 22.04 LTS ARM64 and ROS 2 Humble installed using the official ROS instructions. Replace both placeholders before testing. It verifies dependency resolution, build, install, tests, and package discovery only; it must not start a LiDAR, serial, motor, Ethernet-configuration, or autonomous process.

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep
test -e /etc/ros/rosdep/sources.list.d/20-default.list || sudo rosdep init

git clone <repository-url>
cd sensor-fusion-ugv
git checkout <exact-phase-1-commit>

uname -a
uname -m
cat /proc/device-tree/model
cat /etc/os-release
python3 --version
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
git rev-parse HEAD

rosdep update
rosdep install --from-paths ws/src --ignore-src --rosdistro humble -r -y

chmod +x build_ws.sh scripts/verify_build.sh scripts/validate_ros_packages.py
./scripts/verify_build.sh
source ws/install/setup.bash

colcon list --base-paths ws/src
cat ws/build/unitree_lidar_ros2/unitree_sdk_library.txt
test "$(cat ws/build/unitree_lidar_ros2/unitree_sdk_library.txt)" = \
  "${PWD}/ws/src/unitree_lidar_sdk/lib/aarch64/libunilidar_sdk2.a"

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

bash --noprofile --norc -c \
  'source /opt/ros/humble/setup.bash && source ws/install/setup.bash && ros2 pkg executables unitree_lidar_ros2 && ros2 pkg executables vision_uart_pkg'

git status --short
sudo reboot
```

After the Pi reconnects, repeat the critical verification without relying on `.bashrc` state:

```bash
cd sensor-fusion-ugv
git rev-parse HEAD
./scripts/verify_build.sh
source /opt/ros/humble/setup.bash
source ws/install/setup.bash
ros2 pkg executables unitree_lidar_ros2
ros2 pkg executables vision_uart_pkg
git status --short
```

Record the clone commit, environment inventory, rosdep output, build/test summaries, executable output, and the Unitree link result. The SDK link must resolve the checked-in `ws/src/unitree_lidar_sdk/lib/aarch64/libunilidar_sdk2.a` without a source edit.

The recovered legacy value `ROS_DOMAIN_ID=10` is not needed for these build/discovery checks and must not be added to `.bashrc` by this procedure.

## Phase 1 evidence boundaries

| Evidence class | What it can establish | Current status |
|---|---|---|
| Static validation on macOS | Manifest/import/CMake/script consistency, archive format/architecture inspection, and clean missing-Humble diagnostics. | Pass for the recorded commands only; cannot prove ROS or target linkage. |
| Target ARM64 build validation | Rosdep resolution, all eleven packages discovered by colcon, clean compile/link/install, tests, and executable discovery on the supported baseline. | Clean discovery/build/install passed for the current Phase 2 working tree (`BUILD-002-R03`); full tests fail on inherited lint (`BUILD-003-R05`). Fresh-image rosdep/clone/reboot and complete executable discovery remain pending. |
| Runtime dependency validation | Availability of later legacy/manual/visualization packages and ROS graph/runtime behavior when explicitly installed and launched. | Not Run; not a Phase 1 build prerequisite. |
| Hardware validation | LiDAR, serial, motors, networking, safety, and autonomy behavior. | Deferred to later phases; no Phase 1 pass is claimed. |

## Phase 2 Pi/LiDAR acceptance procedure

The consolidated build-only focused command has passed on the tested Pi; every physical step still requires execution. Hardware portions are not authorized for unattended execution. Begin with the motors physically disabled or disconnected; Phase 2 must not move the robot. Run no navigation, fusion, `/cmd_vel`, ESP bridge, or motor process. Record the exact Pi, operating system, ROS version, physical LiDAR model/hardware/firmware identifiers, connection method, measured configuration provenance, Git commit, and every terminal transcript. The checked-in software's SDK2/L2-family lineage is not a substitute for inspecting the physical sensor.

If the LiDAR uses a dedicated `eth0`, do not flush or readdress the interface carrying the active SSH session. Use Wi-Fi, a second interface, or a local keyboard/console for administration. Configure the dedicated LiDAR interface through a separately reviewed persistent network procedure; the commands below do not change host networking.

First verify the supported environment, build, installed resources, and deterministic invalid-configuration failure without opening the LiDAR transport:

```bash
cd sensor-fusion-ugv
export PHASE2_COMMIT='<accepted-phase-2-commit>'
test "${PHASE2_COMMIT}" != '<accepted-phase-2-commit>'
git checkout --detach "${PHASE2_COMMIT}"
git rev-parse HEAD
git status --short --branch
uname -a
uname -m
cat /proc/device-tree/model
cat /etc/os-release
python3 --version

source /opt/ros/humble/setup.bash
./scripts/test_source_setup.sh
./scripts/verify_unitree_phase2.sh
./scripts/verify_build.sh
source ws/install/setup.bash

python3 scripts/validate_ros_packages.py
python3 scripts/validate_unitree_driver.py
ros2 launch unitree_lidar_ros2 launch.py --show-args

expect_driver_rejection() {
  local label="$1"
  shift
  if ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args "$@"; then
    echo "ERROR: ${label} unexpectedly succeeded" >&2
    return 1
  fi
  echo "PASS: ${label} was rejected before SDK transport initialization"
}

expect_driver_rejection "invalid initialize_type" -p initialize_type:=0
expect_driver_rejection "reserved work_mode bit" -p work_mode:=32
expect_driver_rejection "UDP with serial work-mode bit" -p initialize_type:=2 -p work_mode:=8
expect_driver_rejection "serial with Ethernet work-mode bit" -p initialize_type:=1 -p work_mode:=0
expect_driver_rejection "unsupported 2D work mode" -p work_mode:=2
expect_driver_rejection "wait-for-start work mode" -p work_mode:=16
expect_driver_rejection "unsupported hardware timestamps" -p use_system_timestamp:=false
expect_driver_rejection "IMU-disabled mode with dynamic IMU TF" \
  -p work_mode:=4 -p publish_imu_tf:=true
```

Create the editable test configuration outside the Git worktree and edit it only with confirmed values. Record the source of each changed hardware value. Keep `use_system_timestamp=true`. Valid wrapper modes retain 3D measurement and automatic rotation start: UDP uses `0` (or `1` for wide FOV), serial uses `8` (or `9`), and IMU-disabled variants add `4` while requiring `publish_imu_tf=false`. Do not use 2D, wait-for-start, reserved-bit, or transport-mismatched modes.

To enable the robot-connected tree, set `publish_imu_tf` to `false`, set `publish_robot_to_imu_tf` to `true`, set `robot_frame` to the confirmed robot frame, and add all seven measured `robot_to_imu.translation.{x,y,z}` and `robot_to_imu.rotation.{x,y,z,w}` values. If those measurements are unavailable, leave the measured transform disabled and record TF physical acceptance as Not Run; do not insert an identity transform.

```bash
cd sensor-fusion-ugv
source /opt/ros/humble/setup.bash
source ws/install/setup.bash

export PHASE2_VALIDATION_DIR="${HOME}/phase2-validation"
mkdir -p "${PHASE2_VALIDATION_DIR}"
export PHASE2_CONFIG="${PHASE2_VALIDATION_DIR}/phase2_unitree_measured.yaml"
cp ws/src/unitree_lidar_ros2/config/driver.defaults.yaml "${PHASE2_CONFIG}"
${EDITOR:?Set EDITOR to review the Phase 2 configuration} "${PHASE2_CONFIG}"
sha256sum "${PHASE2_CONFIG}"
git rev-parse HEAD
git status --short
```

In terminal A, launch only the driver with RViz disabled:

```bash
cd sensor-fusion-ugv
source /opt/ros/humble/setup.bash
source ws/install/setup.bash
export PHASE2_CONFIG="${HOME}/phase2-validation/phase2_unitree_measured.yaml"
ros2 launch unitree_lidar_ros2 launch.py \
  config_file:="${PHASE2_CONFIG}" \
  use_rviz:=false
```

In terminal B, verify the exact default namespace/topic/type/header/QoS contract and record rates long enough for a reviewed stable measurement. If IMU packets are disabled or unsupported by the confirmed hardware, record that outcome rather than claiming an IMU pass.

```bash
cd sensor-fusion-ugv
source /opt/ros/humble/setup.bash
source ws/install/setup.bash

ros2 node list
ros2 node info /unitree_lidar_ros2_node
ros2 topic list -t
test "$(ros2 topic type /unilidar/cloud)" = "sensor_msgs/msg/PointCloud2"
test "$(ros2 topic type /unilidar/imu)" = "sensor_msgs/msg/Imu"
ros2 topic info --verbose /unilidar/cloud
ros2 topic info --verbose /unilidar/imu
ros2 param get /unitree_lidar_ros2_node initialize_type
ros2 param get /unitree_lidar_ros2_node work_mode
ros2 param get /unitree_lidar_ros2_node use_system_timestamp
ros2 param get /unitree_lidar_ros2_node cloud_topic
ros2 param get /unitree_lidar_ros2_node cloud_frame
ros2 param get /unitree_lidar_ros2_node imu_topic
ros2 param get /unitree_lidar_ros2_node imu_frame
ros2 topic echo /unilidar/cloud --once --field header
ros2 topic echo /unilidar/cloud --once --field fields
ros2 topic echo /unilidar/cloud --once --field height
ros2 topic echo /unilidar/cloud --once --field width
ros2 topic hz /unilidar/cloud
```

When work-mode bit 2 is zero and IMU is enabled, also run:

```bash
ros2 topic echo /unilidar/imu --once --field header
ros2 topic echo /unilidar/imu --once --field orientation
ros2 topic hz /unilidar/imu
```

When work-mode bit 2 is one and IMU is deliberately disabled, `/unilidar/imu` remains advertised but must be silent. Confirm that no sample arrives during a recorded ten-second observation:

```bash
if timeout 10s ros2 topic echo /unilidar/imu --once; then
  echo "ERROR: IMU data arrived although work_mode disables IMU" >&2
  exit 1
else
  imu_echo_status=$?
  if [[ "${imu_echo_status}" -ne 124 ]]; then
    echo "ERROR: IMU observation failed with status ${imu_echo_status}, not timeout status 124" >&2
    exit 1
  fi
  echo "PASS: no IMU sample arrived during the 10-second disabled-mode observation"
fi
```

Stop each `ros2 topic hz` command with Ctrl-C after the chosen interval and retain its sample count, average, minimum, maximum, and standard deviation. A message's presence is not proof that every point is finite or physically correct; retain a rosbag or equivalent sample for reviewed field-level inspection. The default dynamic initial-IMU TF is a legacy-compatible software mapping: do not mark its physical direction/sign semantics verified until stationary and controlled-motion data establish the quaternion reference-frame interpretation.

Verify static and dynamic TF according to the reviewed configuration:

```bash
ros2 topic echo /tf_static tf2_msgs/msg/TFMessage \
  --qos-durability transient_local --once
ros2 topic echo /tf tf2_msgs/msg/TFMessage --once
ros2 run tf2_ros tf2_echo unilidar_imu unilidar_lidar
ros2 run tf2_tools view_frames
```

For the default sensor-only configuration, also run:

```bash
ros2 run tf2_ros tf2_echo unilidar_imu_initial unilidar_imu
```

For an enabled measured robot-connected configuration, use the confirmed frame name and verify the composed robot-to-cloud transform instead. The command intentionally has no guessed frame default:

```bash
export PHASE2_ROBOT_FRAME='<confirmed-robot-frame>'
test "${PHASE2_ROBOT_FRAME}" != '<confirmed-robot-frame>'
ros2 run tf2_ros tf2_echo "${PHASE2_ROBOT_FRAME}" unilidar_imu
ros2 run tf2_ros tf2_echo "${PHASE2_ROBOT_FRAME}" unilidar_lidar
```

Stop each `tf2_echo` with Ctrl-C after retaining representative output. Review `view_frames` output for exactly one parent per child, expected `/tf` versus `/tf_static` ownership, no duplicate authority, and no unmeasured transform. Confirm that no unrelated process supplied `/scan`:

```bash
if ros2 topic list | grep -Fx /scan; then
  echo "ERROR: unexpected /scan exists during the Phase 2-only test" >&2
  exit 1
else
  echo "PASS: Phase 2 driver does not provide /scan"
fi
```

In terminal A, press Ctrl-C once. Then verify clean process exit in terminal B:

```bash
if ros2 node list | grep -Fx /unitree_lidar_ros2_node; then
  echo "ERROR: Unitree node remains after launch shutdown" >&2
  exit 1
else
  echo "PASS: Unitree node exited"
fi
```

Finally, on a Pi with an available display or correctly configured remote display, deliberately test the optional visualization and record the observation; this is separate from the headless pass:

```bash
cd sensor-fusion-ugv
source /opt/ros/humble/setup.bash
source ws/install/setup.bash
export PHASE2_CONFIG="${HOME}/phase2-validation/phase2_unitree_measured.yaml"
ros2 launch unitree_lidar_ros2 launch.py \
  config_file:="${PHASE2_CONFIG}" \
  use_rviz:=true
```

Press Ctrl-C once and confirm both processes exit. Repeat the approved headless subset after reboot before claiming deployment persistence. Phase 2 records cloud/IMU/TF behavior only; it makes no `/scan`, obstacle-avoidance, command, motor, or autonomy claim.

## Phase 2 evidence boundaries

| Evidence class | What it can establish | Current status |
|---|---|---|
| Host/target static validation | Declared parameter/default agreement, launch structure, source-level frame/TF/QoS invariants, installed-resource rules, architecture-selection logic, and unchanged vendor archive hashes. | Pass on macOS (`LIDAR-SW-001-R02`) and the isolated Pi/Humble target (`LIDAR-SW-001-R03`) for the tested uncommitted working tree. |
| ROS 2 Humble build/install validation | C++ compilation/linking, selected ARM64 SDK input, installed Unitree resources, and launch argument parsing. | Pass for the tested Pi working tree (`BUILD-002-R03`, `UNITREE-BUILD-001-R03`). This is not fresh-image reproducibility or physical runtime evidence. |
| Full workspace test gate | All package tests, including inherited Python lint tests. | Fail (`BUILD-003-R05`) only on inherited Flake8/PEP257 findings in eight non-Unitree packages. No Unitree build/test failure was reported; the gate remains strict. |
| Focused Phase 2 target gate | Repeatable isolated Unitree clean build/test/install/executable/invalid-configuration verification through `scripts/verify_unitree_phase2.sh`. | Pass (`LIDAR-SW-002-R01`) on the isolated Pi. Unitree `colcon test` discovered zero tests, so this passes the command/test gate but establishes no unit-test coverage. |
| Physical Unitree validation | Initialization, transport, actual cloud/IMU content and rate, timestamp behavior, physical TF correctness, clean hardware shutdown, reconnection, and reboot behavior. | Not Run. |
| Robot-connected TF measurement | Confirmed robot frame plus seven measured `robot_to_imu` values and confirmed sensor-internal extrinsic, with one TF parent per child. | Not Run; no numeric robot extrinsic is supplied by default. |

| ID | Subsystem | Requirement / claim | Phase | Verification method | Automated or hardware | Expected evidence | Status | Commit/tag | Notes |
|---|---|---|---:|---|---|---|---|---|---|
| BUILD-001 | Dependency resolution | A fresh target-Pi clone resolves all declared workspace dependencies through the documented rosdep procedure. | 1 | Run `rosdep update` and `rosdep install --from-paths ws/src --ignore-src --rosdistro humble -r -y` on a clean supported Pi image. | Deployment + automated | Environment inventory, clone commit, and complete rosdep transcript. | Not Run | — | Must not use undeclared developer-machine or global-pip state. |
| BUILD-002 | Clean build | The intended workspace builds on Raspberry Pi 4 / Ubuntu 22.04 / ROS 2 Humble / Linux `aarch64` / Python 3.10. | 1 | Run `./scripts/verify_build.sh`; it invokes `./build_ws.sh --clean`. Repeat after BUILD-001 on a fresh image for reproducibility acceptance. | Automated on target | Colcon build summary and compiler/linker output. | Pass | Working tree atop `664bd9f` | `BUILD-002-R03`: all eleven packages built in the isolated Pi environment. BUILD-001 fresh-image dependency/clone reproducibility remains Not Run. |
| BUILD-003 | Workspace tests | Workspace tests complete without build, install, or packaging failures. | 1 | Run `./scripts/verify_build.sh`; after its clean build, it sources the install setup through the strict-shell helper, runs `colcon test`, and runs `colcon test-result --verbose`. | Automated on target | Colcon build/test summary and detailed results. | Fail | Working tree atop `664bd9f` | `BUILD-003-R05`: inherited Flake8/PEP257 debt in eight non-Unitree Python packages. No Unitree failure was reported; the general verifier remains strict. |
| BUILD-004 | Build tooling | Developer build/verification scripts locate the workspace reliably, pass shell/static validation, fail clearly when Humble or a setup file is unavailable, and preserve strict nounset behavior around ROS/ament/colcon setup files. | 1 | Run `bash -n` on shell tools, both static validators, `./scripts/test_source_setup.sh`, and the missing-Humble prerequisite path. | Automated static + target | Phase 1/2 artifacts and exact command output. | Pass | Working tree atop `664bd9f` | `BUILD-004-R04` passed the setup-source regression on the isolated Pi/Humble target. This does not pass BUILD-001 or BUILD-003. |
| PKG-001 | Installed packages | Every checked-in ROS executable is installed and discoverable through `ros2 pkg executables`. | 1 | Run the executable-discovery commands above or `./scripts/verify_build.sh` after a clean build. | Automated on target | Command output for all expected package/executable pairs. | Not Run | — | No hardware connection is required. |
| UNITREE-BUILD-001 | Unitree SDK linkage | The Unitree ROS wrapper resolves the checked-in sibling SDK include path and links the matching `aarch64` Linux archive. | 1 | Inspect configuration diagnostics, `unitree_sdk_library.txt`, and linker input/output during a clean `aarch64` build. | Automated on target | CMake configuration and link log tied to the vendor archive checksum. | Pass | Working tree atop `664bd9f` | `UNITREE-BUILD-001-R03`; installed launch/config/RViz resources and `launch --show-args` also passed. Vendor-example unused-parameter and CMP0074/PCL_ROOT developer warnings were nonfatal and remain recorded. |
| DEP-001 | Dependency metadata | All current runtime Python imports, C++/launch includes, and external serial/PCL requirements are represented in package metadata. | 1 | Run `python3 scripts/validate_ros_packages.py` and perform a direct manifest/source comparison. | Automated static | Static audit output and reviewed manifest diff. | Pass | Working tree atop `664bd9f` | `DEP-001-R05` passed on the isolated target Pi; clean fresh-image rosdep resolution remains BUILD-001. |
| DEP-002 | External dependency roles | Recovered external packages are explicitly separated into build/bootstrap, legacy bring-up, development/manual-test, and optional visualization roles. | 1 | Review the dependency table against source, recovered process evidence, and the clean build procedure; check runtime packages only when their later activity is authorized. | Documentation + later runtime | Reviewed classification and, when applicable, `ros2 pkg prefix`/tool output on the supported Pi. | Pass | Working tree atop `ed47083` | Classification passed as `DEP-002-R01`; fresh-Pi installation/discovery remains Not Run. |
| LIDAR-SW-001 | Unitree static contract | Driver parameters/defaults, SDK2 work-mode and timestamp restrictions, YAML-authoritative launch precedence, headless/optional RViz, topic/frame defaults, TF ownership, QoS, resource installation, SDK architecture selection, and vendor hashes agree across source/config/launch/build files. | 2 | Run both Python static validators, syntax/XML checks, archive hashes, and direct source/config review. | Automated static | Phase 2 static-validation artifact and working-tree diff. | Pass | Working tree atop `664bd9f` | `LIDAR-SW-001-R03` passed the focused validator on the target Pi. This does not pass physical initialization, transport, topics, TF semantics, RViz, shutdown, or hardware claims. |
| LIDAR-SW-002 | Focused Unitree target gate | The isolated Phase 2 verifier clean-builds only `unitree_lidar_ros2`, proves the ARM64 archive link and installed resources, runs package-specific tests and executable discovery, and rejects invalid `initialize_type` before SDK initialization without starting hardware. | 2 | Run `./scripts/verify_unitree_phase2.sh` in an isolated supported Pi shell and retain its complete transcript. | Automated on target, no hardware | Focused verifier transcript, link selection, test result, executable output, and invalid-configuration diagnostic. | Pass | Working tree atop `664bd9f` | `LIDAR-SW-002-R01`; status 0. `colcon test` completed with zero discovered tests, so no Unitree unit-test coverage is claimed. The strict full-workspace gate remains Fail under `BUILD-003-R05`. |
| LINT-001 | Inherited Python style debt | Existing Flake8/PEP257 findings in eight non-Unitree packages are resolved in a separately scoped cleanup without behavioral changes or weakened tests. | Later cleanup | Preserve the strict workspace gate, make reviewed package-local style corrections, then rerun `./scripts/verify_build.sh`. | Automated static/target | Before/after lint findings and a passing full-workspace test transcript. | Fail | Working tree atop `664bd9f` | `BUILD-003-R05`; affects `esp_bridge`, `esp_fusion_pkg`, `fusion_pkg`, `gap_follow`, `lidar_logger_pkg`, `tof_processor_pkg`, `ultrasonic_pkg`, and `vfh_pkg`. Explicitly outside Phase 2. |
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
