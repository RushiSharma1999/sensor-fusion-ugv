# Sensor Fusion-Based Autonomous UGV

This repository contains the ROS 2 workspace for the Sensor Fusion-Based Autonomous UGV project. It includes LiDAR, ultrasonic, ToF, ESP serial, camera UART, fusion, and reactive-navigation packages.

The canonical and planned system boundaries are documented in [`docs/architecture/canonical_architecture.md`](docs/architecture/canonical_architecture.md). Building the workspace does not prove sensor operation, motor operation, safety, or autonomous driving.

## Supported build target

The Phase 1 compatibility baseline is:

- Raspberry Pi 4;
- Ubuntu 22.04 LTS Jammy on Linux `aarch64`;
- ROS 2 Humble;
- Python 3.10, colcon, and rosdep; and
- a C++17 compiler and CMake as supplied by the Ubuntu/ROS build toolchain.

This baseline was recovered from the previous team's physical Raspberry Pi 4. It is the compatibility target to reproduce before considering a separate Ubuntu 24.04 / ROS 2 Jazzy migration. Raspberry Pi 5 and Jazzy compatibility are not verified. See [`docs/decisions/ADR-002-supported-platform.md`](docs/decisions/ADR-002-supported-platform.md) and [`docs/legacy/known_working_pi_baseline.md`](docs/legacy/known_working_pi_baseline.md).

The checked-in Unitree SDK contains Linux static libraries for exactly these processor names:

- `aarch64` — the Phase 1 target asset;
- `x86_64` — a checked-in Linux development asset, not target-Pi evidence.

The Unitree CMake projects reject non-Linux systems and any other processor name with an explicit error. No vendor library is present for 32-bit ARM, `arm64`-named toolchains, or other architectures. Do not add an architecture alias without confirming that the vendor binary ABI is compatible.

The recovered Pi proves a historically working Humble environment. It does not yet prove that a fresh clone of this corrected branch resolves, builds, tests, and remains discoverable after a reboot. See [`docs/verification/test_matrix.md`](docs/verification/test_matrix.md) for that pending acceptance procedure.

## Build prerequisites

Start with Ubuntu 22.04 on ARM64 and install ROS 2 Humble using the [official ROS 2 Humble Ubuntu instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Then install the repository/bootstrap tools:

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep
```

Initialize rosdep once on a new operating-system installation if it has not already been initialized:

```bash
sudo rosdep init
```

If `/etc/ros/rosdep/sources.list.d/20-default.list` already exists, do not initialize rosdep again. Every user who builds should still run `rosdep update`.

The package manifests declare the source-level dependencies used by the workspace. In particular:

- `python3-serial` supplies the `serial` Python module used by `esp_bridge`, `esp_fusion_pkg`, and `vision_uart_pkg`;
- `libpcl-all-dev` supplies the PCL headers/libraries used by the Unitree wrapper;
- `ament_python`, `ament_cmake`, `rclpy`, `rclcpp`, ROS message packages, TF2, PCL conversions, launch, and RViz dependencies are declared by the packages that use them; and
- the Unitree SDK headers and architecture-specific static library are checked into `ws/src/unitree_lidar_sdk` and are not downloaded by the build.

Use rosdep to resolve those declarations. Do not substitute undocumented global `pip` installs for the declared Ubuntu/ROS dependencies.

## Dependency classification

| Dependency | Classification | Phase 1 treatment |
|---|---|---|
| ROS/C++/Python dependencies declared in `package.xml`, including `python3-serial`, PCL, messages, TF2, launch, and RViz | Required for source-package build and installed package runtime | Resolved by rosdep from `ws/src`; no global pip install. |
| Checked-in Unitree headers and `lib/aarch64/libunilidar_sdk2.a` | Required for the Unitree source build and core LiDAR runtime | Version controlled; the target link path is checked by `scripts/verify_build.sh`. |
| `python3-colcon-common-extensions`, `python3-rosdep`, and `git` | Required build/bootstrap tools | Installed explicitly with apt before rosdep/build. |
| `pointcloud_to_laserscan` | Recovered legacy robot bring-up dependency | Not required to compile Phase 1. Production projection/configuration remains planned for Phase 3. |
| `tmux` | Recovered legacy bring-up/process-management tool | Optional for development; not used by the Phase 1 build or verification scripts. |
| `teleop_twist_keyboard` | Development/manual-test dependency | Optional and not used during Phase 1 acceptance. |
| `foxglove_bridge` | Optional visualization/telemetry dependency | Optional and not used during Phase 1 acceptance. |

On Humble, install a later test or legacy tool only for the corresponding explicitly reviewed activity:

```bash
sudo apt install -y ros-humble-pointcloud-to-laserscan
sudo apt install -y ros-humble-teleop-twist-keyboard tmux
sudo apt install -y ros-humble-foxglove-bridge
```

These commands are deliberately separate from the required build prerequisites.

## Fresh-clone build procedure

```bash
git clone <repository-url>
cd sensor-fusion-ugv
git checkout <phase-1-commit>

source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths ws/src --ignore-src --rosdistro humble -r -y

./build_ws.sh --clean
source ws/install/setup.bash
```

`build_ws.sh` finds `ws` relative to its own location, requires ROS 2 Humble and colcon, and runs:

```bash
colcon build --symlink-install
```

It intentionally does not run rosdep or install system software. Dependency installation remains an explicit, reviewable step. `--clean` removes only `ws/build`, `ws/install`, and `ws/log` before the build. Additional colcon arguments may be passed through, for example:

```bash
./build_ws.sh --packages-select fusion_pkg
```

## Build and package verification

On the supported Ubuntu 22.04 ARM64 / ROS 2 Humble target with dependencies already installed, run:

```bash
./scripts/verify_build.sh
```

The script checks the supported OS, architecture, Python, and ROS distribution; confirms all eleven source packages are visible to colcon; performs the package and Unitree Phase 2 static audits; makes a clean build; checks that the Unitree wrapper link command references the checked-in ARM64 archive; verifies installed Unitree launch/config/RViz resources and parses launch arguments without starting the driver; runs `colcon test`; reports test results; and confirms all checked-in ROS console executables are discoverable. It does not open hardware devices or start any robot node.

The ROS-independent static audit can also run on a development host without ROS:

```bash
python3 scripts/validate_ros_packages.py
python3 scripts/validate_unitree_driver.py
```

For an explicit manual test sequence:

```bash
cd ws
colcon test
colcon test-result --verbose
```

Expected package executables can be inspected after sourcing the workspace:

```bash
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

## Workspace layout

```text
sensor-fusion-ugv/
├── build_ws.sh
├── run_lidar.sh
├── scripts/
│   ├── validate_ros_packages.py
│   ├── validate_unitree_driver.py
│   └── verify_build.sh
└── ws/
    └── src/
        ├── esp_bridge/
        ├── esp_fusion_pkg/
        ├── fusion_pkg/
        ├── gap_follow/
        ├── lidar_logger_pkg/
        ├── tof_processor_pkg/
        ├── ultrasonic_pkg/
        ├── unitree_lidar_ros2/
        ├── unitree_lidar_sdk/
        ├── vfh_pkg/
        └── vision_uart_pkg/
```

`ws/build`, `ws/install`, and `ws/log` are generated by colcon and are ignored by Git. `data/lidar_data_*.csv` logger outputs are also ignored; preserve valuable experiment data in an approved evidence location before cleaning a worktree.

## Runtime hardware requirements

Runtime nodes require their configured LiDAR, serial, UART, and sensor interfaces. Those devices are not required for Phase 1 build/install verification, and this README does not assert that their models, ports, network configuration, transforms, or behavior have been confirmed.

The recovered runtime used `ROS_DOMAIN_ID=10`. That value is historical runtime configuration, not a compilation prerequisite, and Phase 1 does not modify a user's global shell startup files.

After a successful build, `./run_lidar.sh` launches the Unitree ROS wrapper headlessly using installed launch and parameter resources. It requires a separately configured LiDAR connection. Run `./run_lidar.sh use_rviz:=true` only when visualization is deliberate. The wrapper publishes `PointCloud2` and IMU data; it does not provide the planned robot-centric `/scan` integration. Its exact topics, frames, TF behavior, parameters, QoS, and evidence limits are defined in [`docs/interfaces/unitree_driver_tf_contract.md`](docs/interfaces/unitree_driver_tf_contract.md).

## Later deployment requirements

Robot provisioning, stable device mapping, preflight checks, stopped-state bring-up, explicit enable, and production launch are deferred to later roadmap phases. `build_ws.sh` and `scripts/verify_build.sh` are developer/build tools, not robot deployment or autonomous bring-up commands.
