# Codebase Overview

This repository contains the ROS 2 workspace for a Sensor Fusion-Based Autonomous UGV. Based on the code, the project is intended to run on a Raspberry Pi or another Ubuntu 24.04 machine with ROS 2 Jazzy, and to combine several sensor streams for small unmanned ground vehicle navigation.

The system appears to support:

- A Unitree LiDAR L2, through `ws/src/unitree_lidar_ros2` and the vendored SDK in `ws/src/unitree_lidar_sdk`.
- An ESP32 or similar microcontroller connected over serial, through `ws/src/esp_bridge`.
- Ultrasonic distance sensors reported by the ESP serial protocol.
- A front ToF sensor summary reported by the ESP serial protocol.
- A camera or external vision computer connected over UART, through `ws/src/vision_uart_pkg`.
- Several obstacle-avoidance or navigation experiments: gap following, VFH, sensor-feature fusion, and a combined ESP plus LiDAR VFH node.

ROS 2 is used as the message bus between hardware bridges, sensor processors, fusion logic, navigation nodes, and the final velocity command path. Most Python packages are `ament_python` ROS 2 packages. The Unitree LiDAR ROS driver is an `ament_cmake` package. The Unitree SDK is a vendored C++ SDK/library folder rather than a normal ROS 2 package.

At a high level, the intended system likely looks like this:

```text
Unitree LiDAR hardware
  -> unitree_lidar_ros2
  -> PointCloud2 + IMU topics
  -> Unclear from code: conversion to /scan is needed for the navigation nodes

ESP32 serial device
  -> esp_bridge
  -> /odom, TF odom->base_link, /tof/front/summary, /esp/ultrasonic_raw
  <- /cmd_vel commands encoded as serial CMD lines

ToF and ultrasonic processors
  -> /tof/features and /ultrasonic/features
  -> fusion_pkg
  -> /fusion/sector_penalties, /fusion/critical_stop, /fusion/speed_limit

LiDAR scan and fusion outputs
  -> vfh_pkg or gap_follow or esp_fusion_pkg
  -> velocity commands

Camera UART stream
  -> vision_uart_receiver_node
  -> camera_follow_node
  -> /cmd_vel_camera

Safety mux
  -> selects /cmd_vel_camera or /cmd_vel_vfh
  -> publishes final /cmd_vel
  -> esp_bridge sends final command to ESP32
```

Important assumptions from the code:

- The ESP32 is probably responsible for motor control, odometry feedback, ultrasonic readings, and ToF summaries. This is inferred from `ws/src/esp_bridge/esp_bridge/cmd_vel_to_serial.py`.
- The navigation nodes expect a `sensor_msgs/LaserScan` on `/scan`, but `ws/src/unitree_lidar_ros2` publishes `sensor_msgs/PointCloud2` on `unilidar/cloud` and `sensor_msgs/Imu` on `unilidar/imu`. Unclear from code: a `PointCloud2` to `LaserScan` conversion node, alternate LiDAR driver, or another `/scan` publisher is needed but not included.
- The camera UART source is probably a Jetson, camera microcontroller, or vision processor sending either two-character direction messages or comma-separated detection messages.
- The project has multiple competing or experimental control paths. `gap_follow` publishes directly to `/cmd_vel`, `vfh_pkg` publishes to `/cmd_vel_vfh`, `vision_uart_pkg/safety_mux_node.py` publishes final `/cmd_vel`, and `esp_fusion_pkg` also publishes `/cmd_vel`.
- `esp_bridge` and `esp_fusion_pkg` both default to `/dev/ttyACM0`. Running both at the same time may fail unless the hardware exposes two serial devices or one node is changed.

## Repository Structure

```text
sensor-fusion-ugv/
|-- README.md
|-- CODEBASE_OVERVIEW.md
|-- github_beginner_guide.md
|-- build_ws.sh
|-- run_lidar.sh
|-- .gitignore
|-- .vscode/
|   `-- settings.json
|-- data/
|   |-- lidar_data_1773843456.csv
|   `-- lidar_data_1773843570.csv
|-- docs/
|   `-- images/
|-- scripts/
`-- ws/
    `-- src/
        |-- esp_bridge/
        |-- esp_fusion_pkg/
        |-- fusion_pkg/
        |-- gap_follow/
        |-- lidar_logger_pkg/
        |-- tof_processor_pkg/
        |-- ultrasonic_pkg/
        |-- unitree_lidar_ros2/
        |-- unitree_lidar_sdk/
        |-- vfh_pkg/
        |-- vision_uart_pkg/
        `-- project_structure.text
```

Major folders and files:

- `README.md`: Short project README with package list, ROS 2 Jazzy requirement, build instructions, and LiDAR run instructions.
- `github_beginner_guide.md`: Beginner-facing GitHub Desktop workflow guide. It is documentation for team collaboration, not robot runtime logic.
- `build_ws.sh`: Root build helper. It enters `ws/`, sources `/opt/ros/jazzy/setup.bash`, optionally runs `rosdep`, and runs `colcon build --symlink-install`.
- `run_lidar.sh`: Root run helper. It enters `ws/`, sources ROS 2 Jazzy and the local install setup file, then runs `ros2 launch unitree_lidar_ros2 launch.py`.
- `.gitignore`: Ignores common macOS, ROS 2 build, Python cache, CMake, log, temporary, rosbag, and generated TF graph files.
- `.vscode/settings.json`: VS Code CMake setting pointing at `ws/src/unitree_lidar_ros2`. This file is tracked even though `.gitignore` ignores `.vscode/`.
- `data/`: Contains two tracked LiDAR CSV logs. Both inspected files currently contain only the header `time,angle,distance`.
- `docs/images/`: Empty image folder.
- `scripts/`: Empty helper scripts folder.
- `ws/src/`: ROS 2 source workspace containing Python packages, the Unitree ROS driver, and the Unitree SDK.
- `ws/src/project_structure.text`: A manually generated tree snapshot of `ws/src/`; useful but should not be treated as authoritative over the actual file tree.

## ROS 2 Packages Under `ws/src`

### `esp_bridge`

- Package type: Python ROS 2 package using `ament_python`.
- Main purpose: Bridge final ROS velocity commands to the ESP32 over serial, and translate incoming ESP serial telemetry into ROS topics and odometry TF.
- Important files:
  - `ws/src/esp_bridge/package.xml`
  - `ws/src/esp_bridge/setup.py`
  - `ws/src/esp_bridge/setup.cfg`
  - `ws/src/esp_bridge/esp_bridge/cmd_vel_to_serial.py`
- ROS 2 executable:
  - `cmd_vel_to_serial = esp_bridge.cmd_vel_to_serial:main`
- ROS 2 node name:
  - `cmd_vel_to_serial`
- Subscribed topics:
  - Default `/cmd_vel` as `geometry_msgs/Twist`
- Published topics:
  - Default `/odom` as `nav_msgs/Odometry`
  - Default `/tof/front/summary` as `std_msgs/String`
  - Default `/esp/ultrasonic_raw` as `std_msgs/String`
  - TF transform from `odom` to `base_link`
- Parameters:
  - `port`: default `/dev/ttyACM0`
  - `baud`: default `115200`
  - `rate_hz`: default `30.0`
  - `rx_hz`: default `50.0`
  - `debug_serial`: default `False`
  - `cmd_vel_topic`: default `/cmd_vel`
  - `odom_topic`: default `/odom`
  - `tof_front_topic`: default `/tof/front/summary`
  - `ultra_raw_topic`: default `/esp/ultrasonic_raw`
- Hardware interfaces:
  - Opens a pyserial connection to the configured ESP serial device.
  - Writes ASCII commands like `CMD v 0.123 w -0.456`.
  - Reads line-oriented ASCII telemetry.
- How it fits:
  - This is the likely final actuator bridge for the robot. It consumes final `/cmd_vel` commands from `gap_follow`, `safety_mux_node`, or another controller and sends them to the ESP32.
  - It also exposes ESP sensor telemetry for the ToF and ultrasonic processing packages.
- Warnings and limitations:
  - `package.xml` declares `rclpy` and `geometry_msgs`, but the source also uses `serial`, `nav_msgs`, `tf2_ros`, and `std_msgs`.
  - Serial port and frame names are defaults in code; there is no checked-in YAML config.
  - If no `/cmd_vel` arrives for more than `0.25` seconds, the serial command is forced to zero speed.
  - Serial open happens during node construction. There is no reconnect loop if the ESP is unplugged or resets.
  - The ESP serial protocol is implicit in code, not documented as a separate interface.

### `esp_fusion_pkg`

- Package type: Python ROS 2 package using `ament_python`.
- Main purpose: Combined VFH-style LiDAR obstacle avoidance plus direct ESP serial sensor fusion.
- Important files:
  - `ws/src/esp_fusion_pkg/package.xml`
  - `ws/src/esp_fusion_pkg/setup.py`
  - `ws/src/esp_fusion_pkg/esp_fusion_pkg/esp_lidar_fusion_driver.py`
- ROS 2 executable:
  - `esp_lidar_fusion_driver = esp_fusion_pkg.esp_lidar_fusion_driver:main`
- ROS 2 node name:
  - `vfh_with_esp_fusion`
- Subscribed topics:
  - Default `/scan` as `sensor_msgs/LaserScan`
- Published topics:
  - Default `/cmd_vel` as `geometry_msgs/Twist`
- Parameters:
  - Topics and serial: `scan_topic`, `cmd_topic`, `serial_port`, `baud`, `debug_log`
  - VFH parameters: `sector_angle_deg`, `front_fov_deg`, `robot_radius`, `safety_margin`, `max_range_clip`, `obstacle_influence_dist`, `hist_threshold`, `min_open_sectors`, speed and steering weights
  - Escape parameters: `escape_trigger_cycles`, `blocked_open_fraction`, `escape_turn_angle_deg`, `escape_turn_rate`, `escape_clearance_dist`, `escape_holdoff_cycles`
  - ESP fusion thresholds: `lidar_confirm_trigger_cm`, `confirm_window_s`, ultrasonic thresholds, front/back far thresholds, recovery parameters
- Hardware interfaces:
  - Opens serial on default `/dev/ttyACM0` at `115200`.
  - Sends `PI_READY` once after a two-second sleep.
  - Reads ESP state packets beginning with `STATE|`.
- Expected ESP `STATE` fields:
  - `EMG`
  - `FAR_FRONT`, `FAR_BACK`
  - `US_FRONT`, `US_LEFT`, `US_RIGHT`, `US_BACK`
  - `CONF_FRONT`, `CONF_BACK`
- How it fits:
  - This appears to be an alternate integrated autonomy node. It reads `/scan` and ESP proximity state, computes fused risks, and publishes a drive command directly to `/cmd_vel`.
- Warnings and limitations:
  - It opens the same default serial port as `esp_bridge`. If `esp_bridge` is also needed to send `/cmd_vel` to the ESP, both nodes may compete for `/dev/ttyACM0`.
  - This node publishes `/cmd_vel` but does not write `CMD v ... w ...` command lines to the ESP, except for the startup `PI_READY`. Unclear from code: another node must send `/cmd_vel` to the motor controller, but that other node would likely need the same serial port.
  - `package.xml` declares `rclpy` and `sensor_msgs`, but the source also uses `geometry_msgs` and `serial`.
  - The node ignores scans until at least one ESP `STATE` packet is read.
  - Many behavior thresholds are parameters, but no YAML file or launch file is provided.

### `fusion_pkg`

- Package type: Python ROS 2 package using `ament_python`.
- Main purpose: Fuse processed ToF and ultrasonic features into sector penalties, a critical stop flag, and a speed limit for navigation.
- Important files:
  - `ws/src/fusion_pkg/package.xml`
  - `ws/src/fusion_pkg/setup.py`
  - `ws/src/fusion_pkg/fusion_pkg/fusion_node.py`
- ROS 2 executable:
  - `fusion_node = fusion_pkg.fusion_node:main`
- ROS 2 node name:
  - `fusion_node`
- Subscribed topics:
  - Default `/tof/features` as `std_msgs/String`
  - Default `/ultrasonic/features` as `std_msgs/String`
- Published topics:
  - `/fusion/sector_penalties` as `std_msgs/Float32MultiArray`
  - `/fusion/critical_stop` as `std_msgs/Bool`
  - `/fusion/speed_limit` as `std_msgs/Float32`
  - `/fusion/debug_state` as `std_msgs/String`
- Parameters:
  - `tof_features_topic`, `ultrasonic_features_topic`
  - Sectorization: `front_fov_deg`, `sector_angle_deg`
  - ToF bearing/spread/weights and stop thresholds
  - Ultrasonic bearings/spreads/weights and stop thresholds
- How it fits:
  - `tof_processor_pkg` and `ultrasonic_pkg` reduce raw string telemetry into hazard features.
  - `fusion_pkg` converts those features into navigation-friendly sector penalties and speed/stop signals.
  - `vfh_pkg` consumes the fusion topics.
  - `vision_uart_pkg/safety_mux_node.py` also consumes `/fusion/critical_stop` and `/fusion/speed_limit`.
- Warnings and limitations:
  - Input messages are string fields such as `VALID:1|STATE:ok|...`, not custom ROS message types. This is flexible but easy to break with formatting changes.
  - `camera_data` exists in state but is never populated by a subscription.
  - No freshness timeout is implemented for ToF or ultrasonic data inside `fusion_node`; it will keep using the last parsed messages until replaced. The ultrasonic processor publishes stale invalid messages, but ToF has no equivalent stale wrapper here.

### `gap_follow`

- Package type: Python ROS 2 package using `ament_python`.
- Main purpose: Pure LiDAR gap-following obstacle avoidance.
- Important files:
  - `ws/src/gap_follow/package.xml`
  - `ws/src/gap_follow/setup.py`
  - `ws/src/gap_follow/gap_follow/gap_follow_node.py`
- ROS 2 executable:
  - `gap_follow = gap_follow.gap_follow_node:main`
- ROS 2 node name:
  - `gap_follow`
- Subscribed topics:
  - Hardcoded `/scan` as `sensor_msgs/LaserScan`
- Published topics:
  - Hardcoded `/cmd_vel` as `geometry_msgs/Twist`
- Parameters:
  - No ROS parameters are declared. Tuning values are hardcoded in the class constructor.
- How it fits:
  - This is a simpler alternative to the VFH plus fusion stack. It can drive the robot from `/scan` alone.
  - If `esp_bridge` is running, `gap_follow` can publish `/cmd_vel` and `esp_bridge` can forward commands to the ESP32.
- Warnings and limitations:
  - Hardcoded topic names and tuning values make it less reusable than `vfh_pkg`.
  - It does not consume `/fusion/*` safety outputs or camera commands.
  - It depends on `/scan`, but this repository does not include a confirmed `/scan` producer.

### `lidar_logger_pkg`

- Package type: Python ROS 2 package using `ament_python`.
- Main purpose: Record a narrow front slice of `LaserScan` data into a CSV file.
- Important files:
  - `ws/src/lidar_logger_pkg/package.xml`
  - `ws/src/lidar_logger_pkg/setup.py`
  - `ws/src/lidar_logger_pkg/lidar_logger_pkg/lidar_logger.py`
- ROS 2 executable:
  - `lidar_logger = lidar_logger_pkg.lidar_logger:main`
- ROS 2 node name:
  - `lidar_logger`
- Subscribed topics:
  - Hardcoded `/scan` as `sensor_msgs/LaserScan`
- Published topics:
  - None
- File outputs:
  - `lidar_data_<unix_time>.csv` in the current working directory.
  - Columns: `time`, `angle`, `distance`.
  - Only records angles between `-0.087` and `0.087` radians, about +/-5 degrees.
- How it fits:
  - Useful for simple front-distance experiments or debugging the `/scan` stream.
- Warnings and limitations:
  - `package.xml` contains no runtime dependencies even though the code imports `rclpy` and `sensor_msgs`.
  - Output path is not parameterized.
  - It opens a file directly and does not flush on every callback. A crash may lose buffered rows.
  - The two tracked files in root `data/` contain only CSV headers.

### `tof_processor_pkg`

- Package type: Python ROS 2 package using `ament_python`.
- Main purpose: Convert ESP-provided ToF front summary strings into higher-level hazard features for floor obstacles and dropoffs.
- Important files:
  - `ws/src/tof_processor_pkg/package.xml`
  - `ws/src/tof_processor_pkg/setup.py`
  - `ws/src/tof_processor_pkg/tof_processor_pkg/tof_processor_node.py`
- ROS 2 executable:
  - `tof_processor_node = tof_processor_pkg.tof_processor_node:main`
- ROS 2 node name:
  - `tof_processor_node`
- Subscribed topics:
  - Default `/tof/front/summary` as `std_msgs/String`
- Published topics:
  - Default `/tof/features` as `std_msgs/String`
- Parameters:
  - Topics: `summary_topic`, `features_topic`
  - Publish rate: `publish_hz`, default `15.0`
  - Validity thresholds: `far_avg_invalid_high`, `near_avg_invalid_high`, `center_invalid_low`
  - Baseline learning: `baseline_window`, `baseline_min_samples`
  - Floor obstacle thresholds
  - Edge/dropoff thresholds
  - Persistence counters
- Input message format:
  - Produced by `esp_bridge` from a `TOF|...` serial line.
  - Expected fields include `ID`, `T`, `VALID`, `CENTER`, `NEAR_MIN`, `NEAR_AVG`, `FAR_AVG`.
- Output message format:
  - Publishes string fields including `VALID`, `BEARING`, raw values, learned baselines, deltas, `CENTER_GAP`, `FLOOR`, `DROPOFF`, and `STATE`.
  - `STATE` can be `invalid`, `calibrating`, `clear`, `dropoff`, or `floor_obstacle`.
- How it fits:
  - `fusion_pkg` consumes `/tof/features` and turns ToF hazard scores into sector penalties and speed/stop decisions.
- Warnings and limitations:
  - Uses string parsing instead of typed messages.
  - The baseline is learned online from initial clear frames. It is not saved to disk and not explicitly reset through a service.
  - Units are not documented in the code. Values like `300.0`, `80.0`, and `110.0` suggest sensor-specific distance units, but the code does not state whether they are millimeters or centimeters.
  - No timeout is implemented if `/tof/front/summary` stops updating.

### `ultrasonic_pkg`

- Package type: Python ROS 2 package using `ament_python`.
- Main purpose: Convert raw ESP ultrasonic string readings into normalized hazard features.
- Important files:
  - `ws/src/ultrasonic_pkg/package.xml`
  - `ws/src/ultrasonic_pkg/setup.py`
  - `ws/src/ultrasonic_pkg/ultrasonic_pkg/ultrasonic_processor.py`
- ROS 2 executable:
  - `ultrasonic_processor = ultrasonic_pkg.ultrasonic_processor:main`
- ROS 2 node name:
  - `ultrasonic_processor`
- Subscribed topics:
  - Default `/esp/ultrasonic_raw` as `std_msgs/String`
- Published topics:
  - Default `/ultrasonic/features` as `std_msgs/String`
- Parameters:
  - `ultrasonic_raw_topic`, `ultrasonic_features_topic`
  - `max_range_cm`, default `50.0`
  - `min_range_cm`, default `0.0`
  - `stale_timeout_sec`, default `0.5`
  - `left_bearing_rad`, `center_bearing_rad`, `right_bearing_rad`
- Input message format:
  - Produced by `esp_bridge` from `ULTRA|...` serial lines.
  - `esp_bridge` publishes fields such as `T`, `FRONT`, `LEFT`, `RIGHT`, `BACK`, and `VALID_FRONT`, `VALID_LEFT`, `VALID_RIGHT`, `VALID_BACK`.
- Output message format:
  - Publishes fields such as `VALID`, `STATE`, `LEFT_CM`, `CENTER_CM`, `RIGHT_CM`, per-direction hazards, `MIN_CM`, `AVG_CM`, `STRONGEST`, `BEARING`, and `HAZARD`.
- How it fits:
  - `fusion_pkg` consumes `/ultrasonic/features`.
- Warnings and limitations:
  - The processor maps `FRONT` to the center ultrasonic reading and ignores `BACK`.
  - It does not use the `VALID_FRONT`, `VALID_LEFT`, `VALID_RIGHT`, or `VALID_BACK` flags from `esp_bridge`; it treats readings as valid if they are within `max_range_cm`.
  - `math` is imported but not used.

### `unitree_lidar_ros2`

- Package type: C++ ROS 2 package using `ament_cmake`.
- Main purpose: ROS 2 wrapper around the Unitree LiDAR SDK for publishing point clouds, IMU data, and TF.
- Important files:
  - `ws/src/unitree_lidar_ros2/package.xml`
  - `ws/src/unitree_lidar_ros2/CMakeLists.txt`
  - `ws/src/unitree_lidar_ros2/include/unitree_lidar_ros2.h`
  - `ws/src/unitree_lidar_ros2/src/unitree_lidar_ros2_node.cpp`
  - `ws/src/unitree_lidar_ros2/launch/launch.py`
  - `ws/src/unitree_lidar_ros2/rviz/view.rviz`
  - `ws/src/unitree_lidar_ros2/README.md`
- ROS 2 executable:
  - `unitree_lidar_ros2_node`
- ROS 2 node names:
  - Constructor uses `unitre_lidar_sdk_node` with a typo.
  - Launch file overrides name to `unitree_lidar_ros2_node`.
- Published topics:
  - Default `unilidar/cloud` as `sensor_msgs/PointCloud2`
  - Default `unilidar/imu` as `sensor_msgs/Imu`
- Published TF:
  - `unilidar_imu_initial` to `unilidar_imu`
  - `unilidar_imu` to `unilidar_lidar`
- Parameters:
  - `initialize_type`: default `2`, where `1` means serial and `2` means UDP
  - `work_mode`
  - `use_system_timestamp`
  - `range_min`, `range_max`
  - `cloud_scan_num`
  - Serial settings: `serial_port`, `baudrate`
  - UDP settings: `lidar_port`, `lidar_ip`, `local_port`, `local_ip`
  - Frame/topic settings: `cloud_frame`, `cloud_topic`, `imu_frame`, `imu_topic`
- Hardware interfaces:
  - Uses the Unitree SDK through `UnitreeLidarReader`.
  - Supports serial initialization and UDP initialization.
  - Launch defaults are UDP mode, LiDAR IP `192.168.1.62`, local IP `192.168.1.2`, LiDAR port `6101`, local port `6201`.
- How it fits:
  - Provides the LiDAR point cloud and IMU stream.
  - The rest of the navigation stack needs `/scan`, so an extra conversion step is implied.
- Warnings and limitations:
  - The package does not publish `sensor_msgs/LaserScan`.
  - `CMakeLists.txt` uses relative paths `../../../unitree_lidar_sdk/include` and `../../../unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}`. In the current repository layout, `unitree_lidar_sdk` is a sibling of `unitree_lidar_ros2`, so this path looks suspicious. Unclear from code: this may fail unless the build directory layout makes that relative path resolve differently.
  - `launch/launch.py` always starts RViz, which may not be desired on a headless robot.
  - Launch file imports `subprocess` and runs `ros2 pkg prefix` at launch generation time to find the RViz config.
  - `package.xml` does not declare launch-time dependencies such as `launch`, `launch_ros`, or `rviz2`.
  - The installed launch and RViz files go to `share/unitree_lidar_ros2`, not `share/unitree_lidar_ros2/launch` and `share/unitree_lidar_ros2/rviz`.

### `unitree_lidar_sdk`

- Package type: Vendored C++ SDK/library folder. It has `CMakeLists.txt` but no `package.xml`, so it is not a normal ROS 2 package.
- Main purpose: Provide the Unitree LiDAR L2 SDK API, headers, static libraries, and example utilities.
- Important files:
  - `ws/src/unitree_lidar_sdk/CMakeLists.txt`
  - `ws/src/unitree_lidar_sdk/README.md`
  - `ws/src/unitree_lidar_sdk/include/unitree_lidar_sdk.h`
  - `ws/src/unitree_lidar_sdk/include/unitree_lidar_sdk_pcl.h`
  - `ws/src/unitree_lidar_sdk/include/unitree_lidar_protocol.h`
  - `ws/src/unitree_lidar_sdk/include/unitree_lidar_utilities.h`
  - `ws/src/unitree_lidar_sdk/include/unitree_lidar_sdk_config.h`
  - `ws/src/unitree_lidar_sdk/include/udp_handler.h`
  - `ws/src/unitree_lidar_sdk/include/example.h`
  - `ws/src/unitree_lidar_sdk/examples/*.cpp`
  - `ws/src/unitree_lidar_sdk/lib/aarch64/libunilidar_sdk2.a`
  - `ws/src/unitree_lidar_sdk/lib/x86_64/libunilidar_sdk2.a`
  - `ws/src/unitree_lidar_sdk/bin/*`
- SDK version:
  - `unitree_lidar_sdk_VERSION` is `2.0.9`.
- Hardware interfaces:
  - Serial default `/dev/ttyACM0` at `4000000` baud.
  - UDP default examples use LiDAR IP `192.168.1.62`, local IP `192.168.1.2`, ports `6101` and `6201`.
- How it fits:
  - `unitree_lidar_ros2` includes SDK headers and links the static SDK library.
- Warnings and limitations:
  - The checked-in `bin/*` files are ARM aarch64 Linux executables. They will not run on x86_64 Linux or macOS.
  - The SDK CMake writes executable output back into the source `bin/` directory.
  - The folder is vendored into `ws/src` but lacks ROS package metadata, which may confuse new ROS 2 users.
  - Example utilities contain hardcoded IP addresses and serial ports.

### `vfh_pkg`

- Package type: Python ROS 2 package using `ament_python`.
- Main purpose: Vector Field Histogram obstacle avoidance using `/scan` and optional fusion penalties from ToF/ultrasonic processors.
- Important files:
  - `ws/src/vfh_pkg/package.xml`
  - `ws/src/vfh_pkg/setup.py`
  - `ws/src/vfh_pkg/vfh_pkg/vfh_node.py`
- ROS 2 executable:
  - `vfh_node = vfh_pkg.vfh_node:main`
- ROS 2 node name:
  - `vfh_obstacle_avoidance`
- Subscribed topics:
  - Default `/scan` as `sensor_msgs/LaserScan`
  - Default `/fusion/sector_penalties` as `std_msgs/Float32MultiArray`
  - Default `/fusion/critical_stop` as `std_msgs/Bool`
  - Default `/fusion/speed_limit` as `std_msgs/Float32`
- Published topics:
  - Default `/cmd_vel_vfh` as `geometry_msgs/Twist`
- Parameters:
  - Topic parameters for scan, command, and fusion topics.
  - VFH geometry and histogram parameters.
  - Speed and steering parameters.
  - Escape/corner recovery parameters.
  - Hazard escape parameters.
  - Fusion priority tuning parameters.
- How it fits:
  - This looks like the main navigation node for the richer stack:
    - `/scan` provides LiDAR obstacle geometry.
    - `/fusion/*` adds ToF/ultrasonic safety constraints.
    - `/cmd_vel_vfh` is consumed by `vision_uart_pkg/safety_mux_node.py`.
- Warnings and limitations:
  - `package.xml` declares `rclpy`, `sensor_msgs`, and `geometry_msgs`, but the source also uses `std_msgs`.
  - The `fusion_critical_stop` flag starts a hazard escape turn inside `vfh_node`. However, `safety_mux_node` also subscribes to `/fusion/critical_stop` and forces final `/cmd_vel` to zero. If both nodes run as intended, the mux may suppress VFH's hazard escape commands while critical stop is true.
  - Fusion penalty freshness is not checked. Old penalties can remain active until overwritten.
  - Depends on `/scan`, whose producer is unclear in this repo.

### `vision_uart_pkg`

- Package type: Python ROS 2 package using `ament_python`.
- Main purpose: Receive camera/vision data over UART, generate camera-follow velocity commands, and mux camera/VFH commands into final `/cmd_vel`.
- Important files:
  - `ws/src/vision_uart_pkg/package.xml`
  - `ws/src/vision_uart_pkg/setup.py`
  - `ws/src/vision_uart_pkg/vision_uart_pkg/vision_uart_receiver_node.py`
  - `ws/src/vision_uart_pkg/vision_uart_pkg/camera_follow_node.py`
  - `ws/src/vision_uart_pkg/vision_uart_pkg/safety_mux_node.py`
- ROS 2 executables:
  - `vision_uart_receiver_node = vision_uart_pkg.vision_uart_receiver_node:main`
  - `camera_follow_node = vision_uart_pkg.camera_follow_node:main`
  - `safety_mux_node = vision_uart_pkg.safety_mux_node:main`
- ROS 2 node names:
  - `vision_uart_receiver`
  - `camera_follow_node`
  - `safety_mux_node`
- Hardware interfaces:
  - `vision_uart_receiver_node.py` opens pyserial on default `/dev/serial0` at `115200`.
- Published topics from `vision_uart_receiver_node.py`:
  - `/vision/direction` as `std_msgs/String`
  - `/vision/stop_active` as `std_msgs/Bool`
  - `/vision/target_seen` as `std_msgs/Bool`
  - `/vision/target_angle` as `std_msgs/Float32`
  - `/vision/target_confidence` as `std_msgs/Float32`
  - `/vision/target_box` as `std_msgs/Int32MultiArray`
  - `/vision/debug` as `std_msgs/String`
- Subscribed topics by `camera_follow_node.py`:
  - `/vision/target_seen`
  - `/vision/target_angle`
  - `/vision/target_confidence`
  - `/vision/stop_active`
  - `/vision/target_box`
- Published topics from `camera_follow_node.py`:
  - Default `/cmd_vel_camera` as `geometry_msgs/Twist`
  - `/vision/follow_debug` as `std_msgs/String`
- Subscribed topics by `safety_mux_node.py`:
  - `/cmd_vel_camera` as `geometry_msgs/Twist`
  - `/cmd_vel_vfh` as `geometry_msgs/Twist`
  - `/vision/target_seen` as `std_msgs/Bool`
  - `/fusion/critical_stop` as `std_msgs/Bool`
  - `/fusion/speed_limit` as `std_msgs/Float32`
- Published topics from `safety_mux_node.py`:
  - `/cmd_vel` as `geometry_msgs/Twist`
  - `/mux/debug` as `std_msgs/String`
- How it fits:
  - The receiver turns UART camera data into ROS vision topics.
  - The camera follow node creates camera-based follow commands.
  - The safety mux chooses camera follow when a target is seen, otherwise VFH, and publishes the final `/cmd_vel` expected by `esp_bridge`.
- Warnings and limitations:
  - `package.xml` declares `rclpy` and `std_msgs`, but the source also uses `geometry_msgs` and `serial`.
  - Serial open happens during construction with no reconnect loop.
  - The UART protocol is implicit:
    - Two-character messages use first char `R`, `L`, or `C`, and second char `S` to mean stop.
    - Detection messages are `class_id,x_min,y_min,x_max,y_max,confidence,angle`.
  - The receiver handles confidence values greater than `1.0` by dividing by `100.0`.
  - Image size defaults to `640x480`; target area thresholds depend on that.

## Important Source Code Files

### Root-level files

#### `README.md`

- Purpose: Top-level project introduction and basic setup instructions.
- Important logic: Documents that the project integrates LiDAR, ultrasonic sensors, ToF processing, ESP32 serial communication, camera UART data, and obstacle avoidance.
- Inputs/outputs: Documentation only.
- Notes for maintainers:
  - It says the target platform is Raspberry Pi or Ubuntu with ROS 2 Jazzy.
  - It suggests checking `/scan` and `/cmd_vel`, but the checked-in Unitree driver does not publish `/scan`.

#### `build_ws.sh`

- Purpose: Build helper for the ROS 2 workspace.
- Important logic:
  - Uses `set -e`.
  - Enters `ws/` relative to the script location.
  - Requires `/opt/ros/jazzy/setup.bash`.
  - Runs `rosdep update` and `rosdep install --from-paths src --ignore-src -r -y` if `rosdep` is available.
  - Runs `colcon build --symlink-install`.
- Inputs/outputs:
  - Input: ROS 2 Jazzy installation and package sources under `ws/src`.
  - Output: `ws/build`, `ws/install`, and `ws/log` generated by colcon.
- Notes for maintainers:
  - The script is Ubuntu/ROS specific and will not work on a normal macOS environment.
  - `rosdep update || true` hides update failure but continues.

#### `run_lidar.sh`

- Purpose: Run the Unitree LiDAR launch file.
- Important logic:
  - Enters `ws/`.
  - Sources `/opt/ros/jazzy/setup.bash`.
  - Sources `install/setup.bash`.
  - Runs `ros2 launch unitree_lidar_ros2 launch.py`.
- Inputs/outputs:
  - Input: Built workspace and Unitree LiDAR hardware/network.
  - Output: LiDAR ROS node and RViz from the launch file.
- Notes for maintainers:
  - It assumes the workspace has already been built.
  - It starts RViz through the launch file, which may be unsuitable for headless robot runs.

#### `github_beginner_guide.md`

- Purpose: Team onboarding guide for GitHub Desktop and beginner Git workflows.
- Important logic: Documentation only.
- Inputs/outputs: None.
- Notes for maintainers:
  - It duplicates some build/run guidance from `README.md`.
  - It is useful for collaboration but does not document the robot architecture.

#### `.gitignore`

- Purpose: Ignore generated and local files.
- Important logic:
  - Ignores `.DS_Store`, `.vscode/`, ROS 2 generated folders, Python cache, CMake output, logs, rosbag formats, and generated TF graph files.
- Notes for maintainers:
  - `.vscode/settings.json` is already tracked despite `.gitignore` ignoring `.vscode/`.

#### `.vscode/settings.json`

- Purpose: VS Code CMake integration setting.
- Important logic:
  - Sets `cmake.sourceDirectory` to an absolute user path ending in `ws/src/unitree_lidar_ros2`.
- Notes for maintainers:
  - The absolute path is user-specific and will not match other team members' machines.

#### `data/lidar_data_1773843456.csv` and `data/lidar_data_1773843570.csv`

- Purpose: Sample LiDAR logger outputs.
- Important logic: Data files only.
- Inputs/outputs:
  - Each contains only the header `time,angle,distance`.
- Notes for maintainers:
  - These appear to be empty logs and may not be useful as committed data.

### Common Python package files

Most Python ROS 2 packages contain the same supporting files:

- `package.xml`: ROS package manifest. Several manifests still contain TODO descriptions/licenses and incomplete dependencies.
- `setup.py`: Python package install metadata and `console_scripts` entry points.
- `setup.cfg`: Installs scripts under `$base/lib/<package_name>`.
- `resource/<package_name>`: Ament resource marker.
- `<package_name>/__init__.py`: Empty Python package marker.
- `test/test_flake8.py`, `test/test_pep257.py`, `test/test_copyright.py`: Generated linter test stubs. Copyright tests are skipped with the default generated message.

### `ws/src/esp_bridge/esp_bridge/cmd_vel_to_serial.py`

- Purpose: Convert final ROS velocity commands to ESP serial commands and convert ESP serial telemetry to ROS messages.
- Main classes/functions:
  - `yaw_to_quat(yaw)`: Converts planar yaw to a quaternion tuple.
  - `CmdVelToSerial`: ROS 2 node.
  - `on_cmd_vel`: Stores latest command and timestamp.
  - `on_cmd_timer`: Writes `CMD v ... w ...` to serial at `rate_hz`.
  - `on_rx_timer`: Reads serial bytes, buffers line fragments, and dispatches complete lines.
  - `handle_odom_line`: Parses `ODOM` lines and publishes odometry plus TF.
  - `handle_tof_line`: Parses front ToF summary lines and republishes normalized fields.
  - `handle_ultra_line`: Parses ultrasonic lines and republishes normalized raw fields.
- Inputs:
  - ROS `/cmd_vel` Twist.
  - Serial lines beginning with `ODOM`, `TOF|`, or `ULTRA|`.
- Outputs:
  - Serial command lines to ESP.
  - `/odom`, TF `odom` to `base_link`, `/tof/front/summary`, and `/esp/ultrasonic_raw`.
- Hardware interfaces:
  - pyserial on `/dev/ttyACM0` by default.
- Notes for future maintainers:
  - This file is central to final robot actuation.
  - If a future bringup stack uses `safety_mux_node`, this node should subscribe to the mux's final `/cmd_vel`.
  - Add reconnect handling and document the serial protocol in a standalone file.

### `ws/src/esp_fusion_pkg/esp_fusion_pkg/esp_lidar_fusion_driver.py`

- Purpose: Standalone VFH plus ESP sensor fusion controller.
- Main classes/functions:
  - `VFHWithEspFusion`: ROS 2 node.
  - `parse_state_packet`, `extract_esp_state`, `read_serial_state`: Parse ESP `STATE|` packets.
  - `scan_callback`: Main control loop triggered by `/scan`.
  - `build_histogram`, `find_open_valleys`, `choose_best_sector`: VFH obstacle avoidance.
  - `compute_lidar_sector_mins`: Reduces LaserScan into front/left/right/back minima.
  - `update_confirm_state`, `compute_fused_risks`: Merge LiDAR and ESP ultrasonic/ToF confirmation.
  - `build_normal_command`, `start_escape`, `run_escape`: Command generation and blocked recovery.
- Inputs:
  - `/scan` LaserScan.
  - Serial `STATE|` packets from ESP.
- Outputs:
  - `/cmd_vel` Twist.
- Hardware interfaces:
  - pyserial on `/dev/ttyACM0` by default.
- Notes for future maintainers:
  - Clarify whether this node is meant to replace `fusion_pkg` plus `vfh_pkg`, or be retired.
  - Resolve serial ownership with `esp_bridge`.
  - If kept, add a launch file and YAML config.

### `ws/src/fusion_pkg/fusion_pkg/fusion_node.py`

- Purpose: Convert ToF and ultrasonic feature strings into sector-level navigation constraints.
- Main classes/functions:
  - `FusionNode`: ROS 2 node.
  - `parse_fields`: Parses `KEY:VALUE` pipe-separated strings.
  - `on_tof_features`, `on_ultrasonic_features`: Store latest parsed feature dictionaries.
  - `sector_center_angle`: Computes sector center bearing.
  - `add_directional_penalty`: Adds triangular penalty around a bearing.
  - `publish_fusion`: Runs at 10 Hz and publishes penalties, critical stop, speed limit, and debug state.
- Inputs:
  - `/tof/features` String.
  - `/ultrasonic/features` String.
- Outputs:
  - `/fusion/sector_penalties`, `/fusion/critical_stop`, `/fusion/speed_limit`, `/fusion/debug_state`.
- Notes for future maintainers:
  - This file is an important integration point for ToF and ultrasonic safety.
  - Add stale checks or use message timestamps if sensor data loss matters.
  - Consider custom ROS messages for features and fusion outputs.

### `ws/src/gap_follow/gap_follow/gap_follow_node.py`

- Purpose: Simple reactive gap-following controller using only LaserScan.
- Main classes/functions:
  - `GapFollow`: ROS 2 node.
  - `moving_average`: Smooths scan ranges.
  - `scan_callback`: Cleans scan data, restricts to front FOV, finds gaps, picks a target angle, and publishes speed/turn commands.
  - `publish_cmd`, `maybe_log`, `clamp`, `angle_of_index`: Helpers.
- Inputs:
  - `/scan` LaserScan.
- Outputs:
  - `/cmd_vel` Twist.
- Notes for future maintainers:
  - Good baseline algorithm for LiDAR-only testing.
  - Convert hardcoded values into ROS parameters before field tuning.
  - If used with camera or ToF/ultrasonic fusion, route it through a mux or make it publish to a non-final topic.

### `ws/src/lidar_logger_pkg/lidar_logger_pkg/lidar_logger.py`

- Purpose: Log front LaserScan distances to CSV.
- Main classes/functions:
  - `LidarLogger`: ROS 2 node.
  - `callback`: Writes rows for scan angles within about +/-5 degrees.
  - `destroy_node`: Closes CSV file.
- Inputs:
  - `/scan` LaserScan.
- Outputs:
  - `lidar_data_<time>.csv` in the current working directory.
- Notes for future maintainers:
  - Parameterize topic, angular window, and output path.
  - Use ROS logging to report row counts and write failures.

### `ws/src/tof_processor_pkg/tof_processor_pkg/tof_processor_node.py`

- Purpose: Detect floor obstacles and dropoffs from front ToF summary data.
- Main classes/functions:
  - `TofProcessorNode`: ROS 2 node.
  - `parse_summary`, `on_front`: Parse and store incoming ToF summary.
  - `is_valid_frame`: Reject invalid ToF frames.
  - `baseline_ready`, `maybe_update_baseline`: Online baseline learning while clear.
  - `publish_features`: Main processing loop.
- Inputs:
  - `/tof/front/summary` String.
- Outputs:
  - `/tof/features` String.
- Important logic:
  - Invalid frames publish `VALID:0`.
  - Initial valid frames populate baseline queues and publish `STATE:calibrating`.
  - After baseline is ready, deltas and geometry gaps estimate `FLOOR` and `DROPOFF` hazard scores.
- Notes for future maintainers:
  - Document ToF units and sensor geometry.
  - Add reset/calibrate service if the floor baseline changes.
  - Add stale timeout.

### `ws/src/ultrasonic_pkg/ultrasonic_pkg/ultrasonic_processor.py`

- Purpose: Normalize raw ultrasonic readings into hazards and strongest direction.
- Main classes/functions:
  - `UltrasonicProcessorNode`: ROS 2 node.
  - `parse_fields`: Parses raw pipe-separated fields.
  - `on_raw_ultrasonic`: Stores latest raw data and timestamp.
  - `get_distance_cm`: Reads values using multiple aliases.
  - `normalize_distance`: Converts distance to hazard in `[0, 1]`.
  - `is_stale`: Checks age of last raw message.
  - `publish_features`: Publishes stale/no_echo/ok feature messages at 10 Hz.
- Inputs:
  - `/esp/ultrasonic_raw` String.
- Outputs:
  - `/ultrasonic/features` String.
- Notes for future maintainers:
  - Decide whether the rear ultrasonic sensor should be represented.
  - Use ESP validity flags if they are reliable.
  - Replace string messages with a typed interface.

### `ws/src/vfh_pkg/vfh_pkg/vfh_node.py`

- Purpose: VFH obstacle avoidance with fusion penalties and recovery behaviors.
- Main classes/functions:
  - `VFHObstacleAvoidance`: ROS 2 node.
  - Fusion callbacks: `on_fusion_penalties`, `on_fusion_stop`, `on_fusion_speed`.
  - `scan_callback`: Main control loop.
  - `build_histogram`, `find_open_valleys`, `choose_best_sector`: VFH logic.
  - Escape logic: `start_escape`, `run_escape`, `finish_escape`.
  - Hazard escape logic: `start_hazard_escape`, `run_hazard_escape`.
  - `compute_fusion_urgency`: Converts fusion speed/penalty state into an urgency score.
- Inputs:
  - `/scan` LaserScan.
  - `/fusion/sector_penalties`, `/fusion/critical_stop`, `/fusion/speed_limit`.
- Outputs:
  - `/cmd_vel_vfh` Twist.
- Important logic:
  - Builds a histogram over a front FOV, inflates obstacles by robot radius plus safety margin, finds open valleys, and scores candidate sectors.
  - Adds fusion penalties into the histogram.
  - Applies speed limits and recovery turns.
- Notes for future maintainers:
  - Clarify interaction between VFH hazard escape and mux critical stop.
  - Add stale checks for fusion topics.
  - Add launch/YAML parameter files for field tuning.

### `ws/src/vision_uart_pkg/vision_uart_pkg/vision_uart_receiver_node.py`

- Purpose: Read camera/vision UART data and publish normalized ROS vision topics.
- Main classes/functions:
  - `VisionUartReceiver`: ROS 2 node.
  - `handle_direction`: Parses two-character direction/stop messages.
  - `parse_data_line`: Parses detection CSV lines.
  - `check_timeout`: Publishes target lost when data is stale.
  - `read_serial`: Polls serial and dispatches parsed messages.
- Inputs:
  - Serial lines on `/dev/serial0` by default.
- Outputs:
  - `/vision/direction`, `/vision/stop_active`, `/vision/target_seen`, `/vision/target_angle`, `/vision/target_confidence`, `/vision/target_box`, `/vision/debug`.
- Hardware interfaces:
  - pyserial UART.
- Notes for future maintainers:
  - Document the UART sender and coordinate convention for target angle.
  - Add reconnect handling.
  - Use a checksum or structured packet format if UART noise becomes an issue.

### `ws/src/vision_uart_pkg/vision_uart_pkg/camera_follow_node.py`

- Purpose: Convert vision target information into a camera-follow velocity command.
- Main classes/functions:
  - `CameraFollowNode`: ROS 2 node.
  - Vision callbacks: `on_target_seen`, `on_target_angle`, `on_confidence`, `on_stop_active`, `on_target_box`.
  - `control_loop`: Main 20 Hz controller.
  - `publish_cmd`: Smooths and publishes Twist plus debug state.
- Inputs:
  - `/vision/target_seen`, `/vision/target_angle`, `/vision/target_confidence`, `/vision/stop_active`, `/vision/target_box`.
- Outputs:
  - `/cmd_vel_camera` Twist.
  - `/vision/follow_debug` String.
- Important logic:
  - Stops if vision stop is active.
  - Uses a lost-target grace period before fully stopping.
  - Turns based on target angle, slows/stops based on target box area, and smooths velocity commands.
- Notes for future maintainers:
  - Image dimensions and area thresholds should be in a YAML config.
  - Confirm sign convention for angular velocity before changing camera angle calculations.

### `ws/src/vision_uart_pkg/vision_uart_pkg/safety_mux_node.py`

- Purpose: Select final motion command from camera follow and VFH, while enforcing fusion safety limits.
- Main classes/functions:
  - `SafetyMuxNode`: ROS 2 node.
  - `on_camera_cmd`, `on_vfh_cmd`, `on_target_seen`, `on_critical_stop`, `on_speed_limit`: Store latest inputs.
  - `clamp_speed`: Applies scalar speed limit.
  - `control_loop`: Selects final command at 20 Hz.
- Inputs:
  - `/cmd_vel_camera`, `/cmd_vel_vfh`, `/vision/target_seen`, `/fusion/critical_stop`, `/fusion/speed_limit`.
- Outputs:
  - `/cmd_vel` Twist.
  - `/mux/debug` String.
- Important logic:
  - If `critical_stop` is true, final command is zero.
  - Else, if camera is preferred and recently valid, use camera command.
  - Else, if VFH is recent, use VFH command.
  - Else, stop.
- Notes for future maintainers:
  - This is the intended final command source when both camera and VFH are running.
  - Make priority policy explicit in a bringup launch/config package.

### `ws/src/unitree_lidar_ros2/include/unitree_lidar_ros2.h`

- Purpose: Contains the `UnitreeLidarSDKNode` class implementation for the Unitree ROS 2 driver.
- Main classes/functions:
  - `UnitreeLidarSDKNode`: ROS 2 node class.
  - Constructor: Declares parameters, initializes SDK reader, creates publishers, creates 1 ms timer.
  - `timer_callback`: Parses SDK packets and publishes IMU or point cloud messages.
- Inputs:
  - Unitree SDK parsed packets from serial or UDP.
- Outputs:
  - `PointCloud2`, `Imu`, and TF.
- Hardware interfaces:
  - Unitree SDK reader initialized through `initializeSerial` or `initializeUDP`.
- Notes for future maintainers:
  - Most of the implementation lives in the header rather than the `.cpp`.
  - Check quaternion ordering. The IMU message assigns `imu.quaternion[0..3]` to x/y/z/w, but the TF transform assigns `[1]`, `[2]`, `[3]`, `[0]`.
  - Node name typo in constructor is `unitre_lidar_sdk_node`.

### `ws/src/unitree_lidar_ros2/src/unitree_lidar_ros2_node.cpp`

- Purpose: Minimal C++ main entrypoint for the Unitree ROS 2 node.
- Main functions:
  - `main`: Initializes ROS, spins `UnitreeLidarSDKNode`, shuts down.
- Inputs/outputs:
  - Delegated to `UnitreeLidarSDKNode`.
- Notes for future maintainers:
  - Most behavior changes happen in `include/unitree_lidar_ros2.h`.

### `ws/src/unitree_lidar_ros2/launch/launch.py`

- Purpose: Launch the Unitree LiDAR ROS node and RViz.
- Main functions:
  - `generate_launch_description`: Creates the LiDAR node and an RViz node.
- Inputs:
  - Launch-time parameters are hardcoded inside the file.
- Outputs:
  - Runs `unitree_lidar_ros2_node`.
  - Runs `rviz2` with `view.rviz`.
- Notes for future maintainers:
  - Make RViz optional for robot bringup.
  - Move hardware parameters to launch arguments or YAML.
  - Consider using `FindPackageShare` instead of shelling out to `ros2 pkg prefix`.

### `ws/src/unitree_lidar_ros2/rviz/view.rviz`

- Purpose: RViz visualization config for Unitree point cloud and TF.
- Important logic:
  - Fixed frame is `unilidar_lidar`.
  - Displays `/unilidar/cloud` as `PointCloud2`.
  - Displays TF frames `unilidar_imu_initial`, `unilidar_imu`, and `unilidar_lidar`.
- Notes for future maintainers:
  - It visualizes point clouds, not `/scan`.

### `ws/src/unitree_lidar_sdk/include/unitree_lidar_sdk.h`

- Purpose: Main abstract SDK interface.
- Main classes/functions:
  - `UnitreeLidarReader`: Abstract interface with serial/UDP initialization, parse loop, data getters, version getters, control commands, timestamp sync, rotation control, and IP/MAC configuration.
  - `createUnitreeLidarReader`: Factory function implemented by the static SDK library.
- Inputs/outputs:
  - Provides point cloud and IMU data through SDK data structures.
- Notes for future maintainers:
  - The implementation is in the prebuilt static library, not source.

### `ws/src/unitree_lidar_sdk/include/unitree_lidar_sdk_pcl.h`

- Purpose: Convert Unitree SDK point clouds into PCL point clouds.
- Main classes/functions:
  - `PointType`: PCL point type with x/y/z, intensity, ring, and time.
  - `transformUnitreeCloudToPCL`: Copies `PointCloudUnitree` into a `pcl::PointCloud<PointType>`.
- Inputs:
  - `PointCloudUnitree`.
- Outputs:
  - PCL point cloud suitable for `pcl::toROSMsg`.
- Notes for future maintainers:
  - Used directly by the ROS 2 driver before publishing `PointCloud2`.

### `ws/src/unitree_lidar_sdk/include/unitree_lidar_protocol.h`

- Purpose: Defines Unitree LiDAR packet constants and packet structures.
- Main content:
  - Frame header/tail definitions.
  - Packet type constants for point data, 2D point data, IMU, version, timestamp, work mode, IP config, MAC config, and user commands.
  - Data structures for calibration, inside state, point data, IMU data, ACK, versions, IP config, MAC config, and user control commands.
- Notes for future maintainers:
  - Low-level protocol definitions should generally be treated as vendor code unless updating SDK versions.

### `ws/src/unitree_lidar_sdk/include/unitree_lidar_utilities.h`

- Purpose: Utility types and parsing helpers for the Unitree SDK.
- Main classes/functions:
  - `PointUnitree`, `PointCloudUnitree`
  - `getSystemTimeStamp`
  - `crc32`
  - `parseFromPacketToPointCloud`
  - `parseFromPacketPointCloud2D`
- Notes for future maintainers:
  - Handles raw range conversion, timestamp selection, and filtering by min/max range.

### `ws/src/unitree_lidar_sdk/include/udp_handler.h`

- Purpose: UDP helper declarations for SDK communication.
- Main classes/functions:
  - `UDPHandler`: Create, bind, send, receive, close, and timeout methods.
  - `dataStructToUDPBuffer`: Template declaration for serializing data to a UDP buffer.
- Notes for future maintainers:
  - Implementation is likely inside the prebuilt static library.

### `ws/src/unitree_lidar_sdk/include/example.h`

- Purpose: Shared example processing loop for SDK examples.
- Main logic:
  - Reads LiDAR firmware/hardware/SDK version.
  - Stops and starts LiDAR rotation.
  - Reads dirty percentage and time delay.
  - Loops parsing IMU and point cloud packets and printing values.
- Notes for future maintainers:
  - Example code loops forever and prints to stdout.

### `ws/src/unitree_lidar_sdk/examples/*.cpp`

- Purpose: SDK example utilities.
- Important files:
  - `example_lidar_serial.cpp`: Opens `/dev/ttyACM0` at `4000000`, starts rotation, sets work mode `8`, resets LiDAR, then calls `exampleProcess`.
  - `example_lidar_udp.cpp`: Opens UDP with LiDAR IP `192.168.1.62`, local IP `192.168.1.2`, ports `6101` and `6201`, sets work mode `0`, resets LiDAR, then calls `exampleProcess`.
  - `set_ip_address.cpp`: Opens UDP and changes LiDAR/user IP config to `192.168.123.110` and `192.168.123.120`.
  - `set_to_serial_mode.cpp`: Opens UDP at `192.168.123.110`/`192.168.123.120` and sets work mode `8`.
  - `set_to_udp_mode.cpp`: Opens serial `/dev/ttyACM0` and sets work mode `0`.
- Notes for future maintainers:
  - These are hardware utilities with hardcoded addresses and ports. Treat them carefully during field setup.

## Likely Runtime Flow

### Main full-stack flow, inferred from code

This is the most coherent architecture if `vfh_pkg`, `fusion_pkg`, `vision_uart_pkg`, and `esp_bridge` are meant to run together:

1. `unitree_lidar_ros2` connects to the Unitree LiDAR and publishes:
   - `unilidar/cloud`
   - `unilidar/imu`
   - TF frames for Unitree IMU and LiDAR

2. Unclear from code: another node must convert `unilidar/cloud` into `/scan`, or another LiDAR driver must publish `/scan`.
   - `gap_follow`, `vfh_pkg`, `esp_fusion_pkg`, and `lidar_logger_pkg` all require `/scan`.
   - No checked-in launch or package creates `/scan`.

3. `esp_bridge` opens the ESP32 serial port and sends final `/cmd_vel` commands to the microcontroller.
   - It writes `CMD v ... w ...` at 30 Hz by default.
   - It receives `ODOM`, `TOF|`, and `ULTRA|` serial lines.
   - It publishes `/odom`, TF `odom` to `base_link`, `/tof/front/summary`, and `/esp/ultrasonic_raw`.

4. `tof_processor_pkg` subscribes to `/tof/front/summary`.
   - It learns a floor baseline, detects possible floor obstacles/dropoffs, and publishes `/tof/features`.

5. `ultrasonic_pkg` subscribes to `/esp/ultrasonic_raw`.
   - It normalizes left/front/right distance readings into hazard features and publishes `/ultrasonic/features`.

6. `fusion_pkg` subscribes to `/tof/features` and `/ultrasonic/features`.
   - It publishes sector penalties, a critical stop flag, and a speed limit.

7. `vfh_pkg` subscribes to `/scan` and `/fusion/*`.
   - It computes obstacle-avoidance commands and publishes `/cmd_vel_vfh`.

8. `vision_uart_receiver_node` reads target information from UART.
   - It publishes `/vision/*` topics.

9. `camera_follow_node` uses `/vision/*` topics.
   - It publishes `/cmd_vel_camera`.

10. `safety_mux_node` chooses final command:
    - Stop if `/fusion/critical_stop` is true.
    - Else prefer camera command if a target is seen and recent.
    - Else use VFH command if recent.
    - Else stop.
    - It publishes final `/cmd_vel`.

11. `esp_bridge` consumes final `/cmd_vel` and sends it to the ESP32/motor controller.

### Simpler LiDAR-only flow

For early testing, the intended flow may be:

```text
/scan -> gap_follow -> /cmd_vel -> esp_bridge -> ESP32/motors
```

This bypasses ToF, ultrasonic, camera, fusion, and safety mux logic.

### Alternate combined ESP fusion flow

`esp_fusion_pkg` appears to attempt:

```text
/scan + ESP STATE serial packets -> esp_lidar_fusion_driver -> /cmd_vel
```

Unclear from code:

- How `/cmd_vel` from `esp_fusion_pkg` reaches the motor controller if `esp_bridge` is not running.
- How both `esp_fusion_pkg` and `esp_bridge` can share `/dev/ttyACM0` if they are intended to run together.
- Whether this package is a current runtime path or an experimental alternative to `fusion_pkg` plus `vfh_pkg`.

## Build and Run Notes

### Supported build script

From the repository root:

```bash
./build_ws.sh
```

The script:

- Requires `/opt/ros/jazzy/setup.bash`.
- Enters `ws/`.
- Runs `rosdep` if available.
- Runs `colcon build --symlink-install`.

After building, source the workspace:

```bash
source ws/install/setup.bash
```

### Manual build path from `README.md`

```bash
source /opt/ros/jazzy/setup.bash
cd ws
colcon build --symlink-install
source install/setup.bash
```

### Existing run script

From the repository root:

```bash
./run_lidar.sh
```

This runs:

```bash
ros2 launch unitree_lidar_ros2 launch.py
```

The launch file starts:

- `unitree_lidar_ros2_node`
- `rviz2` with `view.rviz`

### Existing launch files

Only one launch file was found:

```bash
ros2 launch unitree_lidar_ros2 launch.py
```

Important launch defaults:

- UDP LiDAR initialization: `initialize_type=2`
- `work_mode=0`
- `serial_port=/dev/ttyACM0`
- `baudrate=4000000`
- `lidar_ip=192.168.1.62`
- `local_ip=192.168.1.2`
- `lidar_port=6101`
- `local_port=6201`
- `cloud_topic=unilidar/cloud`
- `imu_topic=unilidar/imu`

### Inferred ROS run commands from `setup.py`

These commands are inferred from package entry points and should work after a successful colcon build and workspace source:

```bash
ros2 run esp_bridge cmd_vel_to_serial
ros2 run esp_fusion_pkg esp_lidar_fusion_driver
ros2 run fusion_pkg fusion_node
ros2 run gap_follow gap_follow
ros2 run lidar_logger_pkg lidar_logger
ros2 run tof_processor_pkg tof_processor_node
ros2 run ultrasonic_pkg ultrasonic_processor
ros2 run vfh_pkg vfh_node
ros2 run vision_uart_pkg vision_uart_receiver_node
ros2 run vision_uart_pkg camera_follow_node
ros2 run vision_uart_pkg safety_mux_node
```

The Unitree C++ ROS executable can also be run directly after build:

```bash
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node
```

### Unclear runtime setup

Unclear from code:

- Which exact nodes are intended to be launched together for the full robot.
- How `/scan` is produced from the Unitree point cloud.
- Whether `esp_fusion_pkg` is current or experimental.
- Which serial device is the ESP32 versus the Unitree LiDAR when both default to `/dev/ttyACM0` in different packages/examples.
- Whether camera UART comes from a Jetson, another microcontroller, or a different board.

## Cleanup and Maintainability Suggestions

1. Add a central bringup package.
   - Why it matters: New team members currently have to infer which nodes should run together.
   - Suggested fix: Create `ugv_bringup` with launch files for LiDAR-only, full autonomy, camera-follow, and logging modes.

2. Add a `/scan` producer or document the external dependency.
   - Why it matters: Most navigation nodes require `/scan`, but the included Unitree driver publishes point clouds.
   - Suggested fix: Add a `pointcloud_to_laserscan` launch node or a custom scan projection package.

3. Resolve serial-port ownership and hardcoded port defaults.
   - Why it matters: `esp_bridge`, `esp_fusion_pkg`, and Unitree SDK examples all use `/dev/ttyACM0` defaults.
   - Suggested fix: Use stable `/dev/serial/by-id/...` paths and a shared hardware YAML file.

4. Complete `package.xml` dependencies.
   - Why it matters: `rosdep` and colcon depend on accurate manifests.
   - Examples:
     - `esp_bridge` uses `serial`, `nav_msgs`, `tf2_ros`, and `std_msgs`.
     - `esp_fusion_pkg` uses `serial` and `geometry_msgs`.
     - `lidar_logger_pkg` uses `rclpy` and `sensor_msgs`.
     - `vfh_pkg` uses `std_msgs`.
     - `vision_uart_pkg` uses `serial` and `geometry_msgs`.

5. Replace string protocols with typed ROS messages where practical.
   - Why it matters: Pipe-separated strings are easy to break and hard to test.
   - Suggested fix: Define custom messages such as `TofFeatures.msg`, `UltrasonicFeatures.msg`, and `FusionDebug.msg`.

6. Add checked-in parameter YAML files.
   - Why it matters: Many thresholds are currently embedded as defaults in code.
   - Suggested fix: Add `config/robot.yaml`, `config/simulation.yaml`, and per-node configs.

7. Clarify safety semantics.
   - Why it matters: `fusion_pkg` publishes `critical_stop`; `vfh_pkg` may start hazard escape; `safety_mux_node` forces final stop. These behaviors may conflict.
   - Suggested fix: Define one safety supervisor as the final authority and document the meaning of stop versus escape.

8. Remove or relocate user-specific/editor files.
   - Why it matters: `.vscode/settings.json` contains an absolute path for one developer machine.
   - Suggested fix: Remove tracked `.vscode/settings.json` or replace with portable workspace recommendations.

9. Remove or regenerate empty data logs.
   - Why it matters: `data/lidar_data_*.csv` files contain only headers and do not help reproduce experiments.
   - Suggested fix: Move real data to a documented `data/README.md` workflow or keep datasets in cloud storage.

10. Improve Unitree SDK integration.
    - Why it matters: The ROS driver CMake relative SDK path appears inconsistent with the repo layout.
    - Suggested fix: Make `unitree_lidar_sdk` a proper vendor package or use CMake variables for SDK include/library directories.

11. Make RViz optional in LiDAR launch.
    - Why it matters: Robots often run headless.
    - Suggested fix: Add a launch argument like `use_rviz:=false`.

12. Add real tests.
    - Why it matters: Current tests are generated lint stubs only.
    - Suggested fix: Add unit tests for parsers, stale logic, fusion penalties, VFH sector selection, mux priority, and serial command formatting.

13. Add reconnect and error handling for serial nodes.
    - Why it matters: USB serial devices often reset or disconnect during robotics testing.
    - Suggested fix: Add retry loops, heartbeat status, and diagnostics messages.

14. Parameterize `gap_follow`.
    - Why it matters: It currently has many hardcoded tuning values and topic names.
    - Suggested fix: Declare ROS parameters matching `vfh_pkg` style.

15. Parameterize logger output.
    - Why it matters: `lidar_logger` writes to the current working directory, which varies by launch method.
    - Suggested fix: Add `output_dir`, `angle_min`, `angle_max`, and `scan_topic` parameters.

16. Add freshness checks to fusion consumers.
    - Why it matters: Stale sensor data can continue to affect navigation.
    - Suggested fix: Include timestamps in typed messages or track ROS receive times per input.

17. Document hardware frames and calibration.
    - Why it matters: TF frames, LiDAR-to-IMU translation, ToF bearing, and ultrasonic bearings are hardcoded defaults.
    - Suggested fix: Put frame IDs and sensor extrinsics in config.

18. Update package metadata.
    - Why it matters: Many packages still say `TODO: Package description` and `TODO: License declaration`.
    - Suggested fix: Add accurate descriptions, maintainers, and license fields.

19. Add package READMEs.
    - Why it matters: New contributors need local context at each package boundary.
    - Suggested fix: Add a short README to each package with topics, parameters, and launch/run examples.

20. Decide which navigation stack is canonical.
    - Why it matters: `gap_follow`, `vfh_pkg`, and `esp_fusion_pkg` can all publish commands in different ways.
    - Suggested fix: Mark old/experimental packages or create launch modes that make command ownership explicit.

## Suggested Future Modules and Features

| Suggestion                             | What it would do                                                                    | Why it would help                                        | Interacts with                                                                               | Rough implementation idea                                                                                                          | Priority |
| -------------------------------------- | ----------------------------------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- | -------- |
| `ugv_bringup` package                  | Centralize launch files for robot modes                                             | Removes guesswork from startup                           | All packages                                                                                 | Create `ament_python` or `ament_cmake` package with launch files for LiDAR, full autonomy, camera follow, logging, and diagnostics | High     |
| Hardware configuration package         | Store serial ports, IPs, frames, and thresholds                                     | Avoids hardcoded robot-specific values                   | `esp_bridge`, `vision_uart_pkg`, `unitree_lidar_ros2`, `vfh_pkg`, `fusion_pkg`               | Add YAML files loaded by launch arguments                                                                                          | High     |
| `/scan` adapter package                | Convert Unitree point cloud to LaserScan                                            | Enables existing navigation nodes with Unitree driver    | `unitree_lidar_ros2`, `gap_follow`, `vfh_pkg`, `lidar_logger_pkg`, `esp_fusion_pkg`          | Launch `pointcloud_to_laserscan` or write a projection node                                                                        | High     |
| Diagnostics and health monitor         | Publish status for LiDAR, ESP, ToF, ultrasonic, camera, and command streams         | Makes field failures easier to diagnose                  | All hardware-facing nodes                                                                    | Use `diagnostic_msgs` and timeout checks per topic/serial device                                                                   | High     |
| Safety supervisor node                 | Be the single final authority for stop, escape, and command limits                  | Clarifies current safety conflicts                       | `fusion_pkg`, `vfh_pkg`, `vision_uart_pkg`, `esp_bridge`                                     | Replace or expand `safety_mux_node` with explicit modes and priorities                                                             | High     |
| Serial-port manager                    | Discover stable serial devices and expose them as parameters                        | Reduces `/dev/ttyACM0` conflicts                         | `esp_bridge`, `esp_fusion_pkg`, `vision_uart_pkg`, Unitree SDK examples                      | Use `/dev/serial/by-id` mappings and a startup validation script                                                                   | High     |
| Calibration package                    | Calibrate ToF baseline, ultrasonic bearings, LiDAR frames, and camera angle mapping | Improves sensor fusion accuracy                          | `tof_processor_pkg`, `ultrasonic_pkg`, `fusion_pkg`, `vision_uart_pkg`, `unitree_lidar_ros2` | Add calibration commands/services and saved YAML outputs                                                                           | High     |
| Rosbag workflow                        | Record and replay experiments                                                       | Makes debugging repeatable without hardware              | All ROS topics                                                                               | Add scripts/launch files for `ros2 bag record` profiles and replay instructions                                                    | High     |
| Unit and integration tests             | Test parsers, fusion, VFH, mux, and serial command formatting                       | Prevents regressions during tuning                       | Python packages                                                                              | Use `pytest` and ROS 2 launch tests with sample messages                                                                           | High     |
| Sensor heartbeat system                | Publish last-seen time and validity for each sensor                                 | Prevents stale data from silently controlling navigation | Sensor processors and mux                                                                    | Add heartbeat topics or diagnostics status for each node                                                                           | High     |
| Simulation mode                        | Run navigation without physical hardware                                            | Helps development when the robot is unavailable          | `vfh_pkg`, `fusion_pkg`, `vision_uart_pkg`, `esp_bridge` replacement                         | Use Gazebo/Ignition or simple synthetic scan publishers and fake ESP/camera nodes                                                  | Medium   |
| Foxglove or web dashboard setup        | Visualize topics, safety state, and serial health                                   | Easier field debugging than terminal-only checks         | All runtime nodes                                                                            | Add Foxglove layout, topic naming guide, and optional bridge launch                                                                | Medium   |
| Data logging and report generator      | Save sensor snapshots and generate experiment summaries                             | Helps tune thresholds and compare runs                   | `lidar_logger_pkg`, fusion/debug topics                                                      | Extend logger to record selected topics and produce CSV/Markdown summaries                                                         | Medium   |
| Behavior tree or state machine package | Manage modes like idle, autonomous, camera follow, stop, recover                    | Makes robot behavior explicit                            | `safety_mux_node`, `vfh_pkg`, `vision_uart_pkg`, `esp_bridge`                                | Use `py_trees`, SMACH-like logic, or a simple ROS state machine                                                                    | Medium   |
| Custom message interfaces              | Replace string feature protocols                                                    | Improves type safety and docs                            | `tof_processor_pkg`, `ultrasonic_pkg`, `fusion_pkg`                                          | Add `ugv_msgs` with ToF, ultrasonic, fusion, and vision messages                                                                   | Medium   |
| Package-level READMEs                  | Document each package locally                                                       | Helps new contributors work in one package at a time     | Every package                                                                                | Add concise README files generated from this overview                                                                              | Low      |

## Summary Table

| Package/File                                   | Purpose                                | Main inputs                                                           | Main outputs                                                               | Hardware dependency             | Suggested improvements                                    |
| ---------------------------------------------- | -------------------------------------- | --------------------------------------------------------------------- | -------------------------------------------------------------------------- | ------------------------------- | --------------------------------------------------------- |
| `README.md`                                    | Top-level intro and build notes        | None                                                                  | Human-readable setup docs                                                  | None                            | Mention missing `/scan` conversion and full bringup path  |
| `build_ws.sh`                                  | Build ROS workspace                    | ROS 2 Jazzy, `ws/src`                                                 | Colcon build/install/log folders                                           | Ubuntu/ROS environment          | Add clearer failure handling and optional no-rosdep flag  |
| `run_lidar.sh`                                 | Run Unitree LiDAR launch               | Built workspace                                                       | LiDAR node and RViz                                                        | Unitree LiDAR                   | Add headless option through launch arguments              |
| `github_beginner_guide.md`                     | GitHub onboarding                      | None                                                                  | Team workflow instructions                                                 | None                            | Keep in sync with README build commands                   |
| `.vscode/settings.json`                        | VS Code CMake path                     | Local path                                                            | Editor config                                                              | None                            | Remove user-specific absolute path                        |
| `data/*.csv`                                   | Sample LiDAR logs                      | Logger output                                                         | Empty CSV headers                                                          | `/scan` originally              | Replace with meaningful datasets or remove                |
| `esp_bridge`                                   | Final command and ESP telemetry bridge | `/cmd_vel`, ESP serial lines                                          | Serial `CMD`, `/odom`, TF, `/tof/front/summary`, `/esp/ultrasonic_raw`     | ESP32 serial                    | Add reconnect, complete dependencies, document protocol   |
| `esp_fusion_pkg`                               | Alternate VFH plus ESP serial fusion   | `/scan`, ESP `STATE` serial lines                                     | `/cmd_vel`                                                                 | ESP32 serial and `/scan`        | Resolve serial conflict and clarify whether canonical     |
| `fusion_pkg`                                   | ToF/ultrasonic feature fusion          | `/tof/features`, `/ultrasonic/features`                               | `/fusion/sector_penalties`, `/fusion/critical_stop`, `/fusion/speed_limit` | None directly                   | Use typed messages and stale checks                       |
| `gap_follow`                                   | Simple LiDAR-only obstacle avoidance   | `/scan`                                                               | `/cmd_vel`                                                                 | LiDAR scan source               | Parameterize topics and tuning                            |
| `lidar_logger_pkg`                             | Log front scan slice to CSV            | `/scan`                                                               | CSV file                                                                   | LiDAR scan source               | Parameterize output path/window and declare dependencies  |
| `tof_processor_pkg`                            | ToF floor/dropoff feature extraction   | `/tof/front/summary`                                                  | `/tof/features`                                                            | ESP ToF telemetry               | Document units, add stale timeout and calibration service |
| `ultrasonic_pkg`                               | Ultrasonic hazard extraction           | `/esp/ultrasonic_raw`                                                 | `/ultrasonic/features`                                                     | ESP ultrasonic telemetry        | Use validity flags, consider rear sensor, typed messages  |
| `unitree_lidar_ros2`                           | ROS wrapper for Unitree LiDAR SDK      | Unitree serial/UDP packets                                            | `unilidar/cloud`, `unilidar/imu`, TF                                       | Unitree LiDAR L2                | Fix SDK paths, optional RViz, add `/scan` conversion      |
| `unitree_lidar_sdk`                            | Vendored Unitree SDK                   | LiDAR serial/UDP                                                      | SDK point cloud and IMU data                                               | Unitree LiDAR L2                | Make a proper vendor package and document binaries        |
| `vfh_pkg`                                      | VFH navigation with fusion penalties   | `/scan`, `/fusion/*`                                                  | `/cmd_vel_vfh`                                                             | LiDAR scan source               | Clarify critical-stop behavior, stale fusion handling     |
| `vision_uart_pkg/vision_uart_receiver_node.py` | Parse camera UART data                 | UART serial lines                                                     | `/vision/*` topics                                                         | Camera/vision UART source       | Add reconnect and protocol spec                           |
| `vision_uart_pkg/camera_follow_node.py`        | Generate camera-follow commands        | `/vision/*` topics                                                    | `/cmd_vel_camera`, debug                                                   | Camera/vision topic source      | Move tuning to YAML and verify angle sign convention      |
| `vision_uart_pkg/safety_mux_node.py`           | Select final command                   | `/cmd_vel_camera`, `/cmd_vel_vfh`, `/fusion/*`, `/vision/target_seen` | `/cmd_vel`, `/mux/debug`                                                   | None directly                   | Expand into explicit safety supervisor                    |
| `unitree_lidar_ros2/launch/launch.py`          | Launch LiDAR and RViz                  | Hardcoded launch params                                               | Node processes                                                             | Unitree LiDAR, RViz environment | Add launch args and config files                          |
| `unitree_lidar_ros2/rviz/view.rviz`            | Visualize Unitree cloud and TF         | `/unilidar/cloud`, TF                                                 | RViz display                                                               | None directly                   | Add `/scan` display once available                        |
