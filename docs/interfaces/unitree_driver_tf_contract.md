# Unitree ROS 2 Driver and TF Contract

## Scope and evidence boundary

This document describes the checked-in `unitree_lidar_ros2` wrapper after the Phase 2 software changes. It is an interface contract, not evidence that a physical LiDAR, network, serial link, timestamp source, transform, or publish rate has passed on the robot.

The supported compatibility baseline remains Raspberry Pi 4, Ubuntu 22.04 Jammy, Linux `aarch64`, ROS 2 Humble, and Python 3.10. The checked-in SDK2 software lineage targets the Unitree L2 family. That identifies the software lineage only: the exact physical robot LiDAR model, hardware revision, and firmware are unknown until inspected. The wrapper does not publish `/scan`, start navigation, or start any actuator component.

## Verified from code

### Executable, node, transport, and configuration

| Item | Contract |
|---|---|
| ROS package | `unitree_lidar_ros2` |
| Executable | `unitree_lidar_ros2_node` |
| Default and launched node name | `unitree_lidar_ros2_node` |
| Supported transport selector | `initialize_type=1` for serial; `initialize_type=2` for UDP |
| Default transport | UDP |
| Driver configuration | Installed `share/unitree_lidar_ros2/config/driver.defaults.yaml`; compiled defaults are identical for direct execution |
| Configuration lifetime | Parameters are declared and read during construction; live parameter updates are not implemented |
| SDK polling | One `runParse()` call per wall-timer callback; default `poll_interval_ms=1` is a polling interval, not a claimed publish rate |
| Packet handling | SDK type 102 produces a cloud; type 104 produces IMU; type 103 is not converted to ROS `LaserScan` |

The node validates transport selection, the SDK2 work-mode bitfield, timestamp mode, ranges, UDP ports and IPv4 syntax, serial settings, frame IDs, distinct sensor frames, QoS depth, polling interval, and the static-transform quaternion before calling the SDK. Serial and UDP initialization return codes are checked. A null SDK reader, invalid parameter, or nonzero SDK initialization result makes the process exit with status 1 and a diagnostic.

**Documentation says** the Unitree SDK2 work-mode bitfield uses the following assignments. **Verified from code:** the wrapper enforces those assignments at its boundary rather than passing arbitrary values to `setLidarWorkMode`.

| Bit | SDK2 meaning when `0` / `1` | Wrapper behavior |
|---:|---|---|
| 0 | Standard 180-degree FOV / wide-angle 192-degree FOV | Both are accepted; physical FOV remains hardware-unverified |
| 1 | 3D / 2D measurement | `1` is rejected because this wrapper handles only 3D packet type 102 and does not implement 2D output |
| 2 | IMU enabled / IMU disabled | `1` is accepted only with `publish_imu_tf=false`; the IMU publisher remains advertised but silent, and unexpected IMU packets are ignored |
| 3 | Ethernet / serial transport | Must be `0` for `initialize_type=2` UDP and `1` for `initialize_type=1` serial |
| 4 | Power-on auto-start / wait for explicit start command | `1` is rejected because this node does not call `startLidarRotation()` |
| 5–31 | Reserved | Any set reserved bit is rejected |

Thus the default UDP mode is `0`; the matching basic serial mode is `8`. Wide-angle changes add bit 0. IMU-disabled modes add bit 2 and require the dynamic IMU TF to be disabled. The wrapper is not a utility for changing a connected LiDAR from one transport mode to another.

### Parameter contract

YAML configuration is authoritative. Launch loads `config_file` first and does not add topic/frame parameter overrides for empty launch arguments. A user-supplied non-empty `cloud_topic`, `imu_topic`, `cloud_frame`, `imu_frame`, or `initial_imu_frame` launch argument explicitly overrides only that matching YAML value. Direct `ros2 run` uses the compiled defaults unless a parameter file or command-line override is supplied.

| Parameter | Type | Default | Code-level meaning |
|---|---:|---:|---|
| `initialize_type` | integer | `2` | `1` serial, `2` UDP |
| `work_mode` | integer | `0` | Validated SDK2 bits 0–4; unsupported/reserved combinations fail before SDK initialization |
| `use_system_timestamp` | Boolean | `true` | Must remain `true`; `false` fails because no hardware-clock conversion is implemented |
| `range_min` | double | `0.0` | Minimum SDK point range, intended metres |
| `range_max` | double | `100.0` | Maximum SDK point range, intended metres |
| `cloud_scan_num` | integer | `18` | Vendor aggregation setting; exact physical meaning is not defined in the public header |
| `serial_port` | string | `/dev/ttyACM0` | Used only in serial mode |
| `baudrate` | integer | `4000000` | Used only in serial mode |
| `lidar_port` | integer | `6101` | Remote UDP port |
| `lidar_ip` | string | `192.168.1.62` | Remote LiDAR IPv4 address |
| `local_port` | integer | `6201` | Host UDP port |
| `local_ip` | string | `192.168.1.2` | Host IPv4 address expected to exist before launch |
| `cloud_frame` | string | `unilidar_lidar` | Cloud header frame and static TF child |
| `cloud_topic` | string | `unilidar/cloud` | Relative PointCloud2 topic |
| `imu_frame` | string | `unilidar_imu` | IMU header frame, dynamic TF child, and static TF parent |
| `imu_topic` | string | `unilidar/imu` | Relative IMU topic |
| `initial_imu_frame` | string | `unilidar_imu_initial` | Dynamic IMU-orientation TF parent |
| `publish_imu_tf` | Boolean | `true` | Enables the dynamic IMU-orientation transform |
| `publish_imu_to_lidar_tf` | Boolean | `true` | Enables the sensor-internal static transform |
| `publish_robot_to_imu_tf` | Boolean | `false` | Enables a measured static `robot_frame` to `imu_frame` transform; mutually exclusive with `publish_imu_tf` |
| `robot_frame` | string | empty | Required and validated only when the measured robot transform is enabled |
| `imu_to_lidar.translation.x` | double | `0.007698` | Code-inherited static x translation in metres |
| `imu_to_lidar.translation.y` | double | `0.014655` | Code-inherited static y translation in metres |
| `imu_to_lidar.translation.z` | double | `-0.00667` | Code-inherited static z translation in metres |
| `imu_to_lidar.rotation.x` | double | `0.0` | Static quaternion x |
| `imu_to_lidar.rotation.y` | double | `0.0` | Static quaternion y |
| `imu_to_lidar.rotation.z` | double | `0.0` | Static quaternion z |
| `imu_to_lidar.rotation.w` | double | `1.0` | Static quaternion w |
| `robot_to_imu.translation.x` | double | no default | Required measured x translation when robot TF is enabled |
| `robot_to_imu.translation.y` | double | no default | Required measured y translation when robot TF is enabled |
| `robot_to_imu.translation.z` | double | no default | Required measured z translation when robot TF is enabled |
| `robot_to_imu.rotation.x` | double | no default | Required measured quaternion x when robot TF is enabled |
| `robot_to_imu.rotation.y` | double | no default | Required measured quaternion y when robot TF is enabled |
| `robot_to_imu.rotation.z` | double | no default | Required measured quaternion z when robot TF is enabled |
| `robot_to_imu.rotation.w` | double | no default | Required measured quaternion w when robot TF is enabled |
| `qos_depth` | integer | `10` | Keep-last depth for both sensor publishers |
| `poll_interval_ms` | integer | `1` | SDK polling wall-timer interval |

The installed parameter file is JSON-formatted YAML so the ROS parameter structure and static validator both use a real parser without adding a Python YAML dependency.

### Topic and message contract

With the default empty namespace, relative topic names resolve as follows:

| Topic | ROS type | Publisher QoS | Header frame |
|---|---|---|---|
| `/unilidar/cloud` | `sensor_msgs/msg/PointCloud2` | best effort, volatile, keep last 10 | `unilidar_lidar` |
| `/unilidar/imu` | `sensor_msgs/msg/Imu` | best effort, volatile, keep last 10 | `unilidar_imu` |

The cloud is unorganized (`height=1`, `width=number of converted points`) and marked `is_dense=false`. PCL serialization provides `x`, `y`, `z`, `intensity`, `ring`, and point-relative `time` fields from `PointType`; the wrapper adds no filter. The SDK `cloud.id` is not represented because ROS 2 message headers have no sequence field.

The IMU message carries normalized SDK quaternion order `[0,1,2,3] = x,y,z,w`, angular velocity, and linear acceleration. A non-finite vector, non-finite or zero-norm quaternion, or invalid SDK timestamp causes that sample and its dynamic TF to be dropped with a throttled warning. Covariance arrays remain zero, meaning the wrapper supplies no measured covariance.

The driver never republishes cached cloud or IMU data. If input stops, the output topics become silent; no driver health topic or autonomous safe-state decision is implemented. Downstream freshness enforcement remains required before autonomous use. When work-mode bit 2 disables IMU, `/unilidar/imu` remains discoverable because the publisher is constructed, but it deliberately emits no messages; `publish_imu_tf` must be false and the dynamic initial-IMU TF is also absent.

### Timestamp contract

- Phase 2 requires `use_system_timestamp=true`. Setting it false fails parameter validation before SDK initialization because the wrapper has no hardware-clock-to-ROS-time conversion.
- PointCloud2 uses `PointCloudUnitree.stamp`. In the checked-in SDK utility, `use_system_timestamp=true` derives cloud-start time from `CLOCK_REALTIME` minus scan period. The precompiled SDK implementation remains opaque.
- IMU uses `LidarImuData.info.stamp`. The wrapper represents it as ROS system time under the required configuration, but the public SDK does not establish how the precompiled implementation produces or synchronizes this IMU stamp; hardware validation remains required.
- The dynamic IMU TF uses exactly the same header timestamp as its IMU message.
- The static transform is sent once with node time through `/tf_static`; its geometry does not change.
- Invalid timestamps are dropped. No clock-domain reconciliation or message-filter synchronization is provided.

### TF contract

Default sensor-only tree:

```text
unilidar_imu_initial
`-- unilidar_imu       dynamic on /tf; zero translation, SDK IMU quaternion
    `-- unilidar_lidar static on /tf_static; configurable code-inherited extrinsic
```

| Parent | Child | Ownership | Geometry and time |
|---|---|---|---|
| `initial_imu_frame` | `imu_frame` | Dynamic `TransformBroadcaster`, enabled by `publish_imu_tf` | Zero translation and the same normalized `x,y,z,w` quaternion/timestamp published in the IMU message |
| `imu_frame` | `cloud_frame` | `StaticTransformBroadcaster`, enabled by `publish_imu_to_lidar_tf` | Configurable translation and normalized quaternion sent once |

The default tree gives each child one parent. Parameter validation rejects duplicate default sensor-frame identities. The default configuration does not publish `base_link`, `odom`, or another robot-to-sensor transform. The code-inherited IMU-to-LiDAR translation is preserved for compatibility and parameterized; it is not upgraded to a measured physical fact.

The dynamic initial-IMU transform remains available for legacy compatibility. Its array order, normalization, message/TF equality, and timestamp equality are source-verified, but its physical orientation semantics remain unverified: Phase 2 has not established the SDK quaternion's physical reference-frame direction, sign convention, or whether `initial_imu_frame -> imu_frame` is the physically correct parent-child interpretation.

After a robot mounting measurement is recorded, a reviewed parameter file may instead set `publish_imu_tf=false`, `publish_robot_to_imu_tf=true`, a confirmed `robot_frame`, and all seven `robot_to_imu.*` values. The node will then publish this rigid tree:

```text
<confirmed robot_frame>
`-- <imu_frame>   static measured robot_to_imu
    `-- <cloud_frame> static configured imu_to_lidar
```

All seven measured-only parameters are conditionally declared without numeric defaults when `publish_robot_to_imu_tf=true`; they remain undeclared in the default mode. Enabling the transform with a missing, non-finite, or zero-norm value fails startup. Enabling it together with the dynamic `initial_imu_frame -> imu_frame` transform also fails startup because that would give `imu_frame` two parents.

### Failure, shutdown, resources, and SDK selection

- Failed parameter validation or SDK initialization terminates with status 1 instead of starting publishers against a failed transport.
- A negative `runParse()` result and getter/data-validation failures are logged at most once per five seconds per call site.
- Driver destruction cancels the poll timer and calls `closeSerial()` or `closeUDP()` for the selected initialized transport; a `false` return is logged. The public SDK exposes neither a destroy function nor a virtual base destructor, so the reader allocation is process-lifetime rather than deleted unsafely through the abstract base pointer.
- Driver exit triggers launch shutdown, including optional RViz.
- CMake supports only Linux `aarch64` and `x86_64`, selects `../unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}/libunilidar_sdk2.a`, links `Threads::Threads`, and writes the absolute selection to the build-tree `unitree_sdk_library.txt`.
- Launch, config, and RViz resources install under `share/unitree_lidar_ros2/{launch,config,rviz}`. The executable statically links the selected vendor archive, so runtime does not depend on a source-tree SDK path.

## Launch interface

| Launch argument | Default | Behavior |
|---|---|---|
| `config_file` | installed `config/driver.defaults.yaml` | Transport and driver parameter file; missing paths fail with the exact path |
| `namespace` | empty | Node namespace; relative topics follow it |
| `cloud_topic` | empty | No override; a non-empty value explicitly overrides the YAML PointCloud2 topic |
| `imu_topic` | empty | No override; a non-empty value explicitly overrides the YAML IMU topic |
| `cloud_frame` | empty | No override; a non-empty value explicitly overrides the YAML cloud frame |
| `imu_frame` | empty | No override; a non-empty value explicitly overrides the YAML IMU frame |
| `initial_imu_frame` | empty | No override; a non-empty value explicitly overrides the YAML dynamic parent frame |
| `use_rviz` | `false` | Safe headless default; RViz starts only when explicitly true |
| `rviz_config` | installed `rviz/view.rviz` | Used and validated only when RViz is requested |

Headless default:

```bash
ros2 launch unitree_lidar_ros2 launch.py
```

Deliberate visualization:

```bash
ros2 launch unitree_lidar_ros2 launch.py use_rviz:=true
```

Namespace/topic example:

```bash
ros2 launch unitree_lidar_ros2 launch.py \
  namespace:=robot \
  cloud_topic:=sensors/lidar/cloud \
  imu_topic:=sensors/lidar/imu
```

The installed default YAML supplies the effective root-namespace topic/frame defaults listed above in the parameter table. Relative names remain relative to `namespace`; the empty namespace preserves `/unilidar/cloud` and `/unilidar/imu`. The RViz display uses the relative `unilidar/cloud` topic and shares the driver namespace. If a custom YAML changes the cloud topic, pass the same value explicitly as `cloud_topic:=...` when starting RViz so the launch can remap the display; launch deliberately does not parse and rewrite arbitrary YAML.

The repository helper passes all launch overrides through, so `./run_lidar.sh use_rviz:=true` is equivalent after prerequisite checks. Launch never reconfigures a host Ethernet interface.

## Recovered historical behavior

The recovered Pi used:

```bash
ros2 launch unitree_lidar_ros2 launch.py
```

and an external `pointcloud_to_laserscan` process consumed `/unilidar/cloud` with `target_frame:=unilidar_lidar`. That establishes compatibility expectations for the default cloud topic and frame, which this contract preserves. The recovered launcher configured UDP LiDAR `192.168.1.62:6101` and host `192.168.1.2:6201`; the current defaults match those source and launch values.

Before Phase 2, launch always started RViz, direct execution used reversed UDP IP defaults, the constructor node name was misspelled, failed SDK initialization was ignored, IMU was fetched twice, the TF quaternion order differed from the IMU message, the constant IMU-to-LiDAR transform was sent dynamically, and shutdown did not close the selected transport.

## Unverified hardware assumptions

- The checked-in software is Unitree SDK2 version 2.0.9 from the L2-family software lineage, but the exact physical robot LiDAR model, hardware version, firmware version, supported transport, deployed work mode, and compatibility are unconfirmed until inspected.
- The default IP addresses, ports, serial path, baud rate, range bounds, cloud aggregation, and polling interval have not been validated on the current physical robot.
- The SDK quaternion's physical reference convention and sign, IMU units, timestamp domains, measured publish rates, reconnect behavior, and packet-loss behavior are unverified.
- The code-inherited IMU-to-LiDAR translation and identity rotation have no checked-in measurement or vendor-document provenance. Their physical correctness must be confirmed before use as experimental evidence.
- No LiDAR/IMU mounting transform to `base_link` is known or published by the default configuration. A disabled measured-transform hook is available, but it has no numeric defaults and must not be enabled until the actual robot frame and all seven extrinsic values have measurement provenance. Adding an identity transform merely to connect TF would conceal the missing measurement and is prohibited.
- No successful initialization, communication, cloud, IMU, TF, RViz, shutdown, reconnection, or post-reboot result is claimed by static validation.

The exact Phase 2 physical acceptance commands and recording requirements are maintained in [`../verification/test_matrix.md`](../verification/test_matrix.md#phase-2-pilidar-acceptance-procedure).

## Deferred Phase 3 behavior

Phase 3 begins with a reviewed PointCloud2-to-LaserScan component and measured robot-centric geometry. This driver does not publish `/scan`, choose projection height/range filters, connect the sensor tree to `base_link`, or establish scan forward/angle conventions. Obstacle avoidance, fusion, command arbitration, serial actuation, motors, navigation, SLAM, and Nav2 are also outside this contract.
