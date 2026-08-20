# Unitree LiDAR ROS 2 wrapper

This `ament_cmake` package builds `unitree_lidar_ros2_node` against the checked-in sibling SDK. Linux `aarch64` selects the robot-target archive and Linux `x86_64` selects the development archive; other systems and processor names fail configuration.

The stable interface is documented in [`docs/interfaces/unitree_driver_tf_contract.md`](../../../docs/interfaces/unitree_driver_tf_contract.md). The default output remains `/unilidar/cloud` (`sensor_msgs/msg/PointCloud2`, frame `unilidar_lidar`) and `/unilidar/imu` (`sensor_msgs/msg/Imu`, frame `unilidar_imu`). Both use best-effort, volatile sensor-data QoS with keep-last depth 10.

Installed resources are:

```text
share/unitree_lidar_ros2/launch/launch.py
share/unitree_lidar_ros2/config/driver.defaults.yaml
share/unitree_lidar_ros2/rviz/view.rviz
```

After building and sourcing the workspace, launch headlessly:

```bash
ros2 launch unitree_lidar_ros2 launch.py
```

Start RViz deliberately:

```bash
ros2 launch unitree_lidar_ros2 launch.py use_rviz:=true
```

Use `ros2 launch unitree_lidar_ros2 launch.py --show-args` to list namespace, topic, frame, configuration-file, and RViz overrides. Transport and driver defaults live in the installed parameter file. Empty topic/frame launch arguments leave that YAML authoritative; only non-empty launch values override matching parameters. Missing files and invalid parameters fail with diagnostics; driver exit shuts down the launch.

Successful static validation, compilation, or launch-file discovery does not verify LiDAR initialization, communication, timestamps, physical transforms, rates, RViz display, shutdown behavior, or reconnection. This package does not publish `/scan` or start any navigation, fusion, serial-bridge, or motor component.
