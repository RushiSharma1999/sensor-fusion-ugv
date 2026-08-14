# Unitree LiDAR ROS 2 wrapper

This `ament_cmake` package builds `unitree_lidar_ros2_node` against the checked-in sibling SDK at `../unitree_lidar_sdk`.

**Verified from code:** the SDK provides Linux `aarch64` and `x86_64` static libraries. CMake rejects unsupported operating systems or processor names rather than selecting an unverified binary.

The package installs resources conventionally as:

```text
share/unitree_lidar_ros2/launch/launch.py
share/unitree_lidar_ros2/rviz/view.rviz
```

After the workspace is built and sourced, the existing user-facing launch command remains:

```bash
ros2 launch unitree_lidar_ros2 launch.py
```

This launch starts the driver and RViz. Successful compilation or launch-file discovery does not verify LiDAR connectivity, timestamps, transforms, or hardware behavior.
