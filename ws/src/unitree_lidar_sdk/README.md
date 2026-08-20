# Checked-in Unitree LiDAR SDK

**Verified from code:** `include/unitree_lidar_sdk_config.h` identifies this SDK as version 2.0.9. The repository contains `libunilidar_sdk2.a` archives for Linux `aarch64` and Linux `x86_64`; object inspection confirms that the archive contents match those architecture names. The prebuilt utilities under `bin/` are Linux `aarch64` executables. The checked-in software follows Unitree's SDK2 lineage for the L2 family; this software provenance does not identify the exact LiDAR model or firmware installed on the physical robot.

The static libraries are vendor binaries. Their source implementation is not present and their compatibility with the deployed LiDAR model, firmware, and Raspberry Pi image is **not yet hardware verified**.

## Supported standalone builds

The example CMake project supports only Linux toolchains whose `CMAKE_SYSTEM_PROCESSOR` is exactly `aarch64` or `x86_64`. Other systems and architectures fail configuration with a clear diagnostic because no matching checked-in library exists.

From a supported Linux host:

```bash
cmake -S ws/src/unitree_lidar_sdk -B /tmp/unitree_lidar_sdk-build
cmake --build /tmp/unitree_lidar_sdk-build
```

Build outputs are written under the selected build directory, not over the checked-in vendor binaries. The ROS wrapper normally consumes the SDK directly from this sibling directory; a separate SDK example build is not required for the ROS workspace build.
