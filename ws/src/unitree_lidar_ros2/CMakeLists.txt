cmake_minimum_required(VERSION 3.5)
project(unitree_lidar_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(UNITREE_SDK_SUPPORTED_ARCHITECTURES aarch64 x86_64)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR
    "The checked-in Unitree SDK contains Linux libraries only; "
    "the active system is '${CMAKE_SYSTEM_NAME}'."
  )
endif()

list(FIND UNITREE_SDK_SUPPORTED_ARCHITECTURES "${CMAKE_SYSTEM_PROCESSOR}" UNITREE_SDK_ARCH_INDEX)
if(UNITREE_SDK_ARCH_INDEX EQUAL -1)
  message(FATAL_ERROR
    "Unsupported Unitree SDK architecture '${CMAKE_SYSTEM_PROCESSOR}'. "
    "Checked-in Linux libraries support only: ${UNITREE_SDK_SUPPORTED_ARCHITECTURES}."
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Threads REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

set(UNITREE_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_lidar_sdk"
  CACHE PATH "Path to the checked-in Unitree LiDAR SDK"
)
get_filename_component(UNITREE_SDK_ROOT "${UNITREE_SDK_ROOT}" ABSOLUTE)

set(UNITREE_SDK_INCLUDE_DIR "${UNITREE_SDK_ROOT}/include")
set(UNITREE_SDK_LIBRARY
  "${UNITREE_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunilidar_sdk2.a"
)

if(NOT IS_DIRECTORY "${UNITREE_SDK_INCLUDE_DIR}")
  message(FATAL_ERROR "Unitree SDK include directory not found: ${UNITREE_SDK_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${UNITREE_SDK_LIBRARY}")
  message(FATAL_ERROR "Unitree SDK library not found: ${UNITREE_SDK_LIBRARY}")
endif()

add_library(unitree_lidar_sdk STATIC IMPORTED)
set_target_properties(unitree_lidar_sdk PROPERTIES
  IMPORTED_LOCATION "${UNITREE_SDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${UNITREE_SDK_INCLUDE_DIR}"
)
message(STATUS "Unitree SDK library: ${UNITREE_SDK_LIBRARY}")
file(WRITE
  "${CMAKE_CURRENT_BINARY_DIR}/unitree_sdk_library.txt"
  "${UNITREE_SDK_LIBRARY}\n"
)

add_definitions(${PCL_DEFINITIONS})

add_executable(unitree_lidar_ros2_node src/unitree_lidar_ros2_node.cpp)

target_include_directories(unitree_lidar_ros2_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(unitree_lidar_ros2_node
  ${PCL_LIBRARIES}
  unitree_lidar_sdk
  Threads::Threads
)

ament_target_dependencies(
  unitree_lidar_ros2_node
  rclcpp std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_conversions
)

install(TARGETS
  unitree_lidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
