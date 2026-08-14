cmake_minimum_required(VERSION 3.5)
project(unitree_lidar_sdk)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
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

set(UNITREE_SDK_LIBRARY
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunilidar_sdk2.a"
)

if(NOT EXISTS "${UNITREE_SDK_LIBRARY}")
  message(FATAL_ERROR "Unitree SDK library not found: ${UNITREE_SDK_LIBRARY}")
endif()

find_package(Threads REQUIRED)

add_library(unitree_lidar_sdk_vendor STATIC IMPORTED)
set_target_properties(unitree_lidar_sdk_vendor PROPERTIES
  IMPORTED_LOCATION "${UNITREE_SDK_LIBRARY}"
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(example_lidar_udp
  examples/example_lidar_udp.cpp
)

add_executable(example_lidar_serial
    examples/example_lidar_serial.cpp
)

add_executable(set_ip_address
    examples/set_ip_address.cpp
)

add_executable(set_to_serial_mode
    examples/set_to_serial_mode.cpp
)

add_executable(set_to_udp_mode
    examples/set_to_udp_mode.cpp
)

set(UNITREE_SDK_EXAMPLE_TARGETS
  example_lidar_udp
  example_lidar_serial
  set_ip_address
  set_to_serial_mode
  set_to_udp_mode
)

foreach(target IN LISTS UNITREE_SDK_EXAMPLE_TARGETS)
  target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_link_libraries(${target} PRIVATE unitree_lidar_sdk_vendor Threads::Threads)
endforeach()
