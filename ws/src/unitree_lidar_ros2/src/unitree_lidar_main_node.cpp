/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_lidar_main.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UnitreeLidarSDKNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("unitree_lidar_main_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
