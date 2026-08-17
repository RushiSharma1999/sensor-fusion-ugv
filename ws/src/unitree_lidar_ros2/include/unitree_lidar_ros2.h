/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#define BOOST_BIND_NO_PLACEHOLDERS

#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

// Checked-in Unitree SDK.
#include "unitree_lidar_sdk_pcl.h"

class UnitreeLidarSDKNode : public rclcpp::Node
{
public:
  explicit UnitreeLidarSDKNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~UnitreeLidarSDKNode() override;

private:
  enum class TransportMode : int
  {
    kSerial = 1,
    kUdp = 2,
  };

  void load_parameters();
  void validate_parameters() const;
  void initialize_driver();
  void close_transport();
  void publish_static_transforms();
  void timer_callback();
  void handle_imu_packet();
  void handle_point_packet();

  bool normalized_imu_quaternion(
    const LidarImuData & imu,
    geometry_msgs::msg::Quaternion & quaternion) const;
  bool imu_vectors_are_finite(const LidarImuData & imu) const;
  bool frame_id_is_valid(const std::string & frame_id) const;
  bool sdk_stamp_to_ros_time(
    std::uint32_t seconds,
    std::uint32_t nanoseconds,
    rclcpp::Time & stamp) const;
  bool cloud_stamp_to_ros_time(double seconds, rclcpp::Time & stamp) const;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // The public vendor API has no destroy function or virtual destructor. The
  // selected transport is closed explicitly; the process owns this allocation
  // for its remaining lifetime.
  UnitreeLidarReader * lidar_reader_{nullptr};
  bool transport_initialized_{false};

  int initialize_type_{2};
  int work_mode_{0};
  bool use_system_timestamp_{true};
  double range_min_{0.0};
  double range_max_{100.0};
  int cloud_scan_num_{18};

  std::string serial_port_{"/dev/ttyACM0"};
  int baudrate_{4000000};

  int lidar_port_{6101};
  std::string lidar_ip_{"192.168.1.62"};
  int local_port_{6201};
  std::string local_ip_{"192.168.1.2"};

  std::string cloud_frame_{"unilidar_lidar"};
  std::string cloud_topic_{"unilidar/cloud"};
  std::string imu_frame_{"unilidar_imu"};
  std::string imu_topic_{"unilidar/imu"};
  std::string initial_imu_frame_{"unilidar_imu_initial"};

  bool publish_imu_tf_{true};
  bool publish_imu_to_lidar_tf_{true};
  bool publish_robot_to_imu_tf_{false};
  std::string robot_frame_{};
  double imu_to_lidar_x_{0.007698};
  double imu_to_lidar_y_{0.014655};
  double imu_to_lidar_z_{-0.00667};
  double imu_to_lidar_qx_{0.0};
  double imu_to_lidar_qy_{0.0};
  double imu_to_lidar_qz_{0.0};
  double imu_to_lidar_qw_{1.0};
  double robot_to_imu_x_{0.0};
  double robot_to_imu_y_{0.0};
  double robot_to_imu_z_{0.0};
  double robot_to_imu_qx_{0.0};
  double robot_to_imu_qy_{0.0};
  double robot_to_imu_qz_{0.0};
  double robot_to_imu_qw_{0.0};

  int qos_depth_{10};
  int poll_interval_ms_{1};
};
