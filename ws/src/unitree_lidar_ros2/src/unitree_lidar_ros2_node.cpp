/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_lidar_ros2.h"

#include <arpa/inet.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>

#include <pcl_conversions/pcl_conversions.h>

namespace
{
constexpr double kMinimumQuaternionNorm = 1.0e-12;
constexpr std::uint32_t kNanosecondsPerSecond = 1000000000U;
constexpr int kWarningThrottleMilliseconds = 5000;
constexpr std::uint32_t kWorkModeWideAngleFovBit = 1U << 0U;
constexpr std::uint32_t kWorkMode2dMeasurementBit = 1U << 1U;
constexpr std::uint32_t kWorkModeDisableImuBit = 1U << 2U;
constexpr std::uint32_t kWorkModeSerialTransportBit = 1U << 3U;
constexpr std::uint32_t kWorkModeWaitForStartBit = 1U << 4U;
constexpr std::uint32_t kWorkModeDefinedMask =
  kWorkModeWideAngleFovBit |
  kWorkMode2dMeasurementBit |
  kWorkModeDisableImuBit |
  kWorkModeSerialTransportBit |
  kWorkModeWaitForStartBit;

bool is_valid_ipv4_address(const std::string & address)
{
  in_addr parsed_address{};
  return inet_pton(AF_INET, address.c_str(), &parsed_address) == 1;
}

bool is_finite(double value)
{
  return std::isfinite(value);
}
}  // namespace

UnitreeLidarSDKNode::UnitreeLidarSDKNode(const rclcpp::NodeOptions & options)
: Node("unitree_lidar_ros2_node", options)
{
  load_parameters();
  validate_parameters();

  const auto sensor_qos = rclcpp::SensorDataQoS(
    rclcpp::KeepLast(static_cast<std::size_t>(qos_depth_)));
  pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, sensor_qos);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, sensor_qos);

  if (publish_imu_tf_) {
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }
  if (publish_imu_to_lidar_tf_ || publish_robot_to_imu_tf_) {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
  }

  initialize_driver();
  try {
    if (static_tf_broadcaster_) {
      publish_static_transforms();
    }
    timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_interval_ms_),
      std::bind(&UnitreeLidarSDKNode::timer_callback, this));
  } catch (...) {
    close_transport();
    throw;
  }

  RCLCPP_INFO(
    get_logger(),
    "Unitree driver initialized: transport=%s, cloud=%s [%s], imu=%s [%s]",
    initialize_type_ == static_cast<int>(TransportMode::kSerial) ? "serial" : "udp",
    cloud_topic_.c_str(), cloud_frame_.c_str(), imu_topic_.c_str(), imu_frame_.c_str());
}

UnitreeLidarSDKNode::~UnitreeLidarSDKNode()
{
  if (timer_) {
    timer_->cancel();
  }

  close_transport();
}

void UnitreeLidarSDKNode::load_parameters()
{
  initialize_type_ = declare_parameter<int>("initialize_type", 2);
  work_mode_ = declare_parameter<int>("work_mode", 0);
  use_system_timestamp_ = declare_parameter<bool>("use_system_timestamp", true);
  range_min_ = declare_parameter<double>("range_min", 0.0);
  range_max_ = declare_parameter<double>("range_max", 100.0);
  cloud_scan_num_ = declare_parameter<int>("cloud_scan_num", 18);

  serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  baudrate_ = declare_parameter<int>("baudrate", 4000000);

  lidar_port_ = declare_parameter<int>("lidar_port", 6101);
  lidar_ip_ = declare_parameter<std::string>("lidar_ip", "192.168.1.62");
  local_port_ = declare_parameter<int>("local_port", 6201);
  local_ip_ = declare_parameter<std::string>("local_ip", "192.168.1.2");

  cloud_frame_ = declare_parameter<std::string>("cloud_frame", "unilidar_lidar");
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "unilidar/cloud");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "unilidar_imu");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "unilidar/imu");
  initial_imu_frame_ = declare_parameter<std::string>(
    "initial_imu_frame", "unilidar_imu_initial");

  publish_imu_tf_ = declare_parameter<bool>("publish_imu_tf", true);
  publish_imu_to_lidar_tf_ = declare_parameter<bool>("publish_imu_to_lidar_tf", true);
  publish_robot_to_imu_tf_ = declare_parameter<bool>("publish_robot_to_imu_tf", false);
  robot_frame_ = declare_parameter<std::string>("robot_frame", "");
  imu_to_lidar_x_ = declare_parameter<double>("imu_to_lidar.translation.x", 0.007698);
  imu_to_lidar_y_ = declare_parameter<double>("imu_to_lidar.translation.y", 0.014655);
  imu_to_lidar_z_ = declare_parameter<double>("imu_to_lidar.translation.z", -0.00667);
  imu_to_lidar_qx_ = declare_parameter<double>("imu_to_lidar.rotation.x", 0.0);
  imu_to_lidar_qy_ = declare_parameter<double>("imu_to_lidar.rotation.y", 0.0);
  imu_to_lidar_qz_ = declare_parameter<double>("imu_to_lidar.rotation.z", 0.0);
  imu_to_lidar_qw_ = declare_parameter<double>("imu_to_lidar.rotation.w", 1.0);

  if (publish_robot_to_imu_tf_) {
    robot_to_imu_x_ = declare_parameter<double>("robot_to_imu.translation.x");
    robot_to_imu_y_ = declare_parameter<double>("robot_to_imu.translation.y");
    robot_to_imu_z_ = declare_parameter<double>("robot_to_imu.translation.z");
    robot_to_imu_qx_ = declare_parameter<double>("robot_to_imu.rotation.x");
    robot_to_imu_qy_ = declare_parameter<double>("robot_to_imu.rotation.y");
    robot_to_imu_qz_ = declare_parameter<double>("robot_to_imu.rotation.z");
    robot_to_imu_qw_ = declare_parameter<double>("robot_to_imu.rotation.w");
  }

  qos_depth_ = declare_parameter<int>("qos_depth", 10);
  poll_interval_ms_ = declare_parameter<int>("poll_interval_ms", 1);
}

void UnitreeLidarSDKNode::validate_parameters() const
{
  if (
    initialize_type_ != static_cast<int>(TransportMode::kSerial) &&
    initialize_type_ != static_cast<int>(TransportMode::kUdp))
  {
    throw std::invalid_argument("initialize_type must be 1 (serial) or 2 (UDP)");
  }
  if (work_mode_ < 0) {
    throw std::invalid_argument("work_mode must be a non-negative SDK2 bitfield");
  }
  const auto work_mode = static_cast<std::uint32_t>(work_mode_);
  if ((work_mode & ~kWorkModeDefinedMask) != 0U) {
    throw std::invalid_argument("work_mode bits 5 through 31 are reserved and must be zero");
  }
  if ((work_mode & kWorkMode2dMeasurementBit) != 0U) {
    throw std::invalid_argument(
            "work_mode 2D measurement mode is unsupported; this wrapper publishes only 3D PointCloud2 data");
  }
  const bool work_mode_selects_serial =
    (work_mode & kWorkModeSerialTransportBit) != 0U;
  if (
    initialize_type_ == static_cast<int>(TransportMode::kUdp) &&
    work_mode_selects_serial)
  {
    throw std::invalid_argument(
            "initialize_type=2 (UDP) requires work_mode bit 3 to select Ethernet (0)");
  }
  if (
    initialize_type_ == static_cast<int>(TransportMode::kSerial) &&
    !work_mode_selects_serial)
  {
    throw std::invalid_argument(
            "initialize_type=1 (serial) requires work_mode bit 3 to select serial (1)");
  }
  if ((work_mode & kWorkModeWaitForStartBit) != 0U) {
    throw std::invalid_argument(
            "work_mode bit 4 (wait for start command) is unsupported because this node does not start LiDAR rotation");
  }
  if ((work_mode & kWorkModeDisableImuBit) != 0U && publish_imu_tf_) {
    throw std::invalid_argument(
            "work_mode disables IMU data, so publish_imu_tf must be false");
  }
  if (!use_system_timestamp_) {
    throw std::invalid_argument(
            "use_system_timestamp must be true; hardware-clock to ROS-time conversion is not implemented");
  }
  if (!is_finite(range_min_) || !is_finite(range_max_) || range_min_ < 0.0 || range_min_ >= range_max_) {
    throw std::invalid_argument("range_min and range_max must be finite and satisfy 0 <= min < max");
  }
  if (cloud_scan_num_ <= 0 || cloud_scan_num_ > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("cloud_scan_num must be in [1, 65535]");
  }
  if (qos_depth_ <= 0) {
    throw std::invalid_argument("qos_depth must be positive");
  }
  if (poll_interval_ms_ <= 0) {
    throw std::invalid_argument("poll_interval_ms must be positive");
  }

  if (initialize_type_ == static_cast<int>(TransportMode::kSerial)) {
    if (serial_port_.empty()) {
      throw std::invalid_argument("serial_port must not be empty in serial mode");
    }
    if (baudrate_ <= 0) {
      throw std::invalid_argument("baudrate must be positive in serial mode");
    }
  } else {
    if (lidar_port_ <= 0 || lidar_port_ > 65535 || local_port_ <= 0 || local_port_ > 65535) {
      throw std::invalid_argument("lidar_port and local_port must be in [1, 65535]");
    }
    if (!is_valid_ipv4_address(lidar_ip_)) {
      throw std::invalid_argument("lidar_ip must be a valid IPv4 address");
    }
    if (!is_valid_ipv4_address(local_ip_)) {
      throw std::invalid_argument("local_ip must be a valid IPv4 address");
    }
  }

  if (cloud_topic_.empty() || imu_topic_.empty()) {
    throw std::invalid_argument("cloud_topic and imu_topic must not be empty");
  }
  if (!frame_id_is_valid(cloud_frame_) || !frame_id_is_valid(imu_frame_)) {
    throw std::invalid_argument("cloud_frame and imu_frame must be non-empty TF frame IDs without a leading slash or whitespace");
  }
  if (cloud_frame_ == imu_frame_) {
    throw std::invalid_argument("cloud_frame and imu_frame must be distinct");
  }
  if (publish_imu_tf_) {
    if (!frame_id_is_valid(initial_imu_frame_)) {
      throw std::invalid_argument("initial_imu_frame must be a valid TF frame ID when publish_imu_tf is true");
    }
    if (initial_imu_frame_ == imu_frame_ || initial_imu_frame_ == cloud_frame_) {
      throw std::invalid_argument("initial_imu_frame must be distinct from imu_frame and cloud_frame");
    }
  }

  if (publish_robot_to_imu_tf_) {
    if (publish_imu_tf_) {
      throw std::invalid_argument(
              "publish_robot_to_imu_tf and publish_imu_tf cannot both be true because imu_frame would have two parents");
    }
    if (!frame_id_is_valid(robot_frame_)) {
      throw std::invalid_argument(
              "robot_frame must be a valid TF frame ID when publish_robot_to_imu_tf is true");
    }
    if (
      robot_frame_ == imu_frame_ || robot_frame_ == cloud_frame_ ||
      robot_frame_ == initial_imu_frame_)
    {
      throw std::invalid_argument("robot_frame must be distinct from all Unitree sensor frames");
    }

    const double robot_quaternion_norm = std::sqrt(
      robot_to_imu_qx_ * robot_to_imu_qx_ +
      robot_to_imu_qy_ * robot_to_imu_qy_ +
      robot_to_imu_qz_ * robot_to_imu_qz_ +
      robot_to_imu_qw_ * robot_to_imu_qw_);
    if (
      !is_finite(robot_to_imu_x_) || !is_finite(robot_to_imu_y_) ||
      !is_finite(robot_to_imu_z_) || !is_finite(robot_quaternion_norm) ||
      robot_quaternion_norm < kMinimumQuaternionNorm)
    {
      throw std::invalid_argument(
              "robot_to_imu transform must contain measured finite translation and a non-zero finite quaternion");
    }
  }

  const double static_quaternion_norm = std::sqrt(
    imu_to_lidar_qx_ * imu_to_lidar_qx_ +
    imu_to_lidar_qy_ * imu_to_lidar_qy_ +
    imu_to_lidar_qz_ * imu_to_lidar_qz_ +
    imu_to_lidar_qw_ * imu_to_lidar_qw_);
  if (
    !is_finite(imu_to_lidar_x_) || !is_finite(imu_to_lidar_y_) || !is_finite(imu_to_lidar_z_) ||
    !is_finite(static_quaternion_norm) || static_quaternion_norm < kMinimumQuaternionNorm)
  {
    throw std::invalid_argument("imu_to_lidar transform must contain finite translation and a non-zero finite quaternion");
  }
}

void UnitreeLidarSDKNode::initialize_driver()
{
  lidar_reader_ = createUnitreeLidarReader();
  if (lidar_reader_ == nullptr) {
    throw std::runtime_error("Unitree SDK returned a null reader");
  }

  int initialization_result = -1;
  if (initialize_type_ == static_cast<int>(TransportMode::kSerial)) {
    initialization_result = lidar_reader_->initializeSerial(
      serial_port_, static_cast<std::uint32_t>(baudrate_),
      static_cast<std::uint16_t>(cloud_scan_num_), use_system_timestamp_,
      static_cast<float>(range_min_), static_cast<float>(range_max_));
  } else {
    initialization_result = lidar_reader_->initializeUDP(
      static_cast<unsigned short>(lidar_port_), lidar_ip_,
      static_cast<unsigned short>(local_port_), local_ip_,
      static_cast<std::uint16_t>(cloud_scan_num_), use_system_timestamp_,
      static_cast<float>(range_min_), static_cast<float>(range_max_));
  }

  if (initialization_result != 0) {
    const std::string transport =
      initialize_type_ == static_cast<int>(TransportMode::kSerial) ? "serial" : "UDP";
    throw std::runtime_error(
            "Unitree SDK " + transport + " initialization failed with code " +
            std::to_string(initialization_result));
  }

  transport_initialized_ = true;
  lidar_reader_->setLidarWorkMode(static_cast<std::uint32_t>(work_mode_));
}

void UnitreeLidarSDKNode::close_transport()
{
  if (!transport_initialized_ || lidar_reader_ == nullptr) {
    return;
  }

  bool close_succeeded = false;
  if (initialize_type_ == static_cast<int>(TransportMode::kSerial)) {
    close_succeeded = lidar_reader_->closeSerial();
  } else {
    close_succeeded = lidar_reader_->closeUDP();
  }
  transport_initialized_ = false;

  if (!close_succeeded) {
    RCLCPP_WARN(get_logger(), "Unitree SDK returned false while closing the selected transport");
  }
}

void UnitreeLidarSDKNode::publish_static_transforms()
{
  if (publish_imu_to_lidar_tf_) {
    const double norm = std::sqrt(
      imu_to_lidar_qx_ * imu_to_lidar_qx_ +
      imu_to_lidar_qy_ * imu_to_lidar_qy_ +
      imu_to_lidar_qz_ * imu_to_lidar_qz_ +
      imu_to_lidar_qw_ * imu_to_lidar_qw_);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = imu_frame_;
    transform.child_frame_id = cloud_frame_;
    transform.transform.translation.x = imu_to_lidar_x_;
    transform.transform.translation.y = imu_to_lidar_y_;
    transform.transform.translation.z = imu_to_lidar_z_;
    transform.transform.rotation.x = imu_to_lidar_qx_ / norm;
    transform.transform.rotation.y = imu_to_lidar_qy_ / norm;
    transform.transform.rotation.z = imu_to_lidar_qz_ / norm;
    transform.transform.rotation.w = imu_to_lidar_qw_ / norm;
    static_tf_broadcaster_->sendTransform(transform);
  }

  if (publish_robot_to_imu_tf_) {
    const double norm = std::sqrt(
      robot_to_imu_qx_ * robot_to_imu_qx_ +
      robot_to_imu_qy_ * robot_to_imu_qy_ +
      robot_to_imu_qz_ * robot_to_imu_qz_ +
      robot_to_imu_qw_ * robot_to_imu_qw_);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = robot_frame_;
    transform.child_frame_id = imu_frame_;
    transform.transform.translation.x = robot_to_imu_x_;
    transform.transform.translation.y = robot_to_imu_y_;
    transform.transform.translation.z = robot_to_imu_z_;
    transform.transform.rotation.x = robot_to_imu_qx_ / norm;
    transform.transform.rotation.y = robot_to_imu_qy_ / norm;
    transform.transform.rotation.z = robot_to_imu_qz_ / norm;
    transform.transform.rotation.w = robot_to_imu_qw_ / norm;
    static_tf_broadcaster_->sendTransform(transform);
  }
}

void UnitreeLidarSDKNode::timer_callback()
{
  const int result = lidar_reader_->runParse();
  if (result == LIDAR_IMU_DATA_PACKET_TYPE) {
    if ((static_cast<std::uint32_t>(work_mode_) & kWorkModeDisableImuBit) != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kWarningThrottleMilliseconds,
        "Ignoring an unexpected IMU packet because work_mode disables IMU output");
      return;
    }
    handle_imu_packet();
  } else if (result == LIDAR_POINT_DATA_PACKET_TYPE) {
    handle_point_packet();
  } else if (result < 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWarningThrottleMilliseconds,
      "Unitree SDK runParse returned error code %d", result);
  }
}

void UnitreeLidarSDKNode::handle_imu_packet()
{
  LidarImuData imu{};
  if (!lidar_reader_->getImuData(imu)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWarningThrottleMilliseconds,
      "Unitree SDK reported an IMU packet but did not return IMU data");
    return;
  }

  rclcpp::Time stamp(0, 0, RCL_SYSTEM_TIME);
  if (!sdk_stamp_to_ros_time(imu.info.stamp.sec, imu.info.stamp.nsec, stamp)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWarningThrottleMilliseconds,
      "Dropping Unitree IMU data with an invalid timestamp");
    return;
  }

  geometry_msgs::msg::Quaternion quaternion;
  if (!normalized_imu_quaternion(imu, quaternion) || !imu_vectors_are_finite(imu)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWarningThrottleMilliseconds,
      "Dropping Unitree IMU data with a non-finite vector or invalid quaternion");
    return;
  }

  sensor_msgs::msg::Imu message;
  message.header.frame_id = imu_frame_;
  message.header.stamp = stamp;
  message.orientation = quaternion;
  message.angular_velocity.x = imu.angular_velocity[0];
  message.angular_velocity.y = imu.angular_velocity[1];
  message.angular_velocity.z = imu.angular_velocity[2];
  message.linear_acceleration.x = imu.linear_acceleration[0];
  message.linear_acceleration.y = imu.linear_acceleration[1];
  message.linear_acceleration.z = imu.linear_acceleration[2];
  pub_imu_->publish(message);

  if (publish_imu_tf_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message.header.stamp;
    transform.header.frame_id = initial_imu_frame_;
    transform.child_frame_id = message.header.frame_id;
    transform.transform.rotation = quaternion;
    dynamic_tf_broadcaster_->sendTransform(transform);
  }
}

void UnitreeLidarSDKNode::handle_point_packet()
{
  PointCloudUnitree cloud;
  if (!lidar_reader_->getPointCloud(cloud)) {
    const auto now = std::chrono::steady_clock::now();
    const bool recent_valid_cloud =
      has_published_valid_cloud_ &&
      now - last_valid_cloud_time_ <
      std::chrono::milliseconds(kWarningThrottleMilliseconds);
    if (recent_valid_cloud) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), kWarningThrottleMilliseconds,
        "Unitree SDK point indication did not yield a cloud while valid clouds remain fresh");
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kWarningThrottleMilliseconds,
        "Unitree SDK point indication did not yield a cloud and no valid cloud was published recently");
    }
    return;
  }

  rclcpp::Time stamp(0, 0, RCL_SYSTEM_TIME);
  if (!cloud_stamp_to_ros_time(cloud.stamp, stamp)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWarningThrottleMilliseconds,
      "Dropping Unitree point cloud with an invalid timestamp");
    return;
  }

  pcl::PointCloud<PointType>::Ptr pcl_cloud(new pcl::PointCloud<PointType>());
  transformUnitreeCloudToPCL(cloud, pcl_cloud);
  pcl_cloud->width = static_cast<std::uint32_t>(pcl_cloud->points.size());
  pcl_cloud->height = 1U;
  pcl_cloud->is_dense = false;

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(*pcl_cloud, message);
  message.header.frame_id = cloud_frame_;
  message.header.stamp = stamp;
  pub_cloud_->publish(message);
  last_valid_cloud_time_ = std::chrono::steady_clock::now();
  has_published_valid_cloud_ = true;
}

bool UnitreeLidarSDKNode::normalized_imu_quaternion(
  const LidarImuData & imu,
  geometry_msgs::msg::Quaternion & quaternion) const
{
  // The vendor header/example and upstream ROS publishers disagree about this
  // array's component order. The project mapping below is supported by the
  // recorded upright, planar-rotation hardware test and matches Unitree's
  // upstream dynamic-TF conversion.
  const double x = imu.quaternion[1];
  const double y = imu.quaternion[2];
  const double z = imu.quaternion[3];
  const double w = imu.quaternion[0];
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!is_finite(x) || !is_finite(y) || !is_finite(z) || !is_finite(w) ||
    !is_finite(norm) || norm < kMinimumQuaternionNorm)
  {
    return false;
  }

  quaternion.x = x / norm;
  quaternion.y = y / norm;
  quaternion.z = z / norm;
  quaternion.w = w / norm;
  return true;
}

bool UnitreeLidarSDKNode::imu_vectors_are_finite(const LidarImuData & imu) const
{
  return std::all_of(
    std::begin(imu.angular_velocity), std::end(imu.angular_velocity),
    [](float value) {return std::isfinite(value);}) &&
         std::all_of(
    std::begin(imu.linear_acceleration), std::end(imu.linear_acceleration),
    [](float value) {return std::isfinite(value);});
}

bool UnitreeLidarSDKNode::frame_id_is_valid(const std::string & frame_id) const
{
  return !frame_id.empty() && frame_id.front() != '/' &&
         std::none_of(
    frame_id.begin(), frame_id.end(),
    [](unsigned char character) {return std::isspace(character) != 0;});
}

bool UnitreeLidarSDKNode::sdk_stamp_to_ros_time(
  std::uint32_t seconds,
  std::uint32_t nanoseconds,
  rclcpp::Time & stamp) const
{
  if (
    seconds > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()) ||
    nanoseconds >= kNanosecondsPerSecond)
  {
    return false;
  }
  stamp = rclcpp::Time(static_cast<std::int32_t>(seconds), nanoseconds, RCL_SYSTEM_TIME);
  return true;
}

bool UnitreeLidarSDKNode::cloud_stamp_to_ros_time(double seconds, rclcpp::Time & stamp) const
{
  if (!is_finite(seconds) || seconds < 0.0) {
    return false;
  }

  double whole_seconds = 0.0;
  const double fractional_seconds = std::modf(seconds, &whole_seconds);
  if (whole_seconds > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
    return false;
  }

  auto nanoseconds = static_cast<std::uint32_t>(fractional_seconds * kNanosecondsPerSecond);
  if (nanoseconds >= kNanosecondsPerSecond) {
    nanoseconds = kNanosecondsPerSecond - 1U;
  }
  stamp = rclcpp::Time(
    static_cast<std::int32_t>(whole_seconds), nanoseconds, RCL_SYSTEM_TIME);
  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    auto node = std::make_shared<UnitreeLidarSDKNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::cerr << "Unitree LiDAR driver failed: " << error.what() << std::endl;
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
