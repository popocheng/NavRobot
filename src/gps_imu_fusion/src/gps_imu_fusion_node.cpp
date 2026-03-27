#include <memory>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "GeographicLib/LocalCartesian.hpp"

class GpsImuFusionNode : public rclcpp::Node
{
public:
  GpsImuFusionNode() : Node("gps_imu_fusion_node")
  {
    // Initialize the orientation to represent no rotation (identity quaternion)
    current_orientation_.x = 0.0;
    current_orientation_.y = 0.0;
    current_orientation_.z = 0.0;
    current_orientation_.w = 1.0;

    // Declare parameters for origin
    this->declare_parameter("origin_lat", 37.7749);  // Default to San Francisco as example
    this->declare_parameter("origin_lon", -122.4194);
    this->declare_parameter("origin_alt", 0.0);
    this->declare_parameter("yaw_offset", 0.0);  // Yaw offset in radians
    this->declare_parameter("use_yaw_only", false);  // Whether to extract only yaw from IMU

    // Get parameters
    this->get_parameter("origin_lat", origin_lat_);
    this->get_parameter("origin_lon", origin_lon_);
    this->get_parameter("origin_alt", origin_alt_);
    this->get_parameter("yaw_offset", yaw_offset_);
    this->get_parameter("use_yaw_only", use_yaw_only_);

    // Initialize GeographicLib converter
    geo_converter_.reset(new GeographicLib::LocalCartesian(origin_lat_, origin_lon_, origin_alt_));

    // Initialize subscribers
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/data", 10,
      std::bind(&GpsImuFusionNode::gpsCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10,
      std::bind(&GpsImuFusionNode::imuCallback, this, std::placeholders::_1));

    // Initialize publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/world_odom", 10);

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize timers for periodic publishing
    odom_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&GpsImuFusionNode::publishOdomAndTf, this));

    RCLCPP_INFO(this->get_logger(), "GPS/IMU Fusion Node initialized with origin: lat=%.6f, lon=%.6f, alt=%.2f, yaw_offset=%.4f",
                origin_lat_, origin_lon_, origin_alt_, yaw_offset_);
  }

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Convert GPS coordinates to local cartesian coordinates (ENU: East, North, Up)
    double local_x, local_y, local_z;
    geo_converter_->Forward(msg->latitude, msg->longitude, msg->altitude, local_x, local_y, local_z); // ENU: x=E, y=N, z=U

    // Update our stored pose with the new GPS data
    current_x_ = local_x;  // E (East) - corresponds to +X
    current_y_ = local_y;  // N (North) - corresponds to +Y
    current_z_ = local_z;  // U (Up) - corresponds to +Z

    gps_received_ = true;

    RCLCPP_DEBUG(this->get_logger(), "GPS updated: x(East)=%.2f, y(North)=%.2f, z(Up)=%.2f", current_x_, current_y_, current_z_);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Store the IMU timestamp
    latest_timestamp_ = msg->header.stamp;

    tf2::Quaternion imu_quat;

    // Convert the IMU orientation to tf2::Quaternion using tf2::convert
    tf2::fromMsg(msg->orientation, imu_quat);

    tf2::Quaternion result_quat;

    if (use_yaw_only_) {
      // Extract only yaw from IMU orientation, discard roll and pitch
      double roll, pitch, yaw;
      tf2::Matrix3x3(imu_quat).getRPY(roll, pitch, yaw);

      // Create a quaternion with only yaw component (zero roll and pitch)
      result_quat.setRPY(0, 0, yaw + yaw_offset_);
    } else {
      // Apply yaw offset to the IMU orientation as before
      tf2::Quaternion yaw_quat;

      // Create a quaternion representing only the yaw offset
      yaw_quat.setRPY(0, 0, yaw_offset_);

      // Multiply the quaternions: result = imu_orientation * yaw_offset
      result_quat = imu_quat * yaw_quat;
    }

    // Normalize the resulting quaternion to ensure it's a unit quaternion
    result_quat.normalize();

    // Convert back to geometry_msgs::Quaternion
    current_orientation_ = tf2::toMsg(result_quat);

    // For future improvement: use IMU angular rates to predict orientation changes
    // between GPS updates
    imu_received_ = true;

    RCLCPP_DEBUG(this->get_logger(), "IMU updated: orientation updated with yaw offset %.4f", yaw_offset_);
  }

  void publishOdomAndTf()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!gps_received_ && !imu_received_) {
      return;  // Need at least one type of data to publish
    }

    // Create odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    // Use IMU's latest timestamp if available, otherwise use current time
    if (imu_received_) {
      odom_msg.header.stamp = latest_timestamp_;
    } else {
      odom_msg.header.stamp = this->now();
    }
    odom_msg.header.frame_id = "world";  // Fixed world frame
    odom_msg.child_frame_id = "base_link";  // Robot frame

    // Position
    odom_msg.pose.pose.position.x = current_x_;  // E (East)
    odom_msg.pose.pose.position.y = current_y_;  // N (North)
    odom_msg.pose.pose.position.z = current_z_;  // U (Up)

    // TODO: 目前是用了IMU的orientation，这个后面主要是给导航提供yaw角。
    // 但IMU如果不带磁力计这个yaw就是非全局的，不能用。
    // 后续这个要用外部输入的 1)罗盘数据 或 2)全局坐标系odom(如果有的话)的orientation。
    // Orientation - use IMU if available, otherwise keep last value
    if (imu_received_) {
      odom_msg.pose.pose.orientation = current_orientation_;
    } else {
      odom_msg.pose.pose.orientation = current_orientation_;
    }

    // For now, zero velocity - in a real system, this would be estimated
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Publish odometry
    odom_pub_->publish(odom_msg);

    // Create and broadcast TF transform
    geometry_msgs::msg::TransformStamped t;
    // Use IMU's latest timestamp if available, otherwise use current time
    if (imu_received_) {
      t.header.stamp = latest_timestamp_;
    } else {
      t.header.stamp = this->now();
    }
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";

    t.transform.translation.x = current_x_;
    t.transform.translation.y = current_y_;
    t.transform.translation.z = current_z_;

    t.transform.rotation = current_orientation_;

    tf_broadcaster_->sendTransform(t);

    RCLCPP_DEBUG(this->get_logger(), "Published odom and TF: x=%.2f, y=%.2f, z=%.2f", current_x_, current_y_, current_z_);
  }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr odom_timer_;

  // Data storage
  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  geometry_msgs::msg::Quaternion current_orientation_;  // Default: no rotation (will be initialized in constructor)
  builtin_interfaces::msg::Time latest_timestamp_{};
  bool gps_received_{false};
  bool imu_received_{false};

  // Origin coordinates for local cartesian conversion
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};
  double yaw_offset_{0.0};  // Yaw offset in radians
  bool use_yaw_only_{false};  // Whether to extract only yaw from IMU

  // GeographicLib converter
  std::unique_ptr<GeographicLib::LocalCartesian> geo_converter_;

  // Mutex for thread safety
  std::mutex data_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsImuFusionNode>());
  rclcpp::shutdown();
  return 0;
}