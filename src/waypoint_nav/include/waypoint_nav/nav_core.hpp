#ifndef NAV_CORE_HPP_
#define NAV_CORE_HPP_

#include <vector>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "msg_set_msgs/msg/multi_goal.hpp"
#include "msg_set_msgs/msg/multi_goal_point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace waypoint_nav {

enum class NavigationState {
  IDLE,
  WAITING,
  EXECUTING,
  FAILED,
  COMPLETED
};

struct RobotPose {
  double x, y, z;
  double roll, pitch, yaw;
};

class WaypointNavigator : public rclcpp::Node
{
public:
  explicit WaypointNavigator(const rclcpp::NodeOptions & options);

  // Core navigation methods
  void setCurrentGoal(int index);
  bool isGoalReached(double tolerance = 1.0) const;
  geometry_msgs::msg::Twist computeVelocityCommand();
  void initializeOrientationEstimate();

  // TODO: Implement method to interface with global2local package
  // As mentioned in the requirements: "analyze '/home/lenovo/Projects/robotdog_nav/src/global2local'
  // to see if it meets the requirements, use existing GPS topic and estimated azimuth as input,
  // to convert and publish global_odom topic and /tf transforms"

  // Accessors for state
  NavigationState getState() const { return nav_state_; }
  void setState(NavigationState state) { nav_state_ = state; }

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void goalCallback(const msg_set_msgs::msg::MultiGoal::SharedPtr msg);

  // Helper methods
  void updateRobotPose();
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) const;
  void updateOrientationEstimate();
  RobotPose getCurrentPose() const { return current_pose_; }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<msg_set_msgs::msg::MultiGoal>::SharedPtr goal_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // State variables
  NavigationState nav_state_;
  std::vector<msg_set_msgs::msg::MultiGoalPoint> waypoints_;
  size_t current_goal_index_;
  RobotPose current_pose_;
  double estimated_yaw_;
  bool initial_yaw_estimated_;
  bool has_new_gps_data_;

  // Coordinate transformation parameters
  double origin_lat_, origin_lon_, origin_alt_;
  double local_offset_x_, local_offset_y_;

  // Allow access to these members from friend classes
  friend class NavWaypointNode;

  // Parameters
  double goal_tolerance_;
  double yaw_tolerance_;
  double linear_velocity_;
  double angular_velocity_limit_;
  double control_frequency_;
  double lookahead_distance_;

  mutable std::mutex mutex_;
};

}  // namespace waypoint_nav

#endif  // NAV_CORE_HPP_