#include "waypoint_nav/nav_core.hpp"
#include "waypoint_nav/utils.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace waypoint_nav {

WaypointNavigator::WaypointNavigator(const rclcpp::NodeOptions & options)
: Node("waypoint_navigator", options),
  nav_state_(NavigationState::IDLE),
  current_goal_index_(0),
  estimated_yaw_(0.0),
  initial_yaw_estimated_(false),
  has_new_odom_data_(false)
{
  // Declare parameters
  this->declare_parameter("goal_tolerance", 1.0);
  this->declare_parameter("yaw_tolerance", 0.2);
  this->declare_parameter("linear_velocity", 1.0);
  this->declare_parameter("angular_velocity_limit", 1.0);
  this->declare_parameter("control_frequency", 10.0);
  this->declare_parameter("lookahead_distance", 2.0);

  // Get parameters
  this->get_parameter("goal_tolerance", goal_tolerance_);
  this->get_parameter("yaw_tolerance", yaw_tolerance_);
  this->get_parameter("linear_velocity", linear_velocity_);
  this->get_parameter("angular_velocity_limit", angular_velocity_limit_);
  this->get_parameter("control_frequency", control_frequency_);
  this->get_parameter("lookahead_distance", lookahead_distance_);

  // Initialize subscribers - subscribe to world_odom instead of direct GPS and IMU
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/world_odom", 10,
    std::bind(&WaypointNavigator::odomCallback, this, std::placeholders::_1));

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", 10,
    std::bind(&WaypointNavigator::lidarCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<msg_set_msgs::msg::MultiGoal>(
    "/waypoint_goals", 10,
    std::bind(&WaypointNavigator::goalCallback, this, std::placeholders::_1));

  // Initialize publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/nav_status", 10);

  RCLCPP_INFO(this->get_logger(), "Waypoint Navigator initialized");
}

void WaypointNavigator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Extract position from the world_odom message
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  current_pose_.z = msg->pose.pose.position.z;

  // Extract orientation and convert to yaw
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );

  current_pose_.roll = tf2::getYaw(q);

  // Use the orientation from world_odom as the estimated yaw
  if (!initial_yaw_estimated_) {
    estimated_yaw_ = current_pose_.roll;
    initial_yaw_estimated_ = true;
    RCLCPP_INFO(this->get_logger(), "Initial orientation estimated: %.3f rad", estimated_yaw_);
  } else {
    estimated_yaw_ = current_pose_.roll;  // Update with latest orientation
  }

  has_new_odom_data_ = true;

  RCLCPP_DEBUG(this->get_logger(), "Updated position from world_odom: (%.2f, %.2f, %.2f), yaw: %.3f",
              current_pose_.x, current_pose_.y, current_pose_.z, estimated_yaw_);
}

void WaypointNavigator::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // TODO: Process LiDAR data for obstacle detection and avoidance
  // Integrate with nav2's costmap or implement local obstacle detection
  // Based on the user requirement to use nav2's local costmap with spatio_temporal_voxel_layer

  RCLCPP_DEBUG(this->get_logger(), "Received lidar data with %d points", msg->width);
}

void WaypointNavigator::goalCallback(const msg_set_msgs::msg::MultiGoal::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  waypoints_.clear();
  for (const auto& goal_point : msg->multi_goal_points) {
    msg_set_msgs::msg::MultiGoalPoint wp = goal_point;
    waypoints_.push_back(wp);
  }

  current_goal_index_ = 0;
  nav_state_ = NavigationState::WAITING;

  RCLCPP_INFO(this->get_logger(), "Received %zu waypoints", waypoints_.size());
}

void WaypointNavigator::setCurrentGoal(int index)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= 0 && index < static_cast<int>(waypoints_.size())) {
    current_goal_index_ = static_cast<size_t>(index);
  }
}

bool WaypointNavigator::isGoalReached(double tolerance) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_goal_index_ >= waypoints_.size()) {
    return true;
  }

  const auto& goal = waypoints_[current_goal_index_];
  double dist = distance2D(current_pose_.x, current_pose_.y, goal.x_or_lat, goal.y_or_lon);

  return dist <= tolerance;
}

geometry_msgs::msg::Twist WaypointNavigator::computeVelocityCommand()
{
  std::lock_guard<std::mutex> lock(mutex_);
  geometry_msgs::msg::Twist cmd_vel;

  if (waypoints_.empty() || current_goal_index_ >= waypoints_.size()) {
    // Stop if no waypoints or all waypoints reached
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  const auto& current_goal = waypoints_[current_goal_index_];

  // Use pure pursuit algorithm for path following
  cmd_vel = computePurePursuitCommand(
    current_pose_.x, current_pose_.y, estimated_yaw_,
    current_goal.x_or_lat, current_goal.y_or_lon,
    linear_velocity_, angular_velocity_limit_,
    lookahead_distance_);

  return cmd_vel;
}

void WaypointNavigator::initializeOrientationEstimate()
{
  // Since we're using world_odom which already has fused orientation information,
  // the orientation is ready for use. We just need to ensure that we have valid odometry data.

  if (has_new_odom_data_) {
    RCLCPP_INFO(this->get_logger(), "Orientation is ready from world_odom: %.3f rad", estimated_yaw_);
  } else {
    RCLCPP_WARN(this->get_logger(), "No odometry data received yet for orientation initialization");
  }
}

void WaypointNavigator::updateOrientationEstimate()
{
  // Since we're using world_odom which has fused orientation,
  // this function may not be needed anymore
  // But we'll keep it in case we need to implement further fusion
}

}  // namespace waypoint_nav