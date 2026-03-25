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
  has_new_gps_data_(false),
  origin_lat_(0.0),
  origin_lon_(0.0),
  origin_alt_(0.0),
  local_offset_x_(0.0),
  local_offset_y_(0.0)
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

  // Initialize subscribers
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/data", 10,
    std::bind(&WaypointNavigator::gpsCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10,
    std::bind(&WaypointNavigator::imuCallback, this, std::placeholders::_1));

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

void WaypointNavigator::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Initialize reference point if this is the first GPS message
  if (origin_lat_ == 0.0 && origin_lon_ == 0.0) {
    origin_lat_ = msg->latitude;
    origin_lon_ = msg->longitude;
    origin_alt_ = msg->altitude;

    local_offset_x_ = 0.0;
    local_offset_y_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Initialized local coordinate system at (%.6f, %.6f, %.2f)",
                origin_lat_, origin_lon_, origin_alt_);
  }

  // Convert GPS coordinates to local meters (simplified conversion)
  // Approximate conversion: 1 degree latitude ~ 111320 meters
  // 1 degree longitude ~ 111320 * cos(lat) meters
  double lat_scale = 111320.0; // meters per degree
  double lon_scale = 111320.0 * cos(msg->latitude * M_PI / 180.0); // meters per degree

  double local_x = (msg->latitude - origin_lat_) * lat_scale;
  double local_y = (msg->longitude - origin_lon_) * lon_scale;
  double local_z = msg->altitude - origin_alt_;

  current_pose_.x = local_x;
  current_pose_.y = local_y;
  current_pose_.z = local_z;

  has_new_gps_data_ = true;

  RCLCPP_DEBUG(this->get_logger(), "Updated GPS position: (%.2f, %.2f, %.2f)", local_x, local_y, local_z);
}

void WaypointNavigator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Extract orientation from IMU
  current_pose_.roll = tf2::getYaw(tf2::Quaternion(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w));

  // For now, we'll use IMU yaw directly but in practice you'd fuse it with GPS data
  if (!initial_yaw_estimated_) {
    estimated_yaw_ = current_pose_.roll;
    initial_yaw_estimated_ = true;
    RCLCPP_INFO(this->get_logger(), "Initial orientation estimated: %.3f rad", estimated_yaw_);
  } else {
    // Update orientation estimate (simple integration of gyro data would go here)
    updateOrientationEstimate();
  }

  RCLCPP_DEBUG(this->get_logger(), "Updated IMU orientation: %.3f rad", current_pose_.roll);
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
  // TODO: Implement proper initialization that moves the robot slightly
  // to estimate orientation from GPS changes as suggested in the requirements
  // The requirement states: "Consider moving forward to let the dog know its initial azimuth
  // through GPS positioning, then subsequently use IMU combined with GPS to estimate real-time azimuth"

  if (has_new_gps_data_) {
    // Store initial position for comparison after slight movement
    RCLCPP_INFO(this->get_logger(), "Ready to initialize orientation estimate after movement");
  }
}

void WaypointNavigator::updateOrientationEstimate()
{
  // TODO: Implement proper GPS/IMU fusion as mentioned in the requirements:
  // "After each startup, use GPS and IMU to estimate the azimuth of the dog"
  // Use a more sophisticated fusion algorithm (e.g., complementary filter or EKF)
  // instead of the simple approach below

  static double prev_x = current_pose_.x;
  static double prev_y = current_pose_.y;
  static rclcpp::Time prev_time = this->now();

  rclcpp::Time curr_time = this->now();
  double dt = (curr_time - prev_time).seconds();

  if (dt > 0.01) { // Only update if sufficient time has passed
    double dx = current_pose_.x - prev_x;
    double dy = current_pose_.y - prev_y;

    if (sqrt(dx*dx + dy*dy) > 0.01) { // Only if significant movement occurred
      double heading_from_movement = atan2(dy, dx);
      // Simple complementary filter: blend GPS-derived heading with IMU
      double alpha = 0.1; // Weight for GPS heading
      estimated_yaw_ = alpha * heading_from_movement + (1.0 - alpha) * current_pose_.roll;
    }

    prev_x = current_pose_.x;
    prev_y = current_pose_.y;
    prev_time = curr_time;
  }
}

}  // namespace waypoint_nav