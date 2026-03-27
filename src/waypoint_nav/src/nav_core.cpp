#include "waypoint_nav/nav_core.hpp"
#include "waypoint_nav/utils.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/create_client.hpp"

namespace waypoint_nav {

WaypointNavigator::WaypointNavigator(rclcpp::Node* node_ptr)
: node_(node_ptr),
  nav_state_(NavigationState::IDLE),
  current_goal_index_(0),
  estimated_yaw_(0.0),
  initial_yaw_estimated_(false),
  has_new_odom_data_(false),
  path_sent_to_controller_(false),
  last_goal_index_sent_(0),
  use_controller_server_(true)
{
  // Declare parameters
  node_->declare_parameter("goal_tolerance", 1.0);
  node_->declare_parameter("yaw_tolerance", 0.2);
  node_->declare_parameter("linear_velocity", 1.0);
  node_->declare_parameter("angular_velocity_limit", 1.0);
  node_->declare_parameter("control_frequency", 10.0);
  node_->declare_parameter("lookahead_distance", 2.0);
  node_->declare_parameter("origin_lat", 0.0);  // Origin latitude
  node_->declare_parameter("origin_lon", 0.0);  // Origin longitude
  node_->declare_parameter("origin_alt", 0.0);  // Origin altitude
  node_->declare_parameter("use_controller_server", true);  // Use controller server for path following

  // Navigation2 parameters
  node_->declare_parameter("robot_base_frame", "base_link");
  node_->declare_parameter("global_frame", "world");

  // Get parameters
  node_->get_parameter("goal_tolerance", goal_tolerance_);
  node_->get_parameter("yaw_tolerance", yaw_tolerance_);
  node_->get_parameter("linear_velocity", linear_velocity_);
  node_->get_parameter("angular_velocity_limit", angular_velocity_limit_);
  node_->get_parameter("control_frequency", control_frequency_);
  node_->get_parameter("lookahead_distance", lookahead_distance_);
  node_->get_parameter("origin_lat", origin_lat_);
  node_->get_parameter("origin_lon", origin_lon_);
  node_->get_parameter("origin_alt", origin_alt_);
  node_->get_parameter("use_controller_server", use_controller_server_);

  RCLCPP_INFO(node_->get_logger(), "Waypoint Navigator initialized with parameters:");
  RCLCPP_INFO(node_->get_logger(), "goal_tolerance: %.2f", goal_tolerance_);
  RCLCPP_INFO(node_->get_logger(), "yaw_tolerance: %.2f", yaw_tolerance_);
  RCLCPP_INFO(node_->get_logger(), "linear_velocity: %.2f", linear_velocity_);
  RCLCPP_INFO(node_->get_logger(), "angular_velocity_limit: %.2f", angular_velocity_limit_);
  RCLCPP_INFO(node_->get_logger(), "control_frequency: %.2f", control_frequency_);
  RCLCPP_INFO(node_->get_logger(), "lookahead_distance: %.2f", lookahead_distance_);
  RCLCPP_INFO(node_->get_logger(), "use_controller_server: %s", use_controller_server_ ? "true" : "false");

  // Initialize subscribers - subscribe to world_odom instead of direct GPS and IMU
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/world_odom", 10,
    std::bind(&WaypointNavigator::odomCallback, this, std::placeholders::_1));

  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", 10,
    std::bind(&WaypointNavigator::lidarCallback, this, std::placeholders::_1));

  goal_sub_ = node_->create_subscription<msg_set_msgs::msg::MultiGoal>(
    "/waypoint_goals", 10,
    std::bind(&WaypointNavigator::goalCallback, this, std::placeholders::_1));

  // Initialize publishers
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  status_pub_ = node_->create_publisher<std_msgs::msg::String>("/nav_status", 10);

  // Initialize visualization publishers
  waypoint_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoint_markers", 10);
  path_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", 10);
  actual_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/actual_path", 10);

  // Initialize the geographic converter with origin coordinates
  geo_converter_ = std::make_unique<GeographicLib::LocalCartesian>(origin_lat_, origin_lon_, origin_alt_);

  // Initialize Navigation2 components
  // Removed costmap initialization as per requirements

  // Initialize action clients for Navigation2 services
  follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(node_, "follow_path");

  RCLCPP_INFO(node_->get_logger(), "Waypoint Navigator initialized with origin: lat=%.6f, lon=%.6f, alt=%.2f",
              origin_lat_, origin_lon_, origin_alt_);
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

  // 这里计算 estimated_yaw_ 是为了 
  // 1.computePurePursuitCommand（非避障模式），2.用于ActualPath近似计算（其实没必要）
  // 3.后续用于和MultiGoal的对齐，目前MultiGoal没有传yaw，但以后如果有要求，则方便对齐yaw
  // Use the orientation from world_odom as the estimated yaw
  if (!initial_yaw_estimated_) {
    estimated_yaw_ = current_pose_.roll;
    initial_yaw_estimated_ = true;
    RCLCPP_INFO(node_->get_logger(), "Initial orientation estimated: %.3f rad", estimated_yaw_);
  } else {
    estimated_yaw_ = current_pose_.roll;  // Update with latest orientation
  }

  has_new_odom_data_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Updated position from world_odom: (%.2f, %.2f, %.2f), yaw: %.3f",
              current_pose_.x, current_pose_.y, current_pose_.z, estimated_yaw_);
}

void WaypointNavigator::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Processing LiDAR data for potential future use
  RCLCPP_DEBUG(node_->get_logger(), "Received lidar data with %d points", msg->width);
}

void WaypointNavigator::goalCallback(const msg_set_msgs::msg::MultiGoal::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  waypoints_.clear();
  for (const auto& goal_point : msg->multi_goal_points) {
    msg_set_msgs::msg::MultiGoalPoint wp = goal_point;

    // Check if the coordinates are GPS coordinates or local coordinates
    if (msg->is_gps_aid) {
      // Convert GPS coordinates (lat/lon) to local Cartesian coordinates (x/y)
      // x corresponds to East, y corresponds to North
      double local_x, local_y, local_z;
      geo_converter_->Forward(wp.x_or_lat, wp.y_or_lon, wp.z_or_alt, local_x, local_y, local_z);

      // Update the waypoint with converted coordinates
      wp.x_or_lat = local_x;
      wp.y_or_lon = local_y;
      wp.z_or_alt = local_z;
    }
    // If is_gps_aid is false, the coordinates are already in local frame, no conversion needed
    RCLCPP_INFO(node_->get_logger(), "Received goal in ENU frame: (%.2f, %.2f, %.2f)",
                wp.x_or_lat, wp.y_or_lon, wp.z_or_alt);

    waypoints_.push_back(wp);
  }

  current_goal_index_ = 0;
  nav_state_ = NavigationState::WAITING;

  RCLCPP_INFO(node_->get_logger(), "Received %zu waypoints (GPS aid: %s)",
              waypoints_.size(), msg->is_gps_aid ? "true" : "false");
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

  if (use_controller_server_) {
    // Use controller server with follow_path_client_

    // Check if we need to send a new path to the controller server
    if (!path_sent_to_controller_ || current_goal_index_ != last_goal_index_sent_) {
      if (sendPathToControllerServer()) {
        path_sent_to_controller_ = true;
        last_goal_index_sent_ = current_goal_index_;
        RCLCPP_INFO(node_->get_logger(), "Sent new path to controller server for goal %zu", current_goal_index_);
      }
    }
  } else {
    // Use pure pursuit algorithm directly
    cmd_vel = computePurePursuitCommand(
      current_pose_.x, current_pose_.y, estimated_yaw_,
      current_goal.x_or_lat, current_goal.y_or_lon,
      linear_velocity_, angular_velocity_limit_,
      lookahead_distance_);
  }

  return cmd_vel;
}

void WaypointNavigator::initializeOrientationEstimate()
{
  // Since we're using world_odom which already has fused orientation information,
  // the orientation is ready for use. We just need to ensure that we have valid odometry data.

  if (has_new_odom_data_) {
    RCLCPP_INFO(node_->get_logger(), "Orientation is ready from world_odom: %.3f rad", estimated_yaw_);
  } else {
    RCLCPP_WARN(node_->get_logger(), "No odometry data received yet for orientation initialization");
  }
}

void WaypointNavigator::updateOrientationEstimate()
{
  // Since we're using world_odom which has fused orientation,
  // this function may not be needed anymore
  // But we'll keep it in case we need to implement further fusion
}

void WaypointNavigator::publishVisualizations()
{
  publishWaypointMarkers();
  publishPathMarkers();
  publishActualPath();
}

void WaypointNavigator::publishWaypointMarkers()
{
  if (waypoint_marker_pub_ == nullptr) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  // Clear previous markers by publishing delete markers if needed
  if (waypoints_.empty()) {
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "world";
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = "waypoints";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    waypoint_marker_pub_->publish(marker_array);
    return;
  }

  // Create a marker for each waypoint
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const auto& wp = waypoints_[i];

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = node_->now();
    marker.ns = "waypoints";
    marker.id = static_cast<int>(i);

    if (i == current_goal_index_) {
      // Current goal is green
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.scale.x = 0.8;
      marker.scale.y = 0.8;
      marker.scale.z = 0.8;
    } else {
      // Other waypoints are blue
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 0.8;
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
    }

    marker.pose.position.x = wp.x_or_lat;
    marker.pose.position.y = wp.y_or_lon;
    marker.pose.position.z = wp.z_or_alt;
    marker.pose.orientation.w = 1.0;

    marker.lifetime.sec = 0;  // Permanent
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker_array.markers.push_back(marker);
  }

  // Add text markers for waypoint indices
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const auto& wp = waypoints_[i];

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = node_->now();
    text_marker.ns = "waypoint_labels";
    text_marker.id = static_cast<int>(i);
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = wp.x_or_lat;
    text_marker.pose.position.y = wp.y_or_lon;
    text_marker.pose.position.z = wp.z_or_alt + 1.0;  // Position text slightly above waypoint
    text_marker.pose.orientation.w = 1.0;
    text_marker.text = "WP " + std::to_string(i);
    text_marker.scale.z = 0.8;  // Text size
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    text_marker.lifetime.sec = 0;  // Permanent
    marker_array.markers.push_back(text_marker);
  }

  waypoint_marker_pub_->publish(marker_array);
}

void WaypointNavigator::publishPathMarkers()
{
  if (path_marker_pub_ == nullptr || waypoints_.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  // Create a line strip connecting all waypoints
  visualization_msgs::msg::Marker path_line;
  path_line.header.frame_id = "world";
  path_line.header.stamp = node_->now();
  path_line.ns = "path_lines";
  path_line.id = 0;
  path_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_line.action = visualization_msgs::msg::Marker::ADD;
  path_line.pose.orientation.w = 1.0;
  path_line.scale.x = 0.1;  // Line width

  // Set color based on navigation state
  if (nav_state_ == NavigationState::EXECUTING) {
    path_line.color.r = 1.0;  // Red when executing
    path_line.color.g = 0.5;
    path_line.color.b = 0.0;
    path_line.color.a = 0.8;
  } else {
    path_line.color.r = 0.5;  // Gray when not executing
    path_line.color.g = 0.5;
    path_line.color.b = 0.5;
    path_line.color.a = 0.6;
  }

  // Add points to the line strip
  for (const auto& wp : waypoints_) {
    geometry_msgs::msg::Point pt;
    pt.x = wp.x_or_lat;
    pt.y = wp.y_or_lon;
    pt.z = wp.z_or_alt;
    path_line.points.push_back(pt);
  }

  path_line.lifetime.sec = 0;  // Permanent
  marker_array.markers.push_back(path_line);

  // Create arrow markers pointing from each waypoint to the next
  for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
    const auto& current_wp = waypoints_[i];
    const auto& next_wp = waypoints_[i + 1];

    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = "world";
    arrow_marker.header.stamp = node_->now();
    arrow_marker.ns = "path_arrows";
    arrow_marker.id = static_cast<int>(i);
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;

    // Arrow start and end points
    arrow_marker.points.resize(2);
    arrow_marker.points[0].x = current_wp.x_or_lat;
    arrow_marker.points[0].y = current_wp.y_or_lon;
    arrow_marker.points[0].z = current_wp.z_or_alt;
    arrow_marker.points[1].x = next_wp.x_or_lat;
    arrow_marker.points[1].y = next_wp.y_or_lon;
    arrow_marker.points[1].z = next_wp.z_or_alt;

    // Arrow appearance
    arrow_marker.scale.x = 0.2;  // Shaft diameter
    arrow_marker.scale.y = 0.3;  // Head diameter
    arrow_marker.scale.z = 0.0;  // Head length

    // Color based on index
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 1.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 0.7;

    arrow_marker.lifetime.sec = 0;  // Permanent
    marker_array.markers.push_back(arrow_marker);
  }

  path_marker_pub_->publish(marker_array);
}

void WaypointNavigator::publishActualPath()
{
  if (actual_path_pub_ == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(path_mutex_);

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "world";
  path_msg.header.stamp = node_->now();

  // Add current pose to actual path if it's significantly different from the last recorded pose
  {
    std::lock_guard<std::mutex> nav_lock(mutex_);
    if (!actual_path_.empty()) {
      const auto& last_pose = actual_path_.back().pose.position;
      double dist = std::sqrt(std::pow(current_pose_.x - last_pose.x, 2) +
                              std::pow(current_pose_.y - last_pose.y, 2));

      // Only add new point if it's sufficiently far from the last one (to avoid too many points)
      if (dist > 0.1) {  // Minimum distance threshold
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.header.stamp = node_->now();
        pose_stamped.pose.position.x = current_pose_.x;
        pose_stamped.pose.position.y = current_pose_.y;
        pose_stamped.pose.position.z = current_pose_.z;

        // Set orientation based on estimated_yaw_
        tf2::Quaternion q;
        q.setRPY(0, 0, estimated_yaw_);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        actual_path_.push_back(pose_stamped);
      }
    } else {
      // First pose
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "world";
      pose_stamped.header.stamp = node_->now();
      pose_stamped.pose.position.x = current_pose_.x;
      pose_stamped.pose.position.y = current_pose_.y;
      pose_stamped.pose.position.z = current_pose_.z;

      // Set orientation based on estimated_yaw_
      tf2::Quaternion q;
      q.setRPY(0, 0, estimated_yaw_);
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();

      actual_path_.push_back(pose_stamped);
    }
  }

  path_msg.poses = actual_path_;
  actual_path_pub_->publish(path_msg);
}

void WaypointNavigator::resetNavigator()
{
  std::lock_guard<std::mutex> lock(mutex_);
  waypoints_.clear();
  current_goal_index_ = 0;
  nav_state_ = NavigationState::IDLE;
  clearActualPath();  // Clear the actual path history
  initial_yaw_estimated_ = false;  // Reset yaw estimation
  RCLCPP_INFO(node_->get_logger(), "Navigator reset: waypoints cleared, path history cleared, state reset to IDLE");
}

void WaypointNavigator::clearActualPath()
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  actual_path_.clear();
}

bool WaypointNavigator::sendPathToControllerServer()
{
  if (!follow_path_client_) {
    RCLCPP_ERROR(node_->get_logger(), "Follow path client is not available");
    return false;
  }

  // Check if action server is available
  if (!follow_path_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
    RCLCPP_WARN(node_->get_logger(), "Follow path action server not available");
    return false;
  }

  // Create a path from current position to the current goal
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "world";
  path_msg.header.stamp = node_->now();

  // Add current robot pose as the start of the path
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header = path_msg.header;
  start_pose.pose.position.x = current_pose_.x;
  start_pose.pose.position.y = current_pose_.y;
  start_pose.pose.position.z = current_pose_.z;

  // Set orientation to face towards the goal
  const auto& current_goal = waypoints_[current_goal_index_];
  double target_angle = std::atan2(current_goal.y_or_lon - current_pose_.y,
                                  current_goal.x_or_lat - current_pose_.x);

  tf2::Quaternion q;
  q.setRPY(0, 0, target_angle);
  start_pose.pose.orientation.x = q.x();
  start_pose.pose.orientation.y = q.y();
  start_pose.pose.orientation.z = q.z();
  start_pose.pose.orientation.w = q.w();

  path_msg.poses.push_back(start_pose);

  // Add the target goal as the end of the path
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = path_msg.header;
  goal_pose.pose.position.x = current_goal.x_or_lat;
  goal_pose.pose.position.y = current_goal.y_or_lon;
  goal_pose.pose.position.z = current_goal.z_or_alt;

  // Calculate orientation for the goal pose (looking at the next goal if it exists)
  double goal_orientation = target_angle;
  if (current_goal_index_ + 1 < waypoints_.size()) {
    const auto& next_goal = waypoints_[current_goal_index_ + 1];
    goal_orientation = std::atan2(next_goal.y_or_lon - current_goal.y_or_lon,
                                 next_goal.x_or_lat - current_goal.x_or_lat);
  }

  tf2::Quaternion goal_q;
  goal_q.setRPY(0, 0, goal_orientation);
  goal_pose.pose.orientation.x = goal_q.x();
  goal_pose.pose.orientation.y = goal_q.y();
  goal_pose.pose.orientation.z = goal_q.z();
  goal_pose.pose.orientation.w = goal_q.w();

  path_msg.poses.push_back(goal_pose);

  // Send the path to the controller server
  auto goal_msg = nav2_msgs::action::FollowPath::Goal();
  goal_msg.path = path_msg;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "FollowPath action succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(node_->get_logger(), "FollowPath action was aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(node_->get_logger(), "FollowPath action was canceled");
          break;
        default:
          RCLCPP_ERROR(node_->get_logger(), "FollowPath action failed with unknown result code");
          break;
      }
    };

  // Send the goal asynchronously
  follow_path_client_->async_send_goal(goal_msg, send_goal_options);

  return true;
}

}  // namespace waypoint_nav