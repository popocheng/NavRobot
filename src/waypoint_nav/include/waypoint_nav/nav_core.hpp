# ifndef NAV_CORE_HPP_
# define NAV_CORE_HPP_

# include <vector>
# include <memory>
# include <mutex>

# include "rclcpp/rclcpp.hpp"
# include "std_msgs/msg/string.hpp"
# include "sensor_msgs/msg/nav_sat_fix.hpp"
# include "sensor_msgs/msg/imu.hpp"
# include "sensor_msgs/msg/point_cloud2.hpp"
# include "geometry_msgs/msg/twist.hpp"
# include "geometry_msgs/msg/pose_stamped.hpp"
# include "geometry_msgs/msg/quaternion.hpp"
# include "nav_msgs/msg/odometry.hpp"
# include "msg_set_msgs/msg/multi_goal.hpp"
# include "msg_set_msgs/msg/multi_goal_point.hpp"
# include "tf2/LinearMath/Quaternion.h"
# include "tf2/utils.h"
# include "GeographicLib/LocalCartesian.hpp"
# include "visualization_msgs/msg/marker.hpp"
# include "visualization_msgs/msg/marker_array.hpp"
# include "nav_msgs/msg/path.hpp"

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

class WaypointNavigator
{
public:
  explicit WaypointNavigator(rclcpp::Node* node_ptr);

  // Core navigation methods
  void setCurrentGoal(int index);
  bool isGoalReached(double tolerance = 1.0) const;
  geometry_msgs::msg::Twist computeVelocityCommand();
  void initializeOrientationEstimate();
  void resetNavigator();  // Reset navigator after task completion

  // TODO: Implement method to interface with global2local package
  // As mentioned in the requirements: "analyze '/home/lenovo/Projects/robotdog_nav/src/global2local'
  // to see if it meets the requirements, use existing GPS topic and estimated azimuth as input,
  // to convert and publish global_odom topic and /tf transforms"

  // Accessors for state
  NavigationState getState() const { return nav_state_; }
  void setState(NavigationState state) { nav_state_ = state; }

  // Visualization methods
  void publishVisualizations();
  void publishWaypointMarkers();
  void publishPathMarkers();
  void publishActualPath();
  void clearActualPath();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void goalCallback(const msg_set_msgs::msg::MultiGoal::SharedPtr msg);

  // Helper methods
  void updateRobotPose();
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) const;
  void updateOrientationEstimate();
  RobotPose getCurrentPose() const { return current_pose_; }

  // Members
  rclcpp::Node* node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<msg_set_msgs::msg::MultiGoal>::SharedPtr goal_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // Visualization publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;

  // State variables
  NavigationState nav_state_;
  std::vector<msg_set_msgs::msg::MultiGoalPoint> waypoints_;
  size_t current_goal_index_;
  RobotPose current_pose_;
  double estimated_yaw_;
  bool initial_yaw_estimated_;
  bool has_new_odom_data_;

  // Visualization related
  std::vector<geometry_msgs::msg::PoseStamped> actual_path_;
  mutable std::mutex path_mutex_;

  // Allow access to these members from friend classes
  friend class NavWaypointNode;

  // Parameters
  double goal_tolerance_;
  double yaw_tolerance_;
  double linear_velocity_;
  double angular_velocity_limit_;
  double control_frequency_;
  double lookahead_distance_;

  // Geographic conversion
  std::unique_ptr<GeographicLib::LocalCartesian> geo_converter_;
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};

  mutable std::mutex mutex_;
};

}  // namespace waypoint_nav

# endif  // NAV_CORE_HPP_