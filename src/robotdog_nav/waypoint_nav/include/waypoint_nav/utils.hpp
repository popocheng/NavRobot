#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <vector>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace waypoint_nav {

// Mathematical utilities
double distance2D(double x1, double y1, double x2, double y2);
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat);
geometry_msgs::msg::Quaternion getQuaternionFromYaw(double yaw);

// Navigation utilities
geometry_msgs::msg::Twist computePurePursuitCommand(
  double robot_x, double robot_y, double robot_yaw,
  double target_x, double target_y,
  double linear_vel, double max_angular_vel,
  double lookahead_distance);

// Angle utilities
double normalizeAngle(double angle);
double angleDiff(double from, double to);

}  // namespace waypoint_nav

#endif  // UTILS_HPP_