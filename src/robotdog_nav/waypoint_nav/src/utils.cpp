#include "waypoint_nav/utils.hpp"

namespace waypoint_nav {

double distance2D(double x1, double y1, double x2, double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat)
{
  double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion getQuaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quat;
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = std::sin(yaw * 0.5);
  quat.w = std::cos(yaw * 0.5);
  return quat;
}

geometry_msgs::msg::Twist computePurePursuitCommand(
  double robot_x, double robot_y, double robot_yaw,
  double target_x, double target_y,
  double linear_vel, double max_angular_vel,
  double lookahead_distance)
{
  geometry_msgs::msg::Twist cmd_vel;

  // Calculate the vector from robot to target
  double dx = target_x - robot_x;
  double dy = target_y - robot_y;

  // Calculate the distance to the target
  double distance_to_target = std::sqrt(dx * dx + dy * dy);

  // If we're very close to the target, slow down
  double adjusted_linear_vel = linear_vel;
  if (distance_to_target < 0.5) {
    adjusted_linear_vel = std::min(linear_vel * distance_to_target / 0.5, linear_vel);
  }

  // Calculate the angle between robot's heading and target
  double target_angle = std::atan2(dy, dx);
  double angle_diff = normalizeAngle(target_angle - robot_yaw);

  // Simple proportional controller for angular velocity
  // Instead of using the radius-based formula which can be unstable when angle_diff is near 0 or π,
  // use a simpler approach based on the angle difference
  double angular_vel = 2.0 * angle_diff;  // Proportional gain of 2.0

  // Limit the angular velocity
  if (angular_vel > max_angular_vel) {
    angular_vel = max_angular_vel;
  } else if (angular_vel < -max_angular_vel) {
    angular_vel = -max_angular_vel;
  }

  // When the robot is very close to the goal or facing away from it, reduce linear velocity
  if (std::abs(angle_diff) > M_PI / 2.0) {  // More than 90 degrees off
    adjusted_linear_vel *= 0.5;  // Reduce linear velocity by half
  } else if (distance_to_target < lookahead_distance * 0.5) {  // Very close to goal
    adjusted_linear_vel *= 0.3;  // Further reduce as we approach
  }

  // Set velocities
  cmd_vel.linear.x = adjusted_linear_vel;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = angular_vel;

  return cmd_vel;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double angleDiff(double from, double to)
{
  double diff = to - from;
  while (diff > M_PI) {
    diff -= 2.0 * M_PI;
  }
  while (diff < -M_PI) {
    diff += 2.0 * M_PI;
  }
  return diff;
}

}  // namespace waypoint_nav