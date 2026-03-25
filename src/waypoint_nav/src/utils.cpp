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

  // Calculate the angle between robot's heading and target
  double target_angle = std::atan2(dy, dx);
  double angle_diff = normalizeAngle(target_angle - robot_yaw);

  // Use the pure pursuit formula to calculate angular velocity
  // Radius of curvature for the path to follow
  double radius = lookahead_distance / std::sin(angle_diff);

  // Avoid division by zero when sin(angle_diff) is close to zero
  if (std::abs(std::sin(angle_diff)) < 1e-6) {
    cmd_vel.linear.x = linear_vel;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = (angle_diff > 0) ? max_angular_vel : -max_angular_vel;
    return cmd_vel;
  }

  // Calculate angular velocity based on linear velocity and turning radius
  double angular_vel = 2.0 * linear_vel * std::sin(angle_diff) / lookahead_distance;

  // Limit the angular velocity
  if (angular_vel > max_angular_vel) {
    angular_vel = max_angular_vel;
  } else if (angular_vel < -max_angular_vel) {
    angular_vel = -max_angular_vel;
  }

  // Set velocities
  cmd_vel.linear.x = linear_vel;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = angular_vel;

  // TODO: Enhance the pure pursuit algorithm with dynamic lookahead distance
  // based on robot speed and trajectory curvature for better performance

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