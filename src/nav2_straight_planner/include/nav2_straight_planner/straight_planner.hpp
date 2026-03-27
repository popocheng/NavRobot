// Copyright 2026
#ifndef NAV2_STRAIGHT_PLANNER__STRAIGHT_PLANNER_HPP_
#define NAV2_STRAIGHT_PLANNER__STRAIGHT_PLANNER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_straight_planner
{

class StraightPlanner : public nav2_util::LifecycleNode
{
public:
  explicit StraightPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StraightPlanner() override = default;

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onPlanTick();
  nav_msgs::msg::Path buildStraightPath(
    const geometry_msgs::msg::PoseStamped & start_map,
    const geometry_msgs::msg::PoseStamped & goal_map) const;

  bool isActive();

  std::string global_frame_;
  std::string robot_frame_;
  double path_step_m_;
  double goal_xy_tolerance_;
  double planning_period_s_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string action_name_;

  rclcpp::CallbackGroup::SharedPtr reentrant_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex state_mutex_;
  bool goal_valid_{false};
  bool wait_fp_finish_after_reached_{false};
  geometry_msgs::msg::PoseStamped latest_goal_;
  GoalHandleFollowPath::SharedPtr active_goal_handle_;
};

}  // namespace nav2_straight_planner

#endif  // NAV2_STRAIGHT_PLANNER__STRAIGHT_PLANNER_HPP_
