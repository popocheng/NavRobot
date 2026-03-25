#include "waypoint_nav/nav_core.hpp"
#include "waypoint_nav/nav_fsm.hpp"
#include "waypoint_nav/utils.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

namespace waypoint_nav {

class NavWaypointNode : public rclcpp::Node
{
public:
  explicit NavWaypointNode(const rclcpp::NodeOptions & options)
  : Node("nav_waypoint_node", options),
    navigator_(options),
    fsm_()
  {
    // Create timer for control loop
    control_timer_ = this->create_wall_timer(
      100ms, std::bind(&NavWaypointNode::controlLoop, this));

    // Initialize state publisher
    state_publisher_ = this->create_publisher<std_msgs::msg::String>("/nav_status", 10);

    // TODO: Implement service for querying navigation status
    // As mentioned in the requirements: "Status topic or service, you define yourself,
    // for querying the current task status, including waiting, executing, failed, completed, etc."
    // This could be implemented as a service server alongside or instead of the status topic

    RCLCPP_INFO(this->get_logger(), "Nav Waypoint Node initialized");
  }

private:
  void controlLoop()
  {
    // Update navigation state based on FSM
    auto nav_state = navigator_.getState();

    // Publish current state
    std_msgs::msg::String state_msg;
    state_msg.data = fsm_.getStateAsString();
    state_publisher_->publish(state_msg);

    // Handle state transitions based on navigator state
    switch (fsm_.getCurrentState()) {
      case NavFSMState::IDLE:
        handleIdleState();
        break;
      case NavFSMState::WAITING_FOR_GOALS:
        handleWaitingForGoalsState();
        break;
      case NavFSMState::INITIALIZING:
        handleInitializingState();
        break;
      case NavFSMState::EXECUTING_PATH:
        handleExecutingPathState();
        break;
      case NavFSMState::AVOIDING_OBSTACLE:
        handleAvoidingObstacleState();
        break;
      case NavFSMState::GOAL_REACHED:
        handleGoalReachedState();
        break;
      case NavFSMState::FAILED:
        handleFailedState();
        break;
      case NavFSMState::COMPLETED:
        handleCompletedState();
        break;
    }

    // Compute and publish velocity command
    if (fsm_.isNavigating()) {
      geometry_msgs::msg::Twist cmd_vel = navigator_.computeVelocityCommand();

      // In a real implementation, you'd check for obstacles here and possibly modify cmd_vel
      // For now, just publish the computed velocity
      if (!cmd_vel_publisher_) {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      }
      cmd_vel_publisher_->publish(cmd_vel);
    }
  }

  void handleIdleState()
  {
    // Wait for waypoints to be received
    if (!navigator_.waypoints_.empty()) {
      fsm_.onGoalsReceived();
    }
  }

  void handleWaitingForGoalsState()
  {
    // Wait for goals to arrive
    if (!navigator_.waypoints_.empty()) {
      navigator_.setState(NavigationState::EXECUTING);
      fsm_.onNavigationStart();  // This should transition to EXECUTING_PATH
    }
  }

  void handleInitializingState()
  {
    // Initialize orientation estimate
    navigator_.initializeOrientationEstimate();

    // After initialization, start executing the path
    fsm_.transitToExecutingPath();
  }

  void handleExecutingPathState()
  {
    // Check if current goal is reached
    if (navigator_.isGoalReached(navigator_.goal_tolerance_)) {
      RCLCPP_INFO(this->get_logger(), "Reached goal %zu", navigator_.current_goal_index_);

      // Move to next goal if available
      navigator_.current_goal_index_++;

      if (navigator_.current_goal_index_ >= navigator_.waypoints_.size()) {
        // All goals completed
        fsm_.onCompletion();
      } else {
        // Continue to next goal
        fsm_.onGoalReached();

        // Brief pause before moving to next goal, then continue executing
        fsm_.transitToExecutingPath();
      }
    }

    // TODO: Check for obstacles using LiDAR data and transition to AVOIDING_OBSTACLE if needed
    // As per the requirement: "/livox/lidar [sensor_msgs/msg/PointCloud2], used to implement basic obstacle avoidance after basic functions are realized"
  }

  void handleAvoidingObstacleState()
  {
    // TODO: Implement proper obstacle avoidance logic
    // As mentioned in the requirements: "Call the local costmap of nav2 + local planner library"
    // Use the local costmap with layers for point clouds and inflation layer
    // as suggested: "only use one layer of radar point cloud layer + one layer of expansion layer"

    // For now, assume obstacle is cleared after some time
    // In reality, you'd check lidar data to confirm obstacle is cleared

    // For demonstration, after some condition is met:
    fsm_.onObstacleCleared();
  }

  void handleGoalReachedState()
  {
    // Publish zero velocity to stop at goal
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.angular.z = 0.0;

    if (cmd_vel_publisher_) {
      cmd_vel_publisher_->publish(stop_cmd);
    }

    // Decide whether to go to next goal or stay in this state
    if (navigator_.current_goal_index_ < navigator_.waypoints_.size()) {
      fsm_.transitToExecutingPath();
    }
  }

  void handleFailedState()
  {
    // Stop robot
    if (cmd_vel_publisher_) {
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.linear.y = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_vel_publisher_->publish(stop_cmd);
    }

    // Stay in failed state until reset
  }

  void handleCompletedState()
  {
    // Stop robot
    if (cmd_vel_publisher_) {
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.linear.y = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_vel_publisher_->publish(stop_cmd);
    }

    RCLCPP_INFO(this->get_logger(), "Navigation completed successfully!");

    // Stay in completed state
  }

  WaypointNavigator navigator_;
  NavigationFSM fsm_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

}  // namespace waypoint_nav

// Main function for standalone executable
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<waypoint_nav::NavWaypointNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}