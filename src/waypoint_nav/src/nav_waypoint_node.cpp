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
    navigator_(this),
    fsm_()
  {
    // Create timer for control loop
    control_timer_ = this->create_wall_timer(
      100ms, std::bind(&NavWaypointNode::controlLoop, this));

    // Initialize state publisher
    state_publisher_ = this->create_publisher<std_msgs::msg::String>("/nav_status", 10);

    // Transition to waiting for goals state immediately after initialization
    fsm_.transitToWaitingForGoals();

    RCLCPP_INFO(this->get_logger(), "Nav Waypoint Node initialized");
  }

private:
  void controlLoop()
  {
    // Update navigation state based on FSM
    // auto nav_state = navigator_.getState();  // Commented out since not used

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

      // Check if we should publish cmd_vel based on controller selection
      if (navigator_.use_controller_server_) {
        // When using controller server, we don't publish cmd_vel ourselves
        // The external controller_server handles publishing
      } else {
        // When not using controller server, publish cmd_vel locally
        if (!cmd_vel_publisher_) {
          cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }
        cmd_vel_publisher_->publish(cmd_vel);
      }
    }

    // Publish visualization markers
    navigator_.publishVisualizations();
  }

  void handleIdleState()
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000 /*ms*/, "Handling IDLE state");

    // Wait for waypoints to be received
    if (!navigator_.waypoints_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Waypoints received in IDLE, transitioning to INITIALIZING");
      // Inline implementation of onGoalsReceived: if current state is WAITING_FOR_GOALS, transit to INITIALIZING
      if (fsm_.getCurrentState() == NavFSMState::WAITING_FOR_GOALS) {
        fsm_.transitToInitializing();
      }
    }
  }

  void handleWaitingForGoalsState()
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000 /*ms*/, "Handling WAITING_FOR_GOALS state");

    // Wait for goals to arrive
    if (!navigator_.waypoints_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Waypoints received in WAITING_FOR_GOALS, transitioning to INITIALIZING");
      // Inline implementation of onGoalsReceived: if current state is WAITING_FOR_GOALS, transit to INITIALIZING
      if (fsm_.getCurrentState() == NavFSMState::WAITING_FOR_GOALS) {
        fsm_.transitToInitializing();
      }
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
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000 /*ms*/, "Handling EXECUTING_PATH state");

    // Check if current goal is reached
    if (navigator_.isGoalReached(navigator_.goal_tolerance_)) {
      RCLCPP_INFO(this->get_logger(), "Reached goal %zu", navigator_.current_goal_index_);

      // Move to next goal if available
      navigator_.current_goal_index_++;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000 /*ms*/, "Moving to next goal, current index: %zu (of %zu total)",
                           navigator_.current_goal_index_, navigator_.waypoints_.size());

      if (navigator_.current_goal_index_ >= navigator_.waypoints_.size()) {
        // All goals completed
        RCLCPP_INFO(this->get_logger(), "All goals completed, transitioning to COMPLETED");
        // Inline implementation of onCompletion: transit to COMPLETED state
        fsm_.transitToCompleted();
      } else {
        // Continue to next goal
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000 /*ms*/, "Continuing to next goal");
        // Inline implementation of onGoalReached: transit to GOAL_REACHED state
        fsm_.transitToGoalReached();

        // Brief pause before moving to next goal, then continue executing
        fsm_.transitToExecutingPath();
      }
    }

    // Obstacle detection and avoidance is now handled in the computeVelocityCommand method
    // which is called in the controlLoop function. This keeps obstacle avoidance as part
    // of the normal navigation rather than as a separate state, as required by the specifications.
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

    // Reset the navigator to clear history and waypoints
    navigator_.resetNavigator();

    fsm_.transitToWaitingForGoals();
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