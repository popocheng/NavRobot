#include "rclcpp/rclcpp.hpp"

#include "waypoint_nav/nav_fsm.hpp"

namespace waypoint_nav {

NavigationFSM::NavigationFSM() : current_state_(NavFSMState::IDLE)
{
}

void NavigationFSM::transitToIdle()
{
  if (current_state_ != NavFSMState::IDLE) {
    RCLCPP_INFO(rclcpp::get_logger("NavigationFSM"), "Transitioning to IDLE state");
    current_state_ = NavFSMState::IDLE;
  }
}

void NavigationFSM::transitToWaitingForGoals()
{
  if (current_state_ != NavFSMState::WAITING_FOR_GOALS) {
    RCLCPP_INFO(rclcpp::get_logger("NavigationFSM"), "Transitioning to WAITING_FOR_GOALS state");
    current_state_ = NavFSMState::WAITING_FOR_GOALS;
  }
}

void NavigationFSM::transitToInitializing()
{
  if (current_state_ != NavFSMState::INITIALIZING) {
    RCLCPP_INFO(rclcpp::get_logger("NavigationFSM"), "Transitioning to INITIALIZING state");
    current_state_ = NavFSMState::INITIALIZING;
  }
}

void NavigationFSM::transitToExecutingPath()
{
  if (current_state_ != NavFSMState::EXECUTING_PATH) {
    RCLCPP_INFO(rclcpp::get_logger("NavigationFSM"), "Transitioning to EXECUTING_PATH state");
    current_state_ = NavFSMState::EXECUTING_PATH;
  }
}

void NavigationFSM::transitToAvoidingObstacle()
{
  if (current_state_ != NavFSMState::AVOIDING_OBSTACLE) {
    RCLCPP_INFO(rclcpp::get_logger("NavigationFSM"), "Transitioning to AVOIDING_OBSTACLE state");
    current_state_ = NavFSMState::AVOIDING_OBSTACLE;
  }
}

void NavigationFSM::transitToGoalReached()
{
  if (current_state_ != NavFSMState::GOAL_REACHED) {
    RCLCPP_INFO(rclcpp::get_logger("NavigationFSM"), "Transitioning to GOAL_REACHED state");
    current_state_ = NavFSMState::GOAL_REACHED;
  }
}

void NavigationFSM::transitToFailed()
{
  if (current_state_ != NavFSMState::FAILED) {
    RCLCPP_WARN(rclcpp::get_logger("NavigationFSM"), "Transitioning to FAILED state");
    current_state_ = NavFSMState::FAILED;
  }
}

void NavigationFSM::transitToCompleted()
{
  if (current_state_ != NavFSMState::COMPLETED) {
    RCLCPP_INFO(rclcpp::get_logger("NavigationFSM"), "Transitioning to COMPLETED state");
    current_state_ = NavFSMState::COMPLETED;
  }
}

std::string NavigationFSM::getStateAsString() const
{
  switch (current_state_) {
    case NavFSMState::IDLE:
      return "IDLE";
    case NavFSMState::WAITING_FOR_GOALS:
      return "WAITING_FOR_GOALS";
    case NavFSMState::INITIALIZING:
      return "INITIALIZING";
    case NavFSMState::EXECUTING_PATH:
      return "EXECUTING_PATH";
    case NavFSMState::AVOIDING_OBSTACLE:
      return "AVOIDING_OBSTACLE";
    case NavFSMState::GOAL_REACHED:
      return "GOAL_REACHED";
    case NavFSMState::FAILED:
      return "FAILED";
    case NavFSMState::COMPLETED:
      return "COMPLETED";
    default:
      return "UNKNOWN";
  }
}

bool NavigationFSM::isNavigating() const
{
  return current_state_ == NavFSMState::INITIALIZING ||
         current_state_ == NavFSMState::EXECUTING_PATH ||
         current_state_ == NavFSMState::AVOIDING_OBSTACLE ||
         current_state_ == NavFSMState::GOAL_REACHED;
}

void NavigationFSM::onGoalsReceived()
{
  if (current_state_ == NavFSMState::WAITING_FOR_GOALS) {
    transitToInitializing();
  }
}

void NavigationFSM::onNavigationStart()
{
  if (current_state_ == NavFSMState::INITIALIZING) {
    transitToExecutingPath();
  }
}

void NavigationFSM::onGoalReached()
{
  transitToGoalReached();
  // After a brief pause or additional processing, might continue to next goal or complete
}

void NavigationFSM::onObstacleDetected()
{
  if (current_state_ == NavFSMState::EXECUTING_PATH) {
    transitToAvoidingObstacle();
  }
}

void NavigationFSM::onObstacleCleared()
{
  if (current_state_ == NavFSMState::AVOIDING_OBSTACLE) {
    transitToExecutingPath();
  }
}

void NavigationFSM::onError()
{
  transitToFailed();
}

void NavigationFSM::onCompletion()
{
  transitToCompleted();
}

}  // namespace waypoint_nav