#ifndef NAV_FSM_HPP_
#define NAV_FSM_HPP_

#include <string>

namespace waypoint_nav {

enum class NavFSMState {
  IDLE,
  WAITING_FOR_GOALS,
  INITIALIZING,
  EXECUTING_PATH,
  AVOIDING_OBSTACLE,
  GOAL_REACHED,
  FAILED,
  COMPLETED
};

class NavigationFSM
{
public:
  NavigationFSM();

  // State transition methods
  void transitToIdle();
  void transitToWaitingForGoals();
  void transitToInitializing();
  void transitToExecutingPath();
  void transitToAvoidingObstacle();
  void transitToGoalReached();
  void transitToFailed();
  void transitToCompleted();

  // State query methods
  NavFSMState getCurrentState() const { return current_state_; }
  std::string getStateAsString() const;
  bool isNavigating() const;
  bool isCompleted() const { return current_state_ == NavFSMState::COMPLETED; }
  bool isFailed() const { return current_state_ == NavFSMState::FAILED; }

  // Event trigger methods
  void onGoalsReceived();
  void onNavigationStart();
  void onGoalReached();
  void onObstacleDetected();
  void onObstacleCleared();
  void onError();
  void onCompletion();

private:
  NavFSMState current_state_;
};

}  // namespace waypoint_nav

#endif  // NAV_FSM_HPP_