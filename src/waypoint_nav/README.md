# Waypoint Navigation Package

This package implements autonomous navigation for a robot dog that follows a sequence of waypoints. It integrates fused odometry data from GPS/IMU fusion with LiDAR data to achieve accurate navigation with obstacle avoidance.

## Features

- **Integrated Positioning**: Uses fused world_odom data from GPS/IMU fusion for accurate positioning and orientation
- **Waypoint Following**: Navigates through a sequence of coordinates
- **Obstacle Avoidance**: Uses LiDAR data to detect and avoid obstacles (TO-DO)
- **State Management**: Implements a finite state machine for navigation states
- **Pure Pursuit Algorithm**: Path following algorithm for smooth navigation

## Topics

### Subscribed Topics

- `/world_odom` (`nav_msgs/Odometry`) - Fused world odometry from GPS/IMU fusion
- `/livox/lidar` (`sensor_msgs/PointCloud2`) - LiDAR point cloud data
- `/waypoint_goals` (`msg_set_msgs/MultiGoal`) - Sequence of waypoints to follow

### Published Topics

- `/cmd_vel` (`geometry_msgs/Twist`) - Velocity commands (linear x, y and angular z)
- `/nav_status` (`std_msgs/String`) - Current navigation status

## Usage

### Building

```bash
cd /path/to/robotdog_nav
colcon build --packages-select waypoint_nav
source install/setup.bash
```

### Running

```bash
ros2 launch waypoint_nav waypoint_nav.launch.py
```

### Sending Waypoints

To send a sequence of waypoints, publish to the `/waypoint_goals` topic:

```bash
# Example: Send waypoints via command line (you'll need to adapt to your message format)
ros2 topic pub /waypoint_goals msg_set_msgs/msg/MultiGoal "..."
```

## Parameters

- `goal_tolerance`: Distance tolerance to consider a goal reached (default: 1.0 m)
- `yaw_tolerance`: Yaw angle tolerance for goal orientation (default: 0.2 rad)
- `linear_velocity`: Constant linear velocity for navigation (default: 1.0 m/s)
- `angular_velocity_limit`: Maximum angular velocity (default: 1.0 rad/s)
- `control_frequency`: Control loop frequency (default: 10 Hz)
- `lookahead_distance`: Lookahead distance for pure pursuit (default: 2.0 m)

## States

The navigation system operates in the following states:

- `IDLE`: Awaiting waypoint commands
- `WAITING_FOR_GOALS`: Ready to receive goals
- `INITIALIZING`: Initializing with fused odometry data
- `EXECUTING_PATH`: Following the planned path
- `AVOIDING_OBSTACLE`: Temporarily deviating to avoid obstacles (TO-DO)
- `GOAL_REACHED`: Successfully reached a goal
- `FAILED`: Navigation failed due to error
- `COMPLETED`: All waypoints successfully navigated

## Architecture

The package consists of:

- `WaypointNavigator`: Core navigation logic using fused odometry
- `NavigationFSM`: Finite state machine for managing navigation states
- `Utils`: Utility functions for mathematical calculations and path planning
- `NavWaypointNode`: Main ROS2 node that orchestrates the navigation process

## Unfinished/Planned Features

The following features are not yet implemented but planned:

1. **LiDAR-based Obstacle Detection and Avoidance**: Currently, the LiDAR data is subscribed but not processed for obstacle avoidance.
2. **Integration with nav2 libraries**: The original plan mentioned using nav2's local costmap with spatio_temporal_voxel_layer for advanced obstacle processing.
3. **Advanced Path Planning**: Implementing local replanning when obstacles are detected.
4. **Improved Integration with Global2Local Package**: Full integration with the existing global2local package for more sophisticated transformations.
5. **More Robust State Transitions**: Handling edge cases in state machine transitions.
6. **Service Interface**: Implementing a service for querying navigation status or resetting the navigation system.
7. **Parameter Tuning**: Fine-tuning parameters for different robot dynamics and environments.