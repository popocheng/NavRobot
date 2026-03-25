# TODO List for Waypoint Navigation Package

This document lists all the unfinished features and improvements that need to be implemented for the waypoint navigation package.

## High Priority Items

### 1. LiDAR-based Obstacle Detection and Avoidance
- [ ] Process LiDAR data from `/livox/lidar` topic in `lidarCallback` function
- [ ] Integrate with nav2's local costmap using spatio_temporal_voxel_layer as required
- [ ] Implement local path replanning when obstacles are detected
- [ ] Enhance the obstacle avoidance state in the FSM

### 2. Advanced GPS/IMU Fusion
- [ ] Implement proper initialization that moves the robot slightly to estimate orientation from GPS changes
- [ ] Develop a more sophisticated fusion algorithm (complementary filter or EKF) for orientation estimation
- [ ] Improve the initial azimuth estimation as mentioned in requirements

## Medium Priority Items

### 3. Enhanced Path Planning Algorithms
- [ ] Improve the pure pursuit algorithm with dynamic lookahead distance
- [ ] Add support for smoother trajectories between waypoints

### 4. State Machine Improvements
- [ ] Add more robust state transitions with error handling
- [ ] Implement recovery behaviors for failed states
- [ ] Add intermediate states if needed

## Low Priority Items

### 5. Additional Features
- [ ] Add support for dynamic obstacles
- [ ] Implement formation control if needed for multi-robot scenarios
- [ ] Add logging and diagnostics
- [ ] Implement more sophisticated costmap layers
- [ ] Add support for semantic navigation goals

### 6. Testing and Validation
- [ ] Add unit tests for all modules
- [ ] Create simulation scenarios for validation
- [ ] Perform real-world testing and tuning

## Notes

Based on the original requirements:
- Input topics are handled: `/gps/data`, `/imu`, `/livox/lidar`, and cruise point list
- Output topics are implemented: `/cmd_vel` and status topic
- The suggestion about initial movement for azimuth estimation needs implementation
- The requirement to use nav2's local costmap with spatio_temporal_voxel_layer is pending