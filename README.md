# ROBOTDOG_NAV



```bash
source install/setup.bash && ros2 topic pub /waypoint_goals msg_set_msgs/msg/MultiGoal \
"{multi_goal_points: [{x_or_lat: 1.0, y_or_lon: 2.0, z_or_alt: 0.0, yaw: 0.0,
   vel: 1.0}], is_gps_aid: true, is_gps_hgt: false, ctl_mode: 0}"
```