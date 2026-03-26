# ROBOTDOG_NAV



```bash
source install/setup.bash && \
ros2 topic pub --once /waypoint_goals msg_set_msgs/msg/MultiGoal "{
  multi_goal_points: [
    {
      x_or_lat: 0.00002,
      y_or_lon: 0.00002,
      z_or_alt: 0.0,
      yaw: 0.0,
      vel: 1.0
    },
    {
      x_or_lat: 0.00008,
      y_or_lon: 0.00007,
      z_or_alt: 0.0,
      yaw: 0.5,
      vel: 1.0
    },
    {
      x_or_lat: 0.00002,
      y_or_lon: 0.00006,
      z_or_alt: 0.0,
      yaw: 1.0,
      vel: 0.8
    },
    {
      x_or_lat: 0.00001,
      y_or_lon: 0.00008,
      z_or_alt: 0.0,
      yaw: 1.5,
      vel: 0.8
    }
  ],
  is_gps_aid: true,
  is_gps_hgt: false,
  ctl_mode: 0
}"
```