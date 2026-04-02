# ROBOTDOG_NAV

# 功能包介绍
- 公共类
  - global2local: 无人机共用的全局坐标系到本地坐标系的转换包，暂未使用，最终方案不一定接PX4的融合定位
  - msg_set_msgs: 无人机共用的消息类型包，也包含导航相关的消息类型，如MultiGoal，用于和指控端话题对齐
- 基础算法组件类
  - gps_imu_fusion: 一个示例GPS+IMU融合定位包，后续需替换成最终融合定位方案
  - navigation2: 基于navigation2的定制修改后导航组件，目前包含无图导航功能最小功能包集合，后续可能加上全局规划、行为树等组件
- 外层项目应用类
  - nav2_straight_planner: 定点无图导航应用，可基于此进行课题5等开发
  - waypoint_nav：适配了msg_set_msgs中的MultiGoal消息类型输入，可基于此进行ZNY的waypoint多点巡航开发


# 测试命令
- 测试waypoint_nav
```bash
# 算法节点终端
source install/setup.bash && \
ros2 launch robotdog_nav waypoint_nav.launch.py
```
```bash
# 发送指令终端
source install/setup.bash && \
ros2 topic pub --once /waypoint_goals msg_set_msgs/msg/MultiGoal "{
  multi_goal_points: [
    {
      x_or_lat: 0.00012,
      y_or_lon: 0.00002,
      z_or_alt: 0.0,
      yaw: 0.0,
      vel: 1.0
    },
    {
      x_or_lat: -0.00015,
      y_or_lon: 0.00013,
      z_or_alt: 0.0,
      yaw: 1.0,
      vel: 0.8
    },
    {
      x_or_lat: -0.00005,
      y_or_lon: -0.00010,
      z_or_alt: 0.0,
      yaw: 1.5,
      vel: 0.8
    },
    {
      x_or_lat: 0.00008,
      y_or_lon: -0.00008,
      z_or_alt: 0.0,
      yaw: 0.5,
      vel: 1.0
    },
  ],
  is_gps_aid: true,
  is_gps_hgt: false,
  ctl_mode: 0
}"
```