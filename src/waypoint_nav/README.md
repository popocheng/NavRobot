# 航路点导航包

本包实现了机器人狗跟随一系列航路点的自主导航功能。它集成了来自GPS/IMU融合的定位数据，并提供了两种导航模式：使用外部controller_server进行路径规划的模式，或使用纯追踪算法直接导航的模式。

## 功能特性

- **集成定位**：使用来自GPS/IMU融合的世界里程计数据进行精确定位和方向估计
- **航路点跟随**：导航通过一系列坐标点
- **双控制器模式**：支持通过参数切换使用外部controller_server（带障碍物避让）或纯追踪算法
- **状态管理**：实现有限状态机管理导航状态
- **可视化**：提供航路点、路径和实际轨迹的RViz可视化

## 主要主题

### 订阅的主题

- `/world_odom` (`nav_msgs/Odometry`) - 来自GPS/IMU融合的世界里程计数据
- `/livox/lidar` (`sensor_msgs/PointCloud2`) - 激光雷达点云数据（供外部控制器使用）
- `/waypoint_goals` (`msg_set_msgs/MultiGoal`) - 要跟随的一系列航路点

### 发布的主题

- `/cmd_vel` (`geometry_msgs/Twist`) - 速度命令 (线性 x, y 和角 z)
- `/nav_status` (`std_msgs/String`) - 当前导航状态
- `/waypoint_markers` (`visualization_msgs/MarkerArray`) - 航路点标记
- `/path_markers` (`visualization_msgs/MarkerArray`) - 路径标记
- `/actual_path` (`nav_msgs/Path`) - 实际行驶路径

## 使用方法

### 构建

```bash
cd /path/to/robotdog_nav
colcon build --packages-select waypoint_nav
source install/setup.bash
```

### 运行

```bash
ros2 launch waypoint_nav waypoint_nav.launch.py
```

### 发送航路点

要发送一系列航路点，请发布到 `/waypoint_goals` 主题:

```bash
# 示例：通过命令行发送航路点 (需根据消息格式调整)
ros2 topic pub /waypoint_goals msg_set_msgs/msg/MultiGoal "..."
```

## 参数

- `goal_tolerance`: 认为到达目标的距离容差 (默认: 1.0 m)
- `yaw_tolerance`: 目标方向的角度容差 (默认: 0.8 rad)
- `linear_velocity`: 导航的恒定线速度 (默认: 0.8 m/s)
- `angular_velocity_limit`: 最大角速度 (默认: 1.0 rad/s)
- `control_frequency`: 控制循环频率 (默认: 10 Hz)
- `lookahead_distance`: 纯追踪的前瞻距离 (默认: 2.0 m)
- `use_controller_server`: 是否使用外部controller_server进行路径规划和避障 (默认: true)

## 状态

导航系统在以下状态之间运行:

- `IDLE`: 等待航路点命令
- `WAITING_FOR_GOALS`: 准备接收目标
- `INITIALIZING`: 使用融合里程计数据进行初始化
- `EXECUTING_PATH`: 跟随规划的路径
- `GOAL_REACHED`: 成功到达目标
- `FAILED`: 导航因错误失败
- `COMPLETED`: 所有航路点成功完成

## 系统架构

该包由以下组件组成:

- `WaypointNavigator`: 使用融合里程计的核心导航逻辑
- `NavigationFSM`: 管理导航状态的有限状态机
- `Utils`: 数学计算和路径规划的实用函数
- `NavWaypointNode`: 协调导航过程的主要ROS2节点

## 控制器模式

包支持两种导航控制器模式，通过 `use_controller_server` 参数进行切换:

- **外部Controller Server模式** (use_controller_server: true): 导航器将路径发送给外部启动的controller_server，由其执行路径规划和避障
- **纯追踪模式** (use_controller_server: false): 直接使用纯追踪算法计算速度命令

## 算法逻辑流程图

### 数据流层面
```
传感器数据处理线程:
├── /world_odom → 机器人位置更新
├── /livox/lidar → 激光雷达数据（供外部控制器使用）
└── /waypoint_goals → 航路点序列更新
```

### 控制决策层面
```
主控制循环:
┌─────────────────────────────────────────────────────────────────┐
│                    导航状态管理 (FSM)                            │
│  管理IDLE, WAITING_FOR_GOALS, INITIALIZING等状态                 │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│              控制器选择 (根据参数配置)                           │
│  输入: use_controller_server 参数                              │
│  │                                                             │
│  ├─ true → 将路径发送给外部controller_server                   │
│  └─ false → 使用纯追踪算法直接计算速度命令                      │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                   速度指令输出                                  │
│              → /cmd_vel (线性速度, 角速度)                      │
└─────────────────────────────────────────────────────────────────┘
```