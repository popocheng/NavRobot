# 机器人狗巡航点自主导航算法包

## 项目概述

本项目实现了一个用于机器人狗的自主巡航点导航系统，能够处理GPS、IMU、激光雷达数据，并实现路径跟随、避障和状态管理功能。系统根据输入的多目标点序列，生成高频速度控制指令并提供导航状态反馈。

## 架构设计

### 主要组件

1. **导航核心模块 (nav_core)**
   - GPS/IMU融合定位：结合GPS和IMU数据实现精确定位
   - 路径规划与跟踪：实现路径跟踪算法（如纯追踪算法）
   - 坐标转换：GPS坐标系与本地笛卡尔坐标系的转换

2. **导航状态机 (nav_fsm)**
   - 定义导航状态：IDLE, WAITING_FOR_GOALS, INITIALIZING, EXECUTING_PATH, AVOIDING_OBSTACLE, GOAL_REACHED, FAILED, COMPLETED
   - 状态转换逻辑：根据导航情况自动转换状态
   - 状态查询接口：提供外部查询当前导航状态

3. **实用工具模块 (utils)**
   - 数学运算：距离计算、角度归一化、四元数与欧拉角转换
   - 路径跟踪：纯追踪算法实现
   - 导航辅助：目标距离判断、角度差计算

### 输入输出接口

#### 输入话题
- `/gps/data` `[sensor_msgs/msg/NavSatFix]` - 机器狗全局经纬高坐标
- `/imu` `[sensor_msgs/msg/Imu]` - 机器狗姿态信息
- `/livox/lidar` `[sensor_msgs/msg/PointCloud2]` - 激光雷达数据（用于避障）
- `/waypoint_goals` `[msg_set_msgs/msg/MultiGoal]` - 巡航点列表

#### 输出话题
- `/cmd_vel` `[geometry_msgs/msg/Twist]` - 高频速度控制指令 (vx, vy, wz)
- `/nav_status` `[std_msgs/msg/String]` - 导航状态 (等待、进行中、失败、完成等)

## 算法流程

1. **初始化阶段**
   - 接收首次GPS数据，建立本地坐标系原点
   - 根据GPS变化估算初始方向角
   - 等待接收巡航点序列

2. **路径跟踪阶段**
   - 使用纯追踪算法跟随预设路径
   - 持续融合GPS/IMU数据更新当前位置和方向
   - 发布速度控制指令

3. **避障处理**
   - 监测激光雷达数据检测障碍物
   - 暂时切换到避障状态
   - 绕过障碍物后恢复原路径

4. **目标到达**
   - 判断是否到达当前目标点
   - 如还有后续目标点，继续执行
   - 全部完成则结束导航任务

## 安装与运行

### 构建项目
```bash
cd ~/robotdog_nav
colcon build --packages-select waypoint_nav
source install/setup.bash
```

### 启动导航节点
```bash
ros2 launch waypoint_nav waypoint_nav.launch.py
```

### 发布巡航点
```bash
# 通过编程或命令行发布MultiGoal消息到/waypoint_goals话题
```

## 参数配置

- `goal_tolerance`: 到达目标点的容差距离 (默认: 1.0米)
- `yaw_tolerance`: 到达目标点的角度容差 (默认: 0.2弧度)
- `linear_velocity`: 线速度 (默认: 1.0 m/s)
- `angular_velocity_limit`: 最大角速度 (默认: 1.0 rad/s)
- `control_frequency`: 控制频率 (默认: 10 Hz)
- `lookahead_distance`: 路径跟踪前瞻距离 (默认: 2.0米)

## 状态机详解

| 状态 | 描述 | 进入条件 | 退出条件 |
|------|------|----------|----------|
| IDLE | 空闲状态 | 系统启动或导航完成 | 接收到巡航点列表 |
| WAITING_FOR_GOALS | 等待巡航点 | 系统处于空闲但准备接收目标 | 收到巡航点序列 |
| INITIALIZING | 初始化 | 接收到巡航点 | 坐标系初始化完成 |
| EXECUTING_PATH | 执行路径跟踪 | 初始化完成 | 到达目标点或检测到障碍物 |
| AVOIDING_OBSTACLE | 避障 | 在执行路径时检测到障碍物 | 障碍物清除或绕过 |
| GOAL_REACHED | 到达目标 | 当前目标点距离和角度误差在容差范围内 | 开始下一目标点或全部完成 |
| FAILED | 失败 | 系统错误 | 人工干预重启 |
| COMPLETED | 完成 | 所有目标点均到达 | 系统保持完成状态 |

## 实现特点

1. **鲁棒的传感器融合**
   - GPS提供全局位置参考
   - IMU提供连续姿态信息
   - 互补滤波实现精确位置估计

2. **智能路径规划**
   - 纯追踪算法实现平滑路径跟随
   - 可配置的前瞻距离适应不同速度
   - 速度和角速度限制保证安全

3. **状态机管理**
   - 清晰的状态定义和转换
   - 状态监控便于调试和控制
   - 错误处理和恢复机制

4. **扩展性强**
   - 模块化设计便于扩展
   - 参数化配置适应不同场景
   - 预留接口支持更多功能

## 应用场景

- 自主导航巡逻
- 巡航任务执行
- GPS引导的路径跟随
- 障碍物环境下的安全导航