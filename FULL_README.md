# 机器人狗巡航点自主导航系统

## 项目概述

本项目实现了用于机器人狗的自主巡航点导航系统，包括：
1. **GPS/IMU融合定位包** (gps_imu_fusion): 将GPS和IMU数据融合，发布世界坐标系下的里程计和TF变换
2. **巡航点导航包** (waypoint_nav): 基于GPS坐标序列的路径规划和导航控制

## 系统架构

### 1. GPS/IMU融合定位包 (gps_imu_fusion)

#### 功能
- 接收GPS和IMU数据
- 融合并发布world_odom里程计信息
- 发布world到robot_base的TF变换
- 坐标系定义：北为+Y，东为+X，天为+Z

#### 输入/输出
- **订阅**: `/gps/data` (sensor_msgs/NavSatFix), `/imu` (sensor_msgs/Imu)
- **发布**: `/world_odom` (nav_msgs/Odometry), TF (world -> robot_base)

#### 参数
- `origin_lat`: 原点纬度
- `origin_lon`: 原点经度
- `origin_alt`: 原点海拔

### 2. 巡航点导航包 (waypoint_nav)

#### 功能
- 处理GPS坐标序列作为导航目标
- 实现路径规划和跟踪
- 管理导航状态机
- 发布速度控制命令

#### 输入/输出
- **订阅**: `/gps/data`, `/imu`, `/livox/lidar`, `/waypoint_goals`
- **发布**: `/cmd_vel` (geometry_msgs/Twist), `/nav_status` (std_msgs/String)

## 安装与运行

### 编译项目
```bash
cd ~/robotdog_nav
colcon build --packages-select gps_imu_fusion waypoint_nav
source install/setup.bash
```

### 启动系统

#### 方法1：分别启动各节点
```bash
# 启动GPS/IMU融合节点
ros2 run gps_imu_fusion gps_imu_fusion_node --ros-args --params-file src/gps_imu_fusion/config/params.yaml

# 启动导航节点
ros2 run waypoint_nav nav_waypoint_node --ros-args --params-file src/waypoint_nav/config/params.yaml
```

#### 方法2：使用launch文件
```bash
# 启动GPS/IMU融合系统
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py

# 启动导航系统
ros2 launch waypoint_nav waypoint_nav.launch.py
```

### 测试系统

发送导航目标：
```bash
# 使用提供的测试脚本
python3 test_navigation_e2e.py
```

## 数据流

1. **GPS/IMU数据采集** → **GPS/IMU融合节点** → **world_odom + TF**
2. **GPS坐标序列** → **导航节点** → **融合定位数据** → **路径规划** → **cmd_vel**
3. **传感器数据** → **导航节点** → **状态反馈**

## 坐标系约定

- **world**: 固定参考系，以origin_lat/lon/alt为原点
  - +X: 东 (East)
  - +Y: 北 (North)
  - +Z: 天 (Up)
- **robot_base**: 机器人本体坐标系
  - +X: 前
  - +Y: 左
  - +Z: 上

## 参数配置

### GPS/IMU融合包
- `origin_lat`: 37.7749 (示例值)
- `origin_lon`: -122.4194 (示例值)
- `origin_alt`: 0.0 (示例值)

### 巡航点导航包
- `goal_tolerance`: 1.0 (米)
- `yaw_tolerance`: 0.2 (弧度)
- `linear_velocity`: 1.0 (m/s)
- `angular_velocity_limit`: 1.0 (rad/s)
- `control_frequency`: 10.0 (Hz)
- `lookahead_distance`: 2.0 (米)

## 状态机说明

导航系统包含以下状态：
- `IDLE`: 等待任务
- `WAITING_FOR_GOALS`: 等待接收目标点
- `INITIALIZING`: 初始化定位
- `EXECUTING_PATH`: 执行路径跟踪
- `GOAL_REACHED`: 达到目标点
- `FAILED`: 执行失败
- `COMPLETED`: 任务完成

## 未来功能

以下功能计划在未来版本中实现：
- LiDAR障碍物检测与避障
- 更精确的GPS/IMU融合算法
- 动态路径重规划
- 多机器人协调导航

## 测试验证

系统提供了端到端测试脚本`test_navigation_e2e.py`，可验证从传感器输入到导航输出的完整流程。