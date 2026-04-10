# global2local

<details>
<summary><b style="font-size: 24px;">V1.1.0（2023.11.17）</b></summary>
<ul>
<li>最初测试稳定版本


|    序号    |                 消息类型                 |               消息名称               |
| :--------: | :--------------------------------------: | :----------------------------------: |
|     1      |            nav_msgs::Odometry            |          /mavros/world/odom          |
|     2      |        geometry_msgs::PoseStamped        |          /mavros/world/pose          |
|     3      | msg_set::UsmPositionTargetToStateMachine |       /position_cmd/algorithms       |
|     4      |            msg_set::MultiGoal            |           /waypoints/goal            |
|     5      |         msg_set::TargetWorldPose         |          /target_worldpose           |
|     6      |           msg_set::AssignGroup           |   /mqtt/group/drone_mission/local    |
|     7      |        mavros_msgs::WaypointList         | /mqtt/group/drone_mission/path_local |
|  |                                          |                                      |

</details>

<details>
<summary><b style="font-size: 24px;">V1.3.0（2024.03.06）</b></summary>

|    序号    |                 消息类型                 |               消息名称               |
| :--------: | :--------------------------------------: | :----------------------------------: |
|     1      |            nav_msgs::Odometry            |          /mavros/world/odom          |
|     2      |        geometry_msgs::PoseStamped        |          /mavros/world/pose          |
|     3      | msg_set::UsmPositionTargetToStateMachine |       /position_cmd/algorithms       |
|     4      |            msg_set::MultiGoal            |           /waypoints/goal            |
|     5      |         msg_set::TargetWorldPose         |          /target_worldpose           |
|     6      |           msg_set::MultiTask             |             /multitask               |
|     7      |            msg_set::JumpTask             |             /jump_task               |
|  |                                          |                                      |

</details>