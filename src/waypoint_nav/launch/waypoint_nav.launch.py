from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('waypoint_nav'),
        'config',
        'params.yaml'
    )

    waypoint_nav_node = Node(
        package='waypoint_nav',
        executable='nav_waypoint_node',
        name='nav_waypoint_node',
        output='screen',
        parameters=[
            {'goal_tolerance': 1.0},
            {'yaw_tolerance': 0.2},
            {'linear_velocity': 1.0},
            {'angular_velocity_limit': 1.0},
            {'control_frequency': 10.0},
            {'lookahead_distance': 2.0}
        ]
    )

    return LaunchDescription([
        waypoint_nav_node
    ])