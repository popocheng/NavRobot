from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_params_file_argument = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('waypoint_nav'),
            'config',
            'params.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    gps_imu_fusion_node = Node(
        package='gps_imu_fusion',
        executable='gps_imu_fusion_node',
        name='gps_imu_fusion_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('gps_imu_fusion'),
                'config',
                'params.yaml'
            )
        ],
        remappings=[
            ('/imu', '/livox/imu'),  # Map imu/data to /imu/data for IMU data
            ('/gps/data', '/gps/data'),  # Map gps/data to /gps/data for GPS data
        ]
    )

    # RViz2 node to visualize navigation
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('waypoint_nav'),
            'config',
            'nav_visualization.rviz'
        )],
        output='screen'
    )

    # Navigation2 controller server node - now configured to handle timing issues
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    waypoint_nav_node = Node(
        package='waypoint_nav',
        executable='nav_waypoint_node',
        name='nav_waypoint_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('waypoint_nav'),
                'config',
                'params.yaml'
            )
        ],
        remappings=[
            # ('/world_odom', '/mavros/local_position/odom'), 
        ]
    )

    # Lifecyle manager to manage the navigation2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['controller_server']}]
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_params_file_argument,
        gps_imu_fusion_node,
        rviz_node,
        # External Nav2 controller server
        controller_server_node,
        waypoint_nav_node,
        lifecycle_manager
    ])