from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gps_imu_fusion'),
        'config',
        'params.yaml'
    )

    gps_imu_fusion_node = Node(
        package='gps_imu_fusion',
        executable='gps_imu_fusion_node',
        name='gps_imu_fusion_node',
        output='screen',
        parameters=[
            {'origin_lat': 37.7749},
            {'origin_lon': -122.4194},
            {'origin_alt': 0.0}
        ]
    )

    return LaunchDescription([
        gps_imu_fusion_node
    ])