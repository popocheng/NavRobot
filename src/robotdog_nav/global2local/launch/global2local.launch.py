from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # Define LaunchConfigurations with default values
    drone_id = LaunchConfiguration('drone_id', default='0')
    prefix = LaunchConfiguration('prefix', default="drone_")
    drone_name = PythonExpression(["'/' if '", drone_id, "' == '-1' else str('", prefix, "') + str(", drone_id, ")"])
    
    origin_lat = LaunchConfiguration('origin_lat', default='30.8112796')
    origin_lon = LaunchConfiguration('origin_lon', default='120.8379163')
    origin_alt = LaunchConfiguration('origin_alt', default='0.0')
    origin_use_drone = LaunchConfiguration('origin_use_drone', default='False')
    log_save_path = LaunchConfiguration('log_save_path', default='/home/wth/swarmhub/project/swarm_ws/src/global2local/log/')
    
    return LaunchDescription([
        Node(
            package='global2local',
            executable='global2local',
            name='global2local_node',
            output='screen',
            namespace=drone_name,
            parameters=[{
                'drone_id': drone_id,
                'origin_lat': origin_lat,
                'origin_lon': origin_lon,
                'origin_alt': origin_alt,
                'origin_use_drone': origin_use_drone,
                'log_save_path': log_save_path
            }]
        )
    ])

