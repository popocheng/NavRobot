import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression

# 当传入参数“drone_num"时，使用该参数的值作为机器人数量，否则使用默认值5
def launch_setup(context, *args, **kwargs):
    drone_num = LaunchConfiguration('drone_num', default='5')
    num=drone_num.perform(context)
   
    return [ IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('global2local'), 'launch'), 
                    '/global2local.launch.py'
                ]),
                launch_arguments={
                    'drone_id':str(i)
                    }.items()
            ) for i in range(int(num))] 

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
