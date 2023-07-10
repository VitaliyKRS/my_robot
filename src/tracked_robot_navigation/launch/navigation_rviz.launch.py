import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

def generate_launch_description():
    package_navigation= get_package_share_directory('tracked_robot_navigation')

    default_rviz_config_path = os.path.join(package_navigation, 'rviz/nav2_config.rviz')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')


    
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])    

    ld = LaunchDescription()
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
# EOF
