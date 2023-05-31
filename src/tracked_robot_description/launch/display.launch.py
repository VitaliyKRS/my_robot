import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    tracked_robot_description_path = get_package_share_directory('tracked_robot_description')
    
    gui = LaunchConfiguration('gui')
    cover_type = LaunchConfiguration('cover_type')
    diff_drive_emulation = LaunchConfiguration('diff_drive_emulation')
    rvizconfig = LaunchConfiguration('rvizconfig')
    
    default_rviz_config_path = os.path.join(tracked_robot_description_path, 'rviz', 'urdf.rviz')

    declare_cover_type_cmd = DeclareLaunchArgument(
        name='cover_type',
        default_value='fisheye',
        description='Cover type to use. Options: pi, fisheye, realsense, zed.')

    declare_simulation_cmd = DeclareLaunchArgument(
        name='diff_drive_emulation',
        default_value='false',
        description='Enable urdf for differential drive emulation, for simulation.')

    declare_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui')

    declare_rvizconfig_cmd = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui),
        on_exit=Shutdown()
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui)
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        on_exit=Shutdown()
    )
    
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tracked_robot_description_path, '/launch/description.launch.py']),
        launch_arguments={'cover_type': cover_type, 'diff_drive_emulation': diff_drive_emulation}.items()
        )

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()
    
    ld.add_action(declare_cover_type_cmd)
    ld.add_action(declare_simulation_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_rvizconfig_cmd)
    ld.add_action(description_launch)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld
# EOF
