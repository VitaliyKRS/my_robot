import os
import yaml
from pathlib import Path
import logging

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_navigation= get_package_share_directory('tracked_robot_navigation')
    package_worlds = get_package_share_directory('tracked_robot_worlds')

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    nav2_params_path = os.path.join(package_navigation, 'params', 'nav2_params.yaml')

    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')
    static_map_path = os.path.join(package_worlds, 'maps', 'empty.yaml')

    slam = LaunchConfiguration('slam')
    navsat = LaunchConfiguration('navsat')
    use_namespace = LaunchConfiguration('use_namespace')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    namespace = LaunchConfiguration('namespace', default="tracked_robot")

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')


    robot_localization_file_path = os.path.join(package_navigation, 'config', 'ekf.yaml')


    declare_navsat_cmd = DeclareLaunchArgument(
        name='navsat',
        default_value='False',
        description='Set to true to ran navsat.')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')
        
    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', 
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        name='default_bt_xml_filename',
        default_value=behavior_tree_xml_path,
        description='Full path to the behavior tree xml file to use')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')   

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=static_map_path,
        description='Full path to map file to load')

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    tracked_robot_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='tracked namespace name.')

    # Start the navsat transform node which converts GPS data into the world coordinate frame
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('imu', 'imu/data'),
                    ('gps/fix', 'gps/fix'), 
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odometry/global')], 
        condition=IfCondition(LaunchConfiguration('navsat')))

    # Start robot localization using an Extended Kalman filter...map->odom transform
    start_robot_localization_global_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global'),
                    ('/set_pose', '/initialpose')],
        condition=IfCondition(LaunchConfiguration('navsat')))

    # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
    start_robot_localization_local_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local'),
                    ('/set_pose', '/initialpose')])

    start_slam_toolbox_cmd_with_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time,
                          'slam_params_file': params_file}.items(),
        condition=IfCondition(LaunchConfiguration('slam')))


    start_ros2_navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
    launch_arguments = {'namespace': namespace,
                        'use_namespace': use_namespace,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'default_bt_xml_filename': default_bt_xml_filename,
                        'autostart': autostart}.items())

    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(tracked_robot_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_navsat_cmd)
    ld.add_action(start_navsat_transform_cmd)
    ld.add_action(start_robot_localization_global_cmd)
    ld.add_action(start_robot_localization_local_cmd)
    ld.add_action(start_slam_toolbox_cmd_with_params)
    ld.add_action(start_ros2_navigation_cmd)

    return ld
# EOF
