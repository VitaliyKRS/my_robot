# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("tracked_robot_description"), "urdf", "tracked_robot_hardware.urdf"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("tracked_robot_hardware"),
            "controllers",
            "robot_controller.yaml",
        ]
    )

    tracked_robot_bringup_path = get_package_share_directory('tracked_robot_bringup')
    robot_localization_file_path = os.path.join(tracked_robot_bringup_path, 'config/ekf_with_gps.yaml')
    scan_config_file_path = os.path.join(tracked_robot_bringup_path, 'config/laser_scan_filer.yaml')
    
    #twist_mux_params_file = os.path.join(pkg_teleop, 'config/twist_mux.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_robot_localization = LaunchConfiguration('use_robot_localization')

    # Declare the launch arguments  
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='True',
        description='Use ros2_control if true')

    declare_use_robot_localization_cmd = DeclareLaunchArgument(
        name='use_robot_localization',
        default_value='True',
        description='Use robot_localization package if true')

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(tracked_robot_bringup_path, 'launch', 'robot_state_publisher.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time,
                          'use_ros2_control': use_ros2_control}.items())





    share_dir = get_package_share_directory('mpu9250driver')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'mpu9250.yaml'),
                                           description='Path to the ROS2 parameters file to use.')

    imu_parameter_file = os.path.join(tracked_robot_bringup_path, 'params/filter_madgwick.yaml')

    # Launch controller manager (control_node)
    start_controller_manager_cmd = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # Delayed controller manager action   
    start_delayed_controller_manager = TimerAction(period=2.0, actions=[start_controller_manager_cmd])

    # Spawn diff_controller (robot_controller_spawner)
    start_diff_controller_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager" ],
        #remappings=[("/diffbot_base_controller/odom", "/odom")]
    )

    # Delayed diff_drive_spawner action
    start_delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_diff_controller_cmd]))

    # Spawn joint_state_broadcaser (joint_state_broadcaster_spawner)
    start_joint_broadcaster_cmd = Node(
        condition=IfCondition(use_ros2_control),
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])

    # Delayed joint_broadcaster_spawner action
    start_delayed_joint_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_joint_broadcaster_cmd]))

    # Start robot localization using an Extended Kalman Filter ...map->odom transform
    start_robot_localization_local_cmd = Node(
#        condition=IfCondition(use_robot_localization),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        parameters=[robot_localization_file_path,
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local'),
               ('/set_pose', '/initialpose')]
    )


    # Start robot localization using an Extended Kalman Filter ...odom->base_footprint transform
    start_robot_localization_global_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        parameters=[robot_localization_file_path,
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global'),
                ('/set_pose', '/initialpose')]
    )


    mpu9250driver_node = Node(
        package='mpu9250driver',
        executable='mpu9250driver',
        name='mpu9250driver_node',
        output="screen",
        emulate_tty=True,
        parameters=[parameter_file]
    )

    ublox_dir = get_package_share_directory('ublox_dgnss')
    gnss_included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                ublox_dir + '/launch/ublox_rover_hpposllh_navsatfix.launch.py'))

    lidar_dir = get_package_share_directory('sllidar_ros2')
    argument_for_lidar = ""
    DeclareLaunchArgument(
            'argument_for_lidar',
            default_value = argument_for_lidar,
            description = 'Argument for lidar launch file')

    lidar_included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                lidar_dir + '/launch/view_sllidar_s3_launch.py'),                         # view_sllidar_s3_launch.py
        launch_arguments = {'frame_id': 'lidar_link'}.items()
    )


    # imu_dir = get_package_share_directory('wit_ros2_imu')
    # imu_included_launch = launch.actions.IncludeLaunchDescription(
    #     launch.launch_description_sources.PythonLaunchDescriptionSource(
    #             imu_dir + '/rviz_and_imu.launch.py'))


# Laser scan filter chain 
    laser_scan_filter_cmd = Node(
        package='laser_filters',
        executable="scan_to_scan_filter_chain",
        parameters=[scan_config_file_path],
#        remappings=[('/scan/filtered', '/scan')]
    )

    start_imu_filter_madgwick = Node(
	package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[imu_parameter_file]
    )          

# Start the navsat transform node which converts GPS data into the world coordinate frame
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[robot_localization_file_path,
        {'use_sim_time': use_sim_time}],
        remappings=[('imu/data', 'imu/data'),
            ('gps/fix', 'gps/fix'), 
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/global')])

# Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(start_robot_localization_local_cmd)
    #ld.add_action(start_robot_localization_global_cmd)
    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_delayed_controller_manager)
    ld.add_action(start_delayed_diff_drive_spawner)
    ld.add_action(start_delayed_joint_broadcaster_spawner)
    ld.add_action(params_declare)
    ld.add_action(mpu9250driver_node)
    ld.add_action(gnss_included_launch)
    ld.add_action(lidar_included_launch)
    #ld.add_action(imu_included_launch)
    ld.add_action(laser_scan_filter_cmd)
    ld.add_action(start_imu_filter_madgwick)
    ld.add_action(start_navsat_transform_cmd)
    return ld
