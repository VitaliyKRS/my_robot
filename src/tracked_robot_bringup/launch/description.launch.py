import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, support_package):
    namespace = context.perform_substitution(support_package)

    xacro_path = LaunchConfiguration('xacro_path')
    cover_type = LaunchConfiguration('cover_type')
    diff_drive_emulation = LaunchConfiguration('diff_drive_emulation')
    use_nominal_extrinsics = LaunchConfiguration('use_nominal_extrinsics')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            #'frame_prefix': f"{namespace}/", # Reimplemented https://github.com/ros/robot_state_publisher/pull/169
            'robot_description': Command(['xacro ', xacro_path, ' ',
                    'cover_type:=', cover_type, ' ',
                    'diff_drive_emulation:=', diff_drive_emulation, ' ',
                    'use_nominal_extrinsics:=', use_nominal_extrinsics, ' ',
                ])
        }]
    )
    return [robot_state_publisher_node]


def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default="tracked_robot")

    # URDF/xacro file to be loaded by the Robot State Publisher node
    default_xacro_path = os.path.join(
        get_package_share_directory('tracked_robot_description'),
        'urdf',
        'tracked_robot_hardware.urdf'
    )

    declare_model_path_cmd = DeclareLaunchArgument(
        name='xacro_path',
        default_value=default_xacro_path,
        description='Absolute path to robot urdf file')
    
    tracked_robot_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='tracked_robot',
        description='tracked_robot namespace name. If you are working with multiple robot you can change this namespace.')

    declare_cover_type_cmd = DeclareLaunchArgument(
        name='cover_type',
        default_value='fisheye',
        description='Cover type to use. Options: pi, fisheye, realsense, zed.')

    declare_simulation_cmd = DeclareLaunchArgument(
        name='diff_drive_emulation',
        default_value='false',
        description='Enable urdf for differential drive emulation, for simulation.')

    declare_use_nominal_extrinsics_cmd = DeclareLaunchArgument(
        name='use_nominal_extrinsics',
        default_value='false',
        description='Use nominal extrinsics ONLY for Realsense camera.')

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()    
    ld.add_action(tracked_robot_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_simulation_cmd)
    ld.add_action(declare_cover_type_cmd)
    ld.add_action(declare_use_nominal_extrinsics_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup, args=[namespace]))

    return ld
# EOF
