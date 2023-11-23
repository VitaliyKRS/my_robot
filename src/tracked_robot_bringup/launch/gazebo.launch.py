import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription, LaunchContext
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    tracked_robot_bringup_path = get_package_share_directory('tracked_robot_bringup')
    package_worlds = get_package_share_directory('tracked_robot_worlds')

    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    
    default_world_name = 'mines.world' # Empty world: empty
    package_gazebo = get_package_share_directory('tracked_robot_gazebo')

    
    default_xacro_path = os.path.join(package_gazebo, "urdf", "tracked_robot.urdf.xacro")

    robot_description_content = os.popen(f"xacro {default_xacro_path}").read()


    world_name = LaunchConfiguration('world_name')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default="tracked_robot")

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    tracked_robot_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='tracked namespace name.')

    gazebo_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Set to "false" to run headless.')

    gazebo_server_cmd = DeclareLaunchArgument(
        name='server',
        default_value='True',
        description='Set to "false" not to run gzserver.')

    world_name_cmd = DeclareLaunchArgument(
        name='world_name',
        default_value=default_world_name,
        description='Load gazebo world.')

    # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
    # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
    # Reference options
    # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_ros/launch/gzserver.launch.py
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_ros_path, 'launch'), '/gzserver.launch.py']),
        launch_arguments={'world': [package_worlds, "/worlds/", world_name],
                          'verbose': 'true',
                          'init': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('server'))
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_ros_path, 'launch'), '/gzclient.launch.py']),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        # parameters=[params],
        parameters=[
            {"robot_description": robot_description_content},
        ],
    )

    mine_detector = Node(
        package='tracked_robot_controls',
        executable='tracked_robot_controls_minedetector',
        output='screen'
    )   

    mine_position = Node(
        package='tracked_robot_controls',
        executable='tracked_robot_controls_minepositionhandler',
        output='screen'
    )   

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        namespace=namespace,
        arguments=['-entity', 'tracked_robot',
                   '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-R', '0.0', '-P', '0.0', '-Y','0.0',
                   ]
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        remappings=[("/diff_drive_base_controller/cmd_vel_unstamped", "/cmd_vel")],
    )

    load_diff_drive_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        remappings=[("/diff_drive_base_controller/cmd_vel_unstamped", "/cmd_vel")],
    )


    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_diff_drive_base_controller],
                )
            ),
            use_sim_time_cmd,
            tracked_robot_cmd,
            gazebo_gui_cmd,
            gazebo_server_cmd,
            world_name_cmd,
            gazebo_server,
            gazebo_gui,
            mine_detector,
            mine_position, 
            node_robot_state_publisher,
            spawn_robot,
        ]
    )
# EOF
