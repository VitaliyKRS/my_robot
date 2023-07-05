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
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def launch_gazebo_setup(context: LaunchContext, support_namespace, support_world):
    """ Reference:
        https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/ 
        https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_moveit_config/launch/ur_moveit.launch.py
    """
    # render namespace, dumping the support_package.
    namespace = context.perform_substitution(support_namespace)

    # Load configuration from params
    # Spawn robot
    # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_ros/scripts/spawn_entity.py
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        namespace=namespace,
        arguments=['-entity', 'tracked_robot',
                   '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-R', '0.0', '-P', '0.0', '-Y','0.1',
                   ]
    )

    return [spawn_robot]


def generate_launch_description():
    package_gazebo = get_package_share_directory('tracked_robot_gazebo')
    package_worlds = get_package_share_directory('tracked_robot_worlds')
    package_navigation = get_package_share_directory('tracked_robot_navigation')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    
    default_world_name = 'empty.world' # Empty world: empty
    launch_file_dir = os.path.join(package_gazebo, 'launch')

    navlaunch_file_dir = os.path.join(package_navigation, 'launch')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    slam = LaunchConfiguration('slam')
    world_name = LaunchConfiguration('world_name')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default="tracked_robot")

    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        name='default_bt_xml_filename',
        default_value=behavior_tree_xml_path,
        description='Full path to the behavior tree xml file to use')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')   

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    tracked_robot_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='tracked namespace name.')

    gazebo_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to "false" to run headless.')

    gazebo_server_cmd = DeclareLaunchArgument(
        name='server',
        default_value='true',
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
                          'init': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('server'))
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_ros_path, 'launch'), '/gzclient.launch.py']),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    rsp_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/robot_state_publisher.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time,}.items(),
    )

    nav_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [navlaunch_file_dir, '/navigation.launch.py']),
        launch_arguments={'namespace': namespace,
                          'use_namespace': 'False',
                          'slam': slam,
                          'map': '/home/vitalii/my_robot/install/tracked_robot_worlds/share/tracked_robot_worlds/maps/empty.yaml',
                          'use_sim_time': use_sim_time,
                          'params_file': '/home/vitalii/my_robot/install/tracked_robot_navigation/share/tracked_robot_navigation/params/nav2_params.yaml',
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': 'true'
                          }.items(), 
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
    

    ld = LaunchDescription()
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(tracked_robot_cmd)
    ld.add_action(gazebo_gui_cmd)
    ld.add_action(gazebo_server_cmd)
    ld.add_action(world_name_cmd)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_gui)
    ld.add_action(rsp_launcher)
    ld.add_action(nav_launcher)
    ld.add_action(mine_detector)
    ld.add_action(mine_position)
    ld.add_action(OpaqueFunction(function=launch_gazebo_setup, args=[namespace, world_name]))

    return ld
# EOF
