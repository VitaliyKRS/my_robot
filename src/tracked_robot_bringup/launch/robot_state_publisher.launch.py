import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command

def launch_setup(context: LaunchContext, support_package):
    """ Reference:
        https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/ 
        https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_moveit_config/launch/ur_moveit.launch.py
    """
    # render namespace, dumping the support_package.
    namespace = context.perform_substitution(support_package)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_path = LaunchConfiguration('xacro_path')
    # Add option to publish pointcloudz
    publish_pointcloud="True"
    publish_odom_tf="True"

    # Launch Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time,
                     #'frame_prefix': f"{namespace}/", # Reimplemented https://github.com/ros/robot_state_publisher/pull/169
                     'robot_description': Command(
                         [
                             'xacro ', xacro_path, ' ',
                             'robot_name:=', namespace, ' ',
                             'publish_pointcloud:=', publish_pointcloud, ' ',
                             'publish_odom_tf:=', publish_odom_tf, ' ',
                         ])
                     }]
    )
    
    return [robot_state_publisher_node]


def generate_launch_description():
    package_gazebo = get_package_share_directory('tracked_robot_gazebo')

    namespace = LaunchConfiguration('namespace', default="tracked_robot")

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    default_xacro_path = os.path.join(package_gazebo, "urdf", "tracked_robot.gazebo.xacro")

    declare_model_path_cmd = DeclareLaunchArgument(
        name='xacro_path',
        default_value=default_xacro_path,
        description='Absolute path to robot urdf file')
    
    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup, args=[namespace]))

    return ld
# EOF