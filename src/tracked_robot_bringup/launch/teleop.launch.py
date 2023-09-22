import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
def generate_launch_description():

    topic_name_arg = DeclareLaunchArgument(
        'topic_name', default_value=TextSubstitution(text='cmd_vel'))
    
    joy_container = ComposableNodeContainer(
        name='joy_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy',
                parameters=[{'autorepeat_rate': 30.0}],
                namespace='',
            ),
            ComposableNode(
                package='tracked_robot_teleop',
                plugin='TeleopHandler',
                name='teleop_twist_joy_node',
                namespace='',
                remappings=[
                    ('cmd_vel', LaunchConfiguration('topic_name'))
                ],
            )
        ],
    )

    ld = LaunchDescription()
    ld.add_action(topic_name_arg)
    ld.add_action(joy_container)

    return ld
