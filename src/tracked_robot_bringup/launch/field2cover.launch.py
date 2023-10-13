import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    field2cover_package = get_package_share_directory("field2cover_ros2")

    # Define launch arguments for the parameter file path

    data_file = os.path.join(field2cover_package, "data", "field.json")
    parameter_file = os.path.join(field2cover_package, "config", "field2cover.yaml")

    print(f"[DEBUG] {data_file}")
    print(f"[DEBUG] {parameter_file}")

    # declare_parameter_file = DeclareLaunchArgument(
    #     'parameter_file',
    #     default_value=os.path.join(field2cover_package, 'config', 'field2cover.yaml'),
    #     description='Full path to the YAML parameter file to load'
    # )

    # # Define a launch argument for the data file path
    # declare_data_file = DeclareLaunchArgument(
    #     'data_file',
    #     default_value=os.path.join(field2cover_package, 'data', 'field.gml'),
    #     description='Full path to the data file to load as an argument'
    # )

    field2cover_ros2_cmd = Node(
        package="field2cover_ros2",
        executable="field2cover_ros2",
        name="field2cover_node",
        output="screen",
        parameters=[parameter_file, {'data_file': data_file}],
    )

    ld = LaunchDescription()
    ld.add_action(field2cover_ros2_cmd)

    return ld
