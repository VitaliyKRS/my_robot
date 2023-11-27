cmake_minimum_required(VERSION 3.8)
project(tracked_robot_teleop)
cmake_minimum_required(VERSION 3.5)
project(tracked_robot_teleop)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

add_library(tracked_robot_teleop SHARED
  src/TeleopHandler.cpp
)

target_compile_definitions(tracked_robot_teleop PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(
  tracked_robot_teleop
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  rclcpp_components
)

rclcpp_components_register_nodes(tracked_robot_teleop PLUGIN "TeleopHandler")

# Install shared library and header files
install(TARGETS tracked_robot_teleop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION config
)


ament_package()
