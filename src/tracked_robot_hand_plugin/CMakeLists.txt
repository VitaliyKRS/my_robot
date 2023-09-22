cmake_minimum_required(VERSION 3.8)
project(tracked_robot_hand_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tracked_robot_msgs REQUIRED)

include_directories(
  src
  include
  ${rclcpp_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tracked_robot_msgs_INCLUDE_DIRS}
)

add_library(tracked_robot_hand_plugin SHARED
  src/HandPlugin.cpp
)


ament_export_dependencies(rclcpp)

ament_target_dependencies(
  tracked_robot_hand_plugin
  rclcpp
  gazebo
  std_msgs
  tf2_ros
  tracked_robot_msgs
)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY 
    include
  DESTINATION include
)



ament_package()
