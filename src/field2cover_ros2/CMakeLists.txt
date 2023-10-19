
cmake_minimum_required(VERSION 3.8)
project(field2cover_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()
find_package(Fields2Cover REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_localization REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tracked_robot_msgs REQUIRED)


set(dependencies
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav2_msgs
    rcl_interfaces
    visualization_msgs
    rclcpp_action
    nav2_util
    robot_localization  
    tracked_robot_msgs
)

include_directories(
    include
    ${ament_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav2_msgs_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${rclcpp_action_INCLUDE_DIRS}
    ${nav2_util_INCLUDE_DIRS}
    ${robot_localization_INCLUDE_DIRS}
    ${tracked_robot_msgs_INCLUDE_DIRS}
)

set(SOURCES   
src/main.cpp
src/Field2CoverNode.cpp
src/ros/conversor.cpp
)
add_executable(
  ${PROJECT_NAME} ${SOURCES}
)
target_link_libraries(${PROJECT_NAME}
  Fields2Cover
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies} )

install(DIRECTORY include/
        DESTINATION include/
)

install(
  DIRECTORY config data
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS  ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()