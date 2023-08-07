cmake_minimum_required(VERSION 3.8)
project(tracked_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${nav2_costmap_2d_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

add_library(mine_layer SHARED
    src/MineLayer.cpp 
)

ament_target_dependencies(
  mine_layer

  rclcpp
  nav2_costmap_2d
  pluginlib
  geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(
    TARGETS mine_layer  
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  DIRECTORY params rviz config include
  DESTINATION share/${PROJECT_NAME}
)

install(
    FILES costmap_plugins.xml
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
