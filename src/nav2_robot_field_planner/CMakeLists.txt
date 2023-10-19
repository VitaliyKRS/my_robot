cmake_minimum_required(VERSION 3.8)
project(nav2_robot_field_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(tracked_robot_msgs REQUIRED)


include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
    src/FieldPlanner.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    nav2_core
    nav_msgs
    nav2_navfn_planner
    tracked_robot_msgs
)

pluginlib_export_plugin_description_file(nav2_core field_planner_plugin.xml)

install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
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
