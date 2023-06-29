cmake_minimum_required(VERSION 3.8)
project(tracked_robot_controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tracked_robot_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${tracked_robot_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  )

set(MINESOURCES   
  src/Nodes/MineDetector/main.cpp
  src/Nodes/MineDetector/MineDetector.cpp
  )

  set(MINEPOSSOURCES   
    src/Nodes/MinePositionHandler/main.cpp
    src/Nodes/MinePositionHandler/MinePositionHandler.cpp
  )

add_executable(
  ${PROJECT_NAME}_minedetector ${MINESOURCES}
)
add_executable(
  ${PROJECT_NAME}_minepositionhandler ${MINEPOSSOURCES}
)

target_link_libraries(${PROJECT_NAME}_minedetector zbar)

ament_target_dependencies(${PROJECT_NAME}_minedetector rclcpp sensor_msgs std_msgs OpenCV cv_bridge)

ament_target_dependencies(${PROJECT_NAME}_minepositionhandler rclcpp tracked_robot_msgs geometry_msgs nav_msgs)

install(TARGETS  ${PROJECT_NAME}_minedetector DESTINATION lib/${PROJECT_NAME})
install(TARGETS  ${PROJECT_NAME}_minepositionhandler DESTINATION lib/${PROJECT_NAME})

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
