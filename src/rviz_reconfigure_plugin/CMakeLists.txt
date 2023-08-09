cmake_minimum_required(VERSION 3.8)
project(rviz_reconfigure_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

include_directories(
    src
)

add_library(${PROJECT_NAME} SHARED
    src/NodeModel.cpp
    src/NodeModel.h
    src/NoHighlightDelegate.cpp
    src/NoHighlightDelegate.h
    src/ParametersModel.cpp
    src/ParametersModel.h
    src/RclEngine.cpp
    src/RclEngine.h
    src/tools.cpp
    src/tools.h
    src/RvizPluginPanel.h
    src/RvizPluginPanel.cpp
    src/PluginWidget.h
    src/PluginWidget.cpp
    src/PluginWidget.ui
)

ament_target_dependencies(${PROJECT_NAME}
    pluginlib
    rviz_common
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common rviz_plugin.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

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
