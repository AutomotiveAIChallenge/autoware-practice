cmake_minimum_required(VERSION 3.14)
project(autoware_practice_visualization)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/result.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)
ament_auto_add_executable(marker src/marker.cpp)
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package()
