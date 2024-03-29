cmake_minimum_required(VERSION 3.14)
project(autoware_practice_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/simulator.cpp
  src/interface.cpp
  src/geometry.cpp
  src/kinematics.cpp
  src/controller.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware_practice_simulator::Simulator"
  EXECUTABLE simulator
)

install(PROGRAMS
  script/control.py
  RENAME control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
