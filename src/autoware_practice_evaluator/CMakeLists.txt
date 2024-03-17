cmake_minimum_required(VERSION 3.14)
project(autoware_practice_evaluator)

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
  src/evaluator.cpp
  src/condition.cpp
  src/geometry.cpp
  src/visualization.cpp
)
# Suppress boost internal warning (boost/core/no_exceptions_support.hpp).
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DBOOST_ALLOW_DEPRECATED_HEADERS")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware_practice_evaluator::Evaluator"
  EXECUTABLE evaluator
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
