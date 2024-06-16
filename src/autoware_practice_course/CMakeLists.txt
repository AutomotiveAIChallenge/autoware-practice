cmake_minimum_required(VERSION 3.14)
project(autoware_practice_course)

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

ament_auto_add_executable(vehicle_forward  src/vehicle/forward.cpp)
ament_auto_add_executable(vehicle_backward src/vehicle/backward.cpp)
ament_auto_add_executable(p_controller src/velocity_planning/p_controller.cpp)
ament_auto_add_executable(trajectory_planner src/velocity_planning/trajectory_planner.cpp)
ament_auto_add_executable(longitudinal_controller src/velocity_planning/longitudinal_controller.cpp)
ament_auto_add_executable(trajectory_loader src/velocity_planning/trajectory_loader.cpp)
ament_auto_add_executable(trajectory_follower src/velocity_planning/trajectory_follower.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  src/velocity_planning/scripts/run_autoware_practice_course_example02_02.sh
  src/velocity_planning/scripts/run_autoware_practice_course_example02_03.sh
  src/velocity_planning/scripts/run_autoware_practice_course_example02_04.sh
  DESTINATION lib/${PROJECT_NAME}
)
ament_auto_package()
