cmake_minimum_required(VERSION 3.8)
project(fishbot_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)

add_executable(uart_controller src/uart_controller.cpp)

ament_target_dependencies(
  uart_controller
  rclcpp 
  geometry_msgs 
  nlohmann_json
  serial
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

install(
  DIRECTORY launch param maps urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS  uart_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
