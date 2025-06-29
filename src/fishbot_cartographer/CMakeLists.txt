cmake_minimum_required(VERSION 3.8)
project(fishbot_cartographer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)      # 添加 sensor_msgs 依赖
find_package(nav_msgs REQUIRED)         # 添加 nav_msgs 依赖
find_package(diagnostic_msgs REQUIRED)  # 添加 diagnostic_msgs 依赖
find_package(message_filters REQUIRED)  # 添加 message_filters 依赖

add_executable(CartoTransferPose_main src/CartoTransferPose_main.cc)
add_executable(time_sync_node src/time_sync_node.cc)

ament_target_dependencies(
  time_sync_node
  rclcpp
  sensor_msgs
  nav_msgs
  diagnostic_msgs
  message_filters
)

ament_target_dependencies(
  CartoTransferPose_main
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
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
  DIRECTORY config launch rviz urdf map param
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS CartoTransferPose_main time_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
