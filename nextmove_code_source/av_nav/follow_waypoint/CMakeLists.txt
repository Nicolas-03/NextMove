cmake_minimum_required(VERSION 3.20)
project(zoe_follow_waypoint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(can_zoe_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(zoe_waypoint_interfaces REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)

add_executable(zoe_follow_waypoint src/zoe_follow_waypoint.cpp src/math_utils.cpp)
ament_target_dependencies(zoe_follow_waypoint rclcpp std_msgs geometry_msgs nav_msgs can_zoe_msgs novatel_oem7_msgs sensor_msgs tf2_ros  GeographicLib tf2 tf2_geometry_msgs zoe_waypoint_interfaces visualization_msgs)

target_include_directories(zoe_follow_waypoint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  zoe_follow_waypoint
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY meshes urdf
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
