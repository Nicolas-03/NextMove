cmake_minimum_required(VERSION 3.5)
project(can_zoe)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_zoe_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(can_zoe_ze40_msgs REQUIRED)

add_executable(can_ze40_decoder src/ze40/can_ze40_decoder.cpp src/ze40/can_ze40.cpp)
add_executable(can_ze50_decoder src/ze50/can_ze50_decoder.cpp src/ze50/can_ze50.cpp)
add_executable(speed_controller src/speed_controller.cpp src/ze40/can_ze40.cpp)
add_executable(zoe_control src/zoe_control.cpp src/ze40/can_ze40.cpp)
add_executable(can_ze50_encoder src/ze50/can_ze50_encoder.cpp src/can_encoder.cpp src/vehicle_command.cpp src/ze50/vehicle_commands/acceleration_command.cpp src/ze50/vehicle_commands/steer_command.cpp src/ze50/vehicle_commands/rndb_command.cpp src/ze50/vehicle_commands/brake_command.cpp src/ze50/can_ze50.cpp)
add_executable(steer_teleop src/steer_teleop.cpp)
add_executable(steer_teleop_repeater src/steer_teleop_repeater.cpp)
ament_target_dependencies(can_ze50_decoder rclcpp std_msgs can_zoe_msgs can_msgs)
ament_target_dependencies(can_ze50_encoder rclcpp std_msgs can_zoe_msgs can_msgs)
ament_target_dependencies(can_ze40_decoder rclcpp std_msgs can_zoe_ze40_msgs can_msgs)
ament_target_dependencies(speed_controller rclcpp std_msgs can_zoe_msgs can_msgs)
ament_target_dependencies(zoe_control rclcpp std_msgs can_zoe_ze40_msgs can_msgs)
ament_target_dependencies(steer_teleop rclcpp std_msgs can_zoe_ze40_msgs can_msgs)
ament_target_dependencies(steer_teleop_repeater rclcpp std_msgs can_zoe_msgs can_msgs)

target_include_directories(can_ze50_decoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(can_ze50_encoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(can_ze40_decoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(zoe_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_include_directories(speed_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  


install(TARGETS
  can_ze50_encoder
  can_ze50_decoder
  can_ze40_decoder
  speed_controller
  zoe_control
  steer_teleop
  steer_teleop_repeater
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
