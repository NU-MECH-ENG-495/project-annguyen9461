cmake_minimum_required(VERSION 3.8)
project(claybot)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(claybot_interfaces REQUIRED)

include_directories(
  include
)

add_executable(vision src/vision.cpp)
target_include_directories(vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(vision PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(vision rclcpp std_msgs)  # Ensure ROS2 dependencies

add_executable(control_turn src/control_turn.cpp)
ament_target_dependencies(control_turn
  claybot_interfaces
  dynamixel_sdk
  rclcpp
)

# add_executable(listener src/subscriber_lambda_function.cpp)
# ament_target_dependencies(listener rclcpp std_msgs)
# add_executable(talker src/publisher_lambda_function.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

# Install executables
install(TARGETS 
  vision 
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS 
  control_turn
  DESTINATION lib/${PROJECT_NAME})
  
# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Python scripts
install(
  PROGRAMS 
  scripts/cut_shape_srv.py
  scripts/move_updown_srv.py
  scripts/move_updown_cli.py
  scripts/cut_shape_srv.py
  DESTINATION lib/${PROJECT_NAME}
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
