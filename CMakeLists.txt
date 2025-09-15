cmake_minimum_required(VERSION 3.8)
project(random_explorer_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(
  include
)

# Build executable
add_executable(exploration_controller
  src/exploration_controller.cpp
  src/main.cpp
)

# Link dependencies
ament_target_dependencies(exploration_controller
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  action_msgs
  tf2_geometry_msgs
  visualization_msgs
  tf2
)

# Install executable
install(TARGETS
  exploration_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install include directory
install(DIRECTORY
  include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()