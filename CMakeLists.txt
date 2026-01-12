cmake_minimum_required(VERSION 3.8)
project(random_explorer_bot)

# Enable compiler warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required for std::optional and other modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS2 dependencies
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

# Add include directory for header files
include_directories(include)

# Build exploration_controller executable from sources
add_executable(exploration_controller
  src/exploration_controller.cpp
  src/main.cpp
)

# Link all required ROS2 dependencies to executable
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

# Install executable to lib/<package_name> (ROS2 convention)
install(TARGETS
  exploration_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files to share/<package_name>/launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files (YAML params) to share/<package_name>/config
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install headers to include/ for other packages to use
install(DIRECTORY
  include/
  DESTINATION include/
)

# Run linting tests if BUILD_TESTING is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
