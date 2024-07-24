cmake_minimum_required(VERSION 3.5)
project(cmd_vel_to_gpio)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(cmd_vel_to_gpio_node src/cmd_vel_to_gpio.cpp)

# Link libraries
ament_target_dependencies(cmd_vel_to_gpio_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Install executable
install(TARGETS
  cmd_vel_to_gpio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
