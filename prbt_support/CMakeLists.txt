cmake_minimum_required(VERSION 3.5)
project(prbt_support)

# Enable C++11 standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(canopen_chain_node REQUIRED)


# Include directories
include_directories(
  include
)

# Add executable
add_executable(system_info_node
  src/system_info_node.cpp
  src/system_info.cpp
  src/system_info_exception.cpp
)
ament_target_dependencies(system_info_node
  rclcpp
  sensor_msgs
)

# Install targets
install(TARGETS
  system_info_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other package resources
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Package export
ament_package()
