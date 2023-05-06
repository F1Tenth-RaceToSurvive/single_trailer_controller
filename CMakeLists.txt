cmake_minimum_required(VERSION 3.5)
project(single_trailer_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  serial_driver
  )

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Create Cpp executable
add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
add_executable(save_waypoints_node src/save_waypoints.cpp)
add_executable(serial_encoder_bridge_node src/serial_encoder_bridge.cpp)

ament_target_dependencies(pure_pursuit_node 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros tf2_geometry_msgs
)

ament_target_dependencies(save_waypoints_node 
  rclcpp geometry_msgs tf2_ros
)

ament_target_dependencies(serial_encoder_bridge_node 
  rclcpp std_msgs serial_driver
)

# Install Cpp executables
install(TARGETS
  pure_pursuit_node
  save_waypoints_node
  serial_encoder_bridge_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# install(PROGRAMS
#   scripts/pure_pursuit_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch dependencies
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config dependencies
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install waypoints files directory
install(DIRECTORY
  waypoints
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
