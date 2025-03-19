# CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(udp_mavros_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# Build
add_executable(udp_mavros_bridge src/udp_mavros_bridge.cpp)
ament_target_dependencies(udp_mavros_bridge
  rclcpp
  geometry_msgs
  mavros_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(mavros_example src/mavros_example.cpp)
ament_target_dependencies(mavros_example
  rclcpp
  geometry_msgs
  mavros_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(udp_mavros_bridge_2 src/udp_mavros_bridge_2.cpp)
ament_target_dependencies(udp_mavros_bridge_2
  rclcpp
  geometry_msgs
  mavros_msgs
  tf2
  tf2_geometry_msgs
)


add_executable(udp_mavros_bridge_3 src/udp_mavros_bridge_3.cpp)
ament_target_dependencies(udp_mavros_bridge_3
  rclcpp
  geometry_msgs
  mavros_msgs
  tf2
  tf2_geometry_msgs
)

# Install
install(TARGETS
  udp_mavros_bridge
  mavros_example
  udp_mavros_bridge_2
  udp_mavros_bridge_3
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

