cmake_minimum_required(VERSION 3.8)
project(px4_super_bridge)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(super_planner REQUIRED)

add_executable(bridge_node src/bridge_node.cpp)
ament_target_dependencies(bridge_node
  rclcpp geometry_msgs nav_msgs std_msgs px4_msgs super_planner
)

install(TARGETS bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
