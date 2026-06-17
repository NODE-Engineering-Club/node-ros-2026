cmake_minimum_required(VERSION 3.8)
project(boat_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(
  boat_bt_node
  src/boat_bt_node.cpp
)

ament_target_dependencies(
  boat_bt_node
  rclcpp
  nav_msgs
  geometry_msgs
  behaviortree_cpp
)

install(
  TARGETS boat_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()