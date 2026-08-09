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
find_package(ament_index_cpp REQUIRED)
find_package(njord_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(
  boat_bt_node
  src/boat_bt_node.cpp
  src/bt_registration.cpp
  src/cardinal_nodes.cpp
  src/collision_nodes.cpp
  src/docking_nodes.cpp
  src/geo_helpers.cpp
  src/mission_monitor.cpp
  src/parallel_docking_nodes.cpp
)

target_include_directories(
  boat_bt_node
  PRIVATE
  include
)

ament_target_dependencies(
  boat_bt_node
  rclcpp
  nav_msgs
  geometry_msgs
  behaviortree_cpp
  ament_index_cpp
  njord_msgs
  std_srvs
)

install(
  TARGETS boat_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
