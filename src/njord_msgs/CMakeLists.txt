cmake_minimum_required(VERSION 3.8)
project(njord_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geographic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartMission.srv"
  DEPENDENCIES geographic_msgs
)

ament_package()
