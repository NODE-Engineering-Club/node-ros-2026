cmake_minimum_required(VERSION 3.8)
project(njord_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartMission.srv"
  "srv/SetBypassTarget.srv"
  "srv/SetCompetitionTask.srv"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/MissionStatus.msg"
  "msg/CompetitionState.msg"
  "msg/DockTarget.msg"
  "msg/DockTargetArray.msg"
  DEPENDENCIES geographic_msgs geometry_msgs std_msgs
)

ament_package()
