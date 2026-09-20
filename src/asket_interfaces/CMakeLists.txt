cmake_minimum_required(VERSION 3.8)
project(asket_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SonarStatus.msg"
  "msg/MissionState.msg"
  "msg/PicoStatus.msg"
  "msg/LinkStatus.msg"
  "msg/HeadingStatus.msg"
  "msg/SystemTestItem.msg"
  "msg/SystemTestReport.msg"
  "srv/SetPingParameters.srv"
  "srv/SetMode.srv"
  "srv/StartMission.srv"
  "srv/StopMission.srv"
  "srv/ListMissions.srv"
  "srv/ExportMission.srv"
  "srv/DeleteMission.srv"
  "srv/RunSystemTest.srv"
  DEPENDENCIES std_msgs
)

ament_package()
