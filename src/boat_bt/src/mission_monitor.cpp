#include "boat_bt/mission_monitor.hpp"

#include <utility>

#include "njord_msgs/msg/mission_status.hpp"

namespace boat_bt
{

MissionMonitor::MissionMonitor(
  const std::string & name,
  const BT::NodeConfig & config,
  StatusReceivedCallback status_received_callback,
  MissionStateCallback mission_state_callback)
: BT::StatefulActionNode(name, config),
  status_received_callback_(std::move(status_received_callback)),
  mission_state_callback_(std::move(mission_state_callback))
{
}

BT::PortsList MissionMonitor::providedPorts()
{
  return {};
}

BT::NodeStatus MissionMonitor::onStart()
{
  return evaluateMissionState();
}

BT::NodeStatus MissionMonitor::onRunning()
{
  return evaluateMissionState();
}

void MissionMonitor::onHalted()
{
}

BT::NodeStatus MissionMonitor::evaluateMissionState()
{
  if (!status_received_callback_()) {
    return BT::NodeStatus::RUNNING;
  }

  const uint8_t state = mission_state_callback_();

  if (state == njord_msgs::msg::MissionStatus::SUCCEEDED) {
    return BT::NodeStatus::SUCCESS;
  }

  if (
    state == njord_msgs::msg::MissionStatus::FAILED ||
    state == njord_msgs::msg::MissionStatus::ABORTED)
  {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace boat_bt
