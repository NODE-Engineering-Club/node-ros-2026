#ifndef BOAT_BT__MISSION_MONITOR_HPP_
#define BOAT_BT__MISSION_MONITOR_HPP_

#include <cstdint>
#include <functional>
#include <string>

#include "behaviortree_cpp/action_node.h"

namespace boat_bt
{

/**
 * @brief Monitors the mission lifecycle published by mission_manager.
 *
 * Returns:
 *   RUNNING -> mission not finished yet
 *   SUCCESS -> mission SUCCEEDED
 *   FAILURE -> mission FAILED or ABORTED
 */
class MissionMonitor : public BT::StatefulActionNode
{
public:
  using StatusReceivedCallback = std::function<bool()>;
  using MissionStateCallback = std::function<uint8_t()>;

  MissionMonitor(
    const std::string & name,
    const BT::NodeConfig & config,
    StatusReceivedCallback status_received_callback,
    MissionStateCallback mission_state_callback);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus evaluateMissionState();

  StatusReceivedCallback status_received_callback_;
  MissionStateCallback mission_state_callback_;
};

}  // namespace boat_bt

#endif  // BOAT_BT__MISSION_MONITOR_HPP_
