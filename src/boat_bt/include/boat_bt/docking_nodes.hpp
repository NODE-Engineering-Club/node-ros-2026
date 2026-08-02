#ifndef BOAT_BT__DOCKING_NODES_HPP_
#define BOAT_BT__DOCKING_NODES_HPP_

#include <functional>
#include <string>

#include "behaviortree_cpp/action_node.h"


namespace boat_bt
{

/**
 * Long-running BehaviorTree action for the docking controller.
 *
 * BehaviorTree.CPP synchronous actions may only return SUCCESS or FAILURE.
 * Docking needs multiple control cycles, so this stateful node may return
 * RUNNING until the boat completes the final entry.
 *
 * The actual control logic remains inside BoatBTNode. This class only adapts
 * that controller to the BehaviorTree.CPP stateful-action lifecycle.
 */
class ExecuteDockingNode : public BT::StatefulActionNode
{
public:
  using TickCallback =
    std::function<BT::NodeStatus()>;

  using HaltCallback =
    std::function<void()>;

  ExecuteDockingNode(
    const std::string & name,
    const BT::NodeConfig & config,
    TickCallback tick_callback,
    HaltCallback halt_callback);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

  TickCallback tick_callback_;
  HaltCallback halt_callback_;
};

}  // namespace boat_bt

#endif  // BOAT_BT__DOCKING_NODES_HPP_
