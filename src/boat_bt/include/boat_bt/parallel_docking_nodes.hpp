#ifndef BOAT_BT__PARALLEL_DOCKING_NODES_HPP_
#define BOAT_BT__PARALLEL_DOCKING_NODES_HPP_

#include <functional>
#include <string>

#include "behaviortree_cpp/action_node.h"


namespace boat_bt
{

/**
 * Long-running BehaviorTree action for the parallel-docking controller
 * (Task 3.2). Same StatefulActionNode adapter pattern as
 * ExecuteDockingNode — see docking_nodes.hpp for the full rationale.
 *
 * The actual control logic remains inside BoatBTNode. This class only
 * adapts that controller to the BehaviorTree.CPP stateful-action
 * lifecycle.
 */
class ExecuteDockingParallelNode : public BT::StatefulActionNode
{
public:
  using TickCallback =
    std::function<BT::NodeStatus()>;

  using HaltCallback =
    std::function<void()>;

  ExecuteDockingParallelNode(
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

#endif  // BOAT_BT__PARALLEL_DOCKING_NODES_HPP_
