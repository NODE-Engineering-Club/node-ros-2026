#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "njord_msgs/msg/mission_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


/**
 * @brief Behavior Tree node that monitors the mission lifecycle.
 *
 * Returns:
 *   RUNNING  -> while no mission status has been received yet,
 *               or while mission state is IDLE / RUNNING
 *
 *   SUCCESS  -> when mission state is SUCCEEDED
 *
 *   FAILURE  -> when mission state is FAILED / ABORTED
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
    MissionStateCallback mission_state_callback)
  : BT::StatefulActionNode(name, config),
    status_received_callback_(std::move(status_received_callback)),
    mission_state_callback_(std::move(mission_state_callback))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    return evaluateMissionState();
  }

  BT::NodeStatus onRunning() override
  {
    return evaluateMissionState();
  }

  void onHalted() override
  {
  }

private:
  BT::NodeStatus evaluateMissionState()
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

    // IDLE and RUNNING are non-terminal mission states.
    return BT::NodeStatus::RUNNING;
  }

  StatusReceivedCallback status_received_callback_;
  MissionStateCallback mission_state_callback_;
};


class BoatBTNode : public rclcpp::Node
{
public:
  BoatBTNode()
  : Node("boat_bt_node"),
    odom_received_(false),
    mission_status_received_(false),
    mission_state_(njord_msgs::msg::MissionStatus::IDLE),
    tree_finished_(false)
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/gps",
      10,
      std::bind(
        &BoatBTNode::odom_callback,
        this,
        std::placeholders::_1));

    rclcpp::QoS mission_status_qos(1);
    mission_status_qos.transient_local();

    mission_status_sub_ =
      create_subscription<njord_msgs::msg::MissionStatus>(
      "/mission/status",
      mission_status_qos,
      std::bind(
        &BoatBTNode::mission_status_callback,
        this,
        std::placeholders::_1));

    register_bt_nodes();

    const std::string xml_path =
      ament_index_cpp::get_package_share_directory("boat_bt") +
      "/bt_xml/simple_boat.xml";

    tree_ = factory_.createTreeFromFile(xml_path);

    logger_ =
      std::make_unique<BT::StdCoutLogger>(tree_);

    timer_ = create_wall_timer(
      100ms,
      std::bind(
        &BoatBTNode::tick_tree,
        this));

    RCLCPP_INFO(
      get_logger(),
      "boat_bt_node started with tree: %s",
      xml_path.c_str());
  }

private:
  void register_bt_nodes()
  {
    factory_.registerSimpleCondition(
      "WaitForOdom",
      [this](BT::TreeNode &) {
        return odom_received_
          ? BT::NodeStatus::SUCCESS
          : BT::NodeStatus::FAILURE;
      });

    BT::NodeBuilder mission_monitor_builder =
      [this](
      const std::string & name,
      const BT::NodeConfig & config)
      {
        return std::make_unique<MissionMonitor>(
          name,
          config,
          [this]() {
            return mission_status_received_;
          },
          [this]() {
            return mission_state_;
          });
      };

    factory_.registerBuilder<MissionMonitor>(
      "MissionMonitor",
      mission_monitor_builder);

    factory_.registerSimpleAction(
      "MoveForward",
      [this](BT::TreeNode & node) {
        double speed = 0.5;

        if (auto value = node.getInput<double>("speed")) {
          speed = value.value();
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = speed;

        cmd_pub_->publish(cmd);

        RCLCPP_INFO(
          get_logger(),
          "MoveForward: speed=%.2f",
          speed);

        return BT::NodeStatus::SUCCESS;
      },
      {
        BT::InputPort<double>("speed")
      });

    factory_.registerSimpleAction(
      "StopBoat",
      [this](BT::TreeNode &) {
        geometry_msgs::msg::Twist cmd;

        cmd_pub_->publish(cmd);

        RCLCPP_INFO(
          get_logger(),
          "StopBoat");

        return BT::NodeStatus::SUCCESS;
      });
  }

  void odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    (void)msg;

    if (!odom_received_) {
      odom_received_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Received first odometry message");
    }
  }

  void mission_status_callback(
    const njord_msgs::msg::MissionStatus::SharedPtr msg)
  {
    mission_status_received_ = true;
    mission_state_ = msg->state;

    RCLCPP_INFO(
      get_logger(),
      "Mission status received: "
      "state=%u, message='%s', waypoint=%u/%u",
      static_cast<unsigned int>(msg->state),
      msg->message.c_str(),
      msg->current_waypoint,
      msg->total_waypoints);
  }

  void tick_tree()
  {
    if (tree_finished_) {
      return;
    }

    // Do not tick the mission tree until localization is available.
    if (!odom_received_) {
      return;
    }

    const BT::NodeStatus status =
      tree_.tickOnce();

    if (status == BT::NodeStatus::SUCCESS) {
      tree_finished_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Behavior Tree completed successfully");

      return;
    }

    if (status == BT::NodeStatus::FAILURE) {
      tree_finished_ = true;

      RCLCPP_ERROR(
        get_logger(),
        "Behavior Tree failed");

      return;
    }

    // RUNNING means the mission is still active or waiting to start.
  }

  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr
    cmd_pub_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;

  rclcpp::Subscription<
    njord_msgs::msg::MissionStatus>::SharedPtr
    mission_status_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool odom_received_;
  bool mission_status_received_;

  uint8_t mission_state_;

  bool tree_finished_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;

  std::unique_ptr<
    BT::StdCoutLogger>
    logger_;
};


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);

  auto node =
    std::make_shared<BoatBTNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}