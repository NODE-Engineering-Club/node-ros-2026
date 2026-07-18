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
#include "njord_msgs/msg/obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


/**
 * @brief Monitors the mission lifecycle published by mission_manager.
 *
 * Returns:
 *   RUNNING -> no terminal mission state yet
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
    tree_finished_(false),
    cardinal_marker_detected_(false)
  {
    /*
     * YOLO currently publishes numeric class IDs as strings ("0"..."5").
     * Their semantic mapping is not documented in the repository yet.
     *
     * These parameters allow the mapping to be configured without
     * hard-coding potentially incorrect class IDs.
     */
    declare_parameter<std::string>(
      "cardinal_north_class_id",
      "");

    declare_parameter<std::string>(
      "cardinal_east_class_id",
      "");

    declare_parameter<std::string>(
      "cardinal_south_class_id",
      "");

    declare_parameter<std::string>(
      "cardinal_west_class_id",
      "");

    declare_parameter<double>(
      "cardinal_min_confidence",
      0.5);

    declare_parameter<double>(
      "cardinal_max_range_m",
      50.0);

    cardinal_north_class_id_ =
      get_parameter(
      "cardinal_north_class_id").as_string();

    cardinal_east_class_id_ =
      get_parameter(
      "cardinal_east_class_id").as_string();

    cardinal_south_class_id_ =
      get_parameter(
      "cardinal_south_class_id").as_string();

    cardinal_west_class_id_ =
      get_parameter(
      "cardinal_west_class_id").as_string();

    cardinal_min_confidence_ =
      get_parameter(
      "cardinal_min_confidence").as_double();

    cardinal_max_range_m_ =
      get_parameter(
      "cardinal_max_range_m").as_double();

    cmd_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10);

    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
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

    obstacles_sub_ =
      create_subscription<njord_msgs::msg::ObstacleArray>(
      "/obstacles/global",
      10,
      std::bind(
        &BoatBTNode::obstacles_callback,
        this,
        std::placeholders::_1));

    register_bt_nodes();

    const std::string xml_path =
      ament_index_cpp::get_package_share_directory(
      "boat_bt") +
      "/bt_xml/simple_boat.xml";

    tree_ =
      factory_.createTreeFromFile(
      xml_path);

    logger_ =
      std::make_unique<BT::StdCoutLogger>(
      tree_);

    timer_ =
      create_wall_timer(
      100ms,
      std::bind(
        &BoatBTNode::tick_tree,
        this));

    RCLCPP_INFO(
      get_logger(),
      "boat_bt_node started with tree: %s",
      xml_path.c_str());

    if (!cardinalMappingConfigured()) {
      RCLCPP_WARN(
        get_logger(),
        "Cardinal marker class mapping is not configured. "
        "Set cardinal_north_class_id, cardinal_east_class_id, "
        "cardinal_south_class_id and cardinal_west_class_id "
        "when the YOLO class mapping is known.");
    }
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

    /*
     * Task 1:
     * Check whether a valid cardinal marker has been detected
     * in the fused /obstacles/global stream.
     */
    factory_.registerSimpleCondition(
      "CardinalMarkerDetected",
      [this](BT::TreeNode &) {
        return cardinal_marker_detected_
          ? BT::NodeStatus::SUCCESS
          : BT::NodeStatus::FAILURE;
      });

    /*
     * Task 1:
     * Determine which side represents safe water according
     * to the detected cardinal marker.
     *
     * North cardinal -> pass north
     * East cardinal  -> pass east
     * South cardinal -> pass south
     * West cardinal  -> pass west
     *
     * This node currently determines the decision only.
     * A later navigation node will convert this decision
     * into an actual bypass waypoint / path adjustment.
     */
    factory_.registerSimpleAction(
      "DeterminePassingSide",
      [this](BT::TreeNode &) {
        if (!cardinal_marker_detected_) {
          return BT::NodeStatus::FAILURE;
        }

        if (detected_cardinal_type_.empty()) {
          return BT::NodeStatus::FAILURE;
        }

        passing_side_ =
          detected_cardinal_type_;

        RCLCPP_INFO(
          get_logger(),
          "Task 1 cardinal decision: marker=%s, "
          "safe passing side=%s",
          detected_cardinal_type_.c_str(),
          passing_side_.c_str());

        return BT::NodeStatus::SUCCESS;
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

        if (
          auto value =
            node.getInput<double>("speed"))
        {
          speed =
            value.value();
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
      static_cast<unsigned int>(
        msg->state),
      msg->message.c_str(),
      msg->current_waypoint,
      msg->total_waypoints);
  }

  void obstacles_callback(
    const njord_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    cardinal_marker_detected_ = false;
    detected_cardinal_type_.clear();
    detected_cardinal_class_id_.clear();

    float best_confidence = -1.0F;

    for (const auto & obstacle : msg->obstacles) {
      const std::string cardinal_type =
        cardinalTypeFromClassId(
        obstacle.class_id);

      if (cardinal_type.empty()) {
        continue;
      }

      if (
        obstacle.confidence <
        cardinal_min_confidence_)
      {
        continue;
      }

      /*
       * range_m can be zero when lidar confirmation is unavailable.
       * Only apply the maximum-range filter when a metric range exists.
       */
      if (
        obstacle.range_m > 0.0F &&
        obstacle.range_m >
        cardinal_max_range_m_)
      {
        continue;
      }

      if (
        obstacle.confidence >
        best_confidence)
      {
        best_confidence =
          obstacle.confidence;

        cardinal_marker_detected_ =
          true;

        detected_cardinal_type_ =
          cardinal_type;

        detected_cardinal_class_id_ =
          obstacle.class_id;
      }
    }

    if (cardinal_marker_detected_) {
      RCLCPP_DEBUG(
        get_logger(),
        "Cardinal marker detected: "
        "type=%s class_id=%s confidence=%.2f",
        detected_cardinal_type_.c_str(),
        detected_cardinal_class_id_.c_str(),
        best_confidence);
    }
  }

  std::string cardinalTypeFromClassId(
    const std::string & class_id) const
  {
    if (
      !cardinal_north_class_id_.empty() &&
      class_id ==
      cardinal_north_class_id_)
    {
      return "north";
    }

    if (
      !cardinal_east_class_id_.empty() &&
      class_id ==
      cardinal_east_class_id_)
    {
      return "east";
    }

    if (
      !cardinal_south_class_id_.empty() &&
      class_id ==
      cardinal_south_class_id_)
    {
      return "south";
    }

    if (
      !cardinal_west_class_id_.empty() &&
      class_id ==
      cardinal_west_class_id_)
    {
      return "west";
    }

    return "";
  }

  bool cardinalMappingConfigured() const
  {
    return
      !cardinal_north_class_id_.empty() ||
      !cardinal_east_class_id_.empty() ||
      !cardinal_south_class_id_.empty() ||
      !cardinal_west_class_id_.empty();
  }

  void tick_tree()
  {
    if (tree_finished_) {
      return;
    }

    /*
     * Do not start the mission tree until localization
     * is available.
     */
    if (!odom_received_) {
      return;
    }

    const BT::NodeStatus status =
      tree_.tickOnce();

    if (
      status ==
      BT::NodeStatus::SUCCESS)
    {
      tree_finished_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Behavior Tree completed successfully");

      return;
    }

    if (
      status ==
      BT::NodeStatus::FAILURE)
    {
      tree_finished_ = true;

      RCLCPP_ERROR(
        get_logger(),
        "Behavior Tree failed");

      return;
    }
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

  rclcpp::Subscription<
    njord_msgs::msg::ObstacleArray>::SharedPtr
    obstacles_sub_;

  rclcpp::TimerBase::SharedPtr
    timer_;

  bool odom_received_;
  bool mission_status_received_;

  uint8_t mission_state_;

  bool tree_finished_;

  /*
   * Task 1 - cardinal marker state.
   */
  bool cardinal_marker_detected_;

  std::string detected_cardinal_type_;
  std::string detected_cardinal_class_id_;
  std::string passing_side_;

  /*
   * Configurable perception class mapping.
   */
  std::string cardinal_north_class_id_;
  std::string cardinal_east_class_id_;
  std::string cardinal_south_class_id_;
  std::string cardinal_west_class_id_;

  double cardinal_min_confidence_;
  double cardinal_max_range_m_;

  BT::BehaviorTreeFactory
    factory_;

  BT::Tree
    tree_;

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

  rclcpp::spin(
    node);

  rclcpp::shutdown();

  return 0;
}