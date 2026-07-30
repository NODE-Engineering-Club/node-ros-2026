#include <chrono>
#include <cmath>
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
#include "njord_msgs/msg/competition_state.hpp"
#include "njord_msgs/msg/mission_status.hpp"
#include "njord_msgs/msg/obstacle.hpp"
#include "njord_msgs/msg/obstacle_array.hpp"
#include "njord_msgs/srv/set_bypass_target.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


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
    competition_status_received_(false),
    competition_task_(njord_msgs::msg::CompetitionState::TASK_NONE),
    competition_state_(njord_msgs::msg::CompetitionState::STATE_IDLE),
    tree_finished_(false),
    cardinal_marker_detected_(false),
    cardinal_target_ready_(false),
    last_cardinal_request_id_(0),
    collision_risk_detected_(false),
    avoidance_target_ready_(false),
    last_avoidance_request_id_(0)
  {
    // -----------------------------------------------------------------------
    // Task 1: cardinal-marker class mapping
    // -----------------------------------------------------------------------

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

    declare_parameter<double>(
      "cardinal_bypass_offset_m",
      8.0);

    // -----------------------------------------------------------------------
    // Collision Avoidance configuration
    //
    // NOTE:
    // The current fusion tracker estimates velocity in base_link and does not
    // yet compensate for ego motion. Therefore this is intentionally a
    // relative-risk detector, not a full COLREG / CPA implementation.
    // -----------------------------------------------------------------------

    declare_parameter<double>(
      "collision_risk_range_m",
      20.0);

    declare_parameter<double>(
      "collision_forward_sector_deg",
      60.0);

    declare_parameter<double>(
      "collision_min_relative_speed_mps",
      0.15);

    declare_parameter<double>(
      "collision_avoidance_offset_m",
      12.0);

    declare_parameter<double>(
      "request_cooldown_sec",
      5.0);

    cardinal_north_class_id_ =
      get_parameter("cardinal_north_class_id").as_string();

    cardinal_east_class_id_ =
      get_parameter("cardinal_east_class_id").as_string();

    cardinal_south_class_id_ =
      get_parameter("cardinal_south_class_id").as_string();

    cardinal_west_class_id_ =
      get_parameter("cardinal_west_class_id").as_string();

    cardinal_min_confidence_ =
      get_parameter("cardinal_min_confidence").as_double();

    cardinal_max_range_m_ =
      get_parameter("cardinal_max_range_m").as_double();

    cardinal_bypass_offset_m_ =
      get_parameter("cardinal_bypass_offset_m").as_double();

    collision_risk_range_m_ =
      get_parameter("collision_risk_range_m").as_double();

    collision_forward_sector_deg_ =
      get_parameter("collision_forward_sector_deg").as_double();

    collision_min_relative_speed_mps_ =
      get_parameter("collision_min_relative_speed_mps").as_double();

    collision_avoidance_offset_m_ =
      get_parameter("collision_avoidance_offset_m").as_double();

    request_cooldown_sec_ =
      get_parameter("request_cooldown_sec").as_double();

    // -----------------------------------------------------------------------
    // ROS interfaces
    // -----------------------------------------------------------------------

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

    rclcpp::QoS competition_status_qos(1);
    competition_status_qos.transient_local();

    competition_status_sub_ =
      create_subscription<njord_msgs::msg::CompetitionState>(
      "/competition/status",
      competition_status_qos,
      std::bind(
        &BoatBTNode::competition_status_callback,
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

    bypass_client_ =
      create_client<njord_msgs::srv::SetBypassTarget>(
      "/mission/set_bypass_target");

    register_bt_nodes();

    const std::string xml_path =
      ament_index_cpp::get_package_share_directory("boat_bt") +
      "/bt_xml/simple_boat.xml";

    tree_ =
      factory_.createTreeFromFile(xml_path);

    logger_ =
      std::make_unique<BT::StdCoutLogger>(tree_);

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

    RCLCPP_INFO(
      get_logger(),
      "Collision avoidance relative-risk detector enabled: "
      "range=%.1f m, forward sector=+/-%.1f deg",
      collision_risk_range_m_,
      collision_forward_sector_deg_);
  }

private:
  // =========================================================================
  // Behavior Tree registration
  // =========================================================================

  void register_bt_nodes()
  {
    factory_.registerSimpleCondition(
      "WaitForOdom",
      [this](BT::TreeNode &) {
        return odom_received_
          ? BT::NodeStatus::SUCCESS
          : BT::NodeStatus::FAILURE;
      });

    // -----------------------------------------------------------------------
    // Competition task selection
    // -----------------------------------------------------------------------

    factory_.registerSimpleCondition(
      "IsTask",
      [this](BT::TreeNode & node) {
        const auto task_input =
          node.getInput<std::string>("task");

        if (!task_input) {
          RCLCPP_ERROR(
            get_logger(),
            "IsTask requires a 'task' input");

          return BT::NodeStatus::FAILURE;
        }

        if (!competition_status_received_) {
          return BT::NodeStatus::FAILURE;
        }

        const std::string task =
          task_input.value();

        uint8_t expected_task =
          njord_msgs::msg::CompetitionState::TASK_NONE;

        if (task == "maneuvering") {
          expected_task =
            njord_msgs::msg::CompetitionState::TASK_MANEUVERING;
        }
        else if (task == "path_finding") {
          expected_task =
            njord_msgs::msg::CompetitionState::TASK_PATH_FINDING;
        }
        else if (task == "collision_avoidance") {
          expected_task =
            njord_msgs::msg::CompetitionState::
            TASK_COLLISION_AVOIDANCE;
        }
        else if (task == "docking") {
          expected_task =
            njord_msgs::msg::CompetitionState::TASK_DOCKING;
        }
        else if (task == "surprise") {
          expected_task =
            njord_msgs::msg::CompetitionState::TASK_SURPRISE;
        }
        else if (task != "none") {
          RCLCPP_ERROR(
            get_logger(),
            "IsTask received unknown task: %s",
            task.c_str());

          return BT::NodeStatus::FAILURE;
        }

        return competition_task_ == expected_task
          ? BT::NodeStatus::SUCCESS
          : BT::NodeStatus::FAILURE;
      },
      {
        BT::InputPort<std::string>("task")
      });

    // -----------------------------------------------------------------------
    // Maneuvering / Path Finding
    // -----------------------------------------------------------------------

    factory_.registerSimpleCondition(
      "CardinalMarkerDetected",
      [this](BT::TreeNode &) {
        return cardinal_marker_detected_
          ? BT::NodeStatus::SUCCESS
          : BT::NodeStatus::FAILURE;
      });

    factory_.registerSimpleAction(
      "DeterminePassingSide",
      [this](BT::TreeNode &) {
        if (
          !cardinal_marker_detected_ ||
          detected_cardinal_type_.empty())
        {
          return BT::NodeStatus::FAILURE;
        }

        passing_side_ =
          detected_cardinal_type_;

        RCLCPP_INFO(
          get_logger(),
          "Cardinal marker decision: marker=%s, safe-water side=%s",
          detected_cardinal_type_.c_str(),
          passing_side_.c_str());

        return BT::NodeStatus::SUCCESS;
      });

    factory_.registerSimpleAction(
      "DetermineCardinalBypassTarget",
      [this](BT::TreeNode &) {
        if (
          !cardinal_marker_detected_ ||
          passing_side_.empty())
        {
          cardinal_target_ready_ = false;
          return BT::NodeStatus::FAILURE;
        }

        cardinal_bypass_target_ =
          offsetGeoPoint(
          detected_cardinal_position_,
          passing_side_,
          cardinal_bypass_offset_m_);

        cardinal_target_ready_ = true;

        RCLCPP_INFO(
          get_logger(),
          "Cardinal bypass target: "
          "lat=%.8f lon=%.8f side=%s",
          cardinal_bypass_target_.latitude,
          cardinal_bypass_target_.longitude,
          passing_side_.c_str());

        return BT::NodeStatus::SUCCESS;
      });

    factory_.registerSimpleAction(
      "RequestCardinalBypass",
      [this](BT::TreeNode &) {
        if (!cardinal_target_ready_) {
          return BT::NodeStatus::FAILURE;
        }

        if (
          detected_cardinal_id_ != 0 &&
          detected_cardinal_id_ == last_cardinal_request_id_ &&
          !requestCooldownExpired(last_cardinal_request_time_))
        {
          return BT::NodeStatus::SUCCESS;
        }

        const bool requested =
          sendBypassRequest(
          cardinal_bypass_target_,
          "cardinal_" + passing_side_);

        if (!requested) {
          return BT::NodeStatus::FAILURE;
        }

        last_cardinal_request_id_ =
          detected_cardinal_id_;

        last_cardinal_request_time_ =
          now();

        return BT::NodeStatus::SUCCESS;
      });

    // -----------------------------------------------------------------------
    // Collision Avoidance
    // -----------------------------------------------------------------------

    factory_.registerSimpleCondition(
      "CollisionRiskDetected",
      [this](BT::TreeNode &) {
        return collision_risk_detected_
          ? BT::NodeStatus::SUCCESS
          : BT::NodeStatus::FAILURE;
      });

    factory_.registerSimpleAction(
      "DetermineAvoidanceTarget",
      [this](BT::TreeNode &) {
        if (!collision_risk_detected_) {
          avoidance_target_ready_ = false;
          return BT::NodeStatus::FAILURE;
        }

        /*
         * Conservative relative-risk behavior:
         *
         * Until a validated COLREG/CPA layer exists, the default avoidance
         * direction is starboard/right.
         */
        avoidance_side_ = "starboard";

        avoidance_target_ =
          offsetGeoPointRelativeToBoat(
          current_boat_position_,
          current_boat_heading_,
          avoidance_side_,
          collision_avoidance_offset_m_);

        avoidance_target_ready_ = true;

        RCLCPP_WARN(
          get_logger(),
          "Collision avoidance target generated: "
          "obstacle_id=%u range=%.2f bearing=%.2f "
          "target=(%.8f, %.8f)",
          collision_obstacle_id_,
          collision_obstacle_range_m_,
          collision_obstacle_bearing_deg_,
          avoidance_target_.latitude,
          avoidance_target_.longitude);

        return BT::NodeStatus::SUCCESS;
      });

    factory_.registerSimpleAction(
      "RequestAvoidance",
      [this](BT::TreeNode &) {
        if (!avoidance_target_ready_) {
          return BT::NodeStatus::FAILURE;
        }

        if (
          collision_obstacle_id_ != 0 &&
          collision_obstacle_id_ == last_avoidance_request_id_ &&
          !requestCooldownExpired(last_avoidance_request_time_))
        {
          return BT::NodeStatus::SUCCESS;
        }

        const bool requested =
          sendBypassRequest(
          avoidance_target_,
          "collision_avoidance_relative_risk");

        if (!requested) {
          return BT::NodeStatus::FAILURE;
        }

        last_avoidance_request_id_ =
          collision_obstacle_id_;

        last_avoidance_request_time_ =
          now();

        return BT::NodeStatus::SUCCESS;
      });

    // -----------------------------------------------------------------------
    // Mission lifecycle
    // -----------------------------------------------------------------------

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

    // -----------------------------------------------------------------------
    // Utility actions retained from earlier development/testing
    // -----------------------------------------------------------------------

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

  // =========================================================================
  // ROS callbacks
  // =========================================================================

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

  void competition_status_callback(
    const njord_msgs::msg::CompetitionState::SharedPtr msg)
  {
    competition_status_received_ = true;
    competition_task_ = msg->task;
    competition_state_ = msg->state;

    RCLCPP_INFO(
      get_logger(),
      "Competition status received: "
      "task=%u, state=%u, message='%s'",
      static_cast<unsigned int>(msg->task),
      static_cast<unsigned int>(msg->state),
      msg->message.c_str());
  }

  void obstacles_callback(
    const njord_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    current_boat_position_ =
      msg->boat_position;

    current_boat_heading_ =
      msg->boat_heading;

    updateCardinalMarkerState(*msg);
    updateCollisionRiskState(*msg);
  }

  // =========================================================================
  // Task 1: cardinal-marker handling
  // =========================================================================

  void updateCardinalMarkerState(
    const njord_msgs::msg::ObstacleArray & msg)
  {
    cardinal_marker_detected_ = false;
    cardinal_target_ready_ = false;

    detected_cardinal_type_.clear();
    detected_cardinal_class_id_.clear();

    detected_cardinal_id_ = 0;

    float best_confidence = -1.0F;

    for (const auto & obstacle : msg.obstacles) {
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

        detected_cardinal_id_ =
          obstacle.id;

        detected_cardinal_position_ =
          obstacle.position;
      }
    }
  }

  std::string cardinalTypeFromClassId(
    const std::string & class_id) const
  {
    if (
      !cardinal_north_class_id_.empty() &&
      class_id == cardinal_north_class_id_)
    {
      return "north";
    }

    if (
      !cardinal_east_class_id_.empty() &&
      class_id == cardinal_east_class_id_)
    {
      return "east";
    }

    if (
      !cardinal_south_class_id_.empty() &&
      class_id == cardinal_south_class_id_)
    {
      return "south";
    }

    if (
      !cardinal_west_class_id_.empty() &&
      class_id == cardinal_west_class_id_)
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

  // =========================================================================
  // Collision Avoidance
  // =========================================================================

  void updateCollisionRiskState(
    const njord_msgs::msg::ObstacleArray & msg)
  {
    collision_risk_detected_ = false;
    avoidance_target_ready_ = false;

    collision_obstacle_id_ = 0;
    collision_obstacle_range_m_ = 0.0;
    collision_obstacle_bearing_deg_ = 0.0;
    collision_obstacle_speed_mps_ = 0.0;

    double best_risk_range =
      collision_risk_range_m_ + 1.0;

    for (const auto & obstacle : msg.obstacles) {
      if (!obstacle.lidar_confirmed) {
        continue;
      }

      if (obstacle.range_m <= 0.0F) {
        continue;
      }

      if (
        obstacle.range_m >
        collision_risk_range_m_)
      {
        continue;
      }

      if (
        std::abs(obstacle.bearing_deg) >
        collision_forward_sector_deg_)
      {
        continue;
      }

      /*
       * speed_mps is currently relative-track speed in base_link.
       *
       * A close obstacle is still considered a risk even if this speed is
       * below threshold, because static obstacles in the navigation corridor
       * must also be avoided.
       */
      const bool moving_risk =
        obstacle.speed_mps >=
        collision_min_relative_speed_mps_;

      const bool close_static_risk =
        obstacle.range_m <=
        (collision_risk_range_m_ * 0.5);

      if (
        !moving_risk &&
        !close_static_risk)
      {
        continue;
      }

      if (
        obstacle.range_m <
        best_risk_range)
      {
        best_risk_range =
          obstacle.range_m;

        collision_risk_detected_ =
          true;

        collision_obstacle_id_ =
          obstacle.id;

        collision_obstacle_range_m_ =
          obstacle.range_m;

        collision_obstacle_bearing_deg_ =
          obstacle.bearing_deg;

        collision_obstacle_speed_mps_ =
          obstacle.speed_mps;
      }
    }

    if (collision_risk_detected_) {
      RCLCPP_DEBUG(
        get_logger(),
        "Collision risk: id=%u range=%.2f m "
        "bearing=%.2f deg relative_speed=%.2f m/s",
        collision_obstacle_id_,
        collision_obstacle_range_m_,
        collision_obstacle_bearing_deg_,
        collision_obstacle_speed_mps_);
    }
  }

  // =========================================================================
  // Bypass target helpers
  // =========================================================================

  geographic_msgs::msg::GeoPoint offsetGeoPoint(
    const geographic_msgs::msg::GeoPoint & origin,
    const std::string & direction,
    const double offset_m) const
  {
    geographic_msgs::msg::GeoPoint target =
      origin;

    constexpr double metres_per_degree_lat =
      111320.0;

    const double latitude_rad =
      origin.latitude * M_PI / 180.0;

    double metres_per_degree_lon =
      metres_per_degree_lat *
      std::cos(latitude_rad);

    if (
      std::abs(metres_per_degree_lon) <
      1.0)
    {
      metres_per_degree_lon = 1.0;
    }

    if (direction == "north") {
      target.latitude +=
        offset_m /
        metres_per_degree_lat;
    }
    else if (direction == "south") {
      target.latitude -=
        offset_m /
        metres_per_degree_lat;
    }
    else if (direction == "east") {
      target.longitude +=
        offset_m /
        metres_per_degree_lon;
    }
    else if (direction == "west") {
      target.longitude -=
        offset_m /
        metres_per_degree_lon;
    }

    return target;
  }

  geographic_msgs::msg::GeoPoint offsetGeoPointRelativeToBoat(
    const geographic_msgs::msg::GeoPoint & origin,
    const double heading_rad,
    const std::string & side,
    const double offset_m) const
  {
    /*
     * boat_heading is ENU yaw:
     *   0 rad -> east
     *   +pi/2 -> north
     *
     * Starboard is -90 degrees relative to heading.
     * Port is +90 degrees relative to heading.
     */
    double direction_rad =
      heading_rad;

    if (side == "starboard") {
      direction_rad -=
        M_PI / 2.0;
    }
    else if (side == "port") {
      direction_rad +=
        M_PI / 2.0;
    }

    const double east_m =
      offset_m *
      std::cos(direction_rad);

    const double north_m =
      offset_m *
      std::sin(direction_rad);

    return offsetGeoPointENU(
      origin,
      east_m,
      north_m);
  }

  geographic_msgs::msg::GeoPoint offsetGeoPointENU(
    const geographic_msgs::msg::GeoPoint & origin,
    const double east_m,
    const double north_m) const
  {
    geographic_msgs::msg::GeoPoint target =
      origin;

    constexpr double metres_per_degree_lat =
      111320.0;

    const double latitude_rad =
      origin.latitude * M_PI / 180.0;

    double metres_per_degree_lon =
      metres_per_degree_lat *
      std::cos(latitude_rad);

    if (
      std::abs(metres_per_degree_lon) <
      1.0)
    {
      metres_per_degree_lon = 1.0;
    }

    target.latitude +=
      north_m /
      metres_per_degree_lat;

    target.longitude +=
      east_m /
      metres_per_degree_lon;

    return target;
  }

  bool sendBypassRequest(
    const geographic_msgs::msg::GeoPoint & target,
    const std::string & reason)
  {
    if (
      !bypass_client_->service_is_ready())
    {
      RCLCPP_WARN(
        get_logger(),
        "Bypass service /mission/set_bypass_target "
        "is not available yet");

      return false;
    }

    auto request =
      std::make_shared<
      njord_msgs::srv::SetBypassTarget::Request>();

    request->target =
      target;

    request->reason =
      reason;

    bypass_client_->async_send_request(
      request,
      [this, reason](
        rclcpp::Client<
          njord_msgs::srv::SetBypassTarget>::
          SharedFuture future)
      {
        try {
          const auto response =
            future.get();

          if (response->success) {
            RCLCPP_INFO(
              get_logger(),
              "Bypass request accepted: %s - %s",
              reason.c_str(),
              response->message.c_str());
          }
          else {
            RCLCPP_WARN(
              get_logger(),
              "Bypass request rejected: %s - %s",
              reason.c_str(),
              response->message.c_str());
          }
        }
        catch (
          const std::exception & e)
        {
          RCLCPP_ERROR(
            get_logger(),
            "Bypass service call failed: %s",
            e.what());
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "Requested bypass target: "
      "reason=%s lat=%.8f lon=%.8f",
      reason.c_str(),
      target.latitude,
      target.longitude);

    return true;
  }

  bool requestCooldownExpired(
    const rclcpp::Time & last_time) const
  {
    if (
      last_time.nanoseconds() == 0)
    {
      return true;
    }

    const double elapsed =
      (now() - last_time).seconds();

    return elapsed >=
      request_cooldown_sec_;
  }

  // =========================================================================
  // Main BT tick
  // =========================================================================

  void tick_tree()
  {
    if (tree_finished_) {
      return;
    }

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

  // =========================================================================
  // ROS interfaces
  // =========================================================================

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
    njord_msgs::msg::CompetitionState>::SharedPtr
    competition_status_sub_;

  rclcpp::Subscription<
    njord_msgs::msg::ObstacleArray>::SharedPtr
    obstacles_sub_;

  rclcpp::Client<
    njord_msgs::srv::SetBypassTarget>::SharedPtr
    bypass_client_;

  rclcpp::TimerBase::SharedPtr
    timer_;

  // =========================================================================
  // Mission state
  // =========================================================================

  bool odom_received_;
  bool mission_status_received_;

  uint8_t mission_state_;

  bool competition_status_received_;

  uint8_t competition_task_;
  uint8_t competition_state_;

  bool tree_finished_;

  // =========================================================================
  // Boat state from ObstacleArray
  // =========================================================================

  geographic_msgs::msg::GeoPoint
    current_boat_position_;

  double current_boat_heading_{0.0};

  // =========================================================================
  // Maneuvering / Path Finding state
  // =========================================================================

  bool cardinal_marker_detected_;

  std::string detected_cardinal_type_;
  std::string detected_cardinal_class_id_;
  std::string passing_side_;

  uint32_t detected_cardinal_id_{0};

  geographic_msgs::msg::GeoPoint
    detected_cardinal_position_;

  geographic_msgs::msg::GeoPoint
    cardinal_bypass_target_;

  bool cardinal_target_ready_;

  uint32_t last_cardinal_request_id_;

  rclcpp::Time
    last_cardinal_request_time_{0, 0, RCL_ROS_TIME};

  std::string cardinal_north_class_id_;
  std::string cardinal_east_class_id_;
  std::string cardinal_south_class_id_;
  std::string cardinal_west_class_id_;

  double cardinal_min_confidence_;
  double cardinal_max_range_m_;
  double cardinal_bypass_offset_m_;

  // =========================================================================
  // Collision Avoidance state
  // =========================================================================

  bool collision_risk_detected_;

  uint32_t collision_obstacle_id_{0};

  double collision_obstacle_range_m_{0.0};
  double collision_obstacle_bearing_deg_{0.0};
  double collision_obstacle_speed_mps_{0.0};

  std::string avoidance_side_;

  geographic_msgs::msg::GeoPoint
    avoidance_target_;

  bool avoidance_target_ready_;

  uint32_t last_avoidance_request_id_;

  rclcpp::Time
    last_avoidance_request_time_{0, 0, RCL_ROS_TIME};

  double collision_risk_range_m_;
  double collision_forward_sector_deg_;
  double collision_min_relative_speed_mps_;
  double collision_avoidance_offset_m_;

  double request_cooldown_sec_;

  // =========================================================================
  // BehaviorTree.CPP
  // =========================================================================

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