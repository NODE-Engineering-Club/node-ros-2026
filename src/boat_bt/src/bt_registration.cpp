#include "boat_bt/boat_bt_node.hpp"
#include "boat_bt/docking_nodes.hpp"
#include "boat_bt/parallel_docking_nodes.hpp"


void BoatBTNode::register_bt_nodes()
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
      else if (task == "docking_parallel") {
        expected_task =
          njord_msgs::msg::CompetitionState::TASK_DOCKING_PARALLEL;
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
       * Choose the bypass side from the obstacle bearing.
       *
       * Positive bearing means the obstacle is on the boat's port/left side,
       * so the clearer bypass direction is starboard/right.
       *
       * Negative bearing means the obstacle is on the boat's
       * starboard/right side, so the clearer bypass direction is port/left.
       *
       * For an obstacle approximately straight ahead, keep starboard as the
       * conservative fallback until a full COLREG/CPA classifier is available.
       */
      constexpr double centreline_deadband_deg = 2.0;

      if (
        collision_obstacle_bearing_deg_ >
        centreline_deadband_deg)
      {
        avoidance_side_ = "starboard";
      }
      else if (
        collision_obstacle_bearing_deg_ <
        -centreline_deadband_deg)
      {
        avoidance_side_ = "port";
      }
      else {
        avoidance_side_ = "starboard";
      }

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
        "obstacle_id=%u range=%.2f bearing=%.2f side=%s "
        "target=(%.8f, %.8f)",
        collision_obstacle_id_,
        collision_obstacle_range_m_,
        collision_obstacle_bearing_deg_,
        avoidance_side_.c_str(),
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
  // Task 9.2: Collision Avoidance (gate-crossing + marker-vessel COLREG
  // give-way). Layers on top of GlobalSafety's generic reflex above,
  // exactly as ManeuveringTask/PathFindingTask layer cardinal-marker
  // handling on top of it -- see CollisionAvoidanceTask in simple_boat.xml.
  // -----------------------------------------------------------------------

  factory_.registerSimpleCondition(
    "MarkerVesselDetected",
    [this](BT::TreeNode &) {
      return
        (marker_vessel_detected_ && gate1_crossed_ && !gate2_crossed_)
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
    });

  factory_.registerSimpleAction(
    "DetermineGiveWaySide",
    [this](BT::TreeNode &) {
      if (!marker_vessel_detected_) {
        give_way_side_.clear();
        return BT::NodeStatus::FAILURE;
      }

      give_way_side_ =
        colregGiveWaySide(
        marker_vessel_bearing_deg_,
        marker_vessel_velocity_bearing_deg_);

      return
        give_way_side_ != "none"
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
    });

  factory_.registerSimpleAction(
    "DetermineGiveWayTarget",
    [this](BT::TreeNode &) {
      if (give_way_side_.empty() || give_way_side_ == "none") {
        give_way_target_ready_ = false;
        return BT::NodeStatus::FAILURE;
      }

      give_way_target_ =
        offsetGeoPointRelativeToBoat(
        current_boat_position_,
        current_boat_heading_,
        give_way_side_,
        vessel_giveway_offset_m_);

      give_way_target_ready_ = true;

      RCLCPP_WARN(
        get_logger(),
        "Give-way target generated: "
        "obstacle_id=%u side=%s target=(%.8f, %.8f)",
        marker_vessel_id_,
        give_way_side_.c_str(),
        give_way_target_.latitude,
        give_way_target_.longitude);

      return BT::NodeStatus::SUCCESS;
    });

  factory_.registerSimpleAction(
    "RequestGiveWayBypass",
    [this](BT::TreeNode &) {
      if (!give_way_target_ready_) {
        return BT::NodeStatus::FAILURE;
      }

      if (
        marker_vessel_id_ != 0 &&
        marker_vessel_id_ == last_give_way_request_id_ &&
        !requestCooldownExpired(last_give_way_request_time_))
      {
        return BT::NodeStatus::SUCCESS;
      }

      const bool requested =
        sendBypassRequest(
        give_way_target_,
        "collision_avoidance_colreg_giveway");

      if (!requested) {
        return BT::NodeStatus::FAILURE;
      }

      last_give_way_request_id_ =
        marker_vessel_id_;

      last_give_way_request_time_ =
        now();

      return BT::NodeStatus::SUCCESS;
    });

  // -----------------------------------------------------------------------
  // Docking
  // -----------------------------------------------------------------------

  factory_.registerSimpleCondition(
    "DockTargetAvailable",
    [this](BT::TreeNode &) {
      return dockTargetFresh()
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
    });

  factory_.registerSimpleAction(
    "ReportDockTarget",
    [this](BT::TreeNode &) {
      if (!dockTargetFresh()) {
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_INFO(
        get_logger(),
        "Docking target selected: center=(%.2f, %.2f), "
        "heading=%.2f rad, width=%.2f m, depth=%.2f m, "
        "confidence=%.2f",
        dock_opening_center_.x,
        dock_opening_center_.y,
        dock_heading_,
        dock_width_,
        dock_depth_,
        dock_confidence_);

      return BT::NodeStatus::SUCCESS;
    });

  /*
   * Long-running docking action.
   *
   * A StatefulActionNode is required because docking remains RUNNING across
   * many Behavior Tree ticks. A synchronous action is not allowed to return
   * RUNNING in BehaviorTree.CPP.
   */
  BT::NodeBuilder execute_docking_builder =
    [this](
    const std::string & name,
    const BT::NodeConfig & config)
    {
      return std::make_unique<boat_bt::ExecuteDockingNode>(
        name,
        config,
        [this]() {
          return executeDockingController();
        },
        [this]() {
          publishDockingCommand(0.0, 0.0);

          RCLCPP_WARN(
            get_logger(),
            "ExecuteDocking halted. Boat stop command published.");
        });
    };

  factory_.registerBuilder<boat_bt::ExecuteDockingNode>(
    "ExecuteDocking",
    execute_docking_builder);

  /*
   * Long-running parallel-docking action (Task 3.2) — same StatefulActionNode
   * reasoning as ExecuteDocking above. UNVERIFIED, see
   * parallel_docking_nodes.cpp's file header.
   */
  BT::NodeBuilder execute_docking_parallel_builder =
    [this](
    const std::string & name,
    const BT::NodeConfig & config)
    {
      return std::make_unique<boat_bt::ExecuteDockingParallelNode>(
        name,
        config,
        [this]() {
          return executeDockingParallelController();
        },
        [this]() {
          publishDockingParallelCommand(0.0, 0.0);

          RCLCPP_WARN(
            get_logger(),
            "ExecuteDockingParallel halted. Boat stop command published.");
        });
    };

  factory_.registerBuilder<boat_bt::ExecuteDockingParallelNode>(
    "ExecuteDockingParallel",
    execute_docking_parallel_builder);

  // -----------------------------------------------------------------------
  // Mission lifecycle
  // -----------------------------------------------------------------------

  BT::NodeBuilder mission_monitor_builder =
    [this](
    const std::string & name,
    const BT::NodeConfig & config)
    {
      return std::make_unique<boat_bt::MissionMonitor>(
        name,
        config,
        [this]() {
          return mission_status_received_;
        },
        [this]() {
          return mission_state_;
        });
    };

  factory_.registerBuilder<boat_bt::MissionMonitor>(
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
