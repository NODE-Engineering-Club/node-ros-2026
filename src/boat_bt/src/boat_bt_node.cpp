#include "boat_bt/boat_bt_node.hpp"

using namespace std::chrono_literals;


BoatBTNode::BoatBTNode()
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
  dock_target_received_(false),
  dock_target_available_(false),
  docking_complete_(false),
  competition_completion_request_sent_(false),
  competition_completion_confirmed_(false),
  docking_state_(DockingState::WAITING_FOR_TARGET),
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

  // -----------------------------------------------------------------------
  // Docking configuration
  //
  // All important values are ROS parameters so the competition team can
  // tune the controller without editing or rebuilding the C++ source.
  // -----------------------------------------------------------------------

  declare_parameter<double>(
    "docking_min_confidence",
    0.45);

  declare_parameter<double>(
    "docking_target_timeout_sec",
    1.0);

  declare_parameter<double>(
    "docking_reacquire_timeout_sec",
    3.0);

  declare_parameter<double>(
    "docking_steering_hold_sec",
    0.3);

  declare_parameter<double>(
    "docking_alignment_tolerance_rad",
    0.20);

  declare_parameter<double>(
    "docking_entry_trigger_distance_m",
    1.5);

  declare_parameter<double>(
    "docking_lateral_tolerance_m",
    0.45);

  declare_parameter<double>(
    "docking_final_entry_duration_sec",
    2.5);

  declare_parameter<double>(
    "docking_hold_duration_sec",
    10.0);

  declare_parameter<double>(
    "docking_reverse_duration_sec",
    4.0);

  declare_parameter<double>(
    "docking_alignment_speed_mps",
    0.20);

  declare_parameter<double>(
    "docking_approach_speed_mps",
    0.45);

  declare_parameter<double>(
    "docking_final_speed_mps",
    0.20);

  declare_parameter<double>(
    "docking_reverse_speed_mps",
    -0.25);

  declare_parameter<double>(
    "docking_max_yaw_rate_radps",
    0.70);

  declare_parameter<double>(
    "docking_bearing_gain",
    1.20);

  declare_parameter<double>(
    "docking_heading_gain",
    0.40);

  // -----------------------------------------------------------------------
  // Read cardinal-marker parameters
  // -----------------------------------------------------------------------

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

  cardinal_bypass_offset_m_ =
    get_parameter(
    "cardinal_bypass_offset_m").as_double();

  // -----------------------------------------------------------------------
  // Read collision-avoidance parameters
  // -----------------------------------------------------------------------

  collision_risk_range_m_ =
    get_parameter(
    "collision_risk_range_m").as_double();

  collision_forward_sector_deg_ =
    get_parameter(
    "collision_forward_sector_deg").as_double();

  collision_min_relative_speed_mps_ =
    get_parameter(
    "collision_min_relative_speed_mps").as_double();

  collision_avoidance_offset_m_ =
    get_parameter(
    "collision_avoidance_offset_m").as_double();

  request_cooldown_sec_ =
    get_parameter(
    "request_cooldown_sec").as_double();

  // -----------------------------------------------------------------------
  // Read docking parameters
  // -----------------------------------------------------------------------

  docking_min_confidence_ =
    get_parameter(
    "docking_min_confidence").as_double();

  docking_target_timeout_sec_ =
    get_parameter(
    "docking_target_timeout_sec").as_double();

  docking_reacquire_timeout_sec_ =
    get_parameter(
    "docking_reacquire_timeout_sec").as_double();

  docking_steering_hold_sec_ =
    get_parameter(
    "docking_steering_hold_sec").as_double();

  docking_alignment_tolerance_rad_ =
    get_parameter(
    "docking_alignment_tolerance_rad").as_double();

  docking_entry_trigger_distance_m_ =
    get_parameter(
    "docking_entry_trigger_distance_m").as_double();

  docking_lateral_tolerance_m_ =
    get_parameter(
    "docking_lateral_tolerance_m").as_double();

  docking_final_entry_duration_sec_ =
    get_parameter(
    "docking_final_entry_duration_sec").as_double();

  docking_hold_duration_sec_ =
    get_parameter(
    "docking_hold_duration_sec").as_double();

  docking_reverse_duration_sec_ =
    get_parameter(
    "docking_reverse_duration_sec").as_double();

  docking_alignment_speed_mps_ =
    get_parameter(
    "docking_alignment_speed_mps").as_double();

  docking_approach_speed_mps_ =
    get_parameter(
    "docking_approach_speed_mps").as_double();

  docking_final_speed_mps_ =
    get_parameter(
    "docking_final_speed_mps").as_double();

  docking_reverse_speed_mps_ =
    get_parameter(
    "docking_reverse_speed_mps").as_double();

  docking_max_yaw_rate_radps_ =
    get_parameter(
    "docking_max_yaw_rate_radps").as_double();

  docking_bearing_gain_ =
    get_parameter(
    "docking_bearing_gain").as_double();

  docking_heading_gain_ =
    get_parameter(
    "docking_heading_gain").as_double();

  // -----------------------------------------------------------------------
  // ROS interfaces
  // -----------------------------------------------------------------------

  /*
   * Published on a dedicated topic, not /cmd_vel directly: Nav2's own
   * pipeline (controller_server, behavior_server's recovery behaviors,
   * collision_monitor's safety-stop heartbeat) also targets /cmd_vel
   * whenever it's alive, even with no active goal. Publishing there
   * directly caused boat_bt's docking commands to race against Nav2's
   * idle-but-live output. twist_mux arbitrates the two into the real
   * /cmd_vel (see bringup/config/twist_mux.yaml).
   */
  cmd_pub_ =
    create_publisher<geometry_msgs::msg::Twist>(
    "/boat_bt/cmd_vel",
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

  dock_target_sub_ =
    create_subscription<njord_msgs::msg::DockTarget>(
    "/perception/dock_target",
    10,
    std::bind(
      &BoatBTNode::dock_target_callback,
      this,
      std::placeholders::_1));

  bypass_client_ =
    create_client<njord_msgs::srv::SetBypassTarget>(
    "/mission/set_bypass_target");

  competition_complete_client_ =
    create_client<std_srvs::srv::Trigger>(
    "/competition/complete");

  // -----------------------------------------------------------------------
  // Behavior Tree
  // -----------------------------------------------------------------------

  register_bt_nodes();

  const std::string xml_path =
    ament_index_cpp::get_package_share_directory(
    "boat_bt") +
    "/bt_xml/simple_boat.xml";

  tree_ =
    factory_.createTreeFromFile(xml_path);

  logger_ =
    std::make_unique<BT::StdCoutLogger>(
    tree_);

  timer_ =
    create_wall_timer(
    100ms,
    std::bind(
      &BoatBTNode::tick_tree,
      this));

  // -----------------------------------------------------------------------
  // Startup information
  // -----------------------------------------------------------------------

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

  RCLCPP_INFO(
    get_logger(),
    "Docking controller enabled: confidence>=%.2f, "
    "target timeout=%.2f s, reacquire timeout=%.2f s, "
    "steering hold=%.2f s, "
    "approach speed=%.2f m/s, final speed=%.2f m/s, "
    "max yaw=%.2f rad/s",
    docking_min_confidence_,
    docking_target_timeout_sec_,
    docking_reacquire_timeout_sec_,
    docking_steering_hold_sec_,
    docking_approach_speed_mps_,
    docking_final_speed_mps_,
    docking_max_yaw_rate_radps_);
}


void BoatBTNode::odom_callback(
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


void BoatBTNode::mission_status_callback(
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


void BoatBTNode::competition_status_callback(
  const njord_msgs::msg::CompetitionState::SharedPtr msg)
{
  const uint8_t previous_task =
    competition_task_;

  const uint8_t previous_state =
    competition_state_;

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

  /*
   * Reset docking only when a new docking run begins.
   *
   * This allows the same node process to be reused for multiple competition
   * attempts without retaining the previous DOCKED state.
   */
  const bool docking_task_started =
    competition_task_ ==
    njord_msgs::msg::CompetitionState::TASK_DOCKING &&
    competition_state_ ==
    njord_msgs::msg::CompetitionState::STATE_RUNNING &&
    (
      previous_task !=
      njord_msgs::msg::CompetitionState::TASK_DOCKING ||
      previous_state !=
      njord_msgs::msg::CompetitionState::STATE_RUNNING
    );

  if (docking_task_started) {
    tree_finished_ = false;
    competition_completion_request_sent_ = false;
    competition_completion_confirmed_ = false;
    resetDockingController();

    RCLCPP_INFO(
      get_logger(),
      "New docking competition run started");
  }

  const bool docking_task_succeeded =
    competition_task_ ==
    njord_msgs::msg::CompetitionState::TASK_DOCKING &&
    competition_state_ ==
    njord_msgs::msg::CompetitionState::STATE_SUCCEEDED;

  if (docking_task_succeeded) {
    competition_completion_confirmed_ = true;
    publishDockingCommand(0.0, 0.0);

    RCLCPP_INFO(
      get_logger(),
      "Docking completion confirmed through /competition/status");
  }

  /*
   * Stop the boat when docking is externally aborted or failed.
   */
  const bool docking_stopped =
    competition_task_ ==
    njord_msgs::msg::CompetitionState::TASK_DOCKING &&
    (
      competition_state_ ==
      njord_msgs::msg::CompetitionState::STATE_FAILED ||
      competition_state_ ==
      njord_msgs::msg::CompetitionState::STATE_ABORTED
    );

  if (docking_stopped) {
    publishDockingCommand(0.0, 0.0);

    RCLCPP_WARN(
      get_logger(),
      "Docking task stopped externally");
  }
}


void BoatBTNode::obstacles_callback(
  const njord_msgs::msg::ObstacleArray::SharedPtr msg)
{
  current_boat_position_ =
    msg->boat_position;

  current_boat_heading_ =
    msg->boat_heading;

  updateCardinalMarkerState(*msg);
  updateCollisionRiskState(*msg);
}


void BoatBTNode::tick_tree()
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

    publishDockingCommand(0.0, 0.0);

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

    publishDockingCommand(0.0, 0.0);

    RCLCPP_ERROR(
      get_logger(),
      "Behavior Tree failed");

    return;
  }
}


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
