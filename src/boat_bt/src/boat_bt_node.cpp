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
