#ifndef BOAT_BT__BOAT_BT_NODE_HPP_
#define BOAT_BT__BOAT_BT_NODE_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boat_bt/mission_monitor.hpp"
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

class BoatBTNode : public rclcpp::Node
{
public:
  BoatBTNode();

private:
  // =========================================================================
  // Behavior Tree registration
  // =========================================================================

  void register_bt_nodes();

  // =========================================================================
  // ROS callbacks
  // =========================================================================

  void odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg);

  void mission_status_callback(
    const njord_msgs::msg::MissionStatus::SharedPtr msg);

  void competition_status_callback(
    const njord_msgs::msg::CompetitionState::SharedPtr msg);

  void obstacles_callback(
    const njord_msgs::msg::ObstacleArray::SharedPtr msg);

  // =========================================================================
  // Task 1: cardinal-marker handling
  // =========================================================================

  void updateCardinalMarkerState(
    const njord_msgs::msg::ObstacleArray & msg);

  std::string cardinalTypeFromClassId(
    const std::string & class_id) const;

  bool cardinalMappingConfigured() const;

  // =========================================================================
  // Collision Avoidance
  // =========================================================================

  void updateCollisionRiskState(
    const njord_msgs::msg::ObstacleArray & msg);

  // =========================================================================
  // Bypass target helpers
  // =========================================================================

  geographic_msgs::msg::GeoPoint offsetGeoPoint(
    const geographic_msgs::msg::GeoPoint & origin,
    const std::string & direction,
    double offset_m) const;

  geographic_msgs::msg::GeoPoint offsetGeoPointRelativeToBoat(
    const geographic_msgs::msg::GeoPoint & origin,
    double heading_rad,
    const std::string & side,
    double offset_m) const;

  geographic_msgs::msg::GeoPoint offsetGeoPointENU(
    const geographic_msgs::msg::GeoPoint & origin,
    double east_m,
    double north_m) const;

  bool sendBypassRequest(
    const geographic_msgs::msg::GeoPoint & target,
    const std::string & reason);

  bool requestCooldownExpired(
    const rclcpp::Time & last_time) const;

  // =========================================================================
  // Main BT tick
  // =========================================================================

  void tick_tree();

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

#endif  // BOAT_BT__BOAT_BT_NODE_HPP_
