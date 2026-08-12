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
#include "njord_msgs/msg/dock_target.hpp"
#include "njord_msgs/msg/mission_status.hpp"
#include "njord_msgs/msg/obstacle.hpp"
#include "njord_msgs/msg/obstacle_array.hpp"
#include "njord_msgs/msg/wall_target.hpp"
#include "njord_msgs/srv/set_bypass_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// A local-tangent-plane point/offset in metres (east, north), used for
// gate-crossing geometry (Task 9.2) where straight-line math is simpler done
// in a flat local frame than directly in lat/lon.
struct EnuOffset
{
  double east_m{0.0};
  double north_m{0.0};
};

class BoatBTNode : public rclcpp::Node
{
public:
  BoatBTNode();

private:
  // =========================================================================
  // Docking controller states
  //
  // The states are deliberately explicit so the competition team can
  // understand the current phase from ROS logs.
  // =========================================================================

  enum class DockingState
  {
    WAITING_FOR_TARGET,
    ALIGNING,
    APPROACHING,
    FINAL_ENTRY,
    DOCKED
  };

  // =========================================================================
  // Parallel-docking controller states (Task 3.2)
  //
  // Same phase structure as DockingState, retargeted from "enter a U-shaped
  // opening" to "come alongside and lie parallel to a wall": FINAL_ENTRY
  // becomes FINAL_APPROACH (closing the last of the standoff gap against
  // the wall instead of moving through an opening), otherwise identical.
  // =========================================================================

  enum class DockingParallelState
  {
    WAITING_FOR_TARGET,
    ALIGNING,
    APPROACHING,
    FINAL_APPROACH,
    DOCKED
  };

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

  void dock_target_callback(
    const njord_msgs::msg::DockTarget::SharedPtr msg);

  void wall_target_callback(
    const njord_msgs::msg::WallTarget::SharedPtr msg);

  // =========================================================================
  // Task 1: cardinal-marker handling
  // =========================================================================

  void updateCardinalMarkerState(
    const njord_msgs::msg::ObstacleArray & msg);

  std::string cardinalTypeFromClassId(
    const std::string & class_id) const;

  bool cardinalMappingConfigured() const;

  bool isBuoyClassId(
    const std::string & class_id) const;

  // =========================================================================
  // Docking controller
  // =========================================================================

  BT::NodeStatus executeDockingController();

  void setDockingState(
    DockingState new_state);

  void resetDockingController();

  void publishDockingCommand(
    double forward_speed,
    double yaw_rate);

  void requestCompetitionCompletion();

  bool dockTargetFresh() const;

  const char * dockingStateName(
    DockingState state) const;

  // =========================================================================
  // Parallel-docking controller (Task 3.2)
  //
  // UNVERIFIED: this state machine has never been run against a real wall,
  // on the bench or in the water — see parallel_docking_nodes.cpp's file
  // header for the full caveat. It reuses executeDockingController's
  // bearing+heading blended steering law, retargeted at a wall-aligned
  // standoff point instead of a U-opening.
  // =========================================================================

  BT::NodeStatus executeDockingParallelController();

  void setDockingParallelState(
    DockingParallelState new_state);

  void resetDockingParallelController();

  void publishDockingParallelCommand(
    double forward_speed,
    double yaw_rate);

  bool wallTargetFresh() const;

  const char * dockingParallelStateName(
    DockingParallelState state) const;

  // =========================================================================
  // Collision Avoidance
  // =========================================================================

  void updateCollisionRiskState(
    const njord_msgs::msg::ObstacleArray & msg);

  // =========================================================================
  // Task 9.2: gate-crossing + marker-vessel COLREG give-way
  // =========================================================================

  void updateGateState(
    const njord_msgs::msg::ObstacleArray & msg);

  // Shared search used for both gate 1 and gate 2: finds the nearest valid
  // green+red buoy pair (spacing within [gate_pair_min_spacing_m_,
  // gate_pair_max_spacing_m_]) whose midpoint range exceeds min_midpoint_range_m
  // (0.0 for gate 1; gate1's own midpoint range + gate_min_separation_m_ for
  // gate 2, so gate 1's buoys can't be re-matched as gate 2). Returns false if
  // no valid pair is found.
  bool findGatePair(
    const njord_msgs::msg::ObstacleArray & msg,
    double min_midpoint_range_m,
    geographic_msgs::msg::GeoPoint & out_left,
    geographic_msgs::msg::GeoPoint & out_right,
    double & out_midpoint_range_m) const;

  void updateMarkerVesselState(
    const njord_msgs::msg::ObstacleArray & msg);

  std::string colregGiveWaySide(
    double obstacle_bearing_deg,
    double obstacle_velocity_bearing_deg) const;

  // =========================================================================
  // Task 9.4: individual buoy COLREG-side handling (Surprise)
  //
  // Distinct from updateGateState above: that logic pairs buoys into gates
  // and assigns left/right purely by boat-relative side at detection time
  // (no colour convention, per its own comment -- correct for Task 9.2's
  // own course, which doesn't fix one). Surprise's spec does fix one ("keep
  // red to port / green to starboard"), and buoys here are handled
  // individually, not as matched pairs.
  // =========================================================================

  void updateBuoyMarkerState(
    const njord_msgs::msg::ObstacleArray & msg);

  static std::string oppositeSide(
    const std::string & side);

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

  // Inverse of offsetGeoPointENU: the (east_m, north_m) of `to` relative to
  // `from`, using the same metres_per_degree constants for numeric
  // consistency. Used by Task 9.2's gate-crossing geometry (collision_nodes.cpp).
  EnuOffset geoDeltaENU(
    const geographic_msgs::msg::GeoPoint & from,
    const geographic_msgs::msg::GeoPoint & to) const;

  // Signed perpendicular distance of `point` from the infinite line through
  // line_start->line_end (right-hand rule). A sign flip between consecutive
  // calls (same line, boat position each tick) means the line was crossed.
  double lineSide(
    const EnuOffset & line_start,
    const EnuOffset & line_end,
    const EnuOffset & point) const;

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

  rclcpp::Subscription<
    njord_msgs::msg::DockTarget>::SharedPtr
    dock_target_sub_;

  rclcpp::Subscription<
    njord_msgs::msg::WallTarget>::SharedPtr
    wall_target_sub_;

  rclcpp::Client<
    njord_msgs::srv::SetBypassTarget>::SharedPtr
    bypass_client_;

  rclcpp::Client<
    std_srvs::srv::Trigger>::SharedPtr
    competition_complete_client_;

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
  // Docking target and controller state
  // =========================================================================

  bool dock_target_received_;
  bool dock_target_available_;
  bool docking_complete_;

  /*
   * The controller remains RUNNING after the physical docking manoeuvre
   * until Competition Manager acknowledges /competition/complete.
   */
  bool competition_completion_request_sent_;
  bool competition_completion_confirmed_;

  DockingState docking_state_;

  geometry_msgs::msg::Point
    dock_opening_center_;

  double dock_heading_{0.0};
  double dock_width_{0.0};
  double dock_depth_{0.0};
  double dock_confidence_{0.0};

  rclcpp::Time
    last_dock_target_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Time
    docking_target_loss_start_time_{0, 0, RCL_ROS_TIME};

  bool docking_target_loss_active_{false};

  rclcpp::Time
    final_entry_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Time
    docking_hold_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Time
    docking_reverse_start_time_{0, 0, RCL_ROS_TIME};

  bool docking_reverse_started_{false};

  // Perception filtering
  double docking_min_confidence_;
  double docking_target_timeout_sec_;
  double docking_reacquire_timeout_sec_;

  /*
   * How long a single dock_target reading is trusted to keep driving a
   * steering correction before the boat holds (zero yaw) and waits for a
   * fresh one. Without this, a low perception update rate (e.g. software-
   * rendered sim, a dropped frame) lets the same stale bearing keep
   * commanding a turn for many BT ticks in a row, overshooting well past
   * where the boat should have stopped turning -- confirmed in practice:
   * at ~1 Hz updates the boat overshot roughly 80 degrees chasing a single
   * reading. At normal sensor rates a fresh reading arrives well within
   * this window, so this has no effect there.
   */
  double docking_steering_hold_sec_;

  // State transition thresholds
  double docking_alignment_tolerance_rad_;
  double docking_entry_trigger_distance_m_;
  double docking_lateral_tolerance_m_;
  double docking_final_entry_duration_sec_;
  double docking_hold_duration_sec_;
  double docking_reverse_duration_sec_;

  // Motion parameters
  double docking_alignment_speed_mps_;
  double docking_approach_speed_mps_;
  double docking_final_speed_mps_;
  double docking_reverse_speed_mps_;
  double docking_max_yaw_rate_radps_;

  // Steering gains
  double docking_bearing_gain_;
  double docking_heading_gain_;

  // =========================================================================
  // Parallel-docking target and controller state (Task 3.2)
  // =========================================================================

  bool wall_target_received_;
  bool wall_target_available_;
  bool docking_parallel_complete_;

  DockingParallelState docking_parallel_state_;

  geometry_msgs::msg::Point
    wall_aim_point_;

  double wall_heading_{0.0};
  double wall_length_{0.0};
  double wall_confidence_{0.0};

  rclcpp::Time
    last_wall_target_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Time
    docking_parallel_target_loss_start_time_{0, 0, RCL_ROS_TIME};

  bool docking_parallel_target_loss_active_{false};

  rclcpp::Time
    final_approach_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Time
    docking_parallel_hold_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Time
    docking_parallel_reverse_start_time_{0, 0, RCL_ROS_TIME};

  bool docking_parallel_reverse_started_{false};

  // Perception filtering
  double docking_parallel_min_confidence_;
  double docking_parallel_target_timeout_sec_;
  double docking_parallel_reacquire_timeout_sec_;
  double docking_parallel_steering_hold_sec_;

  // State transition thresholds
  double docking_parallel_alignment_tolerance_rad_;
  double docking_parallel_approach_trigger_distance_m_;
  double docking_parallel_final_approach_duration_sec_;
  double docking_parallel_hold_duration_sec_;
  double docking_parallel_reverse_duration_sec_;

  // Motion parameters
  double docking_parallel_alignment_speed_mps_;
  double docking_parallel_approach_speed_mps_;
  double docking_parallel_final_speed_mps_;
  double docking_parallel_reverse_speed_mps_;
  double docking_parallel_max_yaw_rate_radps_;

  // Steering gains
  double docking_parallel_bearing_gain_;
  double docking_parallel_heading_gain_;

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

  /*
   * Buoy minimum standoff (spec 9.1: "maintain safe distance from buoys
   * throughout course"). Unlike the generic relative-risk detector above,
   * this always fires for a buoy-classed obstacle within
   * buoy_min_standoff_m_ regardless of forward-sector bearing or closing
   * speed, and always outranks any other candidate obstacle in the same
   * tick (see updateCollisionRiskState in collision_nodes.cpp).
   */
  std::string buoy_green_class_id_;
  std::string buoy_red_class_id_;
  double buoy_min_standoff_m_;

  // =========================================================================
  // Task 9.4: individual buoy COLREG-side state (Surprise)
  // =========================================================================

  bool buoy_marker_detected_{false};

  std::string detected_buoy_color_;
  uint32_t detected_buoy_id_{0};

  geographic_msgs::msg::GeoPoint
    detected_buoy_position_;

  std::string buoy_passing_side_;

  geographic_msgs::msg::GeoPoint
    buoy_bypass_target_;

  bool buoy_target_ready_{false};

  uint32_t last_buoy_request_id_{0};

  rclcpp::Time
    last_buoy_request_time_{0, 0, RCL_ROS_TIME};

  double buoy_colreg_max_range_m_;
  double buoy_bypass_offset_m_;
  std::string red_buoy_side_;

  // =========================================================================
  // Task 9.2: gate-crossing + marker-vessel state
  // =========================================================================

  // Gate state — a "gate" is a matched green+red buoy pair (spec: 5m apart).
  // gateN_left_/gateN_right_ are assigned by boat-relative bearing sign at
  // detection time (not a fixed colour->side convention — the spec doesn't
  // fix one for this course).
  bool gate1_found_{false};
  bool gate2_found_{false};
  bool gate1_crossed_{false};
  bool gate2_crossed_{false};

  geographic_msgs::msg::GeoPoint gate1_left_;
  geographic_msgs::msg::GeoPoint gate1_right_;
  geographic_msgs::msg::GeoPoint gate2_left_;
  geographic_msgs::msg::GeoPoint gate2_right_;

  double gate1_midpoint_range_m_{0.0};
  double gate2_midpoint_range_m_{0.0};

  bool gate1_prev_side_valid_{false};
  bool gate2_prev_side_valid_{false};
  double gate1_prev_side_m_{0.0};
  double gate2_prev_side_m_{0.0};

  double gate_pair_min_spacing_m_;
  double gate_pair_max_spacing_m_;
  double gate_min_separation_m_;

  // Marker-vessel state — "The Otter of Njord", identified as the nearest
  // moving, unclassified (non-buoy, non-cardinal) obstacle in the forward
  // sector (see updateMarkerVesselState in collision_nodes.cpp).
  bool marker_vessel_detected_{false};
  uint32_t marker_vessel_id_{0};
  double marker_vessel_range_m_{0.0};
  double marker_vessel_bearing_deg_{0.0};
  double marker_vessel_speed_mps_{0.0};
  double marker_vessel_velocity_bearing_deg_{0.0};
  geographic_msgs::msg::GeoPoint marker_vessel_position_;

  double vessel_min_speed_mps_;
  double vessel_max_range_m_;
  double vessel_forward_sector_deg_;
  double vessel_centreline_deadband_deg_;
  double vessel_converging_deg_threshold_;
  double vessel_giveway_offset_m_;

  // Give-way decision + bypass request state (mirrors avoidance_side_/
  // avoidance_target_/last_avoidance_request_* above, but scoped to the
  // Task 9.2 marker-vessel encounter specifically).
  std::string give_way_side_;

  geographic_msgs::msg::GeoPoint give_way_target_;

  bool give_way_target_ready_{false};

  uint32_t last_give_way_request_id_{0};

  rclcpp::Time
    last_give_way_request_time_{0, 0, RCL_ROS_TIME};

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
