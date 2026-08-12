#include "boat_bt/boat_bt_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>


bool BoatBTNode::isBuoyClassId(
  const std::string & class_id) const
{
  return
    (!buoy_green_class_id_.empty() && class_id == buoy_green_class_id_) ||
    (!buoy_red_class_id_.empty() && class_id == buoy_red_class_id_);
}


void BoatBTNode::updateCollisionRiskState(
  const njord_msgs::msg::ObstacleArray & msg)
{
  collision_risk_detected_ = false;
  avoidance_target_ready_ = false;

  collision_obstacle_id_ = 0;
  collision_obstacle_range_m_ = 0.0;
  collision_obstacle_bearing_deg_ = 0.0;
  collision_obstacle_speed_mps_ = 0.0;

  double best_risk_score =
    std::numeric_limits<double>::infinity();

  bool best_is_buoy_standoff_violation = false;

  for (const auto & obstacle : msg.obstacles) {
    if (!obstacle.lidar_confirmed) {
      continue;
    }

    if (
      !std::isfinite(obstacle.range_m) ||
      obstacle.range_m <= 0.0F)
    {
      continue;
    }

    if (
      !std::isfinite(obstacle.bearing_deg) ||
      !std::isfinite(obstacle.speed_mps))
    {
      continue;
    }

    /*
     * Buoy minimum standoff (spec 9.1: "maintain safe distance from buoys
     * throughout course"). This always fires for a buoy-classed obstacle
     * within buoy_min_standoff_m_, regardless of forward-sector bearing,
     * the generic collision_risk_range_m_ cutoff, or closing speed — a
     * buoy grazing the hull to port or astern is just as much a standoff
     * violation as one dead ahead. It is scored to always outrank a
     * normal (non-violation) candidate in the same tick: normal risk
     * scores are always >= 0, so a fixed negative offset guarantees
     * priority, while still ranking multiple simultaneous violations by
     * range (the closer one wins).
     */
    const bool buoy_standoff_violation =
      isBuoyClassId(obstacle.class_id) &&
      obstacle.range_m <= buoy_min_standoff_m_;

    if (!buoy_standoff_violation) {
      if (
        obstacle.range_m >
        collision_risk_range_m_)
      {
        continue;
      }

      const double absolute_bearing_deg =
        std::abs(
        static_cast<double>(
          obstacle.bearing_deg));

      if (
        absolute_bearing_deg >
        collision_forward_sector_deg_)
      {
        continue;
      }

      /*
       * The current tracker exposes relative speed magnitude but not the
       * relative velocity vector. Therefore this remains a conservative
       * forward-corridor risk detector, not a complete COLREG/CPA
       * classifier.
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
    }

    double risk_score;

    if (buoy_standoff_violation) {
      // Always negative (guaranteed below any normal score computed
      // below, which is always >= 0), and ranks closer violations as
      // more urgent than farther ones.
      risk_score =
        static_cast<double>(obstacle.range_m) -
        1000.0;
    }
    else {
      /*
       * Lower score means greater urgency.
       *
       * Range is the primary factor. Obstacles nearer the forward
       * centreline receive a smaller score. Relative speed reduces the
       * score further, making a fast nearby obstacle more urgent than a
       * slow obstacle at a similar distance.
       */
      const double absolute_bearing_deg =
        std::abs(
        static_cast<double>(
          obstacle.bearing_deg));

      const double normalized_bearing =
        std::clamp(
        absolute_bearing_deg /
        std::max(
          collision_forward_sector_deg_,
          1.0),
        0.0,
        1.0);

      const double centreline_factor =
        1.0 +
        (0.75 * normalized_bearing);

      const double speed_factor =
        1.0 +
        std::max(
          0.0,
          static_cast<double>(
            obstacle.speed_mps));

      risk_score =
        (
          static_cast<double>(
            obstacle.range_m) *
          centreline_factor
        ) /
        speed_factor;
    }

    if (risk_score >= best_risk_score) {
      continue;
    }

    best_risk_score =
      risk_score;

    best_is_buoy_standoff_violation =
      buoy_standoff_violation;

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

  if (!collision_risk_detected_) {
    return;
  }

  if (best_is_buoy_standoff_violation) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "BUOY STANDOFF VIOLATED: id=%u, range=%.2f m "
      "(minimum %.2f m), bearing=%.2f deg",
      collision_obstacle_id_,
      collision_obstacle_range_m_,
      buoy_min_standoff_m_,
      collision_obstacle_bearing_deg_);
    return;
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "Collision risk selected: id=%u, range=%.2f m, "
    "bearing=%.2f deg, relative_speed=%.2f m/s, "
    "risk_score=%.3f",
    collision_obstacle_id_,
    collision_obstacle_range_m_,
    collision_obstacle_bearing_deg_,
    collision_obstacle_speed_mps_,
    best_risk_score);
}


namespace
{

double normalizeAngleDeg(double deg)
{
  double result = std::fmod(deg + 180.0, 360.0);

  if (result <= 0.0) {
    result += 360.0;
  }

  return result - 180.0;
}

double angularDifferenceDeg(double a_deg, double b_deg)
{
  return normalizeAngleDeg(a_deg - b_deg);
}

}  // namespace


bool BoatBTNode::findGatePair(
  const njord_msgs::msg::ObstacleArray & msg,
  double min_midpoint_range_m,
  geographic_msgs::msg::GeoPoint & out_left,
  geographic_msgs::msg::GeoPoint & out_right,
  double & out_midpoint_range_m) const
{
  bool found = false;

  double best_range_m =
    std::numeric_limits<double>::infinity();

  for (const auto & green : msg.obstacles) {
    if (!green.lidar_confirmed) {
      continue;
    }

    if (
      buoy_green_class_id_.empty() ||
      green.class_id != buoy_green_class_id_)
    {
      continue;
    }

    for (const auto & red : msg.obstacles) {
      if (!red.lidar_confirmed) {
        continue;
      }

      if (
        buoy_red_class_id_.empty() ||
        red.class_id != buoy_red_class_id_)
      {
        continue;
      }

      const EnuOffset spacing_delta =
        geoDeltaENU(green.position, red.position);

      const double spacing_m =
        std::hypot(spacing_delta.east_m, spacing_delta.north_m);

      if (
        spacing_m < gate_pair_min_spacing_m_ ||
        spacing_m > gate_pair_max_spacing_m_)
      {
        continue;
      }

      // Midpoint via plain lat/lon averaging -- fine at this scale (a few
      // metres between buoys), no need for great-circle math.
      geographic_msgs::msg::GeoPoint midpoint;
      midpoint.latitude =
        (green.position.latitude + red.position.latitude) / 2.0;
      midpoint.longitude =
        (green.position.longitude + red.position.longitude) / 2.0;

      const EnuOffset boat_to_midpoint =
        geoDeltaENU(current_boat_position_, midpoint);

      const double midpoint_range_m =
        std::hypot(boat_to_midpoint.east_m, boat_to_midpoint.north_m);

      if (midpoint_range_m <= min_midpoint_range_m) {
        continue;
      }

      if (midpoint_range_m >= best_range_m) {
        continue;
      }

      best_range_m = midpoint_range_m;
      found = true;
      out_midpoint_range_m = midpoint_range_m;

      // Assign left(port)/right(starboard) by boat-relative side at
      // detection time -- deliberately not a fixed colour->side convention,
      // the spec doesn't fix one for this course.
      const EnuOffset origin{0.0, 0.0};

      const EnuOffset forward{
        std::cos(current_boat_heading_),
        std::sin(current_boat_heading_)};

      const EnuOffset green_delta =
        geoDeltaENU(current_boat_position_, green.position);

      const double green_side =
        lineSide(origin, forward, green_delta);

      if (green_side > 0.0) {
        out_left = green.position;
        out_right = red.position;
      }
      else {
        out_left = red.position;
        out_right = green.position;
      }
    }
  }

  return found;
}


void BoatBTNode::updateGateState(
  const njord_msgs::msg::ObstacleArray & msg)
{
  // Corridor already complete -- freeze state for the rest of the attempt.
  if (gate2_crossed_) {
    return;
  }

  if (!gate1_found_) {
    if (
      findGatePair(
        msg, 0.0, gate1_left_, gate1_right_, gate1_midpoint_range_m_))
    {
      gate1_found_ = true;
      gate1_prev_side_valid_ = false;

      RCLCPP_INFO(
        get_logger(),
        "Gate 1 found: range=%.2f m",
        gate1_midpoint_range_m_);
    }
  }

  if (gate1_found_ && !gate1_crossed_) {
    const EnuOffset local_left{0.0, 0.0};

    const EnuOffset local_right =
      geoDeltaENU(gate1_left_, gate1_right_);

    const EnuOffset local_boat =
      geoDeltaENU(gate1_left_, current_boat_position_);

    const double side =
      lineSide(local_left, local_right, local_boat);

    if (
      gate1_prev_side_valid_ &&
      (side > 0.0) != (gate1_prev_side_m_ > 0.0))
    {
      gate1_crossed_ = true;

      RCLCPP_INFO(get_logger(), "Gate 1 crossed");
    }

    gate1_prev_side_m_ = side;
    gate1_prev_side_valid_ = true;
  }

  if (gate1_crossed_ && !gate2_found_) {
    const double min_midpoint_range_m =
      gate1_midpoint_range_m_ + gate_min_separation_m_;

    if (
      findGatePair(
        msg, min_midpoint_range_m, gate2_left_, gate2_right_,
        gate2_midpoint_range_m_))
    {
      gate2_found_ = true;
      gate2_prev_side_valid_ = false;

      RCLCPP_INFO(
        get_logger(),
        "Gate 2 found: range=%.2f m",
        gate2_midpoint_range_m_);
    }
  }

  if (gate2_found_ && !gate2_crossed_) {
    const EnuOffset local_left{0.0, 0.0};

    const EnuOffset local_right =
      geoDeltaENU(gate2_left_, gate2_right_);

    const EnuOffset local_boat =
      geoDeltaENU(gate2_left_, current_boat_position_);

    const double side =
      lineSide(local_left, local_right, local_boat);

    if (
      gate2_prev_side_valid_ &&
      (side > 0.0) != (gate2_prev_side_m_ > 0.0))
    {
      gate2_crossed_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Gate 2 crossed -- collision-avoidance corridor complete");
    }

    gate2_prev_side_m_ = side;
    gate2_prev_side_valid_ = true;
  }
}


void BoatBTNode::updateMarkerVesselState(
  const njord_msgs::msg::ObstacleArray & msg)
{
  // Per-tick fresh reset, same semantics as updateCollisionRiskState /
  // updateCardinalMarkerState -- a stale detection does not persist.
  marker_vessel_detected_ = false;

  marker_vessel_id_ = 0;
  marker_vessel_range_m_ = 0.0;
  marker_vessel_bearing_deg_ = 0.0;
  marker_vessel_speed_mps_ = 0.0;
  marker_vessel_velocity_bearing_deg_ = 0.0;

  double best_range_m =
    std::numeric_limits<double>::infinity();

  for (const auto & obstacle : msg.obstacles) {
    if (!obstacle.lidar_confirmed) {
      continue;
    }

    if (
      !std::isfinite(obstacle.range_m) ||
      obstacle.range_m <= 0.0F)
    {
      continue;
    }

    if (
      !std::isfinite(obstacle.bearing_deg) ||
      !std::isfinite(obstacle.speed_mps))
    {
      continue;
    }

    // The marker vessel has no matching YOLO class (only buoys and
    // cardinal markers are classified) -- it is identified purely
    // kinematically among the tracker's unclassified obstacles.
    if (isBuoyClassId(obstacle.class_id)) {
      continue;
    }

    if (!cardinalTypeFromClassId(obstacle.class_id).empty()) {
      continue;
    }

    if (
      obstacle.speed_mps <
      vessel_min_speed_mps_)
    {
      continue;
    }

    if (
      obstacle.range_m >
      vessel_max_range_m_)
    {
      continue;
    }

    const double absolute_bearing_deg =
      std::abs(static_cast<double>(obstacle.bearing_deg));

    if (
      absolute_bearing_deg >
      vessel_forward_sector_deg_ / 2.0)
    {
      continue;
    }

    if (obstacle.range_m >= best_range_m) {
      continue;
    }

    best_range_m = obstacle.range_m;

    marker_vessel_detected_ = true;
    marker_vessel_id_ = obstacle.id;
    marker_vessel_range_m_ = obstacle.range_m;
    marker_vessel_bearing_deg_ = obstacle.bearing_deg;
    marker_vessel_speed_mps_ = obstacle.speed_mps;
    marker_vessel_velocity_bearing_deg_ = obstacle.velocity_bearing_deg;
    marker_vessel_position_ = obstacle.position;
  }

  if (marker_vessel_detected_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Marker vessel candidate: id=%u range=%.2f m bearing=%.2f deg "
      "speed=%.2f m/s velocity_bearing=%.2f deg",
      marker_vessel_id_,
      marker_vessel_range_m_,
      marker_vessel_bearing_deg_,
      marker_vessel_speed_mps_,
      marker_vessel_velocity_bearing_deg_);
  }
}


std::string BoatBTNode::colregGiveWaySide(
  double obstacle_bearing_deg,
  double obstacle_velocity_bearing_deg) const
{
  /*
   * Pragmatic, documented simplification of COLREG Rules 14 (head-on) and
   * 15 (crossing) -- not a full CPA/judging-accurate classifier, same
   * honesty level as updateCollisionRiskState's own comment above. A
   * vessel is only treated as an active give-way situation if its
   * estimated relative-velocity direction is roughly converging back
   * toward the boat (within vessel_converging_deg_threshold_ of the
   * reciprocal bearing) -- a vessel crossing the path but moving away
   * again returns "none", leaving GlobalSafety's generic reflex as the
   * only remaining range-closing backstop.
   */
  const double reciprocal_deg =
    normalizeAngleDeg(obstacle_bearing_deg + 180.0);

  const double converging_error_deg =
    std::abs(
    angularDifferenceDeg(
      obstacle_velocity_bearing_deg,
      reciprocal_deg));

  const bool converging =
    converging_error_deg <=
    vessel_converging_deg_threshold_;

  if (!converging) {
    return "none";
  }

  if (obstacle_bearing_deg < -vessel_centreline_deadband_deg_) {
    // Vessel to starboard: Rule 15 give-way -- alter to starboard, pass
    // astern.
    return "starboard";
  }

  if (obstacle_bearing_deg > vessel_centreline_deadband_deg_) {
    // Vessel to port: nominally stand-on -- no action here, GlobalSafety's
    // generic reflex remains the backstop if range genuinely closes.
    return "port";
  }

  // Near dead-ahead: Rule 14 head-on, both vessels alter to starboard.
  return "starboard";
}


std::string BoatBTNode::oppositeSide(
  const std::string & side)
{
  if (side == "port") {
    return "starboard";
  }

  if (side == "starboard") {
    return "port";
  }

  return side;
}


void BoatBTNode::updateBuoyMarkerState(
  const njord_msgs::msg::ObstacleArray & msg)
{
  // Per-tick fresh reset, same semantics as the other update*State
  // functions -- a stale detection does not persist.
  buoy_marker_detected_ = false;

  detected_buoy_color_.clear();
  detected_buoy_id_ = 0;

  double best_range_m =
    std::numeric_limits<double>::infinity();

  for (const auto & obstacle : msg.obstacles) {
    if (!obstacle.lidar_confirmed) {
      continue;
    }

    if (
      !std::isfinite(obstacle.range_m) ||
      obstacle.range_m <= 0.0F)
    {
      continue;
    }

    if (!isBuoyClassId(obstacle.class_id)) {
      continue;
    }

    if (
      obstacle.range_m >
      buoy_colreg_max_range_m_)
    {
      continue;
    }

    if (obstacle.range_m >= best_range_m) {
      continue;
    }

    best_range_m = obstacle.range_m;

    buoy_marker_detected_ = true;
    detected_buoy_id_ = obstacle.id;
    detected_buoy_position_ = obstacle.position;

    detected_buoy_color_ =
      (!buoy_green_class_id_.empty() && obstacle.class_id == buoy_green_class_id_)
      ? "green"
      : "red";
  }

  if (buoy_marker_detected_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Buoy marker candidate: id=%u color=%s range=%.2f m",
      detected_buoy_id_,
      detected_buoy_color_.c_str(),
      best_range_m);
  }
}
