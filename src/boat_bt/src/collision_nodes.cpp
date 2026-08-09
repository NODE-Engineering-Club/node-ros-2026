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
