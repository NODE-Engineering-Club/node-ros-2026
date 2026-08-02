#include "boat_bt/boat_bt_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>


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
     * forward-corridor risk detector, not a complete COLREG/CPA classifier.
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

    /*
     * Lower score means greater urgency.
     *
     * Range is the primary factor. Obstacles nearer the forward centreline
     * receive a smaller score. Relative speed reduces the score further,
     * making a fast nearby obstacle more urgent than a slow obstacle at a
     * similar distance.
     */
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

    const double risk_score =
      (
        static_cast<double>(
          obstacle.range_m) *
        centreline_factor
      ) /
      speed_factor;

    if (risk_score >= best_risk_score) {
      continue;
    }

    best_risk_score =
      risk_score;

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
