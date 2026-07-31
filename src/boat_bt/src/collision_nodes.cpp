#include "boat_bt/boat_bt_node.hpp"


void BoatBTNode::updateCollisionRiskState(
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
