#include "boat_bt/boat_bt_node.hpp"


void BoatBTNode::dock_target_callback(
  const njord_msgs::msg::DockTarget::SharedPtr msg)
{
  dock_target_received_ = true;

  dock_target_available_ =
    msg->detected &&
    !msg->occupied &&
    msg->confidence >= docking_min_confidence_;

  if (!dock_target_available_) {
    return;
  }

  dock_opening_center_ =
    msg->opening_center;

  dock_heading_ =
    msg->heading;

  dock_width_ =
    msg->width;

  dock_depth_ =
    msg->depth;

  dock_confidence_ =
    msg->confidence;

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "Free dock target: center=(%.2f, %.2f), "
    "heading=%.2f rad, width=%.2f m, depth=%.2f m, "
    "confidence=%.2f",
    dock_opening_center_.x,
    dock_opening_center_.y,
    dock_heading_,
    dock_width_,
    dock_depth_,
    dock_confidence_);
}
