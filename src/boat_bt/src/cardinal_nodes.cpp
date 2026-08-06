#include "boat_bt/boat_bt_node.hpp"


void BoatBTNode::updateCardinalMarkerState(
  const njord_msgs::msg::ObstacleArray & msg)
{
  cardinal_marker_detected_ = false;
  cardinal_target_ready_ = false;

  detected_cardinal_type_.clear();
  detected_cardinal_class_id_.clear();

  detected_cardinal_id_ = 0;

  float best_confidence = -1.0F;

  for (const auto & obstacle : msg.obstacles) {
    const std::string cardinal_type =
      cardinalTypeFromClassId(
      obstacle.class_id);

    if (cardinal_type.empty()) {
      continue;
    }

    if (
      obstacle.confidence <
      cardinal_min_confidence_)
    {
      continue;
    }

    if (
      obstacle.range_m > 0.0F &&
      obstacle.range_m >
      cardinal_max_range_m_)
    {
      continue;
    }

    if (
      obstacle.confidence >
      best_confidence)
    {
      best_confidence =
        obstacle.confidence;

      cardinal_marker_detected_ =
        true;

      detected_cardinal_type_ =
        cardinal_type;

      detected_cardinal_class_id_ =
        obstacle.class_id;

      detected_cardinal_id_ =
        obstacle.id;

      detected_cardinal_position_ =
        obstacle.position;
    }
  }
}


std::string BoatBTNode::cardinalTypeFromClassId(
  const std::string & class_id) const
{
  if (
    !cardinal_north_class_id_.empty() &&
    class_id == cardinal_north_class_id_)
  {
    return "north";
  }

  if (
    !cardinal_east_class_id_.empty() &&
    class_id == cardinal_east_class_id_)
  {
    return "east";
  }

  if (
    !cardinal_south_class_id_.empty() &&
    class_id == cardinal_south_class_id_)
  {
    return "south";
  }

  if (
    !cardinal_west_class_id_.empty() &&
    class_id == cardinal_west_class_id_)
  {
    return "west";
  }

  return "";
}


bool BoatBTNode::cardinalMappingConfigured() const
{
  return
    !cardinal_north_class_id_.empty() ||
    !cardinal_east_class_id_.empty() ||
    !cardinal_south_class_id_.empty() ||
    !cardinal_west_class_id_.empty();
}
