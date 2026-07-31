#include "boat_bt/boat_bt_node.hpp"


geographic_msgs::msg::GeoPoint BoatBTNode::offsetGeoPoint(
  const geographic_msgs::msg::GeoPoint & origin,
  const std::string & direction,
  const double offset_m) const
{
  geographic_msgs::msg::GeoPoint target =
    origin;

  constexpr double metres_per_degree_lat =
    111320.0;

  const double latitude_rad =
    origin.latitude * M_PI / 180.0;

  double metres_per_degree_lon =
    metres_per_degree_lat *
    std::cos(latitude_rad);

  if (
    std::abs(metres_per_degree_lon) <
    1.0)
  {
    metres_per_degree_lon = 1.0;
  }

  if (direction == "north") {
    target.latitude +=
      offset_m /
      metres_per_degree_lat;
  }
  else if (direction == "south") {
    target.latitude -=
      offset_m /
      metres_per_degree_lat;
  }
  else if (direction == "east") {
    target.longitude +=
      offset_m /
      metres_per_degree_lon;
  }
  else if (direction == "west") {
    target.longitude -=
      offset_m /
      metres_per_degree_lon;
  }

  return target;
}


geographic_msgs::msg::GeoPoint BoatBTNode::offsetGeoPointRelativeToBoat(
  const geographic_msgs::msg::GeoPoint & origin,
  const double heading_rad,
  const std::string & side,
  const double offset_m) const
{
  /*
   * boat_heading is ENU yaw:
   *   0 rad -> east
   *   +pi/2 -> north
   *
   * Starboard is -90 degrees relative to heading.
   * Port is +90 degrees relative to heading.
   */
  double direction_rad =
    heading_rad;

  if (side == "starboard") {
    direction_rad -=
      M_PI / 2.0;
  }
  else if (side == "port") {
    direction_rad +=
      M_PI / 2.0;
  }

  const double east_m =
    offset_m *
    std::cos(direction_rad);

  const double north_m =
    offset_m *
    std::sin(direction_rad);

  return offsetGeoPointENU(
    origin,
    east_m,
    north_m);
}


geographic_msgs::msg::GeoPoint BoatBTNode::offsetGeoPointENU(
  const geographic_msgs::msg::GeoPoint & origin,
  const double east_m,
  const double north_m) const
{
  geographic_msgs::msg::GeoPoint target =
    origin;

  constexpr double metres_per_degree_lat =
    111320.0;

  const double latitude_rad =
    origin.latitude * M_PI / 180.0;

  double metres_per_degree_lon =
    metres_per_degree_lat *
    std::cos(latitude_rad);

  if (
    std::abs(metres_per_degree_lon) <
    1.0)
  {
    metres_per_degree_lon = 1.0;
  }

  target.latitude +=
    north_m /
    metres_per_degree_lat;

  target.longitude +=
    east_m /
    metres_per_degree_lon;

  return target;
}


bool BoatBTNode::sendBypassRequest(
  const geographic_msgs::msg::GeoPoint & target,
  const std::string & reason)
{
  if (
    !bypass_client_->service_is_ready())
  {
    RCLCPP_WARN(
      get_logger(),
      "Bypass service /mission/set_bypass_target "
      "is not available yet");

    return false;
  }

  auto request =
    std::make_shared<
    njord_msgs::srv::SetBypassTarget::Request>();

  request->target =
    target;

  request->reason =
    reason;

  bypass_client_->async_send_request(
    request,
    [this, reason](
      rclcpp::Client<
        njord_msgs::srv::SetBypassTarget>::
        SharedFuture future)
    {
      try {
        const auto response =
          future.get();

        if (response->success) {
          RCLCPP_INFO(
            get_logger(),
            "Bypass request accepted: %s - %s",
            reason.c_str(),
            response->message.c_str());
        }
        else {
          RCLCPP_WARN(
            get_logger(),
            "Bypass request rejected: %s - %s",
            reason.c_str(),
            response->message.c_str());
        }
      }
      catch (
        const std::exception & e)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Bypass service call failed: %s",
          e.what());
      }
    });

  RCLCPP_INFO(
    get_logger(),
    "Requested bypass target: "
    "reason=%s lat=%.8f lon=%.8f",
    reason.c_str(),
    target.latitude,
    target.longitude);

  return true;
}


bool BoatBTNode::requestCooldownExpired(
  const rclcpp::Time & last_time) const
{
  if (
    last_time.nanoseconds() == 0)
  {
    return true;
  }

  const double elapsed =
    (now() - last_time).seconds();

  return elapsed >=
    request_cooldown_sec_;
}
