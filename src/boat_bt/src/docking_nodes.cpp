#include "boat_bt/boat_bt_node.hpp"
#include "boat_bt/docking_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <utility>


namespace
{

double normalize_angle(double angle)
{
  return std::atan2(
    std::sin(angle),
    std::cos(angle));
}

}  // namespace


namespace boat_bt
{

ExecuteDockingNode::ExecuteDockingNode(
  const std::string & name,
  const BT::NodeConfig & config,
  TickCallback tick_callback,
  HaltCallback halt_callback)
: BT::StatefulActionNode(name, config),
  tick_callback_(std::move(tick_callback)),
  halt_callback_(std::move(halt_callback))
{
}


BT::PortsList ExecuteDockingNode::providedPorts()
{
  return {};
}


BT::NodeStatus ExecuteDockingNode::onStart()
{
  return tick_callback_();
}


BT::NodeStatus ExecuteDockingNode::onRunning()
{
  return tick_callback_();
}


void ExecuteDockingNode::onHalted()
{
  halt_callback_();
}

}  // namespace boat_bt


void BoatBTNode::dock_target_callback(
  const njord_msgs::msg::DockTarget::SharedPtr msg)
{
  dock_target_received_ = true;
  last_dock_target_time_ = now();

  dock_target_available_ =
    msg->detected &&
    !msg->occupied &&
    msg->confidence >= docking_min_confidence_;

  if (!dock_target_available_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Dock target rejected: detected=%s, occupied=%s, "
      "confidence=%.2f, required_confidence=%.2f",
      msg->detected ? "true" : "false",
      msg->occupied ? "true" : "false",
      msg->confidence,
      docking_min_confidence_);

    return;
  }

  /*
   * DockTarget is expressed relative to base_link:
   *
   * opening_center.x = distance forward from the boat
   * opening_center.y = distance left from the boat
   * heading          = desired entry heading relative to the boat
   */
  dock_opening_center_ =
    msg->opening_center;

  dock_heading_ =
    normalize_angle(msg->heading);

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


BT::NodeStatus BoatBTNode::executeDockingController()
{
  if (docking_complete_) {
    publishDockingCommand(0.0, 0.0);
    requestCompetitionCompletion();

    return competition_completion_confirmed_
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::RUNNING;
  }

  /*
   * Never continue moving using an invalid or outdated target.
   * The task stays RUNNING while waiting for perception to recover.
   */
  if (!dockTargetFresh()) {
    if (docking_state_ != DockingState::WAITING_FOR_TARGET) {
      RCLCPP_WARN(
        get_logger(),
        "Dock target lost or stale. "
        "Stopping and waiting for perception.");
    }

    setDockingState(
      DockingState::WAITING_FOR_TARGET);

    publishDockingCommand(0.0, 0.0);

    return BT::NodeStatus::RUNNING;
  }

  const double target_x =
    dock_opening_center_.x;

  const double target_y =
    dock_opening_center_.y;

  const double target_distance =
    std::hypot(target_x, target_y);

  const double target_bearing =
    normalize_angle(
    std::atan2(target_y, target_x));

  const double heading_error =
    normalize_angle(dock_heading_);

  /*
   * WAITING_FOR_TARGET
   *
   * A valid target is available. Decide whether alignment is needed before
   * beginning the main approach.
   */
  if (docking_state_ == DockingState::WAITING_FOR_TARGET) {
    if (
      std::abs(target_bearing) >
      docking_alignment_tolerance_rad_)
    {
      setDockingState(
        DockingState::ALIGNING);
    }
    else {
      setDockingState(
        DockingState::APPROACHING);
    }
  }

  /*
   * ALIGNING
   *
   * A small positive forward speed is used because the physical boat may
   * require water flow over a rudder in order to turn.
   */
  if (docking_state_ == DockingState::ALIGNING) {
    if (
      std::abs(target_bearing) <=
      docking_alignment_tolerance_rad_)
    {
      setDockingState(
        DockingState::APPROACHING);
    }
    else {
      const double yaw_command =
        docking_bearing_gain_ *
        target_bearing;

      publishDockingCommand(
        docking_alignment_speed_mps_,
        yaw_command);

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Docking ALIGNING: distance=%.2f m, "
        "bearing=%.2f rad, "
        "command=(speed %.2f, yaw %.2f)",
        target_distance,
        target_bearing,
        docking_alignment_speed_mps_,
        std::clamp(
          yaw_command,
          -docking_max_yaw_rate_radps_,
          docking_max_yaw_rate_radps_));

      return BT::NodeStatus::RUNNING;
    }
  }

  /*
   * APPROACHING
   *
   * Steering combines the bearing toward the opening center with the
   * desired dock-entry heading.
   */
  if (docking_state_ == DockingState::APPROACHING) {
    const bool close_to_opening =
      target_distance <=
      docking_entry_trigger_distance_m_;

    const bool laterally_aligned =
      std::abs(target_y) <=
      docking_lateral_tolerance_m_;

    if (
      close_to_opening &&
      laterally_aligned)
    {
      final_entry_start_time_ = now();

      setDockingState(
        DockingState::FINAL_ENTRY);
    }
    else {
      const double yaw_command =
        docking_bearing_gain_ *
        target_bearing +
        docking_heading_gain_ *
        heading_error;

      publishDockingCommand(
        docking_approach_speed_mps_,
        yaw_command);

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Docking APPROACHING: distance=%.2f m, "
        "lateral=%.2f m, bearing=%.2f rad, "
        "heading_error=%.2f rad, "
        "command=(speed %.2f, yaw %.2f)",
        target_distance,
        target_y,
        target_bearing,
        heading_error,
        docking_approach_speed_mps_,
        std::clamp(
          yaw_command,
          -docking_max_yaw_rate_radps_,
          docking_max_yaw_rate_radps_));

      return BT::NodeStatus::RUNNING;
    }
  }

  /*
   * FINAL_ENTRY
   *
   * Move slowly through the opening for a fixed period. The detector may
   * lose sight of the opening as the boat enters the berth, so completion
   * does not depend on seeing the opening center reach exactly zero.
   */
  if (docking_state_ == DockingState::FINAL_ENTRY) {
    const double final_entry_elapsed =
      (now() - final_entry_start_time_).seconds();

    if (
      final_entry_elapsed >=
      docking_final_entry_duration_sec_)
    {
      docking_complete_ = true;

      setDockingState(
        DockingState::DOCKED);

      publishDockingCommand(0.0, 0.0);

      RCLCPP_INFO(
        get_logger(),
        "Docking manoeuvre completed. "
        "Waiting for Competition Manager acknowledgement.");

      requestCompetitionCompletion();

      return competition_completion_confirmed_
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::RUNNING;
    }

    const double yaw_command =
      docking_bearing_gain_ *
      target_bearing +
      docking_heading_gain_ *
      heading_error;

    publishDockingCommand(
      docking_final_speed_mps_,
      yaw_command);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Docking FINAL_ENTRY: elapsed=%.2f / %.2f s, "
      "bearing=%.2f rad, heading_error=%.2f rad, "
      "command=(speed %.2f, yaw %.2f)",
      final_entry_elapsed,
      docking_final_entry_duration_sec_,
      target_bearing,
      heading_error,
      docking_final_speed_mps_,
      std::clamp(
        yaw_command,
        -docking_max_yaw_rate_radps_,
        docking_max_yaw_rate_radps_));

    return BT::NodeStatus::RUNNING;
  }

  if (docking_state_ == DockingState::DOCKED) {
    docking_complete_ = true;
    publishDockingCommand(0.0, 0.0);
    requestCompetitionCompletion();

    return competition_completion_confirmed_
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::RUNNING;
  }

  publishDockingCommand(0.0, 0.0);

  return BT::NodeStatus::RUNNING;
}


void BoatBTNode::setDockingState(
  DockingState new_state)
{
  if (new_state == docking_state_) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Docking state: %s -> %s",
    dockingStateName(docking_state_),
    dockingStateName(new_state));

  docking_state_ = new_state;
}


void BoatBTNode::resetDockingController()
{
  docking_complete_ = false;

  docking_state_ =
    DockingState::WAITING_FOR_TARGET;

  final_entry_start_time_ =
    rclcpp::Time(
    0,
    0,
    get_clock()->get_clock_type());

  publishDockingCommand(0.0, 0.0);

  RCLCPP_INFO(
    get_logger(),
    "Docking controller reset");
}


void BoatBTNode::publishDockingCommand(
  double forward_speed,
  double yaw_rate)
{
  geometry_msgs::msg::Twist command;

  /*
   * The first competition implementation is forward-only.
   */
  command.linear.x =
    std::max(0.0, forward_speed);

  command.angular.z =
    std::clamp(
    yaw_rate,
    -docking_max_yaw_rate_radps_,
    docking_max_yaw_rate_radps_);

  cmd_pub_->publish(command);
}


void BoatBTNode::requestCompetitionCompletion()
{
  if (
    competition_completion_confirmed_ ||
    competition_completion_request_sent_)
  {
    return;
  }

  if (!competition_complete_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Waiting for /competition/complete service");

    return;
  }

  competition_completion_request_sent_ = true;

  auto request =
    std::make_shared<std_srvs::srv::Trigger::Request>();

  competition_complete_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
    {
      try {
        const auto response = future.get();

        if (response->success) {
          competition_completion_confirmed_ = true;

          RCLCPP_INFO(
            get_logger(),
            "Competition Manager confirmed docking completion: %s",
            response->message.c_str());

          return;
        }

        competition_completion_request_sent_ = false;

        RCLCPP_ERROR(
          get_logger(),
          "Competition Manager rejected docking completion: %s",
          response->message.c_str());
      }
      catch (const std::exception & error) {
        competition_completion_request_sent_ = false;

        RCLCPP_ERROR(
          get_logger(),
          "Calling /competition/complete failed: %s",
          error.what());
      }
    });

  RCLCPP_INFO(
    get_logger(),
    "Docking completion request sent to Competition Manager");
}


bool BoatBTNode::dockTargetFresh() const
{
  if (
    !dock_target_received_ ||
    !dock_target_available_)
  {
    return false;
  }

  const double target_age =
    (now() - last_dock_target_time_).seconds();

  return
    target_age >= 0.0 &&
    target_age <= docking_target_timeout_sec_;
}


const char * BoatBTNode::dockingStateName(
  DockingState state) const
{
  switch (state) {
    case DockingState::WAITING_FOR_TARGET:
      return "WAITING_FOR_TARGET";

    case DockingState::ALIGNING:
      return "ALIGNING";

    case DockingState::APPROACHING:
      return "APPROACHING";

    case DockingState::FINAL_ENTRY:
      return "FINAL_ENTRY";

    case DockingState::DOCKED:
      return "DOCKED";

    default:
      return "UNKNOWN";
  }
}
