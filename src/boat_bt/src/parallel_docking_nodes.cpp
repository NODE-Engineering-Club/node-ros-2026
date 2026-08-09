/*
 * Parallel-docking controller (Task 3.2).
 *
 * UNVERIFIED — this whole file has never been exercised against a real
 * wall, on the bench or in the water. It exists because
 * docking_parallel_mission's orchestration (transit legs, retry, task
 * clearing) was already built and tested, and simple_boat.xml's
 * DockingParallelTask was an explicit <AlwaysSuccess/> placeholder
 * pending this controller. See wall_detector_node.py's module docstring
 * for the equivalent caveat on the perception side.
 *
 * The control law is a direct adaptation of docking_nodes.cpp's
 * executeDockingController(): both are "steer toward a target point in
 * base_link using blended bearing+heading correction, in phases separated
 * by distance/angle thresholds". The two tasks differ only in what the
 * target point and desired final heading *mean*:
 *
 *   Task 3.1 (ExecuteDocking):    target = U-opening center,
 *                                 heading = inward entry heading
 *   Task 3.2 (ExecuteDockingParallel): target = standoff aim point beside
 *                                 the wall's midpoint,
 *                                 heading = wall-parallel heading
 *
 * WallTarget.aim_point/heading are published in exactly the same
 * base_link (x=forward, y=left) convention as DockTarget.opening_center/
 * heading specifically so this reuse is possible — see WallTarget.msg.
 */

#include "boat_bt/boat_bt_node.hpp"
#include "boat_bt/parallel_docking_nodes.hpp"

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

ExecuteDockingParallelNode::ExecuteDockingParallelNode(
  const std::string & name,
  const BT::NodeConfig & config,
  TickCallback tick_callback,
  HaltCallback halt_callback)
: BT::StatefulActionNode(name, config),
  tick_callback_(std::move(tick_callback)),
  halt_callback_(std::move(halt_callback))
{
}


BT::PortsList ExecuteDockingParallelNode::providedPorts()
{
  return {};
}


BT::NodeStatus ExecuteDockingParallelNode::onStart()
{
  return tick_callback_();
}


BT::NodeStatus ExecuteDockingParallelNode::onRunning()
{
  return tick_callback_();
}


void ExecuteDockingParallelNode::onHalted()
{
  halt_callback_();
}

}  // namespace boat_bt


void BoatBTNode::wall_target_callback(
  const njord_msgs::msg::WallTarget::SharedPtr msg)
{
  wall_target_received_ = true;
  last_wall_target_time_ = now();

  wall_target_available_ =
    msg->detected &&
    !msg->occupied &&
    msg->confidence >= docking_parallel_min_confidence_;

  if (!wall_target_available_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Wall target rejected: detected=%s, occupied=%s, "
      "confidence=%.2f, required_confidence=%.2f",
      msg->detected ? "true" : "false",
      msg->occupied ? "true" : "false",
      msg->confidence,
      docking_parallel_min_confidence_);

    return;
  }

  /*
   * WallTarget is expressed relative to base_link, same convention as
   * DockTarget:
   *
   * aim_point.x = distance forward from the boat
   * aim_point.y = distance left from the boat
   * heading     = desired final (wall-parallel) heading relative to the boat
   */
  wall_aim_point_ =
    msg->aim_point;

  wall_heading_ =
    normalize_angle(msg->heading);

  wall_length_ =
    msg->length;

  wall_confidence_ =
    msg->confidence;

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "Free wall target: aim=(%.2f, %.2f), "
    "heading=%.2f rad, length=%.2f m, "
    "confidence=%.2f",
    wall_aim_point_.x,
    wall_aim_point_.y,
    wall_heading_,
    wall_length_,
    wall_confidence_);
}


BT::NodeStatus BoatBTNode::executeDockingParallelController()
{
  if (docking_parallel_complete_) {
    publishDockingParallelCommand(0.0, 0.0);
    requestCompetitionCompletion();

    return competition_completion_confirmed_
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::RUNNING;
  }

  /*
   * Perception is required while finding and closing on the wall. Once
   * alongside, the detector may lose the wall (own hull occluding the
   * LiDAR's view along it), so HOLD and REVERSE must continue without
   * requiring a fresh target -- same reasoning as docking's DOCKED state.
   */
  const bool wall_target_required =
    docking_parallel_state_ != DockingParallelState::DOCKED;

  if (wall_target_required && !wallTargetFresh()) {
    publishDockingParallelCommand(0.0, 0.0);

    const bool approach_in_progress =
      docking_parallel_state_ == DockingParallelState::ALIGNING ||
      docking_parallel_state_ == DockingParallelState::APPROACHING;

    if (!approach_in_progress) {
      docking_parallel_target_loss_active_ = false;

      setDockingParallelState(
        DockingParallelState::WAITING_FOR_TARGET);

      return BT::NodeStatus::RUNNING;
    }

    if (!docking_parallel_target_loss_active_) {
      docking_parallel_target_loss_active_ = true;
      docking_parallel_target_loss_start_time_ = now();

      RCLCPP_WARN(
        get_logger(),
        "Wall target lost during %s. "
        "Stopping for up to %.1f seconds while waiting for "
        "target reacquisition.",
        dockingParallelStateName(docking_parallel_state_),
        docking_parallel_reacquire_timeout_sec_);

      return BT::NodeStatus::RUNNING;
    }

    const double target_loss_elapsed =
      (now() - docking_parallel_target_loss_start_time_).seconds();

    if (
      target_loss_elapsed <
      docking_parallel_reacquire_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Wall target still unavailable during %s: "
        "elapsed=%.2f / %.2f s",
        dockingParallelStateName(docking_parallel_state_),
        target_loss_elapsed,
        docking_parallel_reacquire_timeout_sec_);

      return BT::NodeStatus::RUNNING;
    }

    RCLCPP_ERROR(
      get_logger(),
      "Wall target was not reacquired within %.1f seconds. "
      "Resetting parallel-docking approach and retrying target selection.",
      docking_parallel_reacquire_timeout_sec_);

    docking_parallel_target_loss_active_ = false;

    setDockingParallelState(
      DockingParallelState::WAITING_FOR_TARGET);

    return BT::NodeStatus::RUNNING;
  }

  if (docking_parallel_target_loss_active_) {
    const double target_loss_elapsed =
      (now() - docking_parallel_target_loss_start_time_).seconds();

    docking_parallel_target_loss_active_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Wall target reacquired after %.2f seconds. "
      "Resuming %s.",
      target_loss_elapsed,
      dockingParallelStateName(docking_parallel_state_));
  }

  const double target_x =
    wall_aim_point_.x;

  const double target_y =
    wall_aim_point_.y;

  const double target_distance =
    std::hypot(target_x, target_y);

  const double target_bearing =
    normalize_angle(
    std::atan2(target_y, target_x));

  const double heading_error =
    normalize_angle(wall_heading_);

  /*
   * Same staleness guard as docking's docking_steering_hold_sec_ -- see
   * that field's comment in boat_bt_node.hpp for why.
   */
  const double target_age =
    (now() - last_wall_target_time_).seconds();

  const bool steering_command_stale =
    target_age > docking_parallel_steering_hold_sec_;

  /*
   * WAITING_FOR_TARGET
   *
   * A valid target is available. Decide whether alignment is needed before
   * beginning the main approach.
   */
  if (docking_parallel_state_ == DockingParallelState::WAITING_FOR_TARGET) {
    if (
      std::abs(target_bearing) >
      docking_parallel_alignment_tolerance_rad_)
    {
      setDockingParallelState(
        DockingParallelState::ALIGNING);
    }
    else {
      setDockingParallelState(
        DockingParallelState::APPROACHING);
    }
  }

  /*
   * ALIGNING
   *
   * A small positive forward speed is used because the physical boat may
   * require water flow over a rudder in order to turn.
   */
  if (docking_parallel_state_ == DockingParallelState::ALIGNING) {
    if (
      std::abs(target_bearing) <=
      docking_parallel_alignment_tolerance_rad_)
    {
      setDockingParallelState(
        DockingParallelState::APPROACHING);
    }
    else if (steering_command_stale) {
      publishDockingParallelCommand(0.0, 0.0);

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "DockingParallel ALIGNING: holding, waiting on a fresh target "
        "(last reading %.2f s old)",
        target_age);

      return BT::NodeStatus::RUNNING;
    }
    else {
      const double yaw_command =
        docking_parallel_bearing_gain_ *
        target_bearing;

      publishDockingParallelCommand(
        docking_parallel_alignment_speed_mps_,
        yaw_command);

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "DockingParallel ALIGNING: distance=%.2f m, "
        "bearing=%.2f rad, "
        "command=(speed %.2f, yaw %.2f)",
        target_distance,
        target_bearing,
        docking_parallel_alignment_speed_mps_,
        std::clamp(
          yaw_command,
          -docking_parallel_max_yaw_rate_radps_,
          docking_parallel_max_yaw_rate_radps_));

      return BT::NodeStatus::RUNNING;
    }
  }

  /*
   * APPROACHING
   *
   * Steering combines the bearing toward the standoff aim point with the
   * desired wall-parallel heading. Transitions to FINAL_APPROACH once both
   * close to the aim point and roughly parallel to the wall -- committing
   * to the final creep while still significantly angled would risk
   * clipping the wall with the bow or stern instead of the hull side.
   */
  if (docking_parallel_state_ == DockingParallelState::APPROACHING) {
    const bool close_to_aim_point =
      target_distance <=
      docking_parallel_approach_trigger_distance_m_;

    const bool roughly_parallel =
      std::abs(heading_error) <=
      docking_parallel_alignment_tolerance_rad_;

    if (
      close_to_aim_point &&
      roughly_parallel)
    {
      final_approach_start_time_ = now();

      setDockingParallelState(
        DockingParallelState::FINAL_APPROACH);
    }
    else if (steering_command_stale) {
      publishDockingParallelCommand(0.0, 0.0);

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "DockingParallel APPROACHING: holding, waiting on a fresh target "
        "(last reading %.2f s old)",
        target_age);

      return BT::NodeStatus::RUNNING;
    }
    else {
      const double yaw_command =
        docking_parallel_bearing_gain_ *
        target_bearing +
        docking_parallel_heading_gain_ *
        heading_error;

      publishDockingParallelCommand(
        docking_parallel_approach_speed_mps_,
        yaw_command);

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "DockingParallel APPROACHING: distance=%.2f m, "
        "bearing=%.2f rad, heading_error=%.2f rad, "
        "command=(speed %.2f, yaw %.2f)",
        target_distance,
        target_bearing,
        heading_error,
        docking_parallel_approach_speed_mps_,
        std::clamp(
          yaw_command,
          -docking_parallel_max_yaw_rate_radps_,
          docking_parallel_max_yaw_rate_radps_));

      return BT::NodeStatus::RUNNING;
    }
  }

  /*
   * FINAL_APPROACH
   *
   * Move slowly toward the wall for a fixed period, closing the remaining
   * standoff gap. The detector may lose the wall as the boat's own hull
   * starts to occlude the LiDAR's view along it, so completion does not
   * depend on seeing the aim point reach exactly zero.
   */
  if (docking_parallel_state_ == DockingParallelState::FINAL_APPROACH) {
    const double final_approach_elapsed =
      (now() - final_approach_start_time_).seconds();

    if (
      final_approach_elapsed >=
      docking_parallel_final_approach_duration_sec_)
    {
      docking_parallel_hold_start_time_ = now();
      docking_parallel_reverse_started_ = false;

      setDockingParallelState(
        DockingParallelState::DOCKED);

      publishDockingParallelCommand(0.0, 0.0);

      RCLCPP_INFO(
        get_logger(),
        "Parallel-docking approach completed. Holding position for "
        "%.1f seconds.",
        docking_parallel_hold_duration_sec_);

      return BT::NodeStatus::RUNNING;
    }

    if (steering_command_stale) {
      publishDockingParallelCommand(0.0, 0.0);

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "DockingParallel FINAL_APPROACH: holding, waiting on a fresh target "
        "(last reading %.2f s old)",
        target_age);

      return BT::NodeStatus::RUNNING;
    }

    const double yaw_command =
      docking_parallel_bearing_gain_ *
      target_bearing +
      docking_parallel_heading_gain_ *
      heading_error;

    publishDockingParallelCommand(
      docking_parallel_final_speed_mps_,
      yaw_command);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "DockingParallel FINAL_APPROACH: elapsed=%.2f / %.2f s, "
      "bearing=%.2f rad, heading_error=%.2f rad, "
      "command=(speed %.2f, yaw %.2f)",
      final_approach_elapsed,
      docking_parallel_final_approach_duration_sec_,
      target_bearing,
      heading_error,
      docking_parallel_final_speed_mps_,
      std::clamp(
        yaw_command,
        -docking_parallel_max_yaw_rate_radps_,
        docking_parallel_max_yaw_rate_radps_));

    return BT::NodeStatus::RUNNING;
  }

  if (docking_parallel_state_ == DockingParallelState::DOCKED) {
    if (!docking_parallel_reverse_started_) {
      const double hold_elapsed =
        (now() - docking_parallel_hold_start_time_).seconds();

      publishDockingParallelCommand(0.0, 0.0);

      if (hold_elapsed < docking_parallel_hold_duration_sec_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "DockingParallel HOLDING: elapsed=%.2f / %.2f s",
          hold_elapsed,
          docking_parallel_hold_duration_sec_);

        return BT::NodeStatus::RUNNING;
      }

      docking_parallel_reverse_started_ = true;
      docking_parallel_reverse_start_time_ = now();

      RCLCPP_INFO(
        get_logger(),
        "Parallel-docking hold completed. Reversing off the wall for "
        "%.1f seconds.",
        docking_parallel_reverse_duration_sec_);
    }

    const double reverse_elapsed =
      (now() - docking_parallel_reverse_start_time_).seconds();

    if (reverse_elapsed < docking_parallel_reverse_duration_sec_) {
      publishDockingParallelCommand(
        docking_parallel_reverse_speed_mps_,
        0.0);

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "DockingParallel REVERSING: elapsed=%.2f / %.2f s, speed=%.2f",
        reverse_elapsed,
        docking_parallel_reverse_duration_sec_,
        docking_parallel_reverse_speed_mps_);

      return BT::NodeStatus::RUNNING;
    }

    publishDockingParallelCommand(0.0, 0.0);
    docking_parallel_complete_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Parallel-docking local sequence completed: hold, reverse and "
      "final stop.");

    requestCompetitionCompletion();

    return competition_completion_confirmed_
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::RUNNING;
  }

  publishDockingParallelCommand(0.0, 0.0);

  return BT::NodeStatus::RUNNING;
}


void BoatBTNode::setDockingParallelState(
  DockingParallelState new_state)
{
  if (new_state == docking_parallel_state_) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "DockingParallel state: %s -> %s",
    dockingParallelStateName(docking_parallel_state_),
    dockingParallelStateName(new_state));

  docking_parallel_state_ = new_state;
}


void BoatBTNode::resetDockingParallelController()
{
  docking_parallel_complete_ = false;

  docking_parallel_state_ =
    DockingParallelState::WAITING_FOR_TARGET;

  final_approach_start_time_ =
    rclcpp::Time(
    0,
    0,
    get_clock()->get_clock_type());

  docking_parallel_hold_start_time_ =
    rclcpp::Time(
    0,
    0,
    get_clock()->get_clock_type());

  docking_parallel_reverse_start_time_ =
    rclcpp::Time(
    0,
    0,
    get_clock()->get_clock_type());

  docking_parallel_target_loss_start_time_ =
    rclcpp::Time(
    0,
    0,
    get_clock()->get_clock_type());

  docking_parallel_target_loss_active_ = false;
  docking_parallel_reverse_started_ = false;

  publishDockingParallelCommand(0.0, 0.0);

  RCLCPP_INFO(
    get_logger(),
    "DockingParallel controller reset");
}


void BoatBTNode::publishDockingParallelCommand(
  double forward_speed,
  double yaw_rate)
{
  geometry_msgs::msg::Twist command;

  command.linear.x =
    forward_speed;

  command.angular.z =
    std::clamp(
    yaw_rate,
    -docking_parallel_max_yaw_rate_radps_,
    docking_parallel_max_yaw_rate_radps_);

  cmd_pub_->publish(command);
}


bool BoatBTNode::wallTargetFresh() const
{
  if (
    !wall_target_received_ ||
    !wall_target_available_)
  {
    return false;
  }

  const double target_age =
    (now() - last_wall_target_time_).seconds();

  return
    target_age >= 0.0 &&
    target_age <= docking_parallel_target_timeout_sec_;
}


const char * BoatBTNode::dockingParallelStateName(
  DockingParallelState state) const
{
  switch (state) {
    case DockingParallelState::WAITING_FOR_TARGET:
      return "WAITING_FOR_TARGET";

    case DockingParallelState::ALIGNING:
      return "ALIGNING";

    case DockingParallelState::APPROACHING:
      return "APPROACHING";

    case DockingParallelState::FINAL_APPROACH:
      return "FINAL_APPROACH";

    case DockingParallelState::DOCKED:
      return "DOCKED";

    default:
      return "UNKNOWN";
  }
}
