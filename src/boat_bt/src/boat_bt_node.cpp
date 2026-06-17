#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class BoatBTNode : public rclcpp::Node
{
public:
  BoatBTNode()
  : Node("boat_bt_node")
  {
    cmd_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10);

    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/gps",
        10,
        std::bind(
          &BoatBTNode::odom_callback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "boat_bt_node started");
  }

private:
  void odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    (void)msg;

    RCLCPP_INFO_ONCE(
      get_logger(),
      "Received first odom message");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<BoatBTNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}