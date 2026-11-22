#ifndef WHEEL_TWIST_RELAY_HPP_
#define WHEEL_TWIST_RELAY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <array>
#include <string>

namespace prior_map_localization
{

// Hunter speed -> twist at rear_axle, the one point of an Ackermann base with no lateral velocity.
// Only the forward speed of the driver odometry is used; its steering yaw rate is not.
class WheelTwistRelay : public rclcpp::Node
{
public:
  explicit WheelTwistRelay(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_odom(const nav_msgs::msg::Odometry & msg);

  std::string twist_frame_;
  double scale_;
  std::array<double, 36> covariance_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

}  // namespace prior_map_localization

#endif  // WHEEL_TWIST_RELAY_HPP_
