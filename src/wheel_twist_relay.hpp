#ifndef PRIOR_MAP_LOCALIZATION__WHEEL_TWIST_RELAY_HPP_
#define PRIOR_MAP_LOCALIZATION__WHEEL_TWIST_RELAY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>

namespace prior_map_localization
{

// Hunter speed -> twist at rear_axle, the one point of an Ackermann base with no lateral velocity.
// The driver message has no header, so the stamp is the reception time.
class WheelTwistRelay : public rclcpp::Node
{
public:
  explicit WheelTwistRelay(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_speed(const std_msgs::msg::Float64 & msg);

  double scale_;
  std::array<double, 36> covariance_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
};

}  // namespace prior_map_localization

#endif  // PRIOR_MAP_LOCALIZATION__WHEEL_TWIST_RELAY_HPP_
