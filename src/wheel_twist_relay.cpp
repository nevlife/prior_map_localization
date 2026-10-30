#include "wheel_twist_relay.hpp"

namespace prior_map_localization
{

WheelTwistRelay::WheelTwistRelay(const rclcpp::NodeOptions & options)
: Node("wheel_twist_relay", options), scale_(declare_parameter<double>("wheel_scale"))
{
  const double vx_sigma = declare_parameter<double>("vx_sigma");
  const double vy_sigma = declare_parameter<double>("vy_sigma");
  covariance_.fill(0.0);
  covariance_[0] = vx_sigma * vx_sigma;
  covariance_[7] = vy_sigma * vy_sigma;
  for (const int i : {14, 21, 28, 35}) {
    covariance_[i] = 1e3;
  }
  pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("~/output/twist", 20);
  sub_ = create_subscription<std_msgs::msg::Float64>(
    "~/input/velocity", rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float64 & msg) { on_speed(msg); });
}

void WheelTwistRelay::on_speed(const std_msgs::msg::Float64 & msg)
{
  geometry_msgs::msg::TwistWithCovarianceStamped out;
  out.header.stamp = now();
  out.header.frame_id = "rear_axle";
  out.twist.twist.linear.x = msg.data * scale_;
  out.twist.covariance = covariance_;
  pub_->publish(out);
}

}  // namespace prior_map_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prior_map_localization::WheelTwistRelay)
