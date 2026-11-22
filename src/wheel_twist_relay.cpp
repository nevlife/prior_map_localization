#include "wheel_twist_relay.hpp"

#include <string>

namespace prior_map_localization
{

WheelTwistRelay::WheelTwistRelay(const rclcpp::NodeOptions & options)
: Node("wheel_twist_relay", options),
  twist_frame_(declare_parameter<std::string>("twist_frame")),
  scale_(declare_parameter<double>("wheel_scale"))
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
  sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odom", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry & msg) { on_odom(msg); });
}

void WheelTwistRelay::on_odom(const nav_msgs::msg::Odometry & msg)
{
  geometry_msgs::msg::TwistWithCovarianceStamped out;
  out.header.stamp = msg.header.stamp;
  out.header.frame_id = twist_frame_;
  out.twist.twist.linear.x = msg.twist.twist.linear.x * scale_;
  out.twist.covariance = covariance_;
  pub_->publish(out);
}

}  // namespace prior_map_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prior_map_localization::WheelTwistRelay)
