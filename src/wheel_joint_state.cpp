#include "wheel_joint_state.hpp"

#include "wheel_speed.hpp"

#include <string>

namespace prior_map_localization
{

WheelJointState::WheelJointState(const rclcpp::NodeOptions & options)
: Node("wheel_joint_state", options),
  joint_name_(declare_parameter<std::string>("joint_name")),
  scale_(declare_parameter<double>("wheel_scale"))
{
  pub_ = create_publisher<sensor_msgs::msg::JointState>("~/output/joint_state", 20);
  sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odom", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry & msg) { on_odom(msg); });
}

void WheelJointState::on_odom(const nav_msgs::msg::Odometry & msg)
{
  const auto velocity = center_wheel_velocity(msg.twist.twist.linear.x, scale_);
  sensor_msgs::msg::JointState out;
  out.header.stamp = msg.header.stamp;
  out.name = {joint_name_};
  out.velocity = {velocity[0], velocity[1]};
  pub_->publish(out);
}

}  // namespace prior_map_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prior_map_localization::WheelJointState)
