#ifndef WHEEL_JOINT_STATE_HPP_
#define WHEEL_JOINT_STATE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <string>

namespace prior_map_localization
{

// Hunter wheel odometry -> the JointState that MINS reads as a centered wheel ([omega, v]).
// MINS keeps only messages whose first name is one of its wheel sub_topics.
class WheelJointState : public rclcpp::Node
{
public:
  explicit WheelJointState(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_odom(const nav_msgs::msg::Odometry & msg);

  std::string joint_name_;
  double scale_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

}  // namespace prior_map_localization

#endif  // WHEEL_JOINT_STATE_HPP_
