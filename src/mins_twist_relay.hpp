#ifndef MINS_TWIST_RELAY_HPP_
#define MINS_TWIST_RELAY_HPP_

#include "lio_twist.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <optional>
#include <string>

namespace prior_map_localization
{

// MINS odometry -> base frame twist for robot_localization.
//
// MINS publishes the IMU pose in its own world frame, the IMU velocity in that world frame and the
// bias-corrected angular velocity in the IMU frame, under frame names the TF tree does not have.
class MinsTwistRelay : public rclcpp::Node
{
public:
  explicit MinsTwistRelay(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  bool has_mount();
  void on_odom(const nav_msgs::msg::Odometry & msg);

  std::string base_frame_;
  std::string imu_frame_;
  std::optional<ImuMount> mount_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

}  // namespace prior_map_localization

#endif  // MINS_TWIST_RELAY_HPP_
