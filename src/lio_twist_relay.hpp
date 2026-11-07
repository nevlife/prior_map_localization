#ifndef LIO_TWIST_RELAY_HPP_
#define LIO_TWIST_RELAY_HPP_

#include "gyro_bias.hpp"
#include "lio_twist.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <optional>
#include <string>

namespace prior_map_localization
{

// FAST-LIO odometry -> base frame twist, and the IMU with the gyro bias that FAST-LIO observes.
//
// FAST-LIO publishes the IMU pose in its start frame and no twist, so the twist is the motion
// between consecutive scans, moved from the IMU to the base frame. robot_localization keeps no gyro
// bias, so the yaw rate goes to the EKF as the IMU minus the bias GyroBias tracks.
class LioTwistRelay : public rclcpp::Node
{
public:
  explicit LioTwistRelay(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  bool has_mount();
  void on_imu(sensor_msgs::msg::Imu msg);
  void on_odom(const nav_msgs::msg::Odometry & msg);
  void publish(const rclcpp::Time & stamp, const Eigen::Vector3d & v, const Eigen::Vector3d & w);

  std::string base_frame_;
  std::string imu_frame_;
  std::array<double, 36> covariance_;
  GyroBias gyro_;
  double check_window_sec_;
  double hold_sec_;

  std::optional<ImuMount> mount_;
  std::optional<rclcpp::Time> prev_stamp_;
  LioPose prev_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

}  // namespace prior_map_localization

#endif  // LIO_TWIST_RELAY_HPP_
