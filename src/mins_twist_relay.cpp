#include "mins_twist_relay.hpp"

#include <Eigen/Geometry>

#include <string>

namespace prior_map_localization
{
namespace
{

Eigen::Matrix3d rotation(const geometry_msgs::msg::Quaternion & q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
}

Eigen::Vector3d vector(const geometry_msgs::msg::Vector3 & v)
{
  return {v.x, v.y, v.z};
}

}  // namespace

MinsTwistRelay::MinsTwistRelay(const rclcpp::NodeOptions & options)
: Node("mins_twist_relay", options),
  base_frame_(declare_parameter<std::string>("base_frame")),
  imu_frame_(declare_parameter<std::string>("imu_frame")),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("~/output/twist", 20);
  sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odom", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry & msg) { on_odom(msg); });
}

bool MinsTwistRelay::has_mount()
{
  if (mount_) {
    return true;
  }
  // tf_static can arrive after the first MINS state.
  if (!tf_buffer_.canTransform(base_frame_, imu_frame_, tf2::TimePointZero)) {
    return false;
  }
  const auto tf = tf_buffer_.lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
  const auto & t = tf.transform.translation;
  mount_ = ImuMount{rotation(tf.transform.rotation), -Eigen::Vector3d(t.x, t.y, t.z)};
  return true;
}

void MinsTwistRelay::on_odom(const nav_msgs::msg::Odometry & msg)
{
  if (!has_mount()) {
    return;
  }
  const Eigen::Matrix3d imu_rotation = rotation(msg.pose.pose.orientation);
  const BaseTwist twist = base_twist(
    *mount_, imu_rotation, vector(msg.twist.twist.linear), vector(msg.twist.twist.angular));
  Eigen::Matrix3d world_covariance;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      world_covariance(r, c) = msg.twist.covariance[6 * r + c];
    }
  }
  const Eigen::Matrix3d covariance = base_covariance(*mount_, imu_rotation, world_covariance);

  geometry_msgs::msg::TwistWithCovarianceStamped out;
  out.header.stamp = msg.header.stamp;
  out.header.frame_id = base_frame_;
  out.twist.twist.linear.x = twist.linear.x();
  out.twist.twist.linear.y = twist.linear.y();
  out.twist.twist.linear.z = twist.linear.z();
  out.twist.twist.angular.x = twist.angular.x();
  out.twist.twist.angular.y = twist.angular.y();
  out.twist.twist.angular.z = twist.angular.z();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out.twist.covariance[6 * r + c] = covariance(r, c);
    }
  }
  // MINS gives the angular covariance as an isotropic gyro noise, the same in every frame.
  for (int i = 3; i < 6; ++i) {
    out.twist.covariance[7 * i] = msg.twist.covariance[7 * i];
  }
  pub_->publish(out);
}

}  // namespace prior_map_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prior_map_localization::MinsTwistRelay)
