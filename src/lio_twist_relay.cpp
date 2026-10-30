#include "lio_twist_relay.hpp"

#include <Eigen/Geometry>

#include <cmath>

namespace prior_map_localization
{
namespace
{

double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec + stamp.nanosec * 1e-9;
}

double radians(double deg) { return deg * M_PI / 180.0; }

Eigen::Matrix3d rotation(const geometry_msgs::msg::Quaternion & q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
}

}  // namespace

LioTwistRelay::LioTwistRelay(const rclcpp::NodeOptions & options)
: Node("lio_twist_relay", options),
  imu_frame_(declare_parameter<std::string>("imu_frame")),
  gyro_(GyroBias::Params{
    declare_parameter<double>("bias_time_constant_sec"),
    declare_parameter<double>("check_window_sec"),
    radians(declare_parameter<double>("check_threshold_deg")),
    declare_parameter<double>("check_turn_ratio"),
    declare_parameter<double>("hold_sec")}),
  check_window_sec_(get_parameter("check_window_sec").as_double()),
  hold_sec_(get_parameter("hold_sec").as_double()),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  const double vx_sigma = declare_parameter<double>("vx_sigma");
  const double vy_sigma = declare_parameter<double>("vy_sigma");
  const double wz_sigma = declare_parameter<double>("wz_sigma");
  covariance_.fill(0.0);
  covariance_[0] = vx_sigma * vx_sigma;
  covariance_[7] = vy_sigma * vy_sigma;
  covariance_[35] = wz_sigma * wz_sigma;
  for (const int i : {14, 21, 28}) {
    covariance_[i] = 1e3;
  }
  twist_pub_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("~/output/twist", 20);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/output/imu", 50);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odom", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry & msg) { on_odom(msg); });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu & msg) { on_imu(msg); });
}

bool LioTwistRelay::has_mount()
{
  if (base_rotation_) {
    return true;
  }
  // tf_static can arrive after the first FAST-LIO pose or IMU sample.
  if (!tf_buffer_.canTransform("base_link", imu_frame_, tf2::TimePointZero)) {
    return false;
  }
  const auto tf = tf_buffer_.lookupTransform("base_link", imu_frame_, tf2::TimePointZero);
  base_rotation_ = rotation(tf.transform.rotation);
  const auto & t = tf.transform.translation;
  imu_to_base_ = -Eigen::Vector3d(t.x, t.y, t.z);
  return true;
}

void LioTwistRelay::on_imu(sensor_msgs::msg::Imu msg)
{
  if (!has_mount()) {
    return;
  }
  auto & av = msg.angular_velocity;
  const Eigen::Vector3d w = *base_rotation_ * Eigen::Vector3d(av.x, av.y, av.z);
  gyro_.add_imu(seconds(msg.header.stamp), w.z());
  const Eigen::Vector3d bias = base_rotation_->transpose() * Eigen::Vector3d(0, 0, gyro_.bias());
  av.x -= bias.x();
  av.y -= bias.y();
  av.z -= bias.z();
  imu_pub_->publish(msg);
}

void LioTwistRelay::on_odom(const nav_msgs::msg::Odometry & msg)
{
  if (!has_mount()) {
    return;
  }
  const rclcpp::Time stamp(msg.header.stamp);
  const Eigen::Matrix3d rot = rotation(msg.pose.pose.orientation);
  const auto & p = msg.pose.pose.position;
  const Eigen::Vector3d position(p.x, p.y, p.z);
  if (prev_) {
    const double dt = (stamp - prev_->stamp).seconds();
    const Eigen::Vector3d v_imu = prev_->rotation.transpose() * (position - prev_->position) / dt;
    const Eigen::AngleAxisd turn(prev_->rotation.transpose() * rot);
    const Eigen::Vector3d w = *base_rotation_ * (turn.angle() * turn.axis() / dt);
    const Eigen::Vector3d v = *base_rotation_ * v_imu + w.cross(imu_to_base_);
    const double t1 = stamp.seconds();
    if (const std::optional<double> gap = gyro_.add_lio(t1 - dt, t1, w.z() * dt)) {
      RCLCPP_WARN(
        get_logger(), "LIO yaw disagrees with the IMU by %.2f deg over %.1f s; withholding LIO "
        "for %.0f s", *gap * 180.0 / M_PI, check_window_sec_, hold_sec_);
    }
    if (!gyro_.lio_withheld(t1)) {
      // The twist is the mean over the scan interval, so it belongs to its middle.
      publish(prev_->stamp + (stamp - prev_->stamp) * 0.5, v, w);
    }
  }
  prev_ = Pose{stamp, rot, position};
}

void LioTwistRelay::publish(
  const rclcpp::Time & stamp, const Eigen::Vector3d & v, const Eigen::Vector3d & w)
{
  geometry_msgs::msg::TwistWithCovarianceStamped out;
  out.header.stamp = stamp;
  out.header.frame_id = "base_link";
  out.twist.twist.linear.x = v.x();
  out.twist.twist.linear.y = v.y();
  out.twist.twist.linear.z = v.z();
  out.twist.twist.angular.x = w.x();
  out.twist.twist.angular.y = w.y();
  out.twist.twist.angular.z = w.z();
  out.twist.covariance = covariance_;
  twist_pub_->publish(out);
}

}  // namespace prior_map_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prior_map_localization::LioTwistRelay)
