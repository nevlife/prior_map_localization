#include "gps_cov_floor.hpp"

#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <algorithm>

namespace prior_map_localization
{

GpsCovFloor::GpsCovFloor(const rclcpp::NodeOptions & options)
: Node("gps_cov_floor", options),
  rtk_floor_(declare_parameter<double>("rtk_sigma_floor")),
  float_floor_(declare_parameter<double>("float_sigma_floor"))
{
  pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("~/output/fix", 10);
  sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "~/input/fix", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::NavSatFix & msg) { on_fix(msg); });
}

void GpsCovFloor::on_fix(sensor_msgs::msg::NavSatFix msg)
{
  using Status = sensor_msgs::msg::NavSatStatus;
  if (msg.status.status < Status::STATUS_FIX) {
    return;
  }
  const bool rtk = msg.status.status >= Status::STATUS_GBAS_FIX;
  const double floor = rtk ? rtk_floor_ : float_floor_;
  const auto reported = msg.position_covariance;
  msg.position_covariance.fill(0.0);
  msg.position_covariance[0] = std::max(reported[0], floor * floor);
  msg.position_covariance[4] = std::max(reported[4], floor * floor);
  msg.position_covariance[8] = std::max(reported[8], 4.0 * floor * floor);
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  pub_->publish(msg);
}

}  // namespace prior_map_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prior_map_localization::GpsCovFloor)
