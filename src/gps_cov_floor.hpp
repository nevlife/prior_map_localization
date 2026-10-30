#ifndef PRIOR_MAP_LOCALIZATION__GPS_COV_FLOOR_HPP_
#define PRIOR_MAP_LOCALIZATION__GPS_COV_FLOOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace prior_map_localization
{

// Per-fix-type covariance floor: float fixes report sub-metre sigma, and the map's georeference
// has 1.2 m RMSE, so GPS cannot agree with the PCD map better than that.
class GpsCovFloor : public rclcpp::Node
{
public:
  explicit GpsCovFloor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_fix(sensor_msgs::msg::NavSatFix msg);

  double rtk_floor_;
  double float_floor_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

}  // namespace prior_map_localization

#endif  // PRIOR_MAP_LOCALIZATION__GPS_COV_FLOOR_HPP_
