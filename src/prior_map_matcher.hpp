#ifndef PRIOR_MAP_MATCHER_HPP_
#define PRIOR_MAP_MATCHER_HPP_

#include "pose_tracker.hpp"
#include "registration.hpp"
#include "scan.hpp"

#include <flux_wrapper/flux_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace prior_map_localization
{

class PriorMapMatcher : public rclcpp::Node
{
public:
  explicit PriorMapMatcher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  Registration make_registration();
  void load_map();
  void subscribe_points();
  void on_points(const RawScan & raw);
  void on_odom(const nav_msgs::msg::Odometry & msg);
  void on_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  void publish_pose(const Eigen::Matrix4f & pose, double stamp_sec, double fitness);
  std::optional<Eigen::Affine3f> base_from_sensor(const std::string & frame_id);
  void publish_status(
    const PreparedScan & scan, const Alignment & result, const Eigen::Matrix4f & seed,
    bool accepted, bool reinitialization);

  Registration registration_;
  std::string map_pcd_;
  std::string map_frame_;
  std::string base_frame_;
  ScanFilter filter_;
  double fitness_threshold_;
  double reinitialization_gap_sec_;
  PoseStd pose_std_;
  PoseTracker tracker_;

  std::optional<double> initial_pose_stamp_sec_;
  std::optional<Eigen::Affine3f> base_from_sensor_;
  bool map_loaded_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reinitialization_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  std::unique_ptr<flux_wrapper::Subscription<sensor_msgs::msg::PointCloud2>> points_sub_;
  rclcpp::TimerBase::SharedPtr load_timer_;
};

}  // namespace prior_map_localization

#endif  // PRIOR_MAP_MATCHER_HPP_
