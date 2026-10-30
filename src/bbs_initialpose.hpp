#ifndef PRIOR_MAP_LOCALIZATION__BBS_INITIALPOSE_HPP_
#define PRIOR_MAP_LOCALIZATION__BBS_INITIALPOSE_HPP_

#include "bbs/bbs_branch_and_bound.hpp"
#include "corner_features.hpp"

#include <flux/subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace prior_map_localization
{

struct Pose2
{
  double x;
  double y;
  double yaw;
};

// /initialpose from one scan: a 2D branch-and-bound search of x, y and the full yaw circle
// inside the GPS circle, against the map bundle's corner layer.
class BbsInitialPose : public rclcpp::Node
{
public:
  explicit BbsInitialPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  struct Vote
  {
    Pose2 map_from_odom;
    double stamp_sec;
  };

  void load_maps(const std::string & corner_pcd, const std::string & map_pcd);
  void subscribe_points(const std::string & transport);
  template<typename Field>
  void on_points(
    const std::vector<Field> & fields, const std::uint8_t * data, std::size_t count,
    std::uint32_t step, double stamp, const std::string & frame_id);
  bool vote(const Pose2 & map_from_odom, double stamp);
  std::vector<std::array<double, 2>> scan_xy(
    const std::vector<Eigen::Vector3f> & corners, const Eigen::Affine3f & base_from_sensor) const;
  std::optional<double> ground_z(double x, double y) const;
  void publish(const Pose2 & pose, double z, double stamp);
  std::optional<Eigen::Affine3f> base_from_sensor(const std::string & frame_id);

  std::string map_frame_;
  std::string base_frame_;
  CornerParams corners_;
  double resolution_;
  double yaw_resolution_;
  int pyramid_depth_;
  double min_scan_range_;
  double max_scan_range_;
  int max_scan_points_;
  double min_score_;
  double radius_sigma_scale_;
  double min_radius_;
  double max_radius_;
  double speed_margin_;
  int required_votes_;
  double vote_xy_tolerance_;
  double vote_yaw_tolerance_;
  double vote_timeout_sec_;
  double ground_radius_;
  double base_height_;
  double xy_sigma_;
  double yaw_sigma_;

  bbs::Grid occupancy_;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::vector<Eigen::Vector3f> ground_;

  bool armed_ = true;
  std::optional<nav_msgs::msg::Odometry> gps_;
  std::optional<Pose2> odom_;
  std::deque<Vote> votes_;
  std::optional<Eigen::Affine3f> base_from_sensor_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reinit_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr dds_sub_;
  std::shared_ptr<flux::Subscription> flux_sub_;
};

}  // namespace prior_map_localization

#endif  // PRIOR_MAP_LOCALIZATION__BBS_INITIALPOSE_HPP_
