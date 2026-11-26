#ifndef BBS_INITIALPOSE_HPP_
#define BBS_INITIALPOSE_HPP_

#include "bbs_search.hpp"
#include "corner_features.hpp"

#include <Eigen/Geometry>
#include <flux_wrapper/flux_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace prior_map_localization
{

// /initialpose from one scan: a 2D branch-and-bound search of x, y and the full yaw circle
// inside the GPS circle, against the map bundle's corner layer.
class BbsInitialPose : public rclcpp::Node
{
public:
  explicit BbsInitialPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_points(const flux_wrapper::View<sensor_msgs::msg::PointCloud2> & cloud);
  void publish(const Pose2 & pose, double z, double stamp);
  std::optional<Eigen::Affine3f> base_from_sensor(const std::string & frame_id);

  std::string map_frame_;
  std::string base_frame_;
  CornerParams corners_;
  BbsParams search_;
  double min_score_;
  GpsRadiusParams gps_radius_;
  double ground_radius_;
  double base_height_;
  double xy_sigma_;
  double yaw_sigma_;

  CornerGrid grid_;
  GroundHeight ground_;
  PoseVote votes_;

  bool armed_ = true;
  std::optional<nav_msgs::msg::Odometry> gps_;
  std::optional<Pose2> odom_;
  std::optional<Eigen::Affine3f> base_from_sensor_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reinit_sub_;
  std::unique_ptr<flux_wrapper::Subscription<sensor_msgs::msg::PointCloud2>> points_sub_;
};

}  // namespace prior_map_localization

#endif  // BBS_INITIALPOSE_HPP_
