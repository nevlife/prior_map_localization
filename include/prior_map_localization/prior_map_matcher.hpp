#ifndef PRIOR_MAP_LOCALIZATION__PRIOR_MAP_MATCHER_HPP_
#define PRIOR_MAP_LOCALIZATION__PRIOR_MAP_MATCHER_HPP_

#include "prior_map_localization/registration.hpp"
#include "prior_map_localization/scan.hpp"

#include <flux/subscription.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace prior_map_localization
{

class PriorMapMatcher : public rclcpp::Node
{
public:
  PriorMapMatcher();

  // Run after the thread is bound: building the target starts the OpenMP team.
  void load_map();

  // Null on the dds transport; on flux the caller's executor must serve it.
  std::shared_ptr<flux::Subscription> flux_subscription() const { return flux_sub_; }

private:
  struct TwistSample
  {
    double stamp_sec;
    Eigen::Vector3f linear;
    Eigen::Vector3f angular;
  };

  struct Alignment
  {
    Eigen::Matrix4f pose;
    bool converged;
    double fitness;
    double time_sec;
    int iterations;
  };

  void subscribe_points(const std::string & transport);
  void on_points(const RawScan & raw);
  void on_odom(const nav_msgs::msg::Odometry & msg);
  void on_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  Eigen::Matrix4f predict(double stamp_sec, double dt_sec) const;
  Alignment align(const Cloud::Ptr & cloud, const Eigen::Matrix4f & seed);
  void accept(const Eigen::Matrix4f & pose, double stamp_sec, double fitness);
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
  double max_prediction_dt_sec_;
  double reinitialization_gap_sec_;
  // [xy, z, roll and pitch, yaw] standard deviations: floor + per_fitness * fitness, capped.
  std::vector<double> std_floor_;
  std::vector<double> std_per_fitness_;
  std::vector<double> std_max_;

  std::optional<double> initial_pose_stamp_sec_;
  Eigen::Matrix4f predicted_pose_ = Eigen::Matrix4f::Identity();
  double predicted_stamp_sec_ = 0.0;
  Eigen::Matrix4f accepted_pose_ = Eigen::Matrix4f::Identity();
  double accepted_stamp_sec_ = 0.0;
  bool last_scan_accepted_ = false;
  std::optional<ScanMotion> motion_;
  int consecutive_rejects_ = 0;
  std::deque<TwistSample> twists_;
  std::optional<Eigen::Affine3f> base_from_sensor_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reinitialization_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr dds_sub_;
  std::shared_ptr<flux::Subscription> flux_sub_;
};

}  // namespace prior_map_localization

#endif  // PRIOR_MAP_LOCALIZATION__PRIOR_MAP_MATCHER_HPP_
