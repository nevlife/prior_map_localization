#include "prior_map_localization/prior_map_matcher.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include "sensor_msgs/flux/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace prior_map_localization
{
namespace
{

using FluxCloud = sensor_msgs::flux_msg::PointCloud2;

// A reliable reader that keeps one scan drops it whenever the next starts arriving mid-fragment.
constexpr std::size_t kCloudQueueDepth = 5;
// About 20 s of 50 Hz odometry, longer than any prediction reaches back.
constexpr std::size_t kTwistHistory = 1024;
constexpr auto kThrottleMs = 5000;

double stamp_sec(const builtin_interfaces::msg::Time & t)
{
  return t.sec + t.nanosec * 1e-9;
}

builtin_interfaces::msg::Time to_stamp(double sec)
{
  return rclcpp::Time(static_cast<std::int64_t>(std::llround(sec * 1e9)));
}

double capped_std(double floor, double per_fitness, double max, double fitness)
{
  return std::clamp(floor + per_fitness * std::max(0.0, fitness), floor, max);
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue out;
  out.key = key;
  out.value = value;
  return out;
}

}  // namespace

PriorMapMatcher::PriorMapMatcher()
: Node("prior_map_matcher"),
  registration_(make_registration(*this)),
  map_pcd_(declare_parameter<std::string>("map_pcd")),
  map_frame_(declare_parameter<std::string>("map_frame")),
  base_frame_(declare_parameter<std::string>("base_frame")),
  filter_{
    declare_parameter<double>("filter.min_range"), declare_parameter<double>("filter.max_range"),
    declare_parameter<double>("filter.voxel_leaf_size"),
    declare_parameter<double>("filter.max_scan_duration_sec")},
  fitness_threshold_(declare_parameter<double>("fitness_threshold")),
  max_prediction_dt_sec_(declare_parameter<double>("max_prediction_dt_sec")),
  reinitialization_gap_sec_(declare_parameter<double>("reinitialization_gap_sec")),
  std_floor_(declare_parameter<std::vector<double>>("covariance.std_floor")),
  std_per_fitness_(declare_parameter<std::vector<double>>("covariance.std_per_fitness")),
  std_max_(declare_parameter<std::vector<double>>("covariance.std_max")),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output/pose", 10);
  reinitialization_pub_ =
    create_publisher<std_msgs::msg::Bool>("~/output/reinitialization_requested", 10);
  status_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/debug/alignment_status", 10);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odom", 50, [this](const nav_msgs::msg::Odometry & msg) { on_odom(msg); });
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/input/initialpose", 1,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped & msg) { on_initial_pose(msg); });
  subscribe_points(declare_parameter<std::string>("transport"));
}

void PriorMapMatcher::subscribe_points(const std::string & transport)
{
  if (transport == "flux") {
    flux_sub_ = flux::ros::create_subscription(
      *this, "~/input/points", FluxCloud::kFingerprint, flux::QoS{},
      [this](const flux::FrameView & frame) {
        const FluxCloud::View v{frame};
        FieldOffsets fields;
        for (std::size_t i = 0; i < v.fields__size(); ++i) {
          fields.emplace_back(v.fields(i).name(), v.fields(i).offset());
        }
        std::optional<RawScan> raw = raw_scan_fields(fields);
        if (!raw) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kThrottleMs, "scan lacks x/y/z/time");
          return;
        }
        raw->data = v.data().data();
        raw->count = std::size_t{v.width()} * v.height();
        raw->step = v.point_step();
        raw->stamp_sec = v.header__sec() + v.header__nanosec() * 1e-9;
        raw->frame_id = v.header__frame_id();
        on_points(*raw);
      });
    return;
  }
  if (transport != "dds") {
    throw std::invalid_argument("transport must be dds or flux, got " + transport);
  }
  // Large clouds go reliable: best effort loses a whole scan to one lost fragment.
  dds_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/points", rclcpp::QoS(kCloudQueueDepth).reliable(),
    [this](const sensor_msgs::msg::PointCloud2 & msg) {
      FieldOffsets fields;
      for (const auto & f : msg.fields) {
        fields.emplace_back(f.name, f.offset);
      }
      std::optional<RawScan> raw = raw_scan_fields(fields);
      if (!raw) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kThrottleMs, "scan lacks x/y/z/time");
        return;
      }
      raw->data = msg.data.data();
      raw->count = std::size_t{msg.width} * msg.height;
      raw->step = msg.point_step;
      raw->stamp_sec = stamp_sec(msg.header.stamp);
      raw->frame_id = msg.header.frame_id;
      on_points(*raw);
    });
}

void PriorMapMatcher::load_map()
{
  Cloud::Ptr map(new Cloud);
  if (pcl::io::loadPCDFile<Point>(map_pcd_, *map) != 0 || map->empty()) {
    throw std::runtime_error("cannot load the map " + map_pcd_);
  }
  const auto start = std::chrono::steady_clock::now();
  registration_.engine->setInputTarget(map);
  // The first align builds the search trees; a sparse probe pays for it here, not on a scan.
  Cloud::Ptr probe(new Cloud);
  for (std::size_t i = 0; i < map->size(); i += std::max<std::size_t>(1, map->size() / 100)) {
    probe->push_back((*map)[i]);
  }
  Cloud aligned;
  registration_.engine->setInputSource(probe);
  registration_.engine->align(aligned, Eigen::Matrix4f::Identity());
  RCLCPP_INFO(
    get_logger(), "%s target from %zu map points ready in %.1f s", registration_.method.c_str(),
    map->size(), std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
}

void PriorMapMatcher::on_odom(const nav_msgs::msg::Odometry & msg)
{
  const auto & v = msg.twist.twist;
  twists_.push_back(
    {stamp_sec(msg.header.stamp), Eigen::Vector3f(v.linear.x, v.linear.y, v.linear.z),
     Eigen::Vector3f(v.angular.x, v.angular.y, v.angular.z)});
  if (twists_.size() > kTwistHistory) {
    twists_.pop_front();
  }
}

void PriorMapMatcher::on_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  if (msg.header.frame_id != map_frame_) {
    RCLCPP_WARN(
      get_logger(), "initial pose in %s ignored, expected %s", msg.header.frame_id.c_str(),
      map_frame_.c_str());
    return;
  }
  Eigen::Isometry3d pose;
  tf2::fromMsg(msg.pose.pose, pose);
  const double stamp = stamp_sec(msg.header.stamp);
  initial_pose_stamp_sec_ = stamp;
  predicted_pose_ = accepted_pose_ = pose.matrix().cast<float>();
  predicted_stamp_sec_ = accepted_stamp_sec_ = stamp;
  last_scan_accepted_ = false;
  motion_.reset();
  consecutive_rejects_ = 0;
  RCLCPP_INFO(
    get_logger(), "initial pose (%.2f, %.2f)", msg.pose.pose.position.x, msg.pose.pose.position.y);
  pose_pub_->publish(msg);
  reinitialization_pub_->publish(std_msgs::msg::Bool());
}

void PriorMapMatcher::on_points(const RawScan & raw)
{
  if (!initial_pose_stamp_sec_) {
    return;
  }
  const std::optional<Eigen::Affine3f> sensor = base_from_sensor(raw.frame_id);
  if (!sensor) {
    return;
  }
  const PreparedScan scan = prepare_scan(raw, *sensor, motion_, filter_);
  if (scan.end_stamp_sec < *initial_pose_stamp_sec_) {
    return;
  }
  const double dt =
    std::clamp(scan.end_stamp_sec - predicted_stamp_sec_, 0.0, max_prediction_dt_sec_);
  const Eigen::Matrix4f seed = predict(scan.end_stamp_sec, dt);
  const Alignment result = align(scan.cloud, seed);
  const bool accepted = result.converged && result.fitness <= fitness_threshold_;
  if (accepted) {
    accept(result.pose, scan.end_stamp_sec, result.fitness);
  } else {
    // A rejected scan still moves the prediction on, so the next seed starts from here.
    predicted_pose_ = seed;
    predicted_stamp_sec_ = scan.end_stamp_sec;
    last_scan_accepted_ = false;
    ++consecutive_rejects_;
  }
  const bool reinitialization =
    scan.end_stamp_sec - accepted_stamp_sec_ >= reinitialization_gap_sec_;
  std_msgs::msg::Bool request;
  request.data = reinitialization;
  reinitialization_pub_->publish(request);
  publish_status(scan, result, seed, accepted, reinitialization);
}

// The odometry twist is in the body frame, so it moves the pose along its own axes.
Eigen::Matrix4f PriorMapMatcher::predict(double stamp_sec, double dt_sec) const
{
  const auto at_or_before = std::find_if(
    twists_.rbegin(), twists_.rend(),
    [stamp_sec](const TwistSample & s) { return s.stamp_sec <= stamp_sec; });
  if (at_or_before == twists_.rend() || dt_sec <= 0.0) {
    return predicted_pose_;
  }
  const auto dt = static_cast<float>(dt_sec);
  const Eigen::Vector3f w = at_or_before->angular * dt;
  Eigen::Affine3f pose(predicted_pose_);
  pose.translation() += pose.linear() * (at_or_before->linear * dt);
  pose.linear() = pose.linear() * (Eigen::AngleAxisf(w.x(), Eigen::Vector3f::UnitX()) *
                                   Eigen::AngleAxisf(w.y(), Eigen::Vector3f::UnitY()) *
                                   Eigen::AngleAxisf(w.z(), Eigen::Vector3f::UnitZ()))
                                    .toRotationMatrix();
  return pose.matrix();
}

PriorMapMatcher::Alignment PriorMapMatcher::align(
  const Cloud::Ptr & cloud, const Eigen::Matrix4f & seed)
{
  auto & engine = *registration_.engine;
  engine.setInputSource(cloud);
  Cloud aligned;
  const auto start = std::chrono::steady_clock::now();
  engine.align(aligned, seed);
  const double time_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  return {
    engine.getFinalTransformation(), engine.hasConverged(), engine.getFitnessScore(), time_sec,
    registration_.iterations()};
}

void PriorMapMatcher::accept(const Eigen::Matrix4f & pose, double stamp_sec, double fitness)
{
  // Deskew needs one sweep of motion, which only two accepted scans in a row measure.
  if (last_scan_accepted_) {
    const Eigen::Affine3f delta(accepted_pose_.inverse() * pose);
    motion_ = ScanMotion{
      Eigen::Quaternionf(delta.linear()).normalized(), delta.translation(),
      stamp_sec - accepted_stamp_sec_};
  }
  accepted_pose_ = predicted_pose_ = pose;
  accepted_stamp_sec_ = predicted_stamp_sec_ = stamp_sec;
  last_scan_accepted_ = true;
  consecutive_rejects_ = 0;

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = to_stamp(stamp_sec);
  msg.header.frame_id = map_frame_;
  msg.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
  const double xy = capped_std(std_floor_[0], std_per_fitness_[0], std_max_[0], fitness);
  const double z = capped_std(std_floor_[1], std_per_fitness_[1], std_max_[1], fitness);
  const double rp = capped_std(std_floor_[2], std_per_fitness_[2], std_max_[2], fitness);
  const double yaw = capped_std(std_floor_[3], std_per_fitness_[3], std_max_[3], fitness);
  msg.pose.covariance[0] = msg.pose.covariance[7] = xy * xy;
  msg.pose.covariance[14] = z * z;
  msg.pose.covariance[21] = msg.pose.covariance[28] = rp * rp;
  msg.pose.covariance[35] = yaw * yaw;
  pose_pub_->publish(msg);
}

std::optional<Eigen::Affine3f> PriorMapMatcher::base_from_sensor(const std::string & frame_id)
{
  // The sensor is rigidly mounted, so the first lookup holds for the run.
  if (!base_from_sensor_) {
    try {
      const auto t = tf_buffer_.lookupTransform(base_frame_, frame_id, tf2::TimePointZero);
      base_from_sensor_ = tf2::transformToEigen(t).cast<float>();
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMs, "%s", e.what());
    }
  }
  return base_from_sensor_;
}

void PriorMapMatcher::publish_status(
  const PreparedScan & scan, const Alignment & result, const Eigen::Matrix4f & seed, bool accepted,
  bool reinitialization)
{
  const Eigen::Matrix4f correction = seed.inverse() * result.pose;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "prior_map_localization/alignment";
  status.hardware_id = registration_.method;
  status.level = accepted ? diagnostic_msgs::msg::DiagnosticStatus::OK
                          : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = accepted            ? "ok"
                   : !result.converged ? "registration_not_converged"
                                       : "fitness_score_over_threshold_rejected";
  status.values = {
    kv("fitness_score", std::to_string(result.fitness)),
    kv("alignment_time_sec", std::to_string(result.time_sec)),
    kv("iterations", std::to_string(result.iterations)),
    kv("filtered_point_count", std::to_string(scan.cloud->size())),
    kv("scan_time_duration_sec", std::to_string(scan.duration_sec)),
    kv("continuous_time_deskew_status", to_string(scan.deskew)),
    kv("correction_translation_m", std::to_string(correction.block<3, 1>(0, 3).norm())),
    kv(
      "correction_yaw_deg",
      std::to_string(std::atan2(correction(1, 0), correction(0, 0)) * 180.0 / M_PI)),
    kv("accepted_gap_sec", std::to_string(scan.end_stamp_sec - accepted_stamp_sec_)),
    kv("consecutive_rejected_updates", std::to_string(consecutive_rejects_)),
    kv("reinitialization_requested", reinitialization ? "true" : "false")};
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = to_stamp(scan.end_stamp_sec);
  array.header.frame_id = base_frame_;
  array.status.push_back(status);
  status_pub_->publish(array);
}

}  // namespace prior_map_localization
