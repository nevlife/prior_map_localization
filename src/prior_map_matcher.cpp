#include "prior_map_matcher.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include "sensor_msgs/flux/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace prior_map_localization
{
namespace
{

using FluxCloud = sensor_msgs::flux_msg::PointCloud2;

// A reliable reader that keeps one scan drops it whenever the next starts arriving mid-fragment.
constexpr std::size_t cloud_queue_depth = 5;
constexpr auto throttle_ms = 5000;

double stamp_sec(const builtin_interfaces::msg::Time & t)
{
  return t.sec + t.nanosec * 1e-9;
}

builtin_interfaces::msg::Time to_stamp(double sec)
{
  return rclcpp::Time(static_cast<std::int64_t>(std::llround(sec * 1e9)));
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue out;
  out.key = key;
  out.value = value;
  return out;
}

}  // namespace

PriorMapMatcher::PriorMapMatcher(const rclcpp::NodeOptions & options)
: Node("prior_map_matcher", options),
  registration_(make_registration()),
  map_pcd_(declare_parameter<std::string>("map_pcd")),
  map_frame_(declare_parameter<std::string>("map_frame")),
  base_frame_(declare_parameter<std::string>("base_frame")),
  filter_{
    declare_parameter<double>("filter.min_range"), declare_parameter<double>("filter.max_range"),
    declare_parameter<double>("filter.voxel_leaf_size"),
    declare_parameter<double>("filter.max_scan_duration_sec")},
  fitness_threshold_(declare_parameter<double>("fitness_threshold")),
  reinitialization_gap_sec_(declare_parameter<double>("reinitialization_gap_sec")),
  pose_std_{
    declare_parameter<std::vector<double>>("covariance.std_floor"),
    declare_parameter<std::vector<double>>("covariance.std_per_fitness"),
    declare_parameter<std::vector<double>>("covariance.std_max")},
  tracker_(declare_parameter<double>("max_prediction_dt_sec")),
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
  // The container binds the group thread before its first callback, and building the target starts
  // the OpenMP team, so the map loads in that callback for the team to inherit the stage.
  load_timer_ = create_wall_timer(std::chrono::seconds(0), [this]() {
    load_timer_->cancel();
    load_map();
  });
}

// Declares the registration.* parameters of the selected method only.
Registration PriorMapMatcher::make_registration()
{
  const auto method = declare_parameter<std::string>("registration.method");
  const auto threads = static_cast<int>(declare_parameter<int>("registration.num_threads"));
  const auto max_iterations =
    static_cast<int>(declare_parameter<int>("registration.max_iterations"));
  if (method == "ndt_omp") {
    const std::string p = "registration.ndt_omp.";
    return make_ndt_omp(
      {declare_parameter<double>(p + "resolution"), declare_parameter<double>(p + "step_size"),
       declare_parameter<double>(p + "transformation_epsilon"),
       declare_parameter<bool>(p + "line_search")},
      threads, max_iterations);
  }
  if (method == "small_gicp") {
    const std::string p = "registration.small_gicp.";
    return make_small_gicp(
      {declare_parameter<std::string>(p + "type"),
       static_cast<int>(declare_parameter<int>(p + "num_neighbors")),
       declare_parameter<double>(p + "max_correspondence_distance"),
       declare_parameter<double>(p + "voxel_resolution"),
       declare_parameter<double>(p + "translation_epsilon"),
       declare_parameter<double>(p + "rotation_epsilon")},
      threads, max_iterations);
  }
  throw std::invalid_argument("registration.method must be ndt_omp or small_gicp, got " + method);
}

void PriorMapMatcher::subscribe_points(const std::string & transport)
{
  if (transport == "flux") {
    subscribe_flux_points();
    return;
  }
  if (transport != "dds") {
    throw std::invalid_argument("transport must be dds or flux, got " + transport);
  }
  subscribe_dds_points();
}

void PriorMapMatcher::subscribe_flux_points()
{
  flux_sub_ = flux::create_subscription(
    *this, "~/input/points", flux::QoS{},
    [this](const flux::Frame & frame) {
      const FluxCloud::View v{frame};
      FieldOffsets fields;
      for (std::size_t i = 0; i < v.fields__size(); ++i) {
        fields.emplace_back(v.fields(i).name(), v.fields(i).offset());
      }
      std::optional<RawScan> raw = raw_scan_fields(fields);
      if (!raw) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), throttle_ms, "scan lacks x/y/z/time");
        return;
      }
      raw->data = v.data().data();
      raw->count = std::size_t{v.width()} * v.height();
      raw->step = v.point_step();
      raw->stamp_sec = v.header__sec() + v.header__nanosec() * 1e-9;
      raw->frame_id = v.header__frame_id();
      on_points(*raw);
    },
    {}, FluxCloud::kFingerprint);
  flux_sub_->add_to_callback_group(*this);
}

void PriorMapMatcher::subscribe_dds_points()
{
  // Large clouds go reliable: best effort loses a whole scan to one lost fragment.
  dds_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/points", rclcpp::QoS(cloud_queue_depth).reliable(),
    [this](const sensor_msgs::msg::PointCloud2 & msg) {
      FieldOffsets fields;
      for (const auto & f : msg.fields) {
        fields.emplace_back(f.name, f.offset);
      }
      std::optional<RawScan> raw = raw_scan_fields(fields);
      if (!raw) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), throttle_ms, "scan lacks x/y/z/time");
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
  set_target(registration_, map);
  RCLCPP_INFO(
    get_logger(), "%s target from %zu map points ready in %.1f s", registration_.method.c_str(),
    map->size(), std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
  map_loaded_ = true;
}

void PriorMapMatcher::on_odom(const nav_msgs::msg::Odometry & msg)
{
  const auto & v = msg.twist.twist;
  tracker_.add_twist(
    {stamp_sec(msg.header.stamp), Eigen::Vector3f(v.linear.x, v.linear.y, v.linear.z),
     Eigen::Vector3f(v.angular.x, v.angular.y, v.angular.z)});
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
  tracker_.reset(pose.matrix().cast<float>(), stamp);
  RCLCPP_INFO(
    get_logger(), "initial pose (%.2f, %.2f)", msg.pose.pose.position.x, msg.pose.pose.position.y);
  pose_pub_->publish(msg);
  reinitialization_pub_->publish(std_msgs::msg::Bool());
}

void PriorMapMatcher::on_points(const RawScan & raw)
{
  if (!map_loaded_ || !initial_pose_stamp_sec_) {
    return;
  }
  const std::optional<Eigen::Affine3f> sensor = base_from_sensor(raw.frame_id);
  if (!sensor) {
    return;
  }
  const PreparedScan scan = prepare_scan(raw, *sensor, tracker_.motion(), filter_);
  if (scan.end_stamp_sec < *initial_pose_stamp_sec_) {
    return;
  }
  const Eigen::Matrix4f seed = tracker_.seed(scan.end_stamp_sec);
  const Alignment result = align(registration_, scan.cloud, seed);
  const bool accepted = result.converged && result.fitness <= fitness_threshold_;
  if (accepted) {
    tracker_.accept(result.pose, scan.end_stamp_sec);
    publish_pose(result.pose, scan.end_stamp_sec, result.fitness);
  } else {
    tracker_.reject(seed, scan.end_stamp_sec);
  }
  const bool reinitialization =
    scan.end_stamp_sec - tracker_.accepted_stamp_sec() >= reinitialization_gap_sec_;
  std_msgs::msg::Bool request;
  request.data = reinitialization;
  reinitialization_pub_->publish(request);
  publish_status(scan, result, seed, accepted, reinitialization);
}

void PriorMapMatcher::publish_pose(const Eigen::Matrix4f & pose, double stamp_sec, double fitness)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = to_stamp(stamp_sec);
  msg.header.frame_id = map_frame_;
  msg.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
  msg.pose.covariance = pose_covariance(pose_std_, fitness);
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
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), throttle_ms, "%s", e.what());
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
  status.message = accepted              ? "ok"
                   : scan.cloud->empty() ? "empty_scan"
                   : !result.converged   ? "registration_not_converged"
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
    kv("accepted_gap_sec", std::to_string(scan.end_stamp_sec - tracker_.accepted_stamp_sec())),
    kv("consecutive_rejected_updates", std::to_string(tracker_.consecutive_rejects())),
    kv("reinitialization_requested", reinitialization ? "true" : "false")};
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = to_stamp(scan.end_stamp_sec);
  array.header.frame_id = base_frame_;
  array.status.push_back(status);
  status_pub_->publish(array);
}

}  // namespace prior_map_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prior_map_localization::PriorMapMatcher)
