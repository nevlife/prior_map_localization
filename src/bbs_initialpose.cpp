#include "bbs_initialpose.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include "sensor_msgs/flux/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace prior_map_localization
{
namespace
{

using FluxCloud = sensor_msgs::flux_msg::PointCloud2;
constexpr auto throttle_ms = 5000;
// The search fixes x, y and yaw only; z, roll and pitch get loose fixed variances.
constexpr double initial_z_variance = 1.0;
constexpr double initial_roll_pitch_variance = 0.01;

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double stamp_sec(const builtin_interfaces::msg::Time & t)
{
  return t.sec + t.nanosec * 1e-9;
}

struct RingFields
{
  std::uint32_t x;
  std::uint32_t y;
  std::uint32_t z;
  std::uint32_t ring;
};

template <typename Field>
std::optional<RingFields> ring_fields(const std::vector<Field> & fields)
{
  std::optional<std::uint32_t> x, y, z, ring;
  for (const auto & [name, offset] : fields) {
    if (name == "x") {
      x = offset;
    }
    if (name == "y") {
      y = offset;
    }
    if (name == "z") {
      z = offset;
    }
    if (name == "ring") {
      ring = offset;
    }
  }
  if (!x || !y || !z || !ring) {
    return std::nullopt;
  }
  return RingFields{*x, *y, *z, *ring};
}

std::vector<RingPoint> read_points(
  const std::uint8_t * data, std::size_t count, std::uint32_t step, const RingFields & f)
{
  std::vector<RingPoint> points(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint8_t * p = data + i * step;
    std::memcpy(&points[i].x, p + f.x, sizeof(float));
    std::memcpy(&points[i].y, p + f.y, sizeof(float));
    std::memcpy(&points[i].z, p + f.z, sizeof(float));
    std::memcpy(&points[i].ring, p + f.ring, sizeof(std::uint16_t));
  }
  return points;
}

std::vector<Eigen::Vector3f> load_xyz(const std::string & path)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(path, cloud) != 0 || cloud.empty()) {
    throw std::runtime_error("cannot load " + path);
  }
  std::vector<Eigen::Vector3f> out;
  out.reserve(cloud.size());
  for (const auto & p : cloud) {
    out.emplace_back(p.x, p.y, p.z);
  }
  return out;
}

}  // namespace

BbsInitialPose::BbsInitialPose(const rclcpp::NodeOptions & options)
: Node("bbs_initialpose", options),
  map_frame_(declare_parameter<std::string>("map_frame")),
  base_frame_(declare_parameter<std::string>("base_frame")),
  corners_{
    static_cast<int>(declare_parameter<int>("corners.n_scan")),
    static_cast<int>(declare_parameter<int>("corners.horizon_scan")),
    declare_parameter<double>("corners.min_range"), declare_parameter<double>("corners.max_range"),
    declare_parameter<double>("corners.edge_threshold")},
  search_{
    declare_parameter<double>("search.resolution"),
    declare_parameter<double>("search.yaw_resolution_deg") * M_PI / 180.0,
    static_cast<int>(declare_parameter<int>("search.pyramid_depth")),
    declare_parameter<double>("search.min_scan_range"),
    declare_parameter<double>("search.max_scan_range"),
    static_cast<int>(declare_parameter<int>("search.max_scan_points"))},
  min_score_(declare_parameter<double>("search.min_score")),
  gps_radius_{
    declare_parameter<double>("gps.radius_sigma_scale"),
    declare_parameter<double>("gps.min_radius"), declare_parameter<double>("gps.max_radius"),
    declare_parameter<double>("gps.speed_margin")},
  ground_radius_(declare_parameter<double>("ground_radius")),
  base_height_(declare_parameter<double>("base_height")),
  xy_sigma_(declare_parameter<double>("initial_xy_sigma")),
  yaw_sigma_(declare_parameter<double>("initial_yaw_sigma_deg") * M_PI / 180.0),
  grid_(load_xyz(declare_parameter<std::string>("corner_pcd")), search_.resolution),
  ground_(load_xyz(declare_parameter<std::string>("map_pcd")), ground_radius_),
  votes_(PoseVote::Params{
    static_cast<int>(declare_parameter<int>("vote.required")),
    declare_parameter<double>("vote.xy_tolerance"),
    declare_parameter<double>("vote.yaw_tolerance_deg") * M_PI / 180.0,
    declare_parameter<double>("vote.timeout_sec")}),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  RCLCPP_INFO(
    get_logger(), "corner grid %dx%d at %.2f m", grid_.width(), grid_.height(), search_.resolution);
  initialpose_pub_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output/initialpose", 1);
  set_pose_pub_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output/set_pose", 1);
  gps_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/gps", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry & msg) { gps_ = msg; });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odom", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry & msg) {
      odom_ = Pose2{
        msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(msg.pose.pose.orientation)};
    });
  reinit_sub_ = create_subscription<std_msgs::msg::Bool>(
    "~/input/reinitialization_requested", 10, [this](const std_msgs::msg::Bool & msg) {
      if (msg.data && !armed_) {
        RCLCPP_WARN(get_logger(), "reinitialization requested, re-arming");
        armed_ = true;
        votes_.clear();
      }
    });
  subscribe_points(declare_parameter<std::string>("transport"));
}

void BbsInitialPose::subscribe_points(const std::string & transport)
{
  if (transport == "flux") {
    flux_sub_ = flux::create_subscription(
      *this, "~/input/points", flux::QoS{},
      [this](const flux::Frame & frame) {
        const FluxCloud::View v{frame};
        std::vector<std::pair<std::string_view, std::uint32_t>> fields;
        for (std::size_t i = 0; i < v.fields__size(); ++i) {
          fields.emplace_back(v.fields(i).name(), v.fields(i).offset());
        }
        on_points(
          fields, v.data().data(), std::size_t{v.width()} * v.height(), v.point_step(),
          v.header__sec() + v.header__nanosec() * 1e-9, std::string(v.header__frame_id()));
      },
      {}, FluxCloud::kFingerprint);
    flux_sub_->add_to_callback_group(*this);
    return;
  }
  if (transport != "dds") {
    throw std::invalid_argument("transport must be dds or flux, got " + transport);
  }
  // Depth 1: a search outlasts several scans, and only the newest is worth searching.
  dds_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/points", rclcpp::QoS(1).reliable(), [this](const sensor_msgs::msg::PointCloud2 & msg) {
      std::vector<std::pair<std::string, std::uint32_t>> fields;
      for (const auto & f : msg.fields) {
        fields.emplace_back(f.name, f.offset);
      }
      on_points(
        fields, msg.data.data(), std::size_t{msg.width} * msg.height, msg.point_step,
        stamp_sec(msg.header.stamp), msg.header.frame_id);
    });
}

template <typename Field>
void BbsInitialPose::on_points(
  const std::vector<Field> & fields, const std::uint8_t * data, std::size_t count,
  std::uint32_t step, double stamp, const std::string & frame_id)
{
  if (!armed_ || !gps_ || !odom_) {
    return;
  }
  const std::optional<RingFields> f = ring_fields(fields);
  if (!f) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), throttle_ms, "scan lacks x/y/z/ring");
    return;
  }
  const std::optional<Eigen::Affine3f> sensor = base_from_sensor(frame_id);
  if (!sensor) {
    return;
  }
  const auto start = std::chrono::steady_clock::now();
  const std::vector<std::array<double, 2>> scan =
    scan_xy(extract_corners(read_points(data, count, step, *f), corners_), *sensor, search_);
  const double radius = search_radius(
    gps_radius_, gps_->pose.covariance[0], gps_->pose.covariance[7], stamp,
    stamp_sec(gps_->header.stamp));
  const std::optional<GridCrop> crop = grid_.crop(
    gps_->pose.pose.position.x, gps_->pose.pose.position.y, radius, search_.max_scan_range);
  if (!crop) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), throttle_ms, "GPS is off the map");
    return;
  }
  const std::optional<BbsMatch> match = grid_.search(*crop, scan, search_);
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  if (!match) {
    return;
  }
  const Pose2 & map_from_base = match->map_from_base;
  RCLCPP_INFO(
    get_logger(),
    "BBS (%.2f, %.2f) yaw %.1f deg score %.3f (next %.3f) scan %zu radius %.1f m in %.0f ms",
    map_from_base.x, map_from_base.y, map_from_base.yaw * 180.0 / M_PI, match->score,
    match->runner_up, scan.size(), radius, elapsed_ms);
  if (match->score < min_score_) {
    votes_.clear();
    return;
  }
  const Pose2 odom_now = *odom_;
  const Pose2 map_from_odom = compose(map_from_base, inverse(odom_now));
  if (!votes_.add(map_from_odom, stamp)) {
    return;
  }
  const Pose2 pose = compose(map_from_odom, odom_now);
  const std::optional<double> z = ground_.at(pose.x, pose.y);
  if (!z) {
    RCLCPP_WARN(get_logger(), "no map points under (%.2f, %.2f)", pose.x, pose.y);
    votes_.clear();
    return;
  }
  publish(pose, *z + base_height_, stamp);
  armed_ = false;
  votes_.clear();
}

void BbsInitialPose::publish(const Pose2 & pose, double z, double stamp)
{
  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp = rclcpp::Time(static_cast<std::int64_t>(std::llround(stamp * 1e9)));
  out.header.frame_id = map_frame_;
  out.pose.pose.position.x = pose.x;
  out.pose.pose.position.y = pose.y;
  out.pose.pose.position.z = z;
  out.pose.pose.orientation.z = std::sin(pose.yaw / 2);
  out.pose.pose.orientation.w = std::cos(pose.yaw / 2);
  out.pose.covariance[0] = out.pose.covariance[7] = xy_sigma_ * xy_sigma_;
  out.pose.covariance[14] = initial_z_variance;
  out.pose.covariance[21] = out.pose.covariance[28] = initial_roll_pitch_variance;
  out.pose.covariance[35] = yaw_sigma_ * yaw_sigma_;
  initialpose_pub_->publish(out);
  set_pose_pub_->publish(out);
  RCLCPP_INFO(
    get_logger(), "initial pose (%.2f, %.2f, z=%.2f) yaw %.1f deg", pose.x, pose.y,
    out.pose.pose.position.z, pose.yaw * 180.0 / M_PI);
}

std::optional<Eigen::Affine3f> BbsInitialPose::base_from_sensor(const std::string & frame_id)
{
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

}  // namespace prior_map_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prior_map_localization::BbsInitialPose)
